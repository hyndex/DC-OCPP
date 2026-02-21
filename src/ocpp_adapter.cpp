// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "tie_gating.hpp"
#include "error_catalog.hpp"
#include "evse_security_file.hpp"
#include "maintenance_manager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <sqlite3.h>
#include <nlohmann/json.hpp>

#include <everest/logging.hpp>
#include <ocpp/common/types.hpp>
#include <ocpp/v16/ocpp_enums.hpp>

namespace charger {

namespace fs = std::filesystem;

namespace {

constexpr uint8_t HLC_STAGE_AUTHORIZATION = 5; // PLC HLC stage enum: WAIT_AUTHORIZATION
constexpr uint8_t HLC_STAGE_PRECHARGE = 8; // PLC HLC stage enum: WAIT_PRECHARGE
constexpr uint8_t HLC_MIN_POWER_STAGE = 9; // minimum stage indicating power delivery readiness
constexpr std::chrono::milliseconds MC_OPEN_TIMEOUT_MS(2000);
constexpr std::chrono::milliseconds GC_OPEN_TIMEOUT_MS(2000);
constexpr std::chrono::milliseconds GC_OPEN_FAULT_CONFIRM_MS(1000);
constexpr std::chrono::milliseconds PRECHARGE_OVERCURRENT_HOLD_MS(100);
constexpr std::chrono::milliseconds PRECHARGE_OVERSHOOT_HOLD_MS(200);
constexpr std::chrono::milliseconds PRECHARGE_STABLE_HOLD_MS(300);
// Avoid tripping local stall faults on short/noisy underdelivery windows.
// EVs and module telemetry can transiently dip for a few seconds without indicating a hard fault.
constexpr std::chrono::milliseconds POWER_DELIVERY_STALL_TIMEOUT_MS(8000);
constexpr std::chrono::seconds RECENT_TOKEN_DEDUP_WINDOW(10);
constexpr std::chrono::seconds RFID_TAP_LATCH_WINDOW(3);
constexpr std::chrono::milliseconds SEAMLESS_RETRY_GRACE_MS(20000);
constexpr std::chrono::milliseconds CP_FAULT_GRACE_MS(3000);
[[maybe_unused]] constexpr std::chrono::seconds STATUS_REFRESH_INTERVAL(60);
constexpr std::chrono::milliseconds STATUS_REFRESH_MIN_GAP(2000);
constexpr std::chrono::milliseconds POST_STOP_HOLD_MS(5000);
constexpr std::chrono::milliseconds POST_STOP_MAX_HOLD_MS(30000);
constexpr std::chrono::milliseconds RESERVATION_CHECK_INTERVAL(1000);
// Safety-first power clamp when EV withdraws CP ready (C/D -> B/A).
// Keep this short to tolerate brief CP jitter while avoiding sustained output
// after the vehicle has de-asserted ready-to-charge.
constexpr std::chrono::milliseconds CP_NOT_READY_POWER_CUT_DELAY_MS(500);

template <typename TimePoint>
std::chrono::steady_clock::time_point to_steady_from_utc(const TimePoint& t_utc) {
    const auto now_utc = ocpp::DateTime().to_time_point();
    const auto now_steady = std::chrono::steady_clock::now();
    return now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(t_utc - now_utc);
}

template <typename T, typename = void>
struct has_register_clear_cache_callback : std::false_type {};
template <typename T>
struct has_register_clear_cache_callback<
    T,
    std::void_t<decltype(std::declval<T&>().register_clear_cache_callback(std::declval<std::function<void()>>()))>>
    : std::true_type {};

template <typename T, typename = void>
struct has_trigger_status_notifications : std::false_type {};
template <typename T>
struct has_trigger_status_notifications<T, std::void_t<decltype(std::declval<T&>().trigger_status_notifications())>>
    : std::true_type {};

template <typename CP>
std::enable_if_t<has_register_clear_cache_callback<CP>::value, void>
register_clear_cache_callback_if_supported(CP& cp, std::function<void()> cb) {
    cp.register_clear_cache_callback(std::move(cb));
}

template <typename CP>
std::enable_if_t<!has_register_clear_cache_callback<CP>::value, void>
register_clear_cache_callback_if_supported(CP&, std::function<void()>) {
    // Older libocpp versions handle ClearCache internally but do not expose a callback. In that case,
    // we cannot clear adapter-side caches on demand.
    EVLOG_warning << "libocpp does not expose register_clear_cache_callback(); skipping adapter cache hook";
}

template <typename CP>
std::enable_if_t<has_trigger_status_notifications<CP>::value, void>
trigger_status_notifications_if_supported(CP& cp) {
    cp.trigger_status_notifications();
}

template <typename CP>
std::enable_if_t<!has_trigger_status_notifications<CP>::value, void>
trigger_status_notifications_if_supported(CP&) {
    // Older libocpp versions do not provide an API to force-refresh StatusNotifications.
}

const char* connector_state_name(ConnectorState state) {
    switch (state) {
    case ConnectorState::Available:
        return "Available";
    case ConnectorState::Preparing:
        return "Preparing";
    case ConnectorState::Charging:
        return "Charging";
    case ConnectorState::SuspendedEV:
        return "SuspendedEV";
    case ConnectorState::SuspendedEVSE:
        return "SuspendedEVSE";
    case ConnectorState::Finishing:
        return "Finishing";
    case ConnectorState::Unavailable:
        return "Unavailable";
    case ConnectorState::Faulted:
        return "Faulted";
    default:
        return "Unknown";
    }
}

ocpp::v16::ChargePointStatus to_ocpp_status(ConnectorState state) {
    switch (state) {
    case ConnectorState::Available:
        return ocpp::v16::ChargePointStatus::Available;
    case ConnectorState::Preparing:
        return ocpp::v16::ChargePointStatus::Preparing;
    case ConnectorState::Charging:
        return ocpp::v16::ChargePointStatus::Charging;
    case ConnectorState::SuspendedEV:
        return ocpp::v16::ChargePointStatus::SuspendedEV;
    case ConnectorState::SuspendedEVSE:
        return ocpp::v16::ChargePointStatus::SuspendedEVSE;
    case ConnectorState::Finishing:
        return ocpp::v16::ChargePointStatus::Finishing;
    case ConnectorState::Unavailable:
        return ocpp::v16::ChargePointStatus::Unavailable;
    case ConnectorState::Faulted:
        return ocpp::v16::ChargePointStatus::Faulted;
    default:
        return ocpp::v16::ChargePointStatus::Available;
    }
}

bool has_hlc_or_power_presence_evidence(const GunStatus& status);
bool infer_vehicle_present(const GunStatus& status, bool has_session_hint = false);

ConnectorState initial_state_from_status(const GunStatus& status) {
    const bool vehicle_present = infer_vehicle_present(status);
    const bool fault_active = !status.safety_ok || status.cp_fault || status.comm_fault || status.isolation_fault ||
                              status.overtemp_fault || status.overcurrent_fault || status.gc_welded ||
                              status.mc_welded || status.earth_fault || status.estop;
    const bool meter_fault_active = status.meter_stale && status.relay_closed;
    if (fault_active || meter_fault_active) {
        return ConnectorState::Faulted;
    }
    if (vehicle_present) {
        return ConnectorState::Preparing;
    }
    return ConnectorState::Available;
}

GunStatus sanitize_status(const GunStatus& in, bool lab_bypass) {
    GunStatus st = in;
    // Normalize HLC phase flags: treat the PLC stage enum as authoritative.
    if (st.hlc_stage >= HLC_MIN_POWER_STAGE && !st.hlc_precharge_active && !st.hlc_charge_complete) {
        st.hlc_power_ready = true;
    }
    if (lab_bypass) {
        // Lab bypass: ignore cable-check, weld, isolation, and temperature-related faults.
        st.hlc_cable_check_ok = true;
        st.isolation_fault = false;
        st.overtemp_fault = false;
        st.gc_welded = false;
        st.mc_welded = false;
    }
    return st;
}

bool should_use_stub_security(const ChargerConfig&) {
    const char* env = std::getenv("DC_OCPP_STUB_SECURITY");
    if (!env) {
        return false;
    }
    return std::string(env) != "0";
}

bool has_hlc_or_power_presence_evidence(const GunStatus& status) {
    return status.hlc_precharge_active || status.hlc_power_ready || status.hlc_stage >= HLC_STAGE_AUTHORIZATION ||
           status.relay_closed || status.authorization_granted;
}

bool infer_vehicle_present(const GunStatus& status, bool has_session_hint) {
    // CP=B can be noisy on some harnesses when unplugged. Require corroboration so
    // startup/noise does not hold the connector in Preparing indefinitely.
    if (status.plugged_in) {
        return true;
    }
    if (status.cp_state == 'C' || status.cp_state == 'D') {
        return true;
    }
    if (status.cp_state != 'B') {
        return false;
    }
    return has_session_hint || has_hlc_or_power_presence_evidence(status);
}

std::chrono::milliseconds evse_limit_ack_timeout(const ChargerConfig& cfg) {
    return std::chrono::milliseconds(std::max(1, cfg.evse_limit_ack_timeout_ms));
}

std::chrono::milliseconds telemetry_timeout(const ChargerConfig& cfg) {
    return std::chrono::milliseconds(std::max(1, cfg.telemetry_timeout_ms));
}

const Slot* find_slot(const std::vector<Slot>& slots, int id) {
    auto it = std::find_if(slots.begin(), slots.end(), [&](const Slot& s) { return s.id == id; });
    return it == slots.end() ? nullptr : &(*it);
}

const Slot* find_slot_for_gun(const std::vector<Slot>& slots, int gun_id) {
    auto it = std::find_if(slots.begin(), slots.end(), [&](const Slot& s) { return s.gun_id == gun_id; });
    if (it != slots.end()) {
        return &(*it);
    }
    return find_slot(slots, gun_id);
}

struct SlotModuleSelection {
    int gun_id{0};
    uint8_t mask{0};
    int module_count{0};
    bool in_island{false};
};

SlotModuleSelection compute_slot_module_selection(const Plan& plan, const Slot& slot,
                                                  const std::map<std::string, int>& module_slot_index) {
    SlotModuleSelection sel{};
    for (const auto& island : plan.islands) {
        const bool slot_in_island = std::find(island.slot_ids.begin(), island.slot_ids.end(), slot.id) != island.slot_ids.end();
        if (!slot_in_island) {
            continue;
        }
        sel.in_island = true;
        sel.gun_id = island.gun_id.value_or(0);
        for (std::size_t idx = 0; idx < slot.modules.size(); ++idx) {
            const auto& module_id = slot.modules[idx];
            if (std::find(island.module_ids.begin(), island.module_ids.end(), module_id) != island.module_ids.end()) {
                int bit = static_cast<int>(idx);
                const auto it = module_slot_index.find(module_id);
                if (it != module_slot_index.end()) {
                    bit = it->second;
                }
                if (bit >= 0 && bit < 8) {
                    sel.mask |= static_cast<uint8_t>(1U << bit);
                }
                sel.module_count++;
            }
        }
        break;
    }
    return sel;
}

bool power_delivery_requested(const GunStatus& status, bool lock_required) {
    if (!status.plugged_in || status.comm_fault || status.cp_fault) {
        return false;
    }
    if (status.hlc_charge_complete) {
        return false;
    }
    if (lock_required && !status.lock_engaged) {
        return false;
    }
    const bool cp_ready = status.cp_state == 'C' || status.cp_state == 'D';
    if (!cp_ready) return false;
    const bool hlc_power_phase =
        status.hlc_power_ready || (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
    if (!hlc_power_phase) {
        constexpr double kEvCurrentRequestThresholdA = 0.1;
        const bool ev_current_req =
            status.target_current_a && status.target_current_a.value() > kEvCurrentRequestThresholdA;
        const bool ev_voltage_req =
            status.target_voltage_v && status.target_voltage_v.value() > 10.0;
        return ev_current_req && ev_voltage_req;
    }
    // By this point the PLC has asserted HLC readiness (stage >= POWER_DELIVERY, cable check ok, precharge complete).
    return true;
}

bool is_hlc_precharge_phase(const GunStatus& status) {
    if (status.hlc_charge_complete) {
        return false;
    }
    // Treat the PLC stage enum as authoritative: once POWER_DELIVERY or later is reached,
    // we are no longer in precharge even if the precharge_active flag is stuck high.
    if (status.hlc_stage >= HLC_MIN_POWER_STAGE) {
        return false;
    }
    if (status.hlc_precharge_active) {
        return true;
    }
    // Some PLC builds may not assert precharge_active reliably; fall back to the explicit stage.
    return status.hlc_stage == HLC_STAGE_PRECHARGE;
}

bool evse_limit_ack_watchdog_relevant(const GunStatus& status, bool lock_required) {
    constexpr double kEvCurrentRequestThresholdA = 0.1;
    const bool hlc_power_phase =
        status.hlc_power_ready || (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
    const bool cp_power_requesting = status.cp_state == 'C' || status.cp_state == 'D';
    const bool power_requested = power_delivery_requested(status, lock_required);
    const bool ev_current_requested =
        status.target_current_a && status.target_current_a.value() > kEvCurrentRequestThresholdA;
    const bool ev_current_measured = status.present_current_a && std::fabs(status.present_current_a.value()) > 1.0;
    const bool ev_power_measured = status.present_power_w && std::fabs(status.present_power_w.value()) > 500.0;
    return status.relay_closed && cp_power_requesting && hlc_power_phase && power_requested &&
           (ev_current_requested || ev_current_measured || ev_power_measured);
}

bool local_disable_latches_until_unplug(const std::string& reason) {
    if (reason.empty()) {
        return false;
    }
    // Planner-side sequencing/timeouts should not be auto-retried while the EV remains plugged.
    // Otherwise, we can end up in relay-flipping / module power cycling loops when HLC gets stuck mid-sequence.
    if (reason.rfind("Stuck", 0) == 0) {
        return true;
    }
    if (reason.rfind("Module", 0) == 0) {
        return true;
    }
    if (reason.find("Timeout") != std::string::npos) {
        return true;
    }
    if (reason == "PowerDeliveryStalled") {
        return true;
    }
    if (reason == "PrechargeOverCurrent" || reason == "PrechargeVoltageOvershoot") {
        return true;
    }
    return false;
}

int popcount(uint8_t mask) {
    int count = 0;
    while (mask) {
        count += (mask & 0x1);
        mask >>= 1U;
    }
    return count;
}

template <typename T> struct is_optional : std::false_type {};
template <typename U> struct is_optional<std::optional<U>> : std::true_type {};

template <typename T, typename = void>
struct has_total : std::false_type {};
template <typename T>
struct has_total<T, std::void_t<decltype(std::declval<T>().total)>> : std::true_type {};

template <typename T, typename = void>
struct has_dc : std::false_type {};
template <typename T>
struct has_dc<T, std::void_t<decltype(std::declval<T>().DC)>> : std::true_type {};

template <typename T>
double extract_total_value(const T& v) {
    if constexpr (is_optional<T>::value) {
        return v ? extract_total_value(*v) : 0.0;
    } else if constexpr (has_total<T>::value) {
        return static_cast<double>(v.total);
    } else {
        return static_cast<double>(v);
    }
}

template <typename T>
std::optional<double> extract_dc_value(const T& v) {
    if constexpr (is_optional<T>::value) {
        if (!v) return std::nullopt;
        return extract_dc_value(*v);
    } else if constexpr (has_dc<T>::value) {
        if constexpr (is_optional<decltype(v.DC)>::value) {
            return v.DC ? std::optional<double>(*v.DC) : std::nullopt;
        } else {
            return std::optional<double>(v.DC);
        }
    } else {
        return std::nullopt;
    }
}

} // namespace

OcppAdapter::OcppAdapter(ChargerConfig cfg, std::shared_ptr<HardwareInterface> hardware) :
    cfg_(std::move(cfg)),
    hardware_(std::move(hardware)),
    planner_cfg_{},
    power_manager_(planner_cfg_) {
    // Only persist pending tokens when an explicit database directory is configured. This avoids
    // polluting the repo root in unit tests or ad-hoc runs with a default-constructed config.
    if (!cfg_.database_dir.empty()) {
        pending_token_store_ = cfg_.database_dir / "pending_tokens.json";
    }
    for (const auto& c : cfg_.connectors) {
        connector_faulted_[c.id] = false;
        connector_state_[c.id] = ConnectorState::Available;
        evse_disabled_[c.id] = false;
        reserved_connectors_[c.id] = false;
        power_constrained_[c.id] = false;
        paused_evse_[c.id] = false;
        plugged_in_state_[c.id] = false;
        plug_event_time_[c.id] = std::chrono::steady_clock::time_point{};
        auth_state_cache_[c.id] = AuthorizationState::Unknown;
        post_stop_plugged_[c.id] = false;
        post_stop_time_[c.id] = std::chrono::steady_clock::time_point{};
        hlc_control_[c.id] = HlcControlState{};
        reservation_required_tag_.erase(c.id);
        reservation_parent_tag_.erase(c.id);
        telemetry_mismatch_count_[c.id] = 0;
        connector_meter_intervals_[c.id] = c.meter_sample_interval_s > 0 ? c.meter_sample_interval_s
                                                                         : cfg_.meter_sample_interval_s;
    }
    load_pending_tokens_from_disk();
    initialize_slots();
}

OcppAdapter::~OcppAdapter() {
    stop();
}

void OcppAdapter::prepare_security_files() const {
    auto ensure_parent = [](const fs::path& path) {
        if (path.empty()) return;
        std::error_code ec;
        fs::create_directories(path.parent_path(), ec);
    };

    ensure_parent(cfg_.security.csms_ca_bundle);
    ensure_parent(cfg_.security.mo_ca_bundle);
    ensure_parent(cfg_.security.mf_ca_bundle);
    ensure_parent(cfg_.security.v2g_ca_bundle);
}

void OcppAdapter::seed_default_evse_limits() {
    for (const auto& c : cfg_.connectors) {
        EvseLimits limits{};
        if (c.max_voltage_v > 0.0) {
            limits.max_voltage_v = c.max_voltage_v;
        }
        if (c.max_current_a > 0.0) {
            limits.max_current_a = c.max_current_a;
        }
        if (c.max_power_w > 0.0) {
            limits.max_power_kw = c.max_power_w / 1000.0;
        }
        hardware_->set_evse_limits(c.id, limits);
    }
}

const Slot* OcppAdapter::find_slot_for_gun(int gun_id) const {
    return charger::find_slot_for_gun(slots_, gun_id);
}

OcppAdapter::HlcControlOutcome
OcppAdapter::apply_hlc_control(std::int32_t connector, const GunStatus& status, bool had_session,
                               const ActiveSession& session, bool post_stop_plugged,
                               const std::optional<std::string>& autocharge_reject_id, bool force_auth_denied,
                               const std::chrono::steady_clock::time_point& now) {
    HlcControlOutcome out{};
    out.force_auth_denied = force_auth_denied;
    const bool autocharge_allowed = autocharge_enabled_.load();
    const bool csms_online = csms_connected_.load();
    const bool mac_autocharge_mode = cfg_.autocharge_id_source == "evmac";

    bool pending_changed = false;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        auto& flow = hlc_control_[connector];
        if (!status.plugged_in) {
            flow.pnc_blocked = false;
            flow.blocked_identity.reset();
            flow.block_expires = std::chrono::steady_clock::time_point{};
            flow.auth_pending_since = std::chrono::steady_clock::time_point{};
            flow.last_autocharge_id.reset();
        } else if (flow.pnc_blocked && flow.block_expires.time_since_epoch().count() != 0 &&
                   now >= flow.block_expires) {
            flow.pnc_blocked = false;
            flow.blocked_identity.reset();
            flow.block_expires = std::chrono::steady_clock::time_point{};
        }

        if (autocharge_reject_id) {
            if (mac_autocharge_mode) {
                // In MAC-ID mode keep PnC open so the same EV identity can be retried
                // without collapsing active digital communication.
                flow.pnc_blocked = false;
                flow.blocked_identity.reset();
                flow.block_expires = std::chrono::steady_clock::time_point{};
                if (flow.auth_pending_since.time_since_epoch().count() == 0) {
                    flow.auth_pending_since = now;
                }
                out.force_auth_denied = false;
                EVLOG_warning << "Autocharge rejected on connector " << connector
                              << " in MAC mode; keeping PnC open for retry";
            } else {
                flow.pnc_blocked = true;
                flow.blocked_identity = autocharge_reject_id;
                flow.block_expires = cfg_.pnc_block_ttl_s > 0
                                         ? (now + std::chrono::seconds(cfg_.pnc_block_ttl_s))
                                         : std::chrono::steady_clock::time_point{};
                flow.auth_pending_since = std::chrono::steady_clock::time_point{};
                if (clear_pending_autocharge_tokens_for_connector_locked(connector)) {
                    pending_changed = true;
                }
                out.force_auth_denied = true;
                EVLOG_warning << "Autocharge rejected; blocking PnC on connector " << connector;
            }
        }

        const bool session_authorized = had_session && session.authorized;
        const bool autocharge_start = autocharge_allowed && !flow.pnc_blocked;
        if (autocharge_start && !session_authorized && status.hlc_stage >= HLC_STAGE_AUTHORIZATION) {
            if (flow.auth_pending_since.time_since_epoch().count() == 0) {
                flow.auth_pending_since = now;
            } else if (cfg_.hlc_auth_timeout_s > 0 &&
                       (now - flow.auth_pending_since) > std::chrono::seconds(cfg_.hlc_auth_timeout_s)) {
                // In MAC-ID mode (or while CSMS is offline), do not tear down digital comms mid-HLC.
                // Keep auth pending and retry instead of forcing fallback that drops SLAC/HLC.
                if (mac_autocharge_mode || !csms_online) {
                    flow.auth_pending_since = now;
                    out.auth_timeout_triggered = true;
                    EVLOG_warning << "Autocharge pending timeout on connector " << connector
                                  << " (mode=" << cfg_.autocharge_id_source
                                  << ", csms_online=" << (csms_online ? "true" : "false")
                                  << "); keeping HLC active";
                } else {
                    flow.pnc_blocked = true;
                    flow.blocked_identity = flow.last_autocharge_id;
                    flow.block_expires = cfg_.pnc_block_ttl_s > 0
                                             ? (now + std::chrono::seconds(cfg_.pnc_block_ttl_s))
                                             : std::chrono::steady_clock::time_point{};
                    flow.auth_pending_since = std::chrono::steady_clock::time_point{};
                    if (clear_pending_autocharge_tokens_for_connector_locked(connector)) {
                        pending_changed = true;
                    }
                    out.auth_timeout_triggered = true;
                    out.force_auth_denied = true;
                    EVLOG_warning << "Autocharge timed out on connector " << connector
                                  << "; forcing EIM fallback";
                }
            }
        } else {
            flow.auth_pending_since = std::chrono::steady_clock::time_point{};
        }

        // Advertise digital communication (SLAC/ISO15118/DIN) while the EV is physically present.
        // Keep this stable across auth retries/blocks to avoid tearing down active SLAC/HLC sessions.
        // Brief CP/telemetry blips (e.g. cp_state='U' for a cycle) must not flap the HLC gate.
        constexpr auto kDigitalDisableDebounce = std::chrono::milliseconds(8000);
        constexpr auto kExplicitAbsentDebounce = std::chrono::milliseconds(2500);
        const bool cp_known = status.cp_state != 'U';
        const bool cp_present = cp_known && status.cp_state != 'A';
        const bool hlc_runtime_active =
            status.hlc_stage > 0 || status.hlc_precharge_active || status.hlc_power_ready || status.relay_closed;
        const bool strong_presence = status.plugged_in || cp_present || had_session || hlc_runtime_active;
        if (strong_presence) {
            flow.last_presence_hint = now;
        }
        const bool explicit_absent_raw =
            cp_known && status.cp_state == 'A' && !status.plugged_in && !had_session && !hlc_runtime_active;
        if (explicit_absent_raw) {
            if (flow.absent_since.time_since_epoch().count() == 0) {
                flow.absent_since = now;
            }
        } else {
            flow.absent_since = std::chrono::steady_clock::time_point{};
        }
        const bool explicit_absent =
            explicit_absent_raw && flow.absent_since.time_since_epoch().count() != 0 &&
            (now - flow.absent_since) >= kExplicitAbsentDebounce;
        bool plugged_hint = strong_presence;
        if (!plugged_hint && !explicit_absent && flow.last_presence_hint.time_since_epoch().count() != 0) {
            plugged_hint = (now - flow.last_presence_hint) <= kDigitalDisableDebounce;
        }
        // Once enabled, keep digital communication up unless an explicit unplug is confirmed.
        // This avoids ENABLED->DISABLED->ENABLED flapping when PLC status briefly destabilizes.
        if (!plugged_hint && flow.digital_enabled && !explicit_absent) {
            plugged_hint = true;
        }
        if (explicit_absent) {
            flow.last_presence_hint = std::chrono::steady_clock::time_point{};
        }
        const bool allow_autocharge = autocharge_allowed && !flow.pnc_blocked;
        if (allow_autocharge) {
            out.desired_digital = plugged_hint;
        } else {
            out.desired_digital = plugged_hint && !post_stop_plugged;
        }
        // Keep PnC available unless explicitly blocked; MAC-based AutoCharge should not force-disable HLC.
        out.desired_pnc_blocked = (!autocharge_allowed) || flow.pnc_blocked;

        if (out.desired_digital != flow.digital_enabled) {
            flow.digital_enabled = out.desired_digital;
            out.digital_update = true;
            EVLOG_info << "HLC digital gate connector " << connector
                       << " -> " << (out.desired_digital ? "ENABLED" : "DISABLED")
                       << " (autocharge=" << (allow_autocharge ? "yes" : "no")
                       << ", session_auth=" << (session_authorized ? "yes" : "no")
                       << ", hlc_stage=" << static_cast<int>(status.hlc_stage)
                       << ", plugged=" << (plugged_hint ? "yes" : "no") << ")";
        }
        if (out.desired_pnc_blocked != flow.pnc_blocked_sent) {
            flow.pnc_blocked_sent = out.desired_pnc_blocked;
            out.pnc_block_update = true;
            EVLOG_info << "PnC block connector " << connector
                       << " -> " << (out.desired_pnc_blocked ? "ON" : "OFF");
        }
    }
    if (pending_changed) {
        persist_pending_tokens();
    }
    return out;
}

void OcppAdapter::initialize_slots() {
    if (slots_initialized_) return;

    if (!hardware_ || !hardware_->supports_cross_slot_islands()) {
        throw std::runtime_error("Split charging requires hardware support for cross-slot islands.");
    }
    if (!cfg_.allow_cross_slot_islands) {
        throw std::runtime_error("Split charging requires planner.allowCrossSlotIslands=true.");
    }

    PlannerConfig pcfg;
    pcfg.module_power_kw = cfg_.module_power_kw > 0.0 ? cfg_.module_power_kw : 30.0;
    pcfg.grid_limit_kw = cfg_.grid_limit_kw > 0.0 ? cfg_.grid_limit_kw : 1000.0;
    pcfg.default_voltage_v = cfg_.default_voltage_v > 0.0 ? cfg_.default_voltage_v : 800.0;
    pcfg.allow_cross_slot_islands = cfg_.allow_cross_slot_islands;
    pcfg.max_modules_per_gun = std::max(1, cfg_.max_modules_per_gun);
    pcfg.min_modules_per_active_gun = std::max(0, cfg_.min_modules_per_active_gun);
    pcfg.max_island_radius = std::max(1, cfg_.max_island_radius);
    pcfg.min_module_hold_ms = std::max(0, cfg_.min_module_hold_ms);
    pcfg.min_mc_hold_ms = std::max(0, cfg_.min_mc_hold_ms);
    pcfg.min_gc_hold_ms = std::max(0, cfg_.min_gc_hold_ms);
    pcfg.mc_open_current_a = cfg_.mc_open_current_a;
    pcfg.gc_open_current_a = cfg_.gc_open_current_a;
    pcfg.ramp_step_a = cfg_.ramp_step_a;
    pcfg.voltage_margin_v = cfg_.planner_voltage_margin_v;
    pcfg.current_margin_a = cfg_.planner_current_margin_a;
    pcfg.voltage_guard_band_v = cfg_.planner_voltage_guard_band_v;
    pcfg.ramp_up_min_a_per_s = cfg_.planner_ramp_up_min_a_per_s;
    pcfg.ramp_up_max_a_per_s = cfg_.planner_ramp_up_max_a_per_s;
    pcfg.ramp_down_min_a_per_s = cfg_.planner_ramp_down_min_a_per_s;
    pcfg.ramp_down_max_a_per_s = cfg_.planner_ramp_down_max_a_per_s;
    pcfg.ramp_down_emergency_a_per_s = cfg_.planner_ramp_down_emergency_a_per_s;
    pcfg.ramp_jerk_a_per_s2 = cfg_.planner_ramp_jerk_a_per_s2;
    pcfg.ramp_response_s = cfg_.planner_ramp_response_s;
    pcfg.ramp_capture_current_a = cfg_.planner_capture_current_a;
    pcfg.ramp_capture_rate_a_per_s = cfg_.planner_capture_rate_a_per_s;
    planner_cfg_ = pcfg;
    power_manager_ = PowerManager(planner_cfg_);

    std::vector<Slot> slots;
    module_states_.clear();
    connector_module_slots_.clear();
    slot_owner_connector_.clear();

    std::map<int, const SlotMapping*> slot_lookup;
    for (const auto& sm : cfg_.slots) {
        slot_lookup[sm.id] = &sm;
    }

    auto build_slot_order = [&]() -> std::vector<int> {
        std::vector<int> order;
        if (cfg_.slots.empty()) {
            return order;
        }
        std::map<int, int> cw;
        std::map<int, int> ccw;
        for (const auto& sm : cfg_.slots) {
            cw[sm.id] = sm.cw_id;
            ccw[sm.id] = sm.ccw_id;
        }
        int start_id = 0;
        for (const auto& sm : cfg_.slots) {
            if (ccw[sm.id] == 0) {
                start_id = sm.id;
                break;
            }
        }
        if (start_id == 0) {
            for (const auto& sm : cfg_.slots) {
                if (cw[sm.id] == 0) {
                    start_id = sm.id;
                    break;
                }
            }
        }
        if (start_id == 0) {
            start_id = cfg_.slots.front().id;
        }

        std::set<int> visited;
        int current = start_id;
        while (current != 0) {
            if (visited.count(current)) {
                break;
            }
            visited.insert(current);
            order.push_back(current);
            int next = cw[current];
            if (next == 0 || next == start_id) {
                break;
            }
            current = next;
        }

        if (order.size() != cfg_.slots.size()) {
            throw std::runtime_error("Invalid slot ring order; check cw/ccw topology for split charging.");
        }
        return order;
    };

    const auto slot_order = build_slot_order();
    if (slot_order.empty()) {
        throw std::runtime_error("No slot topology defined for split charging.");
    }

    int next_slot_id = 1;
    for (int slot_id : slot_order) {
        const auto it = slot_lookup.find(slot_id);
        if (it == slot_lookup.end()) {
            throw std::runtime_error("Slot id=" + std::to_string(slot_id) + " missing from topology.");
        }
        const auto& sm = *it->second;
        std::array<int, 2> module_slot_ids{{0, 0}};
        for (std::size_t idx = 0; idx < 2; ++idx) {
            const ModuleConfig* mptr = (idx < sm.modules.size()) ? &sm.modules[idx] : nullptr;
            Slot s;
            s.id = next_slot_id++;
            s.gun_id = (idx == 0 ? sm.gun_id : 0);
            s.gc_id = (idx == 0 ? sm.gc_id : "");
            if (mptr) {
                s.mc_id = "KM_" + mptr->id;
                s.modules = {mptr->id};
            } else {
                const std::string suffix = (idx == 0) ? "A" : "B";
                s.mc_id = "KM_" + std::to_string(sm.gun_id) + "_" + suffix;
                s.modules.clear();
            }
            slots.push_back(s);
            module_slot_ids[idx] = s.id;
            slot_owner_connector_[s.id] = sm.gun_id;

            if (mptr) {
                const auto& m = *mptr;
                ModuleState ms;
                ms.id = m.id;
                ms.slot_id = s.id;
                ms.mn_id = m.mn_id.empty() ? "MN_" + std::to_string(sm.id) + "_" + std::to_string(idx) : m.mn_id;
                ms.slot_index = static_cast<int>(idx);
                ms.type = m.type;
                ms.can_interface = !m.can_interface.empty() ? m.can_interface : cfg_.can_interface;
                ms.address = m.address;
                ms.group = m.group;
                ms.monitor_address = m.monitor_address;
                ms.production_day = m.production_day;
                ms.serial_low = m.serial_low;
                ms.source_address = m.source_address;
                ms.input_mode = m.input_mode;
                ms.hi_lo_mode = m.hi_lo_mode;
                ms.silent_mode = m.silent_mode;
                ms.rated_power_kw = m.rated_power_kw;
                ms.rated_current_a = m.rated_current_a;
                ms.poll_interval_ms = m.poll_interval_ms;
                ms.cmd_interval_ms = m.cmd_interval_ms;
                ms.poll_budget_fps = m.poll_budget_fps;
                ms.telemetry_stale_ms = m.telemetry_stale_ms;
                ms.broadcast = m.broadcast;
                ms.probe_on_startup = m.probe_on_startup;
                ms.readback_limits = m.readback_limits;
                ms.send_output_current = m.send_output_current;
                ms.send_output_power = m.send_output_power;
                module_states_.push_back(ms);
            }
        }
        connector_module_slots_[sm.gun_id] = module_slot_ids;
    }

    const int total_slots = static_cast<int>(slots.size());
    const bool is_ring =
        !slot_order.empty() &&
        slot_lookup.at(slot_order.back())->cw_id == slot_order.front();
    for (int i = 0; i < total_slots; ++i) {
        slots[i].cw_id = (i + 1 < total_slots) ? slots[i + 1].id : (is_ring ? slots.front().id : 0);
        slots[i].ccw_id = (i > 0) ? slots[i - 1].id : (is_ring ? slots.back().id : 0);
    }

    slots_ = slots;
    power_manager_.set_slots(slots);

    std::vector<ModuleSpec> module_specs;
    ModuleCanTrafficPolicy module_can_policy{};
    module_can_policy.max_total_kbps_per_interface = cfg_.can_traffic.max_total_kbps_per_interface;
    module_can_policy.window_ms = cfg_.can_traffic.window_ms;
    module_can_policy.bits_per_frame_estimate = cfg_.can_traffic.bits_per_frame_estimate;
    module_can_policy.over_cap_debounce_ms = cfg_.can_traffic.over_cap_debounce_ms;
    module_can_policy.enforce = cfg_.can_traffic.enforce;
    // Keep the module budget aligned with the configured total CAN cap and measured runtime traffic.
    // Do not pre-subtract a static PLC reserve per interface; that can double-throttle module control.
    bool missing_module_type = false;
    for (const auto& ms : module_states_) {
        if (ms.type.empty()) {
            missing_module_type = true;
            continue;
        }
        ModuleSpec spec;
        spec.id = ms.id;
        spec.slot_id = ms.slot_id;
        spec.slot_index = ms.slot_index;
        spec.type = ms.type;
        spec.can_interface = !ms.can_interface.empty() ? ms.can_interface : cfg_.can_interface;
        spec.address = ms.address;
        spec.group = ms.group;
        spec.monitor_address = ms.monitor_address;
        spec.production_day = ms.production_day;
        spec.serial_low = ms.serial_low;
        spec.source_address = ms.source_address;
        spec.input_mode = ms.input_mode;
        spec.hi_lo_mode = ms.hi_lo_mode;
        spec.silent_mode = ms.silent_mode;
        spec.rated_power_kw = ms.rated_power_kw > 0.0 ? ms.rated_power_kw : cfg_.module_power_kw;
        spec.rated_current_a = ms.rated_current_a;
        spec.poll_interval_ms = ms.poll_interval_ms;
        spec.cmd_interval_ms = ms.cmd_interval_ms;
        spec.poll_budget_fps = ms.poll_budget_fps;
        spec.telemetry_stale_ms = ms.telemetry_stale_ms;
        spec.broadcast = ms.broadcast;
        spec.probe_on_startup = ms.probe_on_startup;
        spec.readback_limits = ms.readback_limits;
        spec.send_output_current = ms.send_output_current;
        spec.send_output_power = ms.send_output_power;
        module_specs.push_back(spec);
    }
    if (missing_module_type) {
        throw std::runtime_error("Split charging requires module.type for every slots[].modules entry.");
    }
    if (!module_specs.empty()) {
        module_controller_ = std::make_unique<PowerModuleController>(module_specs, module_can_policy);
    } else {
        throw std::runtime_error("Split charging requires module drivers; set module.type in slots[].modules.");
    }

    slots_initialized_ = true;
}

bool OcppAdapter::start() {
    prepare_security_files();
    Everest::Logging::init(cfg_.logging_config.string(), "dc-ocpp");

    const auto config_str = load_and_patch_ocpp_config(cfg_);

    nlohmann::json cfg_json;
    std::string central_system_uri;
    int security_profile = 0;
    bool autocharge_enabled = true;
    try {
        cfg_json = nlohmann::json::parse(config_str);
        if (cfg_json.contains("Security") && cfg_json["Security"].is_object()) {
            const auto& sec = cfg_json["Security"];
            if (sec.contains("SecurityProfile")) {
                const auto& sp = sec["SecurityProfile"];
                if (sp.is_number_integer()) {
                    security_profile = sp.get<int>();
                } else if (sp.is_string()) {
                    try {
                        security_profile = std::stoi(sp.get<std::string>());
                    } catch (const std::exception&) {
                        security_profile = 0;
                    }
                }
            }
        }
        if (cfg_json.contains("Internal") && cfg_json["Internal"].is_object()) {
            central_system_uri = cfg_json["Internal"].value("CentralSystemURI", "");
        }
        if (cfg_json.contains("Custom") && cfg_json["Custom"].is_object()) {
            const auto& custom = cfg_json["Custom"];
            if (custom.contains("AutochargeEnabled")) {
                const auto& val = custom["AutochargeEnabled"];
                if (val.is_boolean()) {
                    autocharge_enabled = val.get<bool>();
                } else if (val.is_number_integer()) {
                    autocharge_enabled = val.get<int>() != 0;
                } else if (val.is_string()) {
                    std::string v = val.get<std::string>();
                    std::transform(v.begin(), v.end(), v.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    if (v == "true" || v == "1" || v == "yes" || v == "on") {
                        autocharge_enabled = true;
                    } else if (v == "false" || v == "0" || v == "no" || v == "off") {
                        autocharge_enabled = false;
                    }
                }
            }
        }
    } catch (const std::exception&) {
        security_profile = 0;
        autocharge_enabled = true;
    }
    if (security_profile < 0) {
        security_profile = 0;
    }
    if (security_profile > 3) {
        EVLOG_error << "Unsupported OCPP SecurityProfile=" << security_profile
                    << " (expected 0..3). Refusing to start.";
        return false;
    }

    if (security_profile >= 2) {
        // libocpp will validate scheme/profile consistency, but fail-fast here with a clearer message.
        if (central_system_uri.rfind("ws://", 0) == 0) {
            EVLOG_error << "SecurityProfile=" << security_profile
                        << " requires a secure websocket endpoint, but CentralSystemURI starts with ws:// ("
                        << central_system_uri << "). Use wss://... or omit the scheme.";
            return false;
        }
        if (central_system_uri.empty()) {
            EVLOG_error << "SecurityProfile=" << security_profile
                        << " requires Internal.CentralSystemURI to be configured";
            return false;
        }
    }

    // Use an explicit file-backed EVSE security backend for TLS and certificate verification/provisioning.
    const bool stub_requested = should_use_stub_security(cfg_);
    const bool permissive = stub_requested && security_profile < 2;
    if (stub_requested && !permissive) {
        EVLOG_warning << "DC_OCPP_STUB_SECURITY=1 ignored for SecurityProfile=" << security_profile
                      << " (TLS profiles require real verification)";
    }
    if (permissive) {
        EVLOG_warning << "Using permissive EVSE security backend (DC_OCPP_STUB_SECURITY=1). NOT production-safe.";
    }

    set_autocharge_enabled(autocharge_enabled, "config");

    const auto evse_security =
        std::make_shared<FileEvseSecurity>(FileEvseSecurity::Paths{cfg_.charge_point_id, cfg_.security}, permissive);
    const auto maintenance_security = permissive
                                          ? std::make_shared<FileEvseSecurity>(
                                                FileEvseSecurity::Paths{cfg_.charge_point_id, cfg_.security}, false)
                                          : evse_security;

    if (security_profile >= 2) {
        const auto verify_loc = evse_security->get_verify_location(ocpp::CaCertificateType::CSMS);
        if (verify_loc.empty()) {
            EVLOG_error << "SecurityProfile=" << security_profile
                        << " requires a CSMS CA bundle, but none is configured/available "
                        << "(set security.csmsCaBundle or ensure system CA bundle exists)";
            return false;
        }
    }
    if (security_profile == 3) {
        const auto info =
            evse_security->get_leaf_certificate_info(ocpp::CertificateSigningUseEnum::ChargingStationCertificate, false);
        if (info.status != ocpp::GetCertificateInfoStatus::Accepted || !info.info.has_value()) {
            const auto expected_cert = (cfg_.security.client_cert_dir / (cfg_.charge_point_id + "_cert.pem")).string();
            const auto expected_key = (cfg_.security.client_key_dir / (cfg_.charge_point_id + "_key.pem")).string();
            EVLOG_error << "SecurityProfile=3 requires a charging station TLS client certificate+key. Expected "
                        << expected_cert << " and " << expected_key
                        << " (or exactly-one *_cert.pem and *_key.pem in the configured dirs). "
                        << "If your CSMS does not require mutual TLS, set SecurityProfile=2 instead. "
                        << "For a dev self-generated cert/key run: ./scripts/provision_ocpp_client_cert.py --config <charger.json> --dev-ca";
            return false;
        }
    }
    if (cfg_.security.mf_ca_bundle.empty()) {
        EVLOG_warning << "security.mfCaBundle is not configured; SignedUpdateFirmware certificate verification will fail";
    }

    charge_point_ = std::make_unique<ocpp::v16::ChargePoint>(config_str, cfg_.share_path, cfg_.user_config,
                                                             cfg_.database_dir, cfg_.sql_migrations,
                                                             cfg_.message_log_path, evse_security, std::nullopt);

    {
        MaintenanceManager::Callbacks cb{};
        cb.on_log_status = [this](int request_id, const std::string& status) {
            if (charge_point_) {
                charge_point_->on_log_status_notification(request_id, status);
            }
        };
        cb.on_firmware_status = [this](int request_id, ocpp::FirmwareStatusNotification status) {
            if (charge_point_) {
                charge_point_->on_firmware_update_status_notification(request_id, status);
            }
        };
        cb.any_active_transaction = [this]() {
            std::lock_guard<std::mutex> lock(session_mutex_);
            for (const auto& kv : sessions_) {
                if (kv.second.transaction_started) {
                    return true;
                }
            }
            return false;
        };
        maintenance_manager_ =
            std::make_unique<MaintenanceManager>(cfg_, maintenance_security, std::move(cb));
    }

    register_callbacks();
    if (cfg_.lab_bypass) {
        EVLOG_warning << "controller.labBypass=true: safety/isolation/weld/temperature faults may be bypassed. "
                         "DO NOT USE IN PRODUCTION.";
    }

    std::map<int, ocpp::v16::ChargePointStatus> connector_status_map;
    connector_status_map.emplace(0, ocpp::v16::ChargePointStatus::Available);
    for (const auto& connector : cfg_.connectors) {
        ConnectorState initial_state = ConnectorState::Available;
        if (hardware_) {
            auto st = sanitize_status(hardware_->get_status(connector.id), cfg_.lab_bypass);
            initial_state = initial_state_from_status(st);
        }
        connector_status_map.emplace(connector.id, to_ocpp_status(initial_state));
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            connector_state_[connector.id] = initial_state;
        }
    }

    if (!charge_point_->start(connector_status_map, ocpp::v16::BootReasonEnum::PowerUp)) {
        EVLOG_error << "Failed to start charge point";
        return false;
    }

    refresh_charging_profile_limits();
    seed_default_evse_limits();

    running_ = true;
    csms_connected_.store(false);
    auth_thread_running_ = true;
    auth_thread_ = std::thread([this]() { auth_loop(); });
    start_metering_threads();
    planner_thread_running_ = true;
    planner_thread_ = std::thread([this]() {
        while (running_ && planner_thread_running_) {
            try {
                const auto now = std::chrono::steady_clock::now();
                bool refresh_due = false;
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    refresh_due = profile_next_refresh_.has_value() && now >= *profile_next_refresh_;
                }
                if (refresh_due) {
                    refresh_charging_profile_limits();
                }
                if (last_reservation_expiry_check_.time_since_epoch().count() == 0 ||
                    (now - last_reservation_expiry_check_) >= RESERVATION_CHECK_INTERVAL) {
                    expire_reservations(now);
                    last_reservation_expiry_check_ = now;
                }
                apply_power_plan();
                maybe_refresh_status_notifications(now);
            } catch (const std::exception& e) {
                EVLOG_warning << "Planner thread error: " << e.what();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    csms_reconnect_thread_running_ = true;
    csms_reconnect_thread_ = std::thread([this]() {
        int backoff_s = 10;
        while (running_ && csms_reconnect_thread_running_) {
            if (csms_connected_.load()) {
                backoff_s = 10;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            const auto wake_at = std::chrono::steady_clock::now() + std::chrono::seconds(backoff_s);
            while (running_ && csms_reconnect_thread_running_ && std::chrono::steady_clock::now() < wake_at) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            if (!running_ || !csms_reconnect_thread_running_) {
                break;
            }
            if (!csms_connected_.load() && charge_point_) {
                EVLOG_warning << "CSMS disconnected; requesting websocket reconnect";
                try {
                    charge_point_->connect_websocket();
                } catch (const std::exception& e) {
                    EVLOG_warning << "connect_websocket() failed: " << e.what();
                }
            }
            backoff_s = std::min(backoff_s * 2, 60);
        }
    });
    return true;
}

void OcppAdapter::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    csms_reconnect_thread_running_ = false;
    planner_thread_running_ = false;
    auth_thread_running_ = false;
    auth_queue_cv_.notify_all();
    if (csms_reconnect_thread_.joinable()) {
        csms_reconnect_thread_.join();
    }
    if (planner_thread_.joinable()) {
        planner_thread_.join();
    }
    if (auth_thread_.joinable()) {
        auth_thread_.join();
    }
    for (auto& thread : meter_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    meter_threads_.clear();
    if (maintenance_manager_) {
        maintenance_manager_->shutdown();
        maintenance_manager_.reset();
    }
    if (charge_point_) {
        charge_point_->stop();
    }
}

void OcppAdapter::register_callbacks() {
    charge_point_->register_enable_evse_callback([this](std::int32_t connector) {
        return handle_enable_evse(connector);
    });

    charge_point_->register_disable_evse_callback([this](std::int32_t connector) {
        return handle_disable_evse(connector);
    });

    charge_point_->register_pause_charging_callback([this](std::int32_t connector) {
        return handle_pause_charging(connector);
    });

    charge_point_->register_resume_charging_callback([this](std::int32_t connector) {
        return handle_resume_charging(connector);
    });

    charge_point_->register_stop_transaction_callback([this](std::int32_t connector, ocpp::v16::Reason reason) {
        return handle_stop_transaction(connector, reason);
    });

    charge_point_->register_provide_token_callback(
        [this](const std::string& id_token, std::vector<std::int32_t> referenced_connectors, bool prevalidated) {
            hardware_->on_remote_start_token(id_token, referenced_connectors, prevalidated);
            AuthToken token;
            token.id_token = id_token;
            token.source = AuthTokenSource::RemoteStart;
            token.prevalidated = prevalidated;
            token.connector_hint = referenced_connectors.empty() ? 0 : referenced_connectors.front();
            token.received_at = std::chrono::steady_clock::now();
            std::optional<std::string> required_tag;
            std::optional<std::string> parent_tag;
            if (token.connector_hint > 0) {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                if (reservation_required_tag_.count(token.connector_hint)) {
                    required_tag = reservation_required_tag_[token.connector_hint];
                }
                if (reservation_parent_tag_.count(token.connector_hint)) {
                    parent_tag = reservation_parent_tag_[token.connector_hint];
                }
            }
            if (token.connector_hint > 0 && required_tag) {
                if (!token_matches_reservation(token.connector_hint, token.id_token, parent_tag)) {
                    EVLOG_warning << "RemoteStart token does not match reservation on connector "
                                  << token.connector_hint << "; ignoring remote start token";
                    return;
                }
            }
            if (token.connector_hint > 0 && required_tag) {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                reserved_connectors_[token.connector_hint] = false;
                reservation_required_tag_.erase(token.connector_hint);
                reservation_parent_tag_.erase(token.connector_hint);
                for (auto it = reservation_lookup_.begin(); it != reservation_lookup_.end();) {
                    if (it->second == token.connector_hint) {
                        it = reservation_lookup_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            ingest_auth_tokens({token}, token.received_at);
        });


    charge_point_->register_reserve_now_callback(
        [this](std::int32_t reservation_id, std::int32_t connector, ocpp::DateTime expiryDate,
               ocpp::CiString<20> idTag, std::optional<ocpp::CiString<20>> parent_id) {
            const auto expiry_tp = to_steady_from_utc(expiryDate.to_time_point());
            const auto now = std::chrono::steady_clock::now();
            if (expiry_tp <= now) {
                EVLOG_warning << "Rejecting reservation " << reservation_id << " on connector " << connector
                              << " (expiry already passed)";
                return ocpp::v16::ReservationStatus::Rejected;
            }
            const auto status = hardware_->reserve(reservation_id, connector, expiryDate, idTag.get(),
                                                   parent_id ? std::optional<std::string>(parent_id->get()) : std::nullopt);
            if (status == ocpp::v16::ReservationStatus::Accepted) {
                std::optional<int> replaced_connector;
                charge_point_->on_reservation_start(connector);
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    auto existing_it = reservation_id_by_connector_.find(connector);
                    if (existing_it != reservation_id_by_connector_.end()) {
                        const int old_id = existing_it->second;
                        reservation_lookup_.erase(old_id);
                        reservation_expiry_.erase(old_id);
                        reservation_id_by_connector_.erase(existing_it);
                        replaced_connector = connector;
                    }
                    reserved_connectors_[connector] = true;
                    reservation_lookup_[reservation_id] = connector;
                    reservation_id_by_connector_[connector] = reservation_id;
                    reservation_expiry_[reservation_id] = expiry_tp;
                    reservation_required_tag_[connector] = idTag.get();
                    reservation_parent_tag_[connector] = parent_id ? std::optional<std::string>(parent_id->get())
                                                                   : std::optional<std::string>{};
                }
                if (replaced_connector && charge_point_) {
                    charge_point_->on_reservation_end(*replaced_connector);
                }
            }
            return status;
        });

    charge_point_->register_cancel_reservation_callback([this](std::int32_t reservation_id) {
        const bool ok = hardware_->cancel_reservation(reservation_id);
        std::optional<int> connector_id;
            if (ok) {
                {
                    std::lock_guard<std::mutex> lock(plan_mutex_);
                    auto it = reservation_lookup_.find(reservation_id);
                    if (it != reservation_lookup_.end()) {
                        const int connector = it->second;
                        connector_id = connector;
                        reserved_connectors_[connector] = false;
                        reservation_lookup_.erase(it);
                        reservation_expiry_.erase(reservation_id);
                        reservation_id_by_connector_.erase(connector);
                        reservation_required_tag_.erase(connector);
                        reservation_parent_tag_.erase(connector);
                    }
                }
            if (connector_id && charge_point_) {
                charge_point_->on_reservation_end(*connector_id);
            }
        }
        return ok;
    });

    charge_point_->register_unlock_connector_callback([this](std::int32_t connector) {
        return unlock_connector(connector);
    });

    charge_point_->register_upload_diagnostics_callback(
        [this](const ocpp::v16::GetDiagnosticsRequest& request) {
            try {
                if (!maintenance_manager_) {
                    ocpp::v16::GetLogResponse resp;
                    resp.status = ocpp::v16::LogStatusEnumType::Rejected;
                    return resp;
                }
                return maintenance_manager_->handle_get_diagnostics(request);
            } catch (const std::exception& e) {
                EVLOG_error << "Diagnostics upload failed: " << e.what();
                ocpp::v16::GetLogResponse resp;
                resp.status = ocpp::v16::LogStatusEnumType::Rejected;
                return resp;
            }
        });

    charge_point_->register_upload_logs_callback(
        [this](const ocpp::v16::GetLogRequest& request) {
            try {
                if (!maintenance_manager_) {
                    ocpp::v16::GetLogResponse resp;
                    resp.status = ocpp::v16::LogStatusEnumType::Rejected;
                    return resp;
                }
                return maintenance_manager_->handle_get_log(request);
            } catch (const std::exception& e) {
                EVLOG_error << "Log upload failed: " << e.what();
                ocpp::v16::GetLogResponse resp;
                resp.status = ocpp::v16::LogStatusEnumType::Rejected;
                return resp;
            }
        });

    charge_point_->register_update_firmware_callback(
        [this](const ocpp::v16::UpdateFirmwareRequest msg) {
            try {
                if (maintenance_manager_) {
                    maintenance_manager_->handle_update_firmware(msg);
                } else if (charge_point_) {
                    charge_point_->on_firmware_update_status_notification(
                        -1, ocpp::FirmwareStatusNotification::InstallationFailed);
                }
            } catch (const std::exception& e) {
                EVLOG_error << "Firmware update failed: " << e.what();
                if (charge_point_) {
                    charge_point_->on_firmware_update_status_notification(
                        -1, ocpp::FirmwareStatusNotification::InstallationFailed);
                }
            }
        });

    charge_point_->register_signed_update_firmware_callback(
        [this](const ocpp::v16::SignedUpdateFirmwareRequest msg) {
            try {
                if (!maintenance_manager_) {
                    return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
                }
                return maintenance_manager_->handle_signed_update_firmware(msg);
            } catch (const std::exception& e) {
                EVLOG_error << "Signed firmware update failed: " << e.what();
                if (charge_point_) {
                    charge_point_->on_firmware_update_status_notification(
                        -1, ocpp::FirmwareStatusNotification::InstallationFailed);
                }
                return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
            }
        });

    charge_point_->register_set_connection_timeout_callback(
        [this](std::int32_t connection_timeout) { hardware_->set_connection_timeout(connection_timeout); });

    charge_point_->register_is_reset_allowed_callback(
        [this](const ocpp::v16::ResetType& reset_type) { return hardware_->is_reset_allowed(reset_type); });

    charge_point_->register_reset_callback(
        [this](const ocpp::v16::ResetType& reset_type) { hardware_->reset(reset_type); });

    charge_point_->register_data_transfer_callback(
        [this](const ocpp::v16::DataTransferRequest& request) { return handle_data_transfer_request(request); });

    register_clear_cache_callback_if_supported(*charge_point_, [this]() { handle_clear_cache(); });

    charge_point_->register_is_token_reserved_for_connector_callback(
        [this](const std::int32_t connector, const std::string& id_token) {
            std::optional<std::string> parent;
            bool reserved = false;
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if (reservation_parent_tag_.count(connector)) {
                    parent = reservation_parent_tag_[connector];
                }
                reserved = reserved_connectors_.count(connector) ? reserved_connectors_[connector] : false;
            }
            const bool matches = token_matches_reservation(connector, id_token, parent);
            if (!reserved) {
                return ocpp::ReservationCheckStatus::NotReserved;
            }
            if (matches) {
                return ocpp::ReservationCheckStatus::ReservedForToken;
            }
            if (parent && !parent->empty()) {
                return ocpp::ReservationCheckStatus::ReservedForOtherTokenAndHasParentToken;
            }
            return ocpp::ReservationCheckStatus::ReservedForOtherToken;
        });

    charge_point_->register_generic_configuration_key_changed_callback(
        [this](const ocpp::v16::KeyValue& key_value) { handle_configuration_key_change(key_value); });

    charge_point_->register_set_system_time_callback(
        [](const std::string& system_time) { EVLOG_info << "CSMS provided system time: " << system_time; });

    charge_point_->register_boot_notification_response_callback(
        [this](const ocpp::v16::BootNotificationResponse& resp) {
            std::stringstream ss;
            ss << resp.currentTime;
            EVLOG_info << "BootNotification response status=" << resp.status << " interval=" << resp.interval
                       << " currentTime=" << ss.str();
            const bool accepted = resp.status == ocpp::v16::RegistrationStatus::Accepted;
            boot_accepted_.store(accepted);
            if (accepted) {
                request_status_refresh("boot_accepted");
            }
        });

    charge_point_->register_connection_state_changed_callback([this](bool is_connected) {
        csms_connected_.store(is_connected);
        EVLOG_info << "CSMS websocket state changed: " << (is_connected ? "connected" : "disconnected");
        if (is_connected) {
            request_status_refresh("csms_connected");
        }
    });

    charge_point_->register_signal_set_charging_profiles_callback([this]() { refresh_charging_profile_limits(); });

    charge_point_->register_all_connectors_unavailable_callback(
        []() { EVLOG_info << "All connectors unavailable; safe for maintenance/firmware actions."; });

    charge_point_->register_security_event_callback(
        [](const std::string& type, const std::string& tech_info) {
            EVLOG_warning << "Security event: " << type << " details=" << tech_info;
        });
}

bool OcppAdapter::handle_enable_evse(std::int32_t connector) {
    if (connector == 0) {
        bool ok = true;
        for (const auto& c : cfg_.connectors) {
            ok = handle_enable_evse(c.id) && ok;
        }
        return ok;
    }

    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        evse_disabled_[connector] = false;
    }
    const bool ok = hardware_->enable(connector);
    if (!ok) {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        evse_disabled_[connector] = true;
    }
    return ok;
}

bool OcppAdapter::handle_disable_evse(std::int32_t connector) {
    if (connector == 0) {
        bool ok = true;
        for (const auto& c : cfg_.connectors) {
            ok = handle_disable_evse(c.id) && ok;
        }
        return ok;
    }

    bool had_session = false;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        had_session = sessions_.count(connector) > 0;
    }
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        evse_disabled_[connector] = true;
    }
    if (had_session) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            stop_origin_hint_[connector] = "ocpp_disable_evse";
        }
        finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
    }
    return hardware_->disable(connector);
}

bool OcppAdapter::handle_pause_charging(std::int32_t connector) {
    if (connector == 0) {
        for (const auto& c : cfg_.connectors) {
            (void)handle_pause_charging(c.id);
        }
        return true;
    }
    std::lock_guard<std::mutex> lock(plan_mutex_);
    paused_evse_[connector] = true;
    return true;
}

bool OcppAdapter::handle_resume_charging(std::int32_t connector) {
    if (connector == 0) {
        for (const auto& c : cfg_.connectors) {
            (void)handle_resume_charging(c.id);
        }
        return true;
    }
    std::lock_guard<std::mutex> lock(plan_mutex_);
    paused_evse_[connector] = false;
    return true;
}

bool OcppAdapter::handle_stop_transaction(std::int32_t connector, ocpp::v16::Reason reason) {
    if (connector == 0) {
        bool ok = true;
        for (const auto& c : cfg_.connectors) {
            ok = handle_stop_transaction(c.id, reason) && ok;
        }
        return ok;
    }

    const bool ok = hardware_->stop_transaction(connector, reason);
    if (ok) {
        EVLOG_info << "Received OCPP stop request connector=" << connector
                   << " reason=" << ocpp::v16::conversions::reason_to_string(reason);
        const auto status = sanitize_status(hardware_->get_status(connector), cfg_.lab_bypass);
        const bool vehicle_present = infer_vehicle_present(status, true);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            post_stop_plugged_[connector] = vehicle_present;
            post_stop_time_[connector] =
                vehicle_present ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
            stop_origin_hint_[connector] = "ocpp_remote_stop";
        }
        finish_transaction(connector, reason, std::nullopt, vehicle_present);
    }
    return ok;
}

void OcppAdapter::start_metering_threads() {
    for (const auto& connector : cfg_.connectors) {
        meter_threads_.emplace_back([this, connector_id = connector.id]() { metering_loop(connector_id); });
    }
}

void OcppAdapter::apply_energy_fallback(std::int32_t connector, const GunStatus& status,
                                        ocpp::Measurement& measurement,
                                        const std::chrono::steady_clock::time_point& now) {
    double measured_wh = measurement.power_meter.energy_Wh_import.total;
    if (!std::isfinite(measured_wh) || measured_wh < 0.0) {
        measured_wh = 0.0;
    }

    double power_w = extract_total_value(measurement.power_meter.power_W);
    if (power_w <= 1.0) {
        power_w = extract_dc_value(measurement.power_meter.power_W).value_or(power_w);
    }
    if ((!std::isfinite(power_w) || power_w <= 1.0) && status.present_power_w) {
        power_w = std::max(power_w, status.present_power_w.value());
    }
    if ((!std::isfinite(power_w) || power_w <= 1.0) && status.present_voltage_v && status.present_current_a) {
        power_w = status.present_voltage_v.value() * status.present_current_a.value();
    }
    if (!std::isfinite(power_w) || power_w < 0.0) {
        power_w = 0.0;
    }

    std::lock_guard<std::mutex> lock(meter_mutex_);
    auto& fallback_wh = fallback_energy_wh_[connector];
    auto& last_ts = fallback_energy_last_update_[connector];
    if (last_ts.time_since_epoch().count() == 0) {
        last_ts = now;
        fallback_wh = std::max(fallback_wh, measured_wh);
    } else {
        auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ts).count();
        if (dt_ms < 0) {
            dt_ms = 0;
        }
        dt_ms = std::min<int64_t>(dt_ms, 2000);
        last_ts = now;
        const bool delivery_context =
            status.relay_closed || status.hlc_power_ready ||
            (status.target_current_a && status.target_current_a.value() > 0.5);
        if (delivery_context && power_w > 1.0 && dt_ms > 0) {
            fallback_wh += (power_w * static_cast<double>(dt_ms)) / 3600000.0;
        }
        fallback_wh = std::max(fallback_wh, measured_wh);
    }

    // Prefer hardware meter when it reports a sane non-zero value. Use fallback only
    // when meter energy is absent/zero so OCPP meterStart/meterStop do not stick at 0.
    if (measured_wh > 0.01) {
        measurement.power_meter.energy_Wh_import.total = measured_wh;
    } else if (fallback_wh > 0.01) {
        measurement.power_meter.energy_Wh_import.total = fallback_wh;
    } else {
        measurement.power_meter.energy_Wh_import.total = 0.0;
    }
}

void OcppAdapter::metering_loop(std::int32_t connector) {
    int current_interval_s = meter_interval_seconds_for_connector(connector);
    auto meter_period = std::chrono::seconds(std::max(1, current_interval_s));
    const auto control_tick = std::chrono::milliseconds(50);
    auto next_meter_push = std::chrono::steady_clock::now();
    bool telemetry_stale_active = false;
    std::chrono::steady_clock::time_point telemetry_stale_log_ts{};
    std::chrono::steady_clock::time_point telemetry_stale_fail_since{};
    std::chrono::steady_clock::time_point telemetry_overdue_since{};
    std::chrono::steady_clock::time_point charge_complete_since{};
    std::chrono::steady_clock::time_point evse_limit_ack_stale_since{};
    std::chrono::steady_clock::time_point evse_limit_ack_stale_log_ts{};
    std::chrono::steady_clock::time_point module_degraded_since{};
    std::chrono::steady_clock::time_point module_degraded_clear_since{};
    std::chrono::steady_clock::time_point module_unavailable_since{};
    std::chrono::steady_clock::time_point suspended_no_power_since{};
    std::chrono::steady_clock::time_point suspended_no_power_log_ts{};
    bool module_degraded_latched = false;

    while (running_) {
        try {
            const auto loop_start = std::chrono::steady_clock::now();
            const auto now = loop_start;
            const int desired_interval = meter_interval_seconds_for_connector(connector);
            if (desired_interval != current_interval_s) {
                current_interval_s = desired_interval;
                meter_period = std::chrono::seconds(std::max(1, current_interval_s));
                next_meter_push = now + meter_period;
            }
            const bool push_meter_now = loop_start >= next_meter_push;
            const bool auth_timeout_enabled = cfg_.auth_wait_timeout_s > 0;
            const bool power_request_timeout_enabled = cfg_.power_request_timeout_s > 0;
            const auto auth_wait_timeout = std::chrono::seconds(auth_timeout_enabled
                                                                     ? std::max(1, cfg_.auth_wait_timeout_s)
                                                                     : 0);
            const auto power_request_timeout = std::chrono::seconds(power_request_timeout_enabled
                                                                         ? std::max(1, cfg_.power_request_timeout_s)
                                                                         : 0);
            auto disable_local = [&](const char* reason) {
                mark_local_hw_disable(connector, reason ? reason : "");
                hardware_->disable(connector);
            };
            auto hw_tokens = hardware_->poll_auth_tokens();
            if (!hw_tokens.empty()) {
                std::set<std::int32_t> rfid_active_connectors;
                std::vector<AuthToken> ingest_tokens;
                ingest_tokens.reserve(hw_tokens.size());
                for (const auto& token : hw_tokens) {
                    if (token.source == AuthTokenSource::RFID && token.connector_hint > 0) {
                        if (rfid_active_connectors.count(token.connector_hint)) {
                            continue;
                        }
                        const auto trimmed = clamp_id_token(token.id_token);
                        std::optional<std::string> active_token;
                        bool tx_started = false;
                        bool suppress_repeat = false;
                        bool matching_active_token = false;
                        {
                            std::lock_guard<std::mutex> lock(session_mutex_);
                            const auto it = sessions_.find(token.connector_hint);
                            if (it != sessions_.end()) {
                                tx_started = it->second.transaction_started;
                                active_token = it->second.id_token;
                            }

                            auto& latch = rfid_tap_latch_[token.connector_hint];
                            const bool has_prev = latch.last_seen.time_since_epoch().count() != 0;
                            const bool same_token = latch.token == trimmed;
                            const bool within_window = has_prev && (now - latch.last_seen) < RFID_TAP_LATCH_WINDOW;
                            const bool same_presence = same_token && within_window;
                            if (!same_presence) {
                                latch.token = trimmed;
                                latch.consumed = false;
                            }
                            latch.last_seen = now;

                            matching_active_token = tx_started && active_token && trimmed == *active_token;
                            const bool should_ingest = !tx_started;
                            const bool actionable = should_ingest || matching_active_token;
                            suppress_repeat = actionable && same_presence && latch.consumed;
                            if (actionable && !suppress_repeat) {
                                latch.consumed = true;
                            }
                        }

                        if (suppress_repeat) {
                            continue;
                        }

                        if (tx_started) {
                            rfid_active_connectors.insert(token.connector_hint);
                            if (matching_active_token) {
                                const auto st = sanitize_status(hardware_->get_status(token.connector_hint), cfg_.lab_bypass);
                                const bool power_delivering = st.relay_closed || st.hlc_power_ready;
                                if (!power_delivering) {
                                    EVLOG_debug << "Ignoring RFID token on connector " << token.connector_hint
                                                << " (duplicate tap before power delivery)";
                                    continue;
                                }
                                EVLOG_info << "RFID token matched active session on connector "
                                           << token.connector_hint << "; stopping transaction";
                                const bool ok = hardware_->stop_transaction(token.connector_hint,
                                                                            ocpp::v16::Reason::Local);
                                if (ok) {
                                    const bool vehicle_present = infer_vehicle_present(st, true);
                                    std::optional<ocpp::CiString<20>> id_tag_end;
                                    if (!trimmed.empty()) {
                                        id_tag_end = ocpp::CiString<20>(trimmed);
                                    }
                                    {
                                        std::lock_guard<std::mutex> lock(state_mutex_);
                                        post_stop_plugged_[token.connector_hint] = vehicle_present;
                                        post_stop_time_[token.connector_hint] =
                                            vehicle_present ? now : std::chrono::steady_clock::time_point{};
                                        stop_origin_hint_[token.connector_hint] = "local_rfid_stop";
                                    }
                                    finish_transaction(token.connector_hint, ocpp::v16::Reason::Local, id_tag_end,
                                                       vehicle_present);
                                } else {
                                    EVLOG_warning << "RFID stop request failed on connector "
                                                  << token.connector_hint;
                                }
                            } else {
                                EVLOG_debug << "Ignoring RFID token on connector " << token.connector_hint
                                            << " (active session uses a different id token)";
                            }
                            continue;
                        }

                        AuthToken token_copy = token;
                        token_copy.id_token = trimmed;
                        ingest_tokens.push_back(std::move(token_copy));
                        continue;
                    }
                    ingest_tokens.push_back(token);
                }
                if (!ingest_tokens.empty()) {
                    ingest_auth_tokens(ingest_tokens, now);
                }
            }
            auto status = sanitize_status(hardware_->get_status(connector), cfg_.lab_bypass);
            record_presence_state(connector, status.plugged_in, now);
            const bool vehicle_present = infer_vehicle_present(status);
            auto mark_post_stop = [&](bool active) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                post_stop_plugged_[connector] = active;
                post_stop_time_[connector] = active ? now : std::chrono::steady_clock::time_point{};
            };
            bool post_stop_plugged = false;
            bool pending_session_stop = false;
            bool suppress_available_event = false; // avoid duplicate Available after deferred session stop
            std::optional<std::string> pending_session_stop_id;
            process_post_stop_state(connector, status, now, &post_stop_plugged, &pending_session_stop,
                                    &pending_session_stop_id);
            if (pending_session_stop_id && charge_point_) {
                charge_point_->on_session_stopped(connector, *pending_session_stop_id);
                suppress_available_event = true;
            }

            auto finish_and_mark = [&](ocpp::v16::Reason reason,
                                       std::optional<ocpp::CiString<20>> id_tag_end = std::nullopt) {
                if (vehicle_present) {
                    mark_post_stop(true);
                }
                finish_transaction(connector, reason, id_tag_end, vehicle_present);
            };

            ActiveSession session{};
            bool had_session = false;
            bool pending_changed = false;
            bool fault = false;

            auto measurement = hardware_->sample_meter(connector);
            apply_energy_fallback(connector, status, measurement, now);
            // Enrich metering with temperatures derived from PLC/module telemetry when available.
            auto add_temp = [&](double temp_c, const std::string& location) {
                if (!std::isfinite(temp_c)) return;
                if (temp_c < -40.0 || temp_c > 200.0) return;
                ocpp::Temperature t{};
                t.value = static_cast<float>(temp_c);
                t.location = location;
                measurement.temperature_C.push_back(t);
            };
            add_temp(status.connector_temp_c, "Connector");
            for (std::size_t idx = 0; idx < status.module_temp_c.size(); ++idx) {
                add_temp(status.module_temp_c[idx], "Module" + std::to_string(idx));
            }
            // Check measurement vs reported telemetry for consistency.
            auto dc_voltage = extract_dc_value(measurement.power_meter.voltage_V);
            auto dc_current = extract_dc_value(measurement.power_meter.current_A);
            auto dc_power = extract_dc_value(measurement.power_meter.power_W);
            const auto has_measured = dc_voltage || dc_current || dc_power;
            const bool has_status = status.present_voltage_v || status.present_current_a || status.present_power_w;
            if (has_measured && has_status) {
                double v_meas = dc_voltage.value_or(status.present_voltage_v.value_or(0.0));
                double i_meas = dc_current.value_or(status.present_current_a.value_or(0.0));
                double p_meas = dc_power.value_or(status.present_power_w.value_or(0.0));
                double v_stat = status.present_voltage_v.value_or(v_meas);
                double i_stat = status.present_current_a.value_or(i_meas);
                double p_stat = status.present_power_w.value_or(p_meas);
                auto within = [](double a, double b, double rel, double abs_tol) {
                    const double diff = std::fabs(a - b);
                    return diff <= abs_tol || diff <= std::max(std::fabs(a), std::fabs(b)) * rel;
                };
                const bool v_ok = within(v_meas, v_stat, 0.05, 20.0);
                const bool i_ok = within(i_meas, i_stat, 0.10, 2.0);
                const bool p_ok = within(p_meas, p_stat, 0.10, 500.0);
                const int mismatch_axes = (!v_ok ? 1 : 0) + (!i_ok ? 1 : 0) + (!p_ok ? 1 : 0);
                const bool mismatch_under_load =
                    status.relay_closed &&
                    ((std::fabs(i_meas) >= 2.0) || (std::fabs(i_stat) >= 2.0) ||
                     (std::fabs(p_meas) >= 800.0) || (std::fabs(p_stat) >= 800.0));
                bool telemetry_mismatch_trip = false;
                int mismatch_count = 0;
                {
                    std::lock_guard<std::mutex> lock(telemetry_mutex_);
                    auto& count = telemetry_mismatch_count_[connector];
                    // Only arm mismatch trips when at least two channels disagree under real load.
                    // This avoids false trips from transient/current-sensor skew at low power.
                    if (mismatch_under_load && mismatch_axes >= 2) {
                        count++;
                    } else {
                        count = 0;
                    }
                    mismatch_count = count;
                    if (count >= 15) {
                        telemetry_mismatch_trip = true;
                        count = 0;
                    }
                }
                if (telemetry_mismatch_trip) {
                    if (!cfg_.lab_bypass) {
                        EVLOG_error << "Telemetry mismatch on connector " << connector
                                    << " meas(V=" << v_meas << " I=" << i_meas << " P=" << p_meas << ")"
                                    << " status(V=" << v_stat << " I=" << i_stat << " P=" << p_stat << ")"
                                    << " mismatch_axes=" << mismatch_axes
                                    << " mismatch_count=" << mismatch_count;
                        finish_and_mark(ocpp::v16::Reason::PowerLoss, std::nullopt);
                        disable_local("telemetry_mismatch");
                        had_session = false;
                        fault = true;
                    } else {
                        EVLOG_warning << "Telemetry mismatch observed on connector " << connector
                                      << " meas(V=" << v_meas << " I=" << i_meas << " P=" << p_meas << ")"
                                      << " status(V=" << v_stat << " I=" << i_stat << " P=" << p_stat << ")"
                                      << " mismatch_axes=" << mismatch_axes
                                      << " mismatch_count=" << mismatch_count
                                      << " -- lab_bypass=1, keeping session active";
                    }
                }
            } else {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                telemetry_mismatch_count_[connector] = 0;
            }
            bool constrained = false;
            bool paused = false;
            bool disabled_by_csms = false;
            bool disabled_by_local = false;
            bool local_fault_latched = false;
            bool reserved = false;
            std::string required_tag;
            std::optional<std::string> parent_tag;
            std::optional<PendingToken> pending_auth;
            std::string pending_auth_session_id;
            bool auto_auth_granted = false;
            bool auth_in_flight = false;
            {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                constrained = power_constrained_[connector];
                paused = paused_evse_[connector];
                disabled_by_csms = evse_disabled_[connector];
                reserved = reserved_connectors_[connector];
                if (reservation_required_tag_.count(connector)) {
                    required_tag = reservation_required_tag_[connector];
                }
                if (reservation_parent_tag_.count(connector)) {
                    parent_tag = reservation_parent_tag_[connector];
                }
            }
            {
                std::lock_guard<std::mutex> lock(auth_queue_mutex_);
                auth_in_flight = auth_in_flight_.count(connector) > 0;
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                const auto dit = local_hw_disable_.find(connector);
                disabled_by_local = dit != local_hw_disable_.end() && dit->second;
                const auto fit = last_local_fault_reason_.find(connector);
                local_fault_latched = fit != last_local_fault_reason_.end() && !fit->second.empty();
            }
            const bool disabled = disabled_by_csms || disabled_by_local;

            bool notify_session_started = false;
            ocpp::SessionStartedReason session_start_reason = ocpp::SessionStartedReason::EVConnected;
            std::string session_start_id;
            bool clear_reservation_after_start = false;
            bool persist_pending = false;
            {
                std::lock_guard<std::mutex> lock(session_mutex_);
                auto qit = pending_tokens_.find(connector);
                if (qit != pending_tokens_.end()) {
                    for (auto pit = qit->second.begin(); pit != qit->second.end();) {
                        if (pit->expires_at <= now) {
                            pending_changed = true;
                            pit = qit->second.erase(pit);
                        } else {
                            ++pit;
                        }
                    }
                }
                auto it = sessions_.find(connector);
                const std::optional<std::string> reservation_required =
                    (reserved && !required_tag.empty()) ? std::optional<std::string>(required_tag) : std::nullopt;
                const auto reservation_parent = reserved ? parent_tag : std::optional<std::string>{};
                const bool blocked_for_session = !status.safety_ok || status.estop || status.earth_fault ||
                    status.comm_fault || status.cp_fault || status.hlc_charge_complete || post_stop_plugged ||
                    pending_session_stop;
                bool create_session = !disabled && status.plugged_in && it == sessions_.end() && !blocked_for_session;
                if (create_session) {
                    ActiveSession s{};
                    s.session_id = make_session_id();
                    s.meter_start_wh = measurement.power_meter.energy_Wh_import.total;
                    s.connected_at = now;
                    s.ev_connected = true;
                    s.pending_started = now;
                    s.last_seen_plugged = status.plugged_in ? now : std::chrono::steady_clock::time_point{};
                    bool have_token = false;
                    if (!auth_in_flight) {
                        if (auto pending = pop_next_pending_token(connector, now, reservation_required,
                                                                  reservation_parent, &pending_changed)) {
                            pending_auth = pending;
                            pending_auth_session_id = s.session_id;
                            have_token = true;
                        }
                    }
                    if (!have_token && !reserved) {
                        if (cfg_.free_mode) {
                            const auto tag = clamp_id_token(cfg_.default_tag);
                            if (!tag.empty()) {
                                s.authorized = true;
                                s.id_token = tag;
                                s.authorized_at = now;
                                s.power_wait_started_at = now;
                                s.token_source = AuthTokenSource::Autocharge;
                                auto_auth_granted = true;
                                have_token = true;
                            } else {
                                EVLOG_warning << "FreeMode enabled but no DefaultTag configured; falling back to auth pending";
                                set_auth_state(connector, AuthorizationState::Pending);
                                have_token = true;
                            }
                        } else {
                            set_auth_state(connector, AuthorizationState::Pending);
                            have_token = true;
                        }
                    }
                    if (reserved && !have_token) {
                        create_session = false;
                        EVLOG_info << "Connector " << connector << " reserved, waiting for matching token before starting session";
                        set_auth_state(connector, AuthorizationState::Pending);
                    } else {
                        sessions_[connector] = s;
                        {
                            std::lock_guard<std::mutex> meter_lock(meter_mutex_);
                            fallback_energy_wh_[connector] = std::max(0.0, s.meter_start_wh);
                            fallback_energy_last_update_[connector] = now;
                        }
                        session_start_id = s.session_id;
                        session_start_reason = s.authorized ? ocpp::SessionStartedReason::Authorized
                                                            : ocpp::SessionStartedReason::EVConnected;
                        notify_session_started = true;
                        it = sessions_.find(connector);
                        mark_post_stop(false);
                        clear_reservation_after_start = reserved;
                    }
                } else if (it != sessions_.end()) {
                    it->second.ev_connected = status.plugged_in;
                    if (!it->second.pending_started.time_since_epoch().count()) {
                        it->second.pending_started = now;
                    }
                    if (status.plugged_in) {
                        it->second.last_seen_plugged = now;
                    }
                    if (!it->second.authorized) {
                        if (!auth_in_flight) {
                            if (auto pending = pop_next_pending_token(connector, now, std::nullopt, std::nullopt,
                                                                      &pending_changed)) {
                                pending_auth = pending;
                                pending_auth_session_id = it->second.session_id;
                            }
                        }
                        if (!pending_auth) {
                            const auto auth_state = get_auth_state(connector);
                            if (auth_state == AuthorizationState::Denied) {
                                if (cfg_.auth_denied_hold_s <= 0) {
                                    set_auth_state(connector, AuthorizationState::Pending);
                                } else if (auto denied_since = get_auth_denied_since(connector)) {
                                    const auto hold = std::chrono::seconds(cfg_.auth_denied_hold_s);
                                    if ((now - *denied_since) >= hold) {
                                        set_auth_state(connector, AuthorizationState::Pending);
                                    }
                                } else {
                                    set_auth_state(connector, AuthorizationState::Pending);
                                }
                            } else {
                                set_auth_state(connector, AuthorizationState::Pending);
                            }
                        }
                    }
                    mark_post_stop(false);
                }

                if (it != sessions_.end()) {
                    session = it->second;
                    had_session = true;
                }
                if (!status.plugged_in && it == sessions_.end()) {
                    set_auth_state(connector, AuthorizationState::Unknown);
                    autocharge_retry_not_before_.erase(connector);
                    autocharge_retry_fail_count_.erase(connector);
                }
                if (pending_changed) {
                    persist_pending = true;
                }
            }
            if (persist_pending) {
                persist_pending_tokens();
            }
            if (auto_auth_granted) {
                set_auth_state(connector, AuthorizationState::Granted);
            }
            bool force_auth_denied = false;
            std::optional<std::string> autocharge_reject_id;
            if (pending_auth) {
                if (pending_auth->token.prevalidated) {
                    const auto state = apply_authorization_result(connector, pending_auth_session_id, *pending_auth, true);
                    if (pending_auth->token.source == AuthTokenSource::Autocharge &&
                        state == AuthorizationState::Denied) {
                        autocharge_reject_id = pending_auth->token.id_token;
                        force_auth_denied = true;
                    }
                } else {
                    set_auth_state(connector, AuthorizationState::Pending);
                    enqueue_auth_request(connector, pending_auth_session_id, *pending_auth);
                }
                {
                    std::lock_guard<std::mutex> lock(session_mutex_);
                    auto it = sessions_.find(connector);
                    if (it != sessions_.end()) {
                        session = it->second;
                        had_session = true;
                    }
                }
            }
            if (had_session) {
                if (auto auth_result = pop_auth_result(connector, session.session_id)) {
                    const auto state =
                        apply_authorization_result(connector, auth_result->session_id, auth_result->pending,
                                                   auth_result->accepted);
                    if (auth_result->pending.token.source == AuthTokenSource::Autocharge &&
                        state == AuthorizationState::Denied) {
                        autocharge_reject_id = auth_result->pending.token.id_token;
                        force_auth_denied = true;
                    }
                    std::lock_guard<std::mutex> lock(session_mutex_);
                    auto it = sessions_.find(connector);
                    if (it != sessions_.end()) {
                        session = it->second;
                        had_session = true;
                    }
                }
            }
            if (notify_session_started) {
                bool session_active = false;
                {
                    std::lock_guard<std::mutex> lock(session_mutex_);
                    auto it = sessions_.find(connector);
                    session_active = it != sessions_.end() && it->second.session_id == session_start_id;
                }
                if (session_active) {
                    charge_point_->on_session_started(connector, session_start_id, session_start_reason, std::nullopt);
                }
            }
            if (clear_reservation_after_start) {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                reserved_connectors_[connector] = false;
                reservation_required_tag_.erase(connector);
                reservation_parent_tag_.erase(connector);
                reservation_id_by_connector_.erase(connector);
                for (auto rit = reservation_lookup_.begin(); rit != reservation_lookup_.end();) {
                    if (rit->second == connector) {
                        reservation_expiry_.erase(rit->first);
                        rit = reservation_lookup_.erase(rit);
                    } else {
                        ++rit;
                    }
                }
            }

            const auto hlc_out = apply_hlc_control(connector, status, had_session, session, post_stop_plugged,
                                                   autocharge_reject_id, force_auth_denied, now);
            if (hlc_out.digital_update) {
                hardware_->set_digital_comm_enabled(connector, hlc_out.desired_digital);
            }
            if (hlc_out.pnc_block_update) {
                hardware_->set_pnc_blocked(connector, hlc_out.desired_pnc_blocked);
            }
            if (hlc_out.force_auth_denied && !session.authorized) {
                set_auth_state(connector, AuthorizationState::Denied);
            }
            const auto cfg_it = std::find_if(cfg_.connectors.begin(), cfg_.connectors.end(),
                                             [&](const ConnectorConfig& c) { return c.id == connector; });
            const bool lock_required = cfg_it != cfg_.connectors.end() ? cfg_it->require_lock : true;
            bool cp_fault_grace = false;
            if (status.cp_fault) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    auto& since = cp_fault_since_[connector];
                    if (since.time_since_epoch().count() == 0) {
                        since = now;
                    }
                    if ((now - since) < CP_FAULT_GRACE_MS) {
                        cp_fault_grace = true;
                    }
                }
                if (cp_fault_grace) {
                    disable_local("cp_fault_grace");
                }
            } else {
                std::lock_guard<std::mutex> lock(state_mutex_);
                cp_fault_since_[connector] = std::chrono::steady_clock::time_point{};
            }
            if (status.cp_fault && !cp_fault_grace) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (had_session) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("cp_fault");
                    had_session = false;
                } else if (!already_faulted) {
                    disable_local("cp_fault");
                }
                fault = true;
            }
            const bool comm_fault_relevant =
                status.plugged_in || status.relay_closed || status.hlc_stage > 0 || had_session || status.authorization_granted;
            if (!status.safety_ok || status.estop || status.earth_fault || (comm_fault_relevant && status.comm_fault)) {
                ocpp::v16::Reason stop_reason = ocpp::v16::Reason::Other;
                if (status.estop) {
                    stop_reason = ocpp::v16::Reason::EmergencyStop;
                } else if (status.comm_fault) {
                    const bool power_path_active = status.relay_closed || is_hlc_precharge_phase(status) ||
                                                   status.hlc_power_ready || status.hlc_stage >= HLC_MIN_POWER_STAGE;
                    stop_reason = power_path_active ? ocpp::v16::Reason::PowerLoss : ocpp::v16::Reason::Other;
                } else if (status.earth_fault) {
                    stop_reason = ocpp::v16::Reason::Other;
                }
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(stop_reason, std::nullopt);
                    disable_local("safety_or_comm_fault");
                    had_session = false;
                }
                fault = true;
            }
            if (!fault && status.isolation_fault) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("isolation_fault");
                    had_session = false;
                }
                fault = true;
            }
            if (!fault && status.overtemp_fault) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("overtemp_fault");
                    had_session = false;
                }
                fault = true;
            }
            if (!fault && status.overcurrent_fault) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("overcurrent_fault");
                    had_session = false;
                }
                fault = true;
            }
            if (!fault && status.gc_welded) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("gc_welded");
                    had_session = false;
                }
                fault = true;
            }
            if (!fault && status.mc_welded) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("mc_welded");
                    had_session = false;
                }
                fault = true;
            }
            uint8_t module_healthy_mask = status.module_healthy_mask;
            uint8_t module_fault_mask = status.module_fault_mask;
            int modules_requested = 0;
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                const auto it = last_module_alloc_.find(connector);
                if (it != last_module_alloc_.end()) {
                    modules_requested = it->second;
                }
            }
            if (module_controller_) {
                bool any_valid = false;
                uint8_t healthy = 0;
                uint8_t fault = 0;
                const auto slot_it = connector_module_slots_.find(connector);
                if (slot_it != connector_module_slots_.end()) {
                    for (int idx = 0; idx < 2; ++idx) {
                        const int slot_id = slot_it->second[idx];
                        if (slot_id <= 0) continue;
                        const auto snap = module_controller_->snapshot_for_slot(slot_id);
                        if (!snap.valid) continue;
                        any_valid = true;
                        healthy |= snap.healthy_mask;
                        fault |= snap.fault_mask;
                    }
                } else {
                    const Slot* slot = find_slot_for_gun(connector);
                    if (slot) {
                        const auto snap = module_controller_->snapshot_for_slot(slot->id);
                        if (snap.valid) {
                            any_valid = true;
                            healthy = snap.healthy_mask;
                            fault = snap.fault_mask;
                        }
                    }
                }
                if (any_valid) {
                    module_healthy_mask = healthy;
                    module_fault_mask = fault;
                }
            }
            const uint8_t usable_modules =
                static_cast<uint8_t>(module_healthy_mask & static_cast<uint8_t>(~module_fault_mask));
            // Only fault on missing modules when we actually need them (precharge/power delivery).
            const bool precharge_hint = status.plugged_in && !post_stop_plugged && !status.hlc_charge_complete &&
                                        is_hlc_precharge_phase(status) &&
                                        !disabled && !paused && status.authorization_granted;
            const bool power_ready_hint = had_session && session.authorized &&
                (status.relay_closed || power_delivery_requested(status, lock_required)) && !disabled;
            const bool module_commanded = modules_requested > 0;
            const bool module_capability_needed =
                status.relay_closed || is_hlc_precharge_phase(status) || status.hlc_stage >= HLC_MIN_POWER_STAGE ||
                status.hlc_power_ready;
            const bool module_relevant = status.plugged_in || had_session;
            const bool need_modules = module_relevant && (power_ready_hint || precharge_hint) && module_commanded &&
                                      module_capability_needed;
            bool module_unavailable_fault = false;
            if (!fault) {
                if (need_modules && usable_modules == 0) {
                    bool expired = false;
                    {
                        std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                        auto& ts = module_missing_since_[connector];
                        if (ts.time_since_epoch().count() == 0) {
                            ts = now;
                        } else {
                            const auto timeout_ms = std::max({2000, cfg_.precharge_timeout_ms, cfg_.telemetry_timeout_ms});
                            if ((now - ts) > std::chrono::milliseconds(timeout_ms)) {
                                expired = true;
                            }
                        }
                    }
                    module_unavailable_fault = expired;
                } else {
                    std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                    module_missing_since_.erase(connector);
                }
            } else {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                module_missing_since_.erase(connector);
            }
            if (!fault && module_unavailable_fault) {
                constexpr auto kModuleUnavailableGrace = std::chrono::seconds(30);
                if (module_unavailable_since.time_since_epoch().count() == 0) {
                    module_unavailable_since = now;
                }
                if ((now - module_unavailable_since) >= kModuleUnavailableGrace) {
                    EVLOG_error << "No healthy power modules available on connector " << connector
                                << " for >" << std::chrono::duration_cast<std::chrono::seconds>(kModuleUnavailableGrace).count()
                                << "s during precharge/power delivery; stopping session";
                    finish_and_mark(ocpp::v16::Reason::PowerLoss, std::nullopt);
                    disable_local("modules_unavailable");
                    had_session = false;
                    fault = true;
                    module_unavailable_since = std::chrono::steady_clock::time_point{};
                } else {
                    static std::map<int, std::chrono::steady_clock::time_point> last_module_fault_log;
                    auto& last_log = last_module_fault_log[connector];
                    if (last_log.time_since_epoch().count() == 0 ||
                        (now - last_log) > std::chrono::seconds(2)) {
                        const auto age_ms =
                            std::chrono::duration_cast<std::chrono::milliseconds>(now - module_unavailable_since).count();
                        EVLOG_warning << "No healthy power modules available on connector " << connector
                                      << " during precharge/power delivery; waiting for recovery (age_ms=" << age_ms
                                      << ")";
                        last_log = now;
                    }
                }
            } else {
                module_unavailable_since = std::chrono::steady_clock::time_point{};
            }
            // Lock fault: if lock disengaged while plug-in/session active, treat as fault
            if (!fault && lock_required && !status.lock_engaged && (status.plugged_in || had_session)) {
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                    disable_local("lock_fault");
                    had_session = false;
                }
                fault = true;
            }
            // If a started transaction remains in CP=B with zero delivered power/current and no fresh EV targets,
            // close it deterministically to avoid dangling OCPP transactions (SuspendedEV forever + MeterValues).
            if (!fault && had_session && session.transaction_started && session.authorized &&
                status.plugged_in && status.cp_state == 'B' && !status.hlc_charge_complete) {
                const auto target_timeout = telemetry_timeout(cfg_);
                const bool target_recent = status.last_target_update.time_since_epoch().count() != 0 &&
                                           (now - status.last_target_update) <= target_timeout;
                const double present_i_abs = std::fabs(status.present_current_a.value_or(0.0));
                const double present_p_abs = std::fabs(status.present_power_w.value_or(0.0));
                const bool no_output = !status.relay_closed && present_i_abs < 0.5 && present_p_abs < 250.0;
                if (!target_recent && no_output) {
                    if (suspended_no_power_since.time_since_epoch().count() == 0) {
                        suspended_no_power_since = now;
                    } else if ((now - suspended_no_power_since) >= std::chrono::seconds(90)) {
                        EVLOG_info << "Connector " << connector
                                   << " stopping transaction after sustained SuspendedEV/no-output"
                                   << " cp=" << status.cp_state
                                   << " relay=" << (status.relay_closed ? "1" : "0")
                                   << " present_I=" << present_i_abs
                                   << " present_P=" << present_p_abs;
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            stop_origin_hint_[connector] = "suspended_ev_no_output";
                        }
                        finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                        disable_local("suspended_ev_no_output");
                        had_session = false;
                        session = ActiveSession{};
                        suspended_no_power_since = std::chrono::steady_clock::time_point{};
                        suspended_no_power_log_ts = std::chrono::steady_clock::time_point{};
                    } else if (suspended_no_power_log_ts.time_since_epoch().count() == 0 ||
                               (now - suspended_no_power_log_ts) >= std::chrono::seconds(10)) {
                        const auto age_ms =
                            std::chrono::duration_cast<std::chrono::milliseconds>(now - suspended_no_power_since).count();
                        EVLOG_warning << "Connector " << connector
                                      << " sustained SuspendedEV/no-output observed"
                                      << " cp=" << status.cp_state
                                      << " relay=" << (status.relay_closed ? "1" : "0")
                                      << " present_I=" << present_i_abs
                                      << " present_P=" << present_p_abs
                                      << " age_ms=" << age_ms
                                      << " -- waiting for recovery";
                        suspended_no_power_log_ts = now;
                    }
                } else {
                    suspended_no_power_since = std::chrono::steady_clock::time_point{};
                    suspended_no_power_log_ts = std::chrono::steady_clock::time_point{};
                }
            } else {
                suspended_no_power_since = std::chrono::steady_clock::time_point{};
                suspended_no_power_log_ts = std::chrono::steady_clock::time_point{};
            }
            if (push_meter_now) {
                bool send_meter = false;
                const auto keepalive = std::chrono::seconds(std::max(1, cfg_.meter_keepalive_s));
                {
                    std::lock_guard<std::mutex> lock(meter_mutex_);
                    const double energy_wh = measurement.power_meter.energy_Wh_import.total;
                    const double last_sent_wh = last_meter_sent_wh_[connector];
                    const auto last_sent_time_it = last_meter_sent_time_.find(connector);
                    const auto last_sent_time =
                        last_sent_time_it != last_meter_sent_time_.end()
                            ? last_sent_time_it->second
                            : std::chrono::steady_clock::time_point{};
                    const bool tx_active = had_session && session.transaction_started;
                    const bool changed = std::fabs(energy_wh - last_sent_wh) > 0.1;
                    const bool stale = last_sent_time.time_since_epoch().count() == 0 ||
                        (now - last_sent_time) >= keepalive;
                    send_meter = tx_active || changed || stale;
                    if (send_meter) {
                        last_meter_sent_wh_[connector] = energy_wh;
                        last_meter_sent_time_[connector] = now;
                    }
                }
                if (send_meter) {
                    push_meter_values(connector, measurement);
                }
                while (next_meter_push <= loop_start) {
                    next_meter_push += meter_period;
                }
            }
            {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                const auto v_dc = extract_dc_value(measurement.power_meter.voltage_V);
                if (v_dc) {
                    last_voltage_v_[connector] = *v_dc;
                }
                const double p_total = extract_total_value(measurement.power_meter.power_W);
                if (p_total > 0.0) {
                    last_power_w_[connector] = p_total;
                } else {
                    const auto i_dc = extract_dc_value(measurement.power_meter.current_A);
                    if (v_dc && i_dc) {
                        last_power_w_[connector] = (*v_dc) * (*i_dc);
                    }
                }
            }
            const bool tx_active = had_session && session.transaction_started;
            const bool meter_fault_relevant = status.meter_stale && (tx_active || status.relay_closed);
            if (meter_fault_relevant) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    telemetry_timeout_events_[connector]++;
                }
            }

            // Treat telemetry as required only once a connector is actually in a power-capable phase. Power modules may
            // not stream telemetry while OFF/idle (0V/0A/0W is expected then), so don't trip PowerLoss when idle.
            const bool telemetry_required =
                cfg_.telemetry_timeout_ms > 0 && module_relevant && (module_capability_needed || module_commanded);
            const bool telemetry_seen = status.last_telemetry.time_since_epoch().count() != 0;
            const auto telemetry_age =
                telemetry_seen ? (now - status.last_telemetry) : std::chrono::steady_clock::duration::zero();
            const bool telemetry_bad =
                telemetry_required &&
                (!telemetry_seen || telemetry_age > telemetry_timeout(cfg_));
            if (telemetry_bad) {
                if (telemetry_overdue_since.time_since_epoch().count() == 0) {
                    telemetry_overdue_since = now;
                }
            } else {
                telemetry_overdue_since = std::chrono::steady_clock::time_point{};
            }
            // Debounce telemetry staleness longer to avoid false positives when PLC is busy or slow to start.
            constexpr auto kTelemetryStaleDebounce = std::chrono::milliseconds(3000);
            const bool telemetry_stale = telemetry_bad && telemetry_overdue_since.time_since_epoch().count() != 0 &&
                                         (now - telemetry_overdue_since) >= kTelemetryStaleDebounce;

            // Sync OCPP error states (raise on edges, clear on recovery).
            const bool ocpp_cp_fault = status.cp_fault && !cp_fault_grace;
            const bool ocpp_estop = status.estop;
            const bool comm_relevant =
                status.plugged_in || status.relay_closed || status.hlc_stage > 0 || had_session || status.authorization_granted;
            const bool ocpp_comm = !ocpp_estop && comm_relevant && (status.comm_fault || telemetry_stale);
            const bool ocpp_earth = !ocpp_estop && !ocpp_comm && status.earth_fault;
            const bool ocpp_safety = !ocpp_estop && !ocpp_comm && !ocpp_earth && !status.safety_ok;
            const bool ocpp_isolation = status.isolation_fault;
            const bool ocpp_overtemp = status.overtemp_fault;
            const bool ocpp_overcurrent = status.overcurrent_fault;
            const bool ocpp_gc_welded = status.gc_welded;
            const bool ocpp_mc_welded = status.mc_welded;
            const bool ocpp_module_fault = module_unavailable_fault;
            // "Modules degraded" is advisory. Debounce both raise/clear to avoid rapid OCPP
            // oscillation from short telemetry decode jitter.
            const bool module_degraded_raw =
                !ocpp_module_fault && module_fault_mask != 0 && (status.relay_closed || had_session);
            if (module_degraded_raw) {
                if (module_degraded_since.time_since_epoch().count() == 0) {
                    module_degraded_since = now;
                }
                module_degraded_clear_since = std::chrono::steady_clock::time_point{};
                constexpr auto kModuleDegradedRaiseDebounce = std::chrono::milliseconds(5000);
                if ((now - module_degraded_since) >= kModuleDegradedRaiseDebounce) {
                    module_degraded_latched = true;
                }
            } else {
                module_degraded_since = std::chrono::steady_clock::time_point{};
                if (module_degraded_latched) {
                    if (module_degraded_clear_since.time_since_epoch().count() == 0) {
                        module_degraded_clear_since = now;
                    }
                    constexpr auto kModuleDegradedClearDebounce = std::chrono::milliseconds(5000);
                    if ((now - module_degraded_clear_since) >= kModuleDegradedClearDebounce) {
                        module_degraded_latched = false;
                    }
                } else {
                    module_degraded_clear_since = std::chrono::steady_clock::time_point{};
                }
            }
            const bool ocpp_module_degraded = module_degraded_latched;
            const bool ocpp_lock_fault = lock_required && !status.lock_engaged && (status.plugged_in || had_session);

            const auto cp_desc = errors::descriptor(errors::ErrorKey::CpFault);
            sync_ocpp_error(connector, "cp_fault", cp_desc.ocpp_code, cp_desc.is_fault, ocpp_cp_fault,
                            cp_desc.info, cfg_.vendor, cp_desc.vendor_code);

            // OCPP 1.6 has no dedicated "EmergencyStop" error code; avoid mislabeling it as PowerSwitchFailure.
            const auto estop_desc = errors::descriptor(errors::ErrorKey::EmergencyStop);
            sync_ocpp_error(connector, "estop", estop_desc.ocpp_code, estop_desc.is_fault, ocpp_estop,
                            estop_desc.info, cfg_.vendor, estop_desc.vendor_code);

            const auto comm_desc = errors::comm_descriptor(telemetry_stale);
            sync_ocpp_error(connector, "comm", comm_desc.ocpp_code, comm_desc.is_fault, ocpp_comm,
                            comm_desc.info, cfg_.vendor, comm_desc.vendor_code);

            const auto earth_desc = errors::descriptor(errors::ErrorKey::EarthFault);
            sync_ocpp_error(connector, "earth", earth_desc.ocpp_code, earth_desc.is_fault, ocpp_earth,
                            earth_desc.info, cfg_.vendor, earth_desc.vendor_code);

            const auto safety_desc = errors::descriptor(errors::ErrorKey::SafetyTrip);
            sync_ocpp_error(connector, "safety", safety_desc.ocpp_code, safety_desc.is_fault, ocpp_safety,
                            safety_desc.info, cfg_.vendor, safety_desc.vendor_code);

            const auto iso_desc = errors::descriptor(errors::ErrorKey::IsolationFault);
            sync_ocpp_error(connector, "isolation_fault", iso_desc.ocpp_code, iso_desc.is_fault, ocpp_isolation,
                            iso_desc.info, cfg_.vendor, iso_desc.vendor_code);

            const auto overtemp_desc = errors::descriptor(errors::ErrorKey::OverTemp);
            sync_ocpp_error(connector, "overtemp", overtemp_desc.ocpp_code, overtemp_desc.is_fault, ocpp_overtemp,
                            overtemp_desc.info, cfg_.vendor, overtemp_desc.vendor_code);

            const auto overcurrent_desc = errors::descriptor(errors::ErrorKey::OverCurrent);
            sync_ocpp_error(connector, "overcurrent", overcurrent_desc.ocpp_code, overcurrent_desc.is_fault,
                            ocpp_overcurrent, overcurrent_desc.info, cfg_.vendor, overcurrent_desc.vendor_code);

            const auto gc_desc = errors::descriptor(errors::ErrorKey::GcWelded);
            sync_ocpp_error(connector, "gc_welded", gc_desc.ocpp_code, gc_desc.is_fault, ocpp_gc_welded,
                            gc_desc.info, cfg_.vendor, gc_desc.vendor_code);

            const auto mc_desc = errors::descriptor(errors::ErrorKey::McWelded);
            sync_ocpp_error(connector, "mc_welded", mc_desc.ocpp_code, mc_desc.is_fault, ocpp_mc_welded,
                            mc_desc.info, cfg_.vendor, mc_desc.vendor_code);

            const auto modules_desc = errors::descriptor(errors::ErrorKey::ModulesUnavailable);
            sync_ocpp_error(connector, "module_fault", modules_desc.ocpp_code, modules_desc.is_fault,
                            ocpp_module_fault, modules_desc.info, cfg_.vendor, modules_desc.vendor_code);

            const auto degraded_desc = errors::descriptor(errors::ErrorKey::ModuleDegraded);
            sync_ocpp_error(connector, "module_degraded", degraded_desc.ocpp_code, degraded_desc.is_fault,
                            ocpp_module_degraded, degraded_desc.info, cfg_.vendor, degraded_desc.vendor_code);

            const auto lock_desc = errors::descriptor(errors::ErrorKey::LockFault);
            sync_ocpp_error(connector, "lock_fault", lock_desc.ocpp_code, lock_desc.is_fault, ocpp_lock_fault,
                            lock_desc.info, cfg_.vendor, lock_desc.vendor_code);

            const auto meter_desc = errors::descriptor(errors::ErrorKey::MeterStale);
            sync_ocpp_error(connector, "meter_stale", meter_desc.ocpp_code, meter_desc.is_fault,
                            meter_fault_relevant, meter_desc.info, cfg_.vendor, meter_desc.vendor_code);

            if (!fault && !local_fault_latched) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                connector_faulted_[connector] = false;
            }

            if (!fault && had_session && session.transaction_started) {
                const auto ack_timeout = evse_limit_ack_timeout(cfg_);
                // Enforce hard-stop only during actual power-delivery phase.
                // During CP=B pauses, precharge, or auth-hold, keep the watchdog non-fatal.
                const bool ack_relevant = evse_limit_ack_watchdog_relevant(status, lock_required);
                const auto tx_start = session.transaction_started_at;
                const bool tx_start_known = tx_start.time_since_epoch().count() != 0;
                const bool ack_seen = status.last_evse_limit_ack.time_since_epoch().count() != 0;
                const bool ack_after_tx_start = ack_seen && (!tx_start_known || status.last_evse_limit_ack >= tx_start);
                bool stale = false;
                std::int64_t age_ms = 0;
                if (ack_relevant) {
                    if (ack_after_tx_start) {
                        const auto age = now - status.last_evse_limit_ack;
                        age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
                        stale = age > ack_timeout;
                    } else if (tx_start_known) {
                        const auto since_tx_start = now - tx_start;
                        age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(since_tx_start).count();
                        stale = since_tx_start > ack_timeout;
                    }
                } else {
                    evse_limit_ack_stale_since = std::chrono::steady_clock::time_point{};
                }
                if (stale) {
                    if (evse_limit_ack_stale_since.time_since_epoch().count() == 0) {
                        evse_limit_ack_stale_since = now;
                        evse_limit_ack_stale_log_ts = std::chrono::steady_clock::time_point{};
                    }
                    const auto stale_span = now - evse_limit_ack_stale_since;
                    // Require a second full watchdog window before hard-stopping to avoid false trips from
                    // short PLC receive gaps while preserving fail-safe behavior on persistent loss of ACKs.
                    const bool stale_persistent = stale_span > ack_timeout;
                    if (stale_persistent) {
                        constexpr auto kAckStaleStopDelay = std::chrono::seconds(45);
                        const auto stale_for_ms =
                            std::chrono::duration_cast<std::chrono::milliseconds>(stale_span).count();
                        if (evse_limit_ack_stale_log_ts.time_since_epoch().count() == 0 ||
                            (now - evse_limit_ack_stale_log_ts) >= std::chrono::seconds(2)) {
                            if (ack_after_tx_start) {
                                EVLOG_warning << "EVSE limit ACK stale on connector " << connector
                                              << " age=" << age_ms
                                              << "ms stale_for=" << stale_for_ms
                                              << "ms ack_count=" << status.evse_limit_ack_count
                                              << " -- waiting for recovery";
                            } else {
                                EVLOG_warning << "EVSE limit ACK missing since transaction start on connector "
                                              << connector
                                              << " age=" << age_ms
                                              << "ms stale_for=" << stale_for_ms
                                              << "ms ack_count=" << status.evse_limit_ack_count
                                              << " -- waiting for recovery";
                            }
                            evse_limit_ack_stale_log_ts = now;
                        }
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            limit_ack_stale_events_[connector]++;
                        }
                        if (stale_span >= kAckStaleStopDelay) {
                            EVLOG_error << "EVSE limit ACK stale too long on connector " << connector
                                        << " stale_for_ms=" << stale_for_ms
                                        << " -- stopping session";
                            finish_and_mark(ocpp::v16::Reason::PowerLoss, std::nullopt);
                            disable_local("evse_limit_ack_stale");
                            had_session = false;
                            fault = true;
                            evse_limit_ack_stale_since = std::chrono::steady_clock::time_point{};
                            evse_limit_ack_stale_log_ts = std::chrono::steady_clock::time_point{};
                        }
                    }
                } else {
                    evse_limit_ack_stale_since = std::chrono::steady_clock::time_point{};
                    evse_limit_ack_stale_log_ts = std::chrono::steady_clock::time_point{};
                }
            } else {
                evse_limit_ack_stale_since = std::chrono::steady_clock::time_point{};
                evse_limit_ack_stale_log_ts = std::chrono::steady_clock::time_point{};
            }

            if (telemetry_stale) {
                if (!telemetry_stale_active) {
                    telemetry_stale_active = true;
                    telemetry_stale_log_ts = now;
                    telemetry_stale_fail_since = now;
                    EVLOG_warning << "Telemetry stale on connector " << connector
                                  << " age="
                                  << std::chrono::duration_cast<std::chrono::milliseconds>(telemetry_age).count()
                                  << "ms timeout=" << telemetry_timeout(cfg_).count()
                                  << "ms -- keeping session alive and waiting for recovery";
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        telemetry_timeout_events_[connector]++;
                    }
                } else if (telemetry_stale_log_ts.time_since_epoch().count() == 0 ||
                           (now - telemetry_stale_log_ts) > std::chrono::seconds(10)) {
                    telemetry_stale_log_ts = now;
                    EVLOG_warning << "Telemetry still stale on connector " << connector
                                  << " age="
                                  << std::chrono::duration_cast<std::chrono::milliseconds>(telemetry_age).count()
                                  << "ms timeout=" << telemetry_timeout(cfg_).count()
                                  << "ms -- waiting for recovery";
                }
                constexpr auto kTelemetryStaleStopDelay = std::chrono::seconds(25);
                constexpr double kTelemetryStaleCurrentReqA = 1.0;
                const bool cp_power_ready = status.cp_state == 'C' || status.cp_state == 'D';
                const bool relay_or_power_phase = status.relay_closed || cp_power_ready;
                const bool current_requested =
                    status.target_current_a && status.target_current_a.value() >= kTelemetryStaleCurrentReqA;
                if (!fault && had_session && session.transaction_started && relay_or_power_phase && current_requested &&
                    telemetry_stale_fail_since.time_since_epoch().count() != 0 &&
                    (now - telemetry_stale_fail_since) >= kTelemetryStaleStopDelay) {
                    EVLOG_error << "Telemetry stale too long on connector " << connector
                                << " stale_for_ms="
                                << std::chrono::duration_cast<std::chrono::milliseconds>(now - telemetry_stale_fail_since).count()
                                << " -- stopping session";
                    finish_and_mark(ocpp::v16::Reason::PowerLoss, std::nullopt);
                    disable_local("telemetry_stale");
                    had_session = false;
                    fault = true;
                    telemetry_stale_active = false;
                    telemetry_stale_log_ts = std::chrono::steady_clock::time_point{};
                    telemetry_stale_fail_since = std::chrono::steady_clock::time_point{};
                }
            } else if (telemetry_stale_active) {
                telemetry_stale_active = false;
                telemetry_stale_log_ts = std::chrono::steady_clock::time_point{};
                telemetry_stale_fail_since = std::chrono::steady_clock::time_point{};
                EVLOG_info << "Telemetry recovered on connector " << connector;
            } else {
                telemetry_stale_fail_since = std::chrono::steady_clock::time_point{};
            }

            if (!fault && had_session && session.transaction_started && status.hlc_charge_complete) {
                EVLOG_info << "Charge complete reported on connector " << connector << ", ending session";
                finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                disable_local("charge_complete");
                had_session = false;
                session = ActiveSession{};
            }

            bool seamless_retry_active = false;
            const bool power_ready = had_session && session.authorized &&
                (status.relay_closed || power_delivery_requested(status, lock_required)) && !disabled;
            const bool cp_power_requesting = (status.cp_state == 'C' || status.cp_state == 'D');
            const bool hlc_power_phase =
                status.hlc_power_ready || (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
            const bool hlc_waiting_pre_power =
                status.hlc_stage > 0 && status.hlc_stage < HLC_MIN_POWER_STAGE && !status.hlc_charge_complete;
            // Only run this watchdog once the EV has entered (or explicitly requested) power transfer.
            // During pre-power HLC stages (e.g. WAIT_CHARGE_PARAMS) some vehicles can legitimately pause.
            const bool power_request_watchdog_eligible =
                (cp_power_requesting || status.relay_closed || hlc_power_phase) && !hlc_waiting_pre_power;
            std::optional<std::chrono::milliseconds> power_wait_elapsed{};
            if (had_session && session.authorized) {
                std::lock_guard<std::mutex> lock(session_mutex_);
                auto it = sessions_.find(connector);
                if (it != sessions_.end()) {
                    if (power_ready) {
                        it->second.power_requested_at = now;
                        it->second.power_wait_started_at = std::nullopt;
                    } else if (power_request_watchdog_eligible && !constrained && !paused &&
                               !it->second.transaction_started) {
                        if (!it->second.power_wait_started_at) {
                            it->second.power_wait_started_at = now;
                        }
                        power_wait_elapsed =
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - *it->second.power_wait_started_at);
                    } else {
                        it->second.power_wait_started_at = std::nullopt;
                    }
                    session = it->second;
                }
            }
            if (had_session && status.cp_state != 'U' && !status.plugged_in) {
                const auto last_plug = session.last_seen_plugged;
                if (last_plug.time_since_epoch().count() != 0 &&
                    (now - last_plug) < SEAMLESS_RETRY_GRACE_MS) {
                    seamless_retry_active = true;
                    disable_local("seamless_retry");
                } else {
                    finish_and_mark(ocpp::v16::Reason::EVDisconnected, std::nullopt);
                    disable_local("ev_disconnected");
                    had_session = false;
                }
            } else if (had_session && !session.authorized && session.connected_at.time_since_epoch().count() &&
                       auth_timeout_enabled && (now - session.connected_at) > auth_wait_timeout) {
                EVLOG_warning << "Session on connector " << connector
                              << " timed out waiting for authorization, stopping session";
                set_auth_state(connector, AuthorizationState::Denied);
                finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                disable_local("auth_timeout");
                had_session = false;
            } else if (had_session && session.authorized && !session.transaction_started && !power_ready &&
                       power_request_timeout_enabled && power_wait_elapsed.has_value() &&
                       *power_wait_elapsed >
                           std::chrono::duration_cast<std::chrono::milliseconds>(power_request_timeout)) {
                EVLOG_warning << "Session on connector " << connector
                              << " timed out waiting for EV power request after authorization"
                              << " elapsed_ms=" << power_wait_elapsed->count()
                              << " timeout_ms="
                              << std::chrono::duration_cast<std::chrono::milliseconds>(power_request_timeout).count()
                              << " cp=" << status.cp_state
                              << " relay=" << (status.relay_closed ? 1 : 0)
                              << " hlc_stage=" << static_cast<int>(status.hlc_stage);
                finish_and_mark(ocpp::v16::Reason::Other, std::nullopt);
                disable_local("power_request_timeout");
                had_session = false;
            } else if (had_session && session.authorized && session.ev_connected && !session.transaction_started) {
                // Start OCPP transaction only once we are in a real power-delivery phase.
                // This avoids accepted-but-idle transactions with zero power/energy when CP is still B.
                const bool cp_power_requesting = status.cp_state == 'C' || status.cp_state == 'D';
                const bool relay_power_indicative =
                    status.relay_closed && cp_power_requesting &&
                    (status.hlc_power_ready ||
                     (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete));
                const bool tx_start_ready = power_delivery_requested(status, lock_required) || relay_power_indicative;
                if (tx_start_ready) {
                    std::lock_guard<std::mutex> lock(session_mutex_);
                    auto it = sessions_.find(connector);
                    if (it != sessions_.end() && !it->second.transaction_started && it->second.authorized &&
                        it->second.ev_connected && it->second.id_token.has_value()) {
                        it->second.meter_start_wh = measurement.power_meter.energy_Wh_import.total;
                        charge_point_->on_transaction_started(connector, it->second.session_id,
                                                              it->second.id_token.value(), it->second.meter_start_wh,
                                                              std::nullopt, ocpp::DateTime(), std::nullopt);
                        it->second.transaction_started = true;
                        it->second.transaction_started_at = now;
                        it->second.power_wait_started_at = std::nullopt;
                        it->second.power_requested_at = now;
                        session = it->second;
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                post_stop_plugged = post_stop_plugged_[connector];
            }
            if (!had_session) {
                session = ActiveSession{};
            }
            const bool block_reenable = fault || cp_fault_grace || seamless_retry_active;
            maybe_reenable_local_hw(connector, status, block_reenable, disabled_by_csms, paused);
            const bool fault_active = fault || local_fault_latched;
            update_connector_state(connector, status, had_session, session.transaction_started, session.authorized,
                                   fault_active, disabled, post_stop_plugged, seamless_retry_active,
                                   suppress_available_event);
        } catch (const std::exception& e) {
            EVLOG_warning << "Metering loop error on connector " << connector << ": " << e.what();
        }
        std::this_thread::sleep_for(control_tick);
    }
}

bool OcppAdapter::begin_transaction(std::int32_t connector, const std::string& id_token, bool prevalidated,
                                    ocpp::SessionStartedReason reason) {
    (void)reason;
    AuthToken token;
    token.id_token = id_token;
    token.source = AuthTokenSource::RemoteStart;
    token.connector_hint = connector;
    token.prevalidated = prevalidated;
    token.received_at = std::chrono::steady_clock::now();
    ingest_auth_tokens({token}, token.received_at);
    return true;
}

void OcppAdapter::finish_transaction(std::int32_t connector, ocpp::v16::Reason reason,
                                     std::optional<ocpp::CiString<20>> id_tag_end, bool defer_session_stop) {
    const auto stop_now = std::chrono::steady_clock::now();
    GunStatus stop_status{};
    if (hardware_) {
        stop_status = sanitize_status(hardware_->get_status(connector), cfg_.lab_bypass);
    }
    std::string session_id;
    std::optional<std::chrono::steady_clock::time_point> pending_started;
    std::optional<std::chrono::steady_clock::time_point> authorized_at;
    double meter_start_wh = 0.0;
    double last_energy_wh = 0.0;
    bool was_started = false;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        const auto it = sessions_.find(connector);
        if (it == sessions_.end()) {
            return;
        }
        session_id = it->second.session_id;
        pending_started = it->second.pending_started;
        authorized_at = it->second.authorized_at;
        meter_start_wh = it->second.meter_start_wh;
        was_started = it->second.transaction_started;
    }
    {
        std::lock_guard<std::mutex> lock(meter_mutex_);
        last_energy_wh = last_energy_wh_[connector];
    }

    double energy_wh = 0.0;
    if (was_started && charge_point_) {
        auto measurement = hardware_->sample_meter(connector);
        apply_energy_fallback(connector, stop_status, measurement, stop_now);
        energy_wh = measurement.power_meter.energy_Wh_import.total;
        if (!std::isfinite(energy_wh) || energy_wh < 0.0) {
            energy_wh = 0.0;
        }
        // Ensure StopTransaction.meterStop is monotonic vs. StartTransaction.meterStart and previously sent MeterValues.
        const double floor_wh = std::max({0.0, meter_start_wh, last_energy_wh});
        if (energy_wh + 1e-3 < floor_wh) {
            EVLOG_warning << "StopTransaction energy regression on connector " << connector << " (start=" << meter_start_wh
                          << "Wh, last=" << last_energy_wh << "Wh, stop=" << energy_wh << "Wh); clamping to " << floor_wh
                          << "Wh";
            energy_wh = floor_wh;
        } else {
            // Keep the clamp base aligned with the final reading so a late meter tick cannot regress after stop.
            std::lock_guard<std::mutex> lock(meter_mutex_);
            if (energy_wh > last_energy_wh_[connector]) {
                last_energy_wh_[connector] = energy_wh;
            }
        }
        charge_point_->on_transaction_stopped(connector, session_id, reason, ocpp::DateTime(),
                                              energy_wh, id_tag_end, std::nullopt);
        {
            std::lock_guard<std::mutex> lock(meter_mutex_);
            fallback_energy_wh_[connector] = std::max(0.0, energy_wh);
            fallback_energy_last_update_[connector] = stop_now;
        }
    }

    std::string stop_origin;
    std::chrono::steady_clock::time_point cp_drop_time{};
    std::string cp_drop_reason;
    std::string local_fault_reason;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto origin_it = stop_origin_hint_.find(connector);
        if (origin_it != stop_origin_hint_.end()) {
            stop_origin = origin_it->second;
            stop_origin_hint_.erase(origin_it);
        }
        auto cp_drop_it = last_cp_request_drop_.find(connector);
        if (cp_drop_it != last_cp_request_drop_.end()) {
            cp_drop_time = cp_drop_it->second;
            last_cp_request_drop_.erase(cp_drop_it);
        }
        auto cp_reason_it = last_cp_request_drop_reason_.find(connector);
        if (cp_reason_it != last_cp_request_drop_reason_.end()) {
            cp_drop_reason = cp_reason_it->second;
            last_cp_request_drop_reason_.erase(cp_reason_it);
        }
        auto local_fault_it = last_local_fault_reason_.find(connector);
        if (local_fault_it != last_local_fault_reason_.end()) {
            local_fault_reason = local_fault_it->second;
        }
    }
    if (stop_origin.empty()) {
        if (reason == ocpp::v16::Reason::EVDisconnected) {
            stop_origin = "vehicle_disconnected";
        } else if (reason == ocpp::v16::Reason::Local) {
            stop_origin = "local_stop";
        } else if (reason == ocpp::v16::Reason::Remote) {
            stop_origin = "ocpp_remote_stop";
        } else if (reason == ocpp::v16::Reason::EmergencyStop) {
            stop_origin = "safety_estop";
        } else {
            stop_origin = "controller_or_fault_logic";
        }
    }
    const auto auth_elapsed =
        (pending_started && pending_started->time_since_epoch().count())
            ? (authorized_at ? (authorized_at.value() - *pending_started)
                             : std::chrono::steady_clock::duration::zero())
            : std::chrono::steady_clock::duration::zero();
    const auto auth_wait_ms = (pending_started && pending_started->time_since_epoch().count())
                                  ? std::chrono::duration_cast<std::chrono::milliseconds>(auth_elapsed).count()
                                  : -1;
    const auto cp_drop_age_ms = cp_drop_time.time_since_epoch().count() != 0
                                    ? std::chrono::duration_cast<std::chrono::milliseconds>(stop_now - cp_drop_time).count()
                                    : -1;
    EVLOG_info << "StopTransaction summary connector=" << connector
               << " session=" << session_id
               << " reason=" << ocpp::v16::conversions::reason_to_string(reason)
               << " origin=" << stop_origin
               << " auth_wait_ms=" << auth_wait_ms
               << " energy_Wh=" << energy_wh
               << " cp=" << stop_status.cp_state
               << " plugged=" << (stop_status.plugged_in ? "1" : "0")
               << " relay=" << (stop_status.relay_closed ? "1" : "0")
               << " hlc_stage=" << static_cast<int>(stop_status.hlc_stage)
               << " present_V=" << stop_status.present_voltage_v.value_or(0.0)
               << " present_I=" << stop_status.present_current_a.value_or(0.0)
               << " target_I=" << stop_status.target_current_a.value_or(0.0)
               << " cp_drop_age_ms=" << cp_drop_age_ms
               << " cp_drop_reason=" << (cp_drop_reason.empty() ? "none" : cp_drop_reason);
    const auto target_age_ms =
        stop_status.last_target_update.time_since_epoch().count() != 0
            ? std::chrono::duration_cast<std::chrono::milliseconds>(stop_now - stop_status.last_target_update).count()
            : -1;
    const bool module_watchdog_observed =
        (stop_origin.find("PowerDeliveryStalled") != std::string::npos) || (local_fault_reason == "PowerDeliveryStalled");
    if (module_watchdog_observed) {
        EVLOG_info << "Module telemetry watchdog observed in this session.";
    } else {
        EVLOG_info << "Lower likelihood here: module telemetry watchdog (not observed in this session).";
    }
    if (target_age_ms >= 0 && target_age_ms <= (telemetry_timeout(cfg_) + std::chrono::milliseconds(200)).count()) {
        EVLOG_info << "Lower likelihood here: lwIP/QCA transport delay for CurrentDemand (latency is very low in this session).";
    } else {
        EVLOG_info << "CurrentDemand target freshness at stop indicates possible PLC transport/processing delay"
                   << " (target_age_ms=" << target_age_ms << ")";
    }

    bool pending_changed = false;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        pending_changed = pending_tokens_.erase(connector) > 0;
        sessions_.erase(connector);
    }
    clear_auth_queue_for_connector(connector);
    if (pending_changed) {
        persist_pending_tokens();
    }
    set_auth_state(connector, AuthorizationState::Unknown);
    bool notify_session_stop = true;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (defer_session_stop) {
            pending_session_stop_[connector] = session_id;
            pending_session_stop_since_[connector] = std::chrono::steady_clock::now();
            notify_session_stop = false;
        } else {
            pending_session_stop_.erase(connector);
            pending_session_stop_since_.erase(connector);
        }
    }
    if (charge_point_ && notify_session_stop) {
        charge_point_->on_session_stopped(connector, session_id);
    }
}

void OcppAdapter::push_meter_values(std::int32_t connector, const ocpp::Measurement& measurement) {
    if (charge_point_) {
        ocpp::Measurement adjusted = measurement;
        if (adjusted.power_meter.energy_Wh_import.total < 0.0) {
            adjusted.power_meter.energy_Wh_import.total = 0.0;
        }
        {
            std::lock_guard<std::mutex> lock(meter_mutex_);
            const double current = adjusted.power_meter.energy_Wh_import.total;
            const double last = last_energy_wh_[connector];
            if (current + 1e-3 < last) {
                EVLOG_warning << "Energy counter regression on connector " << connector << " (last=" << last
                              << "Wh, current=" << current << "Wh); clamping to monotonic";
                adjusted.power_meter.energy_Wh_import.total = last;
            } else {
                last_energy_wh_[connector] = current;
            }
        }
        charge_point_->on_meter_values(connector, adjusted);
    }
}

void OcppAdapter::report_fault(std::int32_t connector, const ocpp::v16::ErrorInfo& info) {
    if (charge_point_) {
        charge_point_->on_error(connector, info);
    }
}

void OcppAdapter::clear_faults(std::int32_t connector) {
    if (hardware_) {
        hardware_->clear_faults(connector);
    }
    if (charge_point_) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            active_ocpp_errors_.erase(connector);
            connector_faulted_[connector] = false;
        }
        charge_point_->on_all_errors_cleared(connector);
    }
}

void OcppAdapter::sync_ocpp_error(std::int32_t connector, const std::string& uuid,
                                 ocpp::v16::ChargePointErrorCode error_code, bool is_fault, bool active,
                                 const std::optional<std::string>& info, const std::optional<std::string>& vendor_id,
                                 const std::optional<std::string>& vendor_error_code) {
    if (!charge_point_) {
        return;
    }

    bool raise = false;
    bool clear = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto& active_set = active_ocpp_errors_[connector];
        const bool was_active = active_set.count(uuid) != 0;
        if (active && !was_active) {
            active_set.insert(uuid);
            raise = true;
        } else if (!active && was_active) {
            active_set.erase(uuid);
            clear = true;
        }
    }

    if (raise) {
        ocpp::v16::ErrorInfo err(uuid, error_code, is_fault, info, vendor_id, vendor_error_code);
        EVLOG_warning << "Raising OCPP error connector=" << connector << " uuid=" << uuid
                      << " code=" << static_cast<int>(error_code)
                      << " fault=" << (is_fault ? "true" : "false")
                      << (info ? " info=" + *info : "")
                      << (vendor_error_code ? " vendor=" + *vendor_error_code : "");
        charge_point_->on_error(connector, err);
    } else if (clear) {
        EVLOG_info << "Clearing OCPP error connector=" << connector << " uuid=" << uuid;
        charge_point_->on_error_cleared(connector, uuid);
    }
}

void OcppAdapter::record_presence_state(std::int32_t connector, bool plugged_in,
                                        const std::chrono::steady_clock::time_point& now) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    const auto it = plugged_in_state_.find(connector);
    const bool prev = it != plugged_in_state_.end() ? it->second : false;
    if (plugged_in && !prev) {
        plug_event_time_[connector] = now;
    }
    plugged_in_state_[connector] = plugged_in;
}

void OcppAdapter::ingest_auth_tokens(const std::vector<AuthToken>& tokens,
                                     const std::chrono::steady_clock::time_point& now) {
    if (tokens.empty()) {
        return;
    }
    const bool auth_timeout_enabled = cfg_.auth_wait_timeout_s > 0;
    const auto ttl = auth_timeout_enabled ? std::chrono::seconds(cfg_.auth_wait_timeout_s) : std::chrono::seconds(0);
    const bool allow_autocharge = autocharge_enabled_.load();
    std::size_t autocharge_dropped = 0;
    std::size_t autocharge_blocked = 0;
    bool pending_changed = false;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        for (auto token : tokens) {
            token.id_token = clamp_id_token(token.id_token);
            if (token.source == AuthTokenSource::Autocharge && !allow_autocharge) {
                ++autocharge_dropped;
                continue;
            }
            if (token.source == AuthTokenSource::Autocharge && token.connector_hint > 0) {
                auto& flow = hlc_control_[token.connector_hint];
                if (flow.pnc_blocked && flow.block_expires.time_since_epoch().count() != 0 &&
                    now >= flow.block_expires) {
                    flow.pnc_blocked = false;
                    flow.blocked_identity.reset();
                    flow.block_expires = std::chrono::steady_clock::time_point{};
                }
                flow.last_autocharge_id = token.id_token;
                if (flow.pnc_blocked) {
                    ++autocharge_blocked;
                    continue;
                }
            }
            const int target = select_connector_for_token(token);
            const bool bypass_dedup =
                (token.source == AuthTokenSource::RemoteStart) || should_bypass_token_dedup(target, token);
            auto& dedup_cache = recent_token_cache_[target];
            for (auto it = dedup_cache.begin(); it != dedup_cache.end();) {
                if ((now - it->second) >= RECENT_TOKEN_DEDUP_WINDOW) {
                    it = dedup_cache.erase(it);
                } else {
                    ++it;
                }
            }
            if (!bypass_dedup) {
                const auto dedup_it = dedup_cache.find(token.id_token);
                if (dedup_it != dedup_cache.end() && (now - dedup_it->second) < RECENT_TOKEN_DEDUP_WINDOW) {
                    continue;
                }
            }
            dedup_cache[token.id_token] = now;
            PendingToken pending;
            pending.token = token;
            if (pending.token.received_at.time_since_epoch().count() == 0) {
                pending.token.received_at = now;
            }
            pending.expires_at = auth_timeout_enabled ? (pending.token.received_at + ttl)
                                                      : std::chrono::steady_clock::time_point::max();
            pending_tokens_[target].push_back(std::move(pending));
            pending_changed = true;
        }
        if (autocharge_dropped > 0) {
            const auto log_now = now;
            if (last_autocharge_drop_log_.time_since_epoch().count() == 0 ||
                (log_now - last_autocharge_drop_log_) > std::chrono::seconds(10)) {
                EVLOG_info << "Autocharge disabled; ignored " << autocharge_dropped << " autocharge token(s)";
                last_autocharge_drop_log_ = log_now;
            }
        }
        if (autocharge_blocked > 0) {
            const auto log_now = now;
            if (last_autocharge_block_log_.time_since_epoch().count() == 0 ||
                (log_now - last_autocharge_block_log_) > std::chrono::seconds(10)) {
                EVLOG_info << "Autocharge blocked; ignored " << autocharge_blocked << " autocharge token(s)";
                last_autocharge_block_log_ = log_now;
            }
        }
    }
    if (pending_changed) {
        persist_pending_tokens();
    }
}

void OcppAdapter::set_autocharge_enabled(bool enabled, const std::string& source) {
    const bool prev = autocharge_enabled_.exchange(enabled);
    if (prev == enabled) {
        return;
    }
    EVLOG_info << "Autocharge " << (enabled ? "enabled" : "disabled") << " (source=" << source << ")";
    if (!enabled) {
        bool removed = false;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            removed = clear_pending_autocharge_tokens_locked();
        }
        if (removed) {
            persist_pending_tokens();
        }
    }
}

bool OcppAdapter::clear_pending_autocharge_tokens_locked() {
    bool removed = false;
    for (auto& kv : pending_tokens_) {
        auto& queue = kv.second;
        const auto before = queue.size();
        queue.erase(std::remove_if(queue.begin(), queue.end(),
                                   [](const PendingToken& p) {
                                       return p.token.source == AuthTokenSource::Autocharge;
                                   }),
                    queue.end());
        if (queue.size() != before) {
            removed = true;
        }
    }
    return removed;
}

bool OcppAdapter::clear_pending_autocharge_tokens_for_connector_locked(std::int32_t connector) {
    auto it = pending_tokens_.find(connector);
    if (it == pending_tokens_.end()) {
        return false;
    }
    auto& queue = it->second;
    const auto before = queue.size();
    queue.erase(std::remove_if(queue.begin(), queue.end(),
                               [](const PendingToken& p) {
                                   return p.token.source == AuthTokenSource::Autocharge;
                               }),
                queue.end());
    return queue.size() != before;
}

int OcppAdapter::select_connector_for_token(const AuthToken& token) const {
    if (token.source == AuthTokenSource::RFID && cfg_.connectors.size() == 2 && hardware_) {
        int best_candidate = 0;
        int best_rank = -1;
        auto best_seen = std::chrono::steady_clock::time_point{};
        for (const auto& c : cfg_.connectors) {
            const auto status = sanitize_status(hardware_->get_status(c.id), cfg_.lab_bypass);
            const auto sit = sessions_.find(c.id);
            const bool tx_started = sit != sessions_.end() && sit->second.transaction_started;
            if (tx_started) {
                continue;
            }
            const bool preparing_like =
                status.cp_state == 'B' || status.hlc_stage > 0 || status.hlc_precharge_active || status.hlc_power_ready;
            const bool vehicle_present = status.plugged_in || preparing_like || status.relay_closed;
            if (!vehicle_present) {
                continue;
            }
            const int rank = preparing_like ? 2 : 1;
            const auto seen_it = plug_event_time_.find(c.id);
            const auto seen = (seen_it != plug_event_time_.end()) ? seen_it->second
                                                                   : std::chrono::steady_clock::time_point{};
            if (rank > best_rank || (rank == best_rank &&
                                     (best_seen.time_since_epoch().count() == 0 || seen > best_seen))) {
                best_candidate = c.id;
                best_rank = rank;
                best_seen = seen;
            }
        }
        if (best_candidate > 0) {
            return best_candidate;
        }
    }

    if (token.connector_hint > 0) {
        const bool exists = std::any_of(cfg_.connectors.begin(), cfg_.connectors.end(),
                                        [&](const ConnectorConfig& c) { return c.id == token.connector_hint; });
        if (exists) {
            return token.connector_hint;
        }
    }
    int best = cfg_.connectors.empty() ? 1 : cfg_.connectors.front().id;
    auto latest = std::chrono::steady_clock::time_point{};
    for (const auto& kv : plug_event_time_) {
        const bool plugged = plugged_in_state_.count(kv.first) ? plugged_in_state_.at(kv.first) : false;
        if (!plugged) continue;
        if (latest.time_since_epoch().count() == 0 || kv.second > latest) {
            latest = kv.second;
            best = kv.first;
        }
    }
    return best;
}

std::optional<OcppAdapter::PendingToken>
OcppAdapter::pop_next_pending_token(std::int32_t connector, const std::chrono::steady_clock::time_point& now,
                                    const std::optional<std::string>& required_token,
                                    const std::optional<std::string>& parent_token,
                                    bool* pending_changed) {
    auto mark_changed = [&]() {
        if (pending_changed) {
            *pending_changed = true;
        }
    };
    auto it = pending_tokens_.find(connector);
    if (it == pending_tokens_.end()) {
        return std::nullopt;
    }
    auto& queue = it->second;
    auto priority = [](AuthTokenSource src) {
        switch (src) {
        case AuthTokenSource::RemoteStart:
            return 0;
        case AuthTokenSource::RFID:
            return 1;
        case AuthTokenSource::Autocharge:
            return 2;
        default:
            return 3;
        }
    };
    const bool csms_offline = !charge_point_ || !csms_connected_.load();
    const bool allow_autocharge = autocharge_enabled_.load();
    bool removed = false;
    for (auto qit = queue.begin(); qit != queue.end();) {
        if (qit->expires_at <= now) {
            removed = true;
            qit = queue.erase(qit);
        } else if (!allow_autocharge && qit->token.source == AuthTokenSource::Autocharge) {
            removed = true;
            qit = queue.erase(qit);
        } else {
            ++qit;
        }
    }
    if (removed) {
        mark_changed();
    }
    if (queue.empty()) {
        return std::nullopt;
    }
    auto matches_reservation = [&](const PendingToken& p) {
        if (!required_token) return true;
        const auto trimmed = clamp_id_token(p.token.id_token);
        if (trimmed == *required_token) return true;
        if (parent_token && trimmed == *parent_token) return true;
        return false;
    };
    std::optional<std::size_t> best_idx;
    int best_prio = std::numeric_limits<int>::max();
    for (std::size_t idx = 0; idx < queue.size(); ++idx) {
        if (csms_offline && queue[idx].token.source == AuthTokenSource::Autocharge &&
            queue[idx].defer_until_online) {
            continue;
        }
        if (queue[idx].token.source == AuthTokenSource::Autocharge) {
            const auto retry_it = autocharge_retry_not_before_.find(connector);
            if (retry_it != autocharge_retry_not_before_.end() && now < retry_it->second) {
                continue;
            }
        }
        if (!matches_reservation(queue[idx])) continue;
        const int prio = priority(queue[idx].token.source);
        if (!best_idx || prio < best_prio ||
            (prio == best_prio && queue[idx].token.received_at < queue[*best_idx].token.received_at)) {
            best_prio = prio;
            best_idx = idx;
        }
    }
    if (!best_idx) {
        return std::nullopt;
    }
    PendingToken selected = queue[*best_idx];
    queue.erase(queue.begin() + static_cast<std::ptrdiff_t>(*best_idx));
    mark_changed();
    return selected;
}

AuthorizationState OcppAdapter::try_authorize_with_token(std::int32_t connector, ActiveSession& session,
                                                         const PendingToken& pending) {
    if (session.authorized) {
        return AuthorizationState::Granted;
    }
    if (pending.token.source == AuthTokenSource::Autocharge && !autocharge_enabled_.load()) {
        set_auth_state(connector, AuthorizationState::Pending);
        return AuthorizationState::Pending;
    }
    const auto trimmed = clamp_id_token(pending.token.id_token);
    bool accepted = pending.token.prevalidated;
    if (!accepted && charge_point_) {
        set_auth_state(connector, AuthorizationState::Pending);
        const auto info = charge_point_->authorize_id_token(ocpp::CiString<20>(trimmed));
        accepted = (info.id_tag_info.status == ocpp::v16::AuthorizationStatus::Accepted);
    }
    if (accepted) {
        session.authorized = true;
        session.id_token = trimmed;
        session.authorized_at = std::chrono::steady_clock::now();
        session.power_wait_started_at = session.authorized_at;
        session.token_source = pending.token.source;
        autocharge_retry_not_before_.erase(connector);
        autocharge_retry_fail_count_.erase(connector);
        set_auth_state(connector, AuthorizationState::Granted);
        persist_pending_tokens();
        return AuthorizationState::Granted;
    } else {
        AuthorizationState state = AuthorizationState::Denied;
        if (pending.token.source == AuthTokenSource::Autocharge) {
            const bool mac_autocharge_mode = cfg_.autocharge_id_source == "evmac";
            const bool csms_online = csms_connected_.load();
            if (mac_autocharge_mode || !csms_online) {
                state = AuthorizationState::Pending;
                const int base_backoff_s = cfg_.auth_denied_hold_s > 0 ? cfg_.auth_denied_hold_s : 15;
                int& fail_count = autocharge_retry_fail_count_[connector];
                fail_count = std::min(fail_count + 1, 6);
                const int retry_delay_s =
                    std::clamp(base_backoff_s * (1 << (fail_count - 1)), 5, 180);
                autocharge_retry_not_before_[connector] =
                    std::chrono::steady_clock::now() + std::chrono::seconds(retry_delay_s);
                EVLOG_info << "Autocharge not accepted on connector " << connector
                           << "; keeping auth pending for retry (mode=" << cfg_.autocharge_id_source
                           << ", csms_online=" << (csms_online ? "true" : "false")
                           << ", retry_in_s=" << retry_delay_s << ")";
            } else {
                autocharge_retry_not_before_.erase(connector);
                autocharge_retry_fail_count_.erase(connector);
                EVLOG_info << "Autocharge rejected on connector " << connector
                           << "; forcing EIM fallback";
            }
        }
        set_auth_state(connector, state);
        if (state == AuthorizationState::Denied) {
            note_auth_denied(connector, trimmed);
            autocharge_retry_not_before_.erase(connector);
            autocharge_retry_fail_count_.erase(connector);
        }
        persist_pending_tokens();
        return state;
    }
}

AuthorizationState OcppAdapter::authorize_token_for_session(std::int32_t connector,
                                                            const std::string& session_id,
                                                            const PendingToken& pending) {
    if (pending.token.source == AuthTokenSource::Autocharge && !autocharge_enabled_.load()) {
        set_auth_state(connector, AuthorizationState::Pending);
        return AuthorizationState::Pending;
    }
    const auto trimmed = clamp_id_token(pending.token.id_token);
    bool accepted = pending.token.prevalidated;
    if (!accepted && charge_point_) {
        set_auth_state(connector, AuthorizationState::Pending);
        try {
            const auto info = charge_point_->authorize_id_token(ocpp::CiString<20>(trimmed));
            accepted = (info.id_tag_info.status == ocpp::v16::AuthorizationStatus::Accepted);
        } catch (const std::exception& e) {
            EVLOG_warning << "Authorization exception on connector " << connector << ": " << e.what();
            accepted = false;
        }
    }

    return apply_authorization_result(connector, session_id, pending, accepted);
}

AuthorizationState OcppAdapter::apply_authorization_result(std::int32_t connector,
                                                           const std::string& session_id,
                                                           const PendingToken& pending,
                                                           bool accepted) {
    const auto now = std::chrono::steady_clock::now();
    const auto trimmed = clamp_id_token(pending.token.id_token);
    AuthorizationState new_state = accepted ? AuthorizationState::Granted : AuthorizationState::Denied;
    const bool csms_offline = !charge_point_ || !csms_connected_.load();
    bool pending_changed = false;
    if (!accepted && pending.token.source == AuthTokenSource::Autocharge) {
        const bool mac_autocharge_mode = cfg_.autocharge_id_source == "evmac";
        if (csms_offline || mac_autocharge_mode) {
            new_state = AuthorizationState::Pending;
            const int base_backoff_s = cfg_.auth_denied_hold_s > 0 ? cfg_.auth_denied_hold_s : 15;
            int& fail_count = autocharge_retry_fail_count_[connector];
            fail_count = std::min(fail_count + 1, 6);
            const int retry_delay_s = std::clamp(base_backoff_s * (1 << (fail_count - 1)), 5, 180);
            autocharge_retry_not_before_[connector] = now + std::chrono::seconds(retry_delay_s);
            EVLOG_info << "Autocharge not accepted for session " << session_id << " on connector " << connector
                       << "; keeping auth pending for retry (mode=" << cfg_.autocharge_id_source
                       << ", csms_online=" << (csms_offline ? "false" : "true")
                       << ", retry_in_s=" << retry_delay_s << ")";
            if (autocharge_enabled_.load() && pending.expires_at > now) {
                std::lock_guard<std::mutex> lock(session_mutex_);
                const auto& queue = pending_tokens_[connector];
                const bool already_queued = std::any_of(queue.begin(), queue.end(), [&](const PendingToken& existing) {
                    return existing.token.source == AuthTokenSource::Autocharge &&
                           clamp_id_token(existing.token.id_token) == trimmed;
                });
                if (!already_queued) {
                    PendingToken retry = pending;
                    // Retry immediately in MAC mode while online; defer only when CSMS is actually offline.
                    retry.defer_until_online = csms_offline;
                    pending_tokens_[connector].push_back(std::move(retry));
                    pending_changed = true;
                }
            }
        } else {
            autocharge_retry_not_before_.erase(connector);
            autocharge_retry_fail_count_.erase(connector);
            EVLOG_info << "Autocharge rejected for session " << session_id << " on connector " << connector
                       << "; forcing EIM fallback";
        }
    }
    if (pending_changed) {
        persist_pending_tokens();
    }

    bool session_present = false;
    bool session_matches = false;
    bool apply_state = true;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        auto it = sessions_.find(connector);
        if (it != sessions_.end()) {
            session_present = true;
            session_matches = (it->second.session_id == session_id);
            if (!session_matches) {
                EVLOG_warning << "Authorization result for stale session " << session_id
                              << " while connector " << connector << " is on session " << it->second.session_id
                              << "; evaluating retarget safety";
                if (accepted) {
                    // Only retarget stale accepted results when the current session already carries
                    // an explicit matching token. Otherwise stale accepts can authorize a new session.
                    if (!it->second.id_token.has_value()) {
                        EVLOG_warning << "Discarding stale accepted auth result without current token (connector "
                                      << connector << ")";
                        apply_state = false;
                    } else if (it->second.id_token.value() != trimmed) {
                        EVLOG_warning << "Discarding stale accepted auth token mismatch (current="
                                      << it->second.id_token.value() << ", incoming=" << trimmed
                                      << ") connector " << connector;
                        apply_state = false;
                    } else {
                        session_matches = true;
                    }
                } else {
                    // A stale rejection must never be applied to the current session.
                    apply_state = false;
                }
            }
            if (accepted && apply_state) {
                if (it->second.id_token.has_value() && it->second.id_token.value() != trimmed) {
                    EVLOG_warning << "Authorization token mismatch on connector " << connector
                                  << " (have " << it->second.id_token.value() << ", got " << trimmed
                                  << "); ignoring update";
                    apply_state = false;
                } else {
                    it->second.authorized = true;
                    it->second.id_token = trimmed;
                    it->second.authorized_at = now;
                    it->second.power_wait_started_at = now;
                    it->second.token_source = pending.token.source;
                }
            }
        }
    }
    if (apply_state && session_present && session_matches) {
        set_auth_state(connector, new_state);
        if (new_state == AuthorizationState::Denied) {
            note_auth_denied(connector, trimmed);
            autocharge_retry_not_before_.erase(connector);
            autocharge_retry_fail_count_.erase(connector);
        } else if (new_state == AuthorizationState::Granted) {
            autocharge_retry_not_before_.erase(connector);
            autocharge_retry_fail_count_.erase(connector);
        }
    }
    return new_state;
}

void OcppAdapter::enqueue_auth_request(std::int32_t connector, const std::string& session_id, const PendingToken& pending) {
    if (!auth_thread_running_) {
        return;
    }
    AuthRequest req;
    req.connector = connector;
    req.session_id = session_id;
    req.pending = pending;
    req.enqueued_at = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(auth_queue_mutex_);
        auto inflight = auth_in_flight_.find(connector);
        if (inflight != auth_in_flight_.end()) {
            if (inflight->second == session_id) {
                return;
            }
            auth_in_flight_.erase(connector);
            auth_results_.erase(connector);
            auth_queue_.erase(std::remove_if(auth_queue_.begin(), auth_queue_.end(),
                                             [connector](const AuthRequest& r) { return r.connector == connector; }),
                              auth_queue_.end());
        }
        auth_in_flight_[connector] = session_id;
        auth_queue_.push_back(std::move(req));
    }
    auth_queue_cv_.notify_one();
}

std::optional<OcppAdapter::AuthResult> OcppAdapter::pop_auth_result(std::int32_t connector,
                                                                    const std::string& session_id) {
    std::lock_guard<std::mutex> lock(auth_queue_mutex_);
    auto it = auth_results_.find(connector);
    if (it == auth_results_.end()) {
        return std::nullopt;
    }
    AuthResult res = it->second;
    auth_results_.erase(it);
    auth_in_flight_.erase(connector);
    if (!session_id.empty() && res.session_id != session_id) {
        bool retarget_to_current = false;
        if (res.accepted) {
            std::lock_guard<std::mutex> session_lock(session_mutex_);
            auto sit = sessions_.find(connector);
            if (sit != sessions_.end() && sit->second.session_id == session_id && !sit->second.authorized) {
                // Security hardening: only retarget stale accepted auth when the current
                // session already has an explicit matching token.
                if (!sit->second.id_token.has_value()) {
                    EVLOG_warning << "Discarding stale accepted auth result without current token (connector "
                                  << connector << ")";
                } else {
                    const auto token = clamp_id_token(res.pending.token.id_token);
                    if (token.empty() || sit->second.id_token.value() != token) {
                        EVLOG_warning << "Discarding stale accepted auth token mismatch while retargeting (current="
                                      << sit->second.id_token.value() << ", incoming=" << token
                                      << ") connector " << connector;
                    } else {
                        retarget_to_current = true;
                    }
                }
            }
        }
        if (retarget_to_current) {
            EVLOG_warning << "Retargeting auth result from stale session " << res.session_id
                          << " to current session " << session_id << " connector " << connector;
            res.session_id = session_id;
        } else {
            EVLOG_warning << "Discarding auth result for stale session " << res.session_id
                          << " (current=" << session_id << ") connector " << connector;
            return std::nullopt;
        }
    }
    return res;
}

void OcppAdapter::clear_auth_queue_for_connector(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(auth_queue_mutex_);
    auth_in_flight_.erase(connector);
    auth_results_.erase(connector);
    auth_queue_.erase(std::remove_if(auth_queue_.begin(), auth_queue_.end(),
                                     [connector](const AuthRequest& req) { return req.connector == connector; }),
                      auth_queue_.end());
}

void OcppAdapter::auth_loop() {
    while (running_ && auth_thread_running_) {
        AuthRequest req;
        {
            std::unique_lock<std::mutex> lock(auth_queue_mutex_);
            auth_queue_cv_.wait(lock, [this]() {
                return !auth_thread_running_ || !running_ || !auth_queue_.empty();
            });
            if (!running_ || !auth_thread_running_) {
                break;
            }
            if (auth_queue_.empty()) {
                continue;
            }
            req = std::move(auth_queue_.front());
            auth_queue_.pop_front();
        }

        bool accepted = req.pending.token.prevalidated;
        if (!accepted && charge_point_) {
            try {
                const auto trimmed = clamp_id_token(req.pending.token.id_token);
                const auto info = charge_point_->authorize_id_token(ocpp::CiString<20>(trimmed));
                accepted = (info.id_tag_info.status == ocpp::v16::AuthorizationStatus::Accepted);
            } catch (const std::exception& e) {
                EVLOG_warning << "Authorization worker exception on connector " << req.connector << ": " << e.what();
                accepted = false;
            }
        }

        AuthResult res;
        res.connector = req.connector;
        res.session_id = req.session_id;
        res.pending = std::move(req.pending);
        res.accepted = accepted;

        {
            std::lock_guard<std::mutex> lock(auth_queue_mutex_);
            auth_results_[res.connector] = std::move(res);
        }
    }
}

void OcppAdapter::clear_local_auth_cache() {
    fs::path db_path = cfg_.database_dir;
    std::error_code ec;
    if (fs::is_directory(db_path, ec) && !ec) {
        db_path /= (cfg_.charge_point_id + ".db");
    }
    if (db_path.empty() || !fs::exists(db_path, ec) || ec) {
        EVLOG_warning << "Cannot clear authorization cache: database not found at " << db_path;
        return;
    }
    sqlite3* db = nullptr;
    const auto db_path_str = db_path.string();
    if (sqlite3_open(db_path_str.c_str(), &db) != SQLITE_OK) {
        EVLOG_warning << "Failed to open OCPP database for auth cache clear: " << db_path_str
                      << " err=" << (db ? sqlite3_errmsg(db) : "unknown");
        if (db) sqlite3_close(db);
        return;
    }
    char* errmsg = nullptr;
    const int rc = sqlite3_exec(db, "DELETE FROM AUTH_CACHE;", nullptr, nullptr, &errmsg);
    if (rc != SQLITE_OK) {
        EVLOG_warning << "Failed to clear AUTH_CACHE table: " << (errmsg ? errmsg : "unknown error");
        if (errmsg) sqlite3_free(errmsg);
    } else {
        EVLOG_info << "Cleared libocpp AUTH_CACHE table";
    }
    sqlite3_close(db);
}

void OcppAdapter::clear_pending_tokens() {
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        pending_tokens_.clear();
    }
    persist_pending_tokens();
}

void OcppAdapter::clear_auth_queue() {
    std::lock_guard<std::mutex> lock(auth_queue_mutex_);
    auth_queue_.clear();
    auth_results_.clear();
    auth_in_flight_.clear();
}

void OcppAdapter::handle_clear_cache() {
    clear_pending_tokens();
    clear_auth_queue();
    {
        std::lock_guard<std::mutex> lock(auth_mutex_);
        auth_denied_since_.clear();
        last_denied_token_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        recent_token_cache_.clear();
    }
    EVLOG_info << "Cleared adapter-side auth caches after ClearCache";
}

std::string OcppAdapter::clamp_id_token(const std::string& raw) const {
    constexpr std::size_t kMaxLen = 20;
    if (raw.size() <= kMaxLen) {
        return raw;
    }
    return raw.substr(0, kMaxLen);
}

bool OcppAdapter::has_active_session(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    return sessions_.find(connector) != sessions_.end();
}

void OcppAdapter::apply_power_plan() {
    std::lock_guard<std::mutex> plan_lock(plan_mutex_);
    const auto now = std::chrono::steady_clock::now();
    const bool tie_mode = (cfg_.plc_relay_mode == PlcRelayMode::Ties);
    const double switch_i_thresh =
        (cfg_.switch_max_current_a > 0.0) ? cfg_.switch_max_current_a
                                          : (planner_cfg_.mc_open_current_a > 0.0 ? planner_cfg_.mc_open_current_a
                                                                                  : 0.5);
    const auto stable_ms = std::chrono::milliseconds(std::max(0, cfg_.switch_stable_time_ms));
    const double max_dv_v = std::max(0.0, cfg_.tie_close_max_delta_v);
    const double gc_close_max_dv =
        (cfg_.precharge_voltage_tolerance_v > 0.0) ? cfg_.precharge_voltage_tolerance_v : max_dv_v;
    if (module_controller_) {
        module_controller_->poll();
    }

    std::map<int, bool> connector_meter_is_module;
    for (const auto& c : cfg_.connectors) {
        connector_meter_is_module[c.id] = (c.meter_source == "module");
    }

    std::map<int, GunStatus> status_by_connector;
    if (hardware_) {
        for (const auto& c : cfg_.connectors) {
            status_by_connector[c.id] = sanitize_status(hardware_->get_status(c.id), cfg_.lab_bypass);
        }
    }

    // Local disable state is owned by the metering/fault logic (state_mutex_), but power planning must honor it
    // so we don't keep energizing modules/relays while a connector is locally locked out.
    std::map<int, bool> local_disabled;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& c : cfg_.connectors) {
            const auto it = local_hw_disable_.find(c.id);
            local_disabled[c.id] = it != local_hw_disable_.end() && it->second;
        }
    }

    // Snapshot per-slot module telemetry once per control tick so island telemetry aggregation and per-gun
    // metering use a consistent view of module state.
    std::map<int, ModuleHealthSnapshot> module_snapshot_by_slot;
    if (module_controller_) {
        for (const auto& slot : slots_) {
            module_snapshot_by_slot[slot.id] = module_controller_->snapshot_for_slot(slot.id);
        }
    }

    struct IslandTelemetry {
        bool telemetry_valid{false};
        bool telemetry_complete{false};
        bool current_valid{false};
        double voltage_v{0.0};
        double current_a{0.0};
        double power_kw{0.0};
    };
    std::map<int, int> telemetry_slot_to_island;
    std::map<int, std::vector<int>> telemetry_island_slots;
    std::map<int, IslandTelemetry> telemetry_by_island;

    auto gun_telem_ok = [&](int gun_id, double& v, double& i, double& p) -> bool {
        const auto it = status_by_connector.find(gun_id);
        if (it == status_by_connector.end()) {
            return false;
        }
        const auto& st = it->second;
        if (st.last_telemetry.time_since_epoch().count() == 0) {
            return false;
        }
        if ((now - st.last_telemetry) > telemetry_timeout(cfg_)) {
            return false;
        }
        if (!st.present_voltage_v || !st.present_current_a) {
            return false;
        }
        v = st.present_voltage_v.value();
        i = st.present_current_a.value();
        if (st.present_power_w) {
            p = st.present_power_w.value() / 1000.0;
        } else {
            p = (v * i) / 1000.0;
        }
        return true;
    };

    // Compute current islands from the previously commanded tie states (no aux feedback available).
    // This is used to publish island-level metering for the active gun and to avoid opening GC under load
    // when the home slot has no locally-assigned modules.
    if (tie_mode && !slots_.empty()) {
        std::map<int, std::size_t> slot_index;
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            slot_index[slots_[i].id] = i;
        }
        std::vector<std::size_t> parent(slots_.size());
        for (std::size_t i = 0; i < parent.size(); ++i) {
            parent[i] = i;
        }
        auto find_root = [&](std::size_t i) -> std::size_t {
            while (parent[i] != i) {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            return i;
        };
        auto unite = [&](std::size_t a, std::size_t b) {
            const auto ra = find_root(a);
            const auto rb = find_root(b);
            if (ra != rb) {
                parent[rb] = ra;
            }
        };

        for (std::size_t i = 0; i < slots_.size(); ++i) {
            const auto& s = slots_[i];
            if (s.cw_id == 0) continue;
            const auto cw_it = slot_index.find(s.cw_id);
            if (cw_it == slot_index.end()) continue;
            const auto mc_it = last_mc_state_.find(s.mc_id);
            const ContactorState mc_state = (mc_it != last_mc_state_.end()) ? mc_it->second : ContactorState::Open;
            if (mc_state == ContactorState::Closed) {
                unite(i, cw_it->second);
            }
        }

        std::map<std::size_t, int> root_to_island;
        int next_island = 1;
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            const auto root = find_root(i);
            auto it = root_to_island.find(root);
            if (it == root_to_island.end()) {
                it = root_to_island.emplace(root, next_island++).first;
            }
            const int island_id = it->second;
            telemetry_slot_to_island[slots_[i].id] = island_id;
            telemetry_island_slots[island_id].push_back(slots_[i].id);
        }

        for (const auto& kv : telemetry_island_slots) {
            const int island_id = kv.first;
            const auto& slot_ids = kv.second;
            bool any_telem = false;
            bool complete = true;
            bool current_complete = true;
            double v_sum = 0.0;
            int v_count = 0;
            double i_sum = 0.0;
            double p_sum = 0.0;
            for (int slot_id : slot_ids) {
                const auto* slot = find_slot(slots_, slot_id);
                const bool slot_has_modules = slot && !slot->modules.empty();
                const auto snap_it = module_snapshot_by_slot.find(slot_id);
                const bool have_slot_telem =
                    snap_it != module_snapshot_by_slot.end() && snap_it->second.valid && snap_it->second.telemetry_valid;
                bool used_gun_telem = false;
                bool prefer_module_meter = false;
                if (slot && slot->gun_id > 0) {
                    const auto it = connector_meter_is_module.find(slot->gun_id);
                    prefer_module_meter = (it != connector_meter_is_module.end()) && it->second;
                }
                if (slot && slot->gun_id > 0) {
                    double gv = 0.0;
                    double gi = 0.0;
                    double gp = 0.0;
                    // Prefer gun-side telemetry (connector/shunt/PLC) unless this connector's configured meter
                    // source is "module" and the module telemetry is available.
                    const bool allow_gun_telem = (!have_slot_telem) || (!prefer_module_meter);
                    if (allow_gun_telem && gun_telem_ok(slot->gun_id, gv, gi, gp)) {
                        used_gun_telem = true;
                        any_telem = true;
                        v_sum += gv;
                        v_count++;
                        i_sum += gi;
                        p_sum += gp;
                    }
                }
                if (slot_has_modules && !have_slot_telem && !used_gun_telem) {
                    complete = false;
                    current_complete = false;
                }
                if (have_slot_telem && !used_gun_telem) {
                    any_telem = true;
                    v_sum += snap_it->second.voltage_v;
                    v_count++;
                    if (snap_it->second.current_valid) {
                        i_sum += snap_it->second.current_a;
                        p_sum += snap_it->second.power_kw;
                    } else {
                        current_complete = false;
                    }
                }
            }
            IslandTelemetry telem{};
            telem.telemetry_valid = any_telem && v_count > 0;
            telem.current_valid = telem.telemetry_valid && current_complete;
            telem.telemetry_complete = complete && telem.telemetry_valid && telem.current_valid;
            if (telem.telemetry_valid) {
                telem.voltage_v = v_sum / static_cast<double>(v_count);
                if (telem.current_valid) {
                    telem.current_a = i_sum;
                    telem.power_kw = p_sum > 0.0 ? p_sum : (telem.voltage_v * telem.current_a) / 1000.0;
                }
            }
            telemetry_by_island[island_id] = telem;
        }
    }

    auto enforce_hold = [&](const std::string& id, ContactorState desired,
                            std::map<std::string, ContactorState>& last_state,
                            std::map<std::string, std::chrono::steady_clock::time_point>& last_change,
                            int hold_ms, bool prefer_open) -> ContactorState {
        const auto prev_it = last_state.find(id);
        const ContactorState prev = prev_it != last_state.end() ? prev_it->second : ContactorState::Open;
        const auto ts_it = last_change.find(id);
        const bool in_hold = ts_it != last_change.end() &&
            (now - ts_it->second) < std::chrono::milliseconds(std::max(0, hold_ms));
        if (in_hold && desired != prev) {
            if (prefer_open && desired == ContactorState::Open) {
                last_change[id] = now;
                last_state[id] = desired;
                return desired;
            }
            return prev;
        }
        if (desired != prev) {
            last_change[id] = now;
        }
        last_state[id] = desired;
        return desired;
    };
    std::vector<GunState> guns;
    guns.reserve(cfg_.connectors.size());
    std::map<int, GunState> gun_lookup;
    std::set<int> hold_guns_no_current;
    struct ConnSnapshot {
        GunStatus status;
        double measured_voltage_v{0.0};
        double measured_power_kw{0.0};
        double measured_current_a{0.0};
        bool forced_fault{false};
        std::string forced_fault_reason;
        bool island_telem_valid{false};
        bool island_telem_complete{false};
        double island_voltage_v{0.0};
        double island_current_a{0.0};
        double island_power_kw{0.0};
        bool module_telem_valid{false};
        bool module_current_valid{false};
        double module_voltage_v{0.0};
        double module_current_a{0.0};
        double module_power_kw{0.0};
        bool module_can_overload{false};
        double module_can_total_kbps{0.0};
        bool module_health_valid{true};
        bool module_unavailable_fault{false};
    };
    std::map<int, ConnSnapshot> snapshots;
    bool trip_global = false;
    std::string global_reason;
    for (const auto& c : cfg_.connectors) {
        power_constrained_[c.id] = false;
    }

    for (const auto& c : cfg_.connectors) {
        GunStatus st = status_by_connector.count(c.id) ? status_by_connector[c.id]
                                                       : sanitize_status(hardware_->get_status(c.id), cfg_.lab_bypass);
        const uint64_t prev_present_stale = last_present_stale_counts_[c.id];
        const uint64_t prev_limit_stale = last_limit_stale_counts_[c.id];
        if (st.present_stale_events > prev_present_stale) {
            EVLOG_warning << "Connector " << c.id << " EVSE_PRESENT cadence stale events observed ("
                          << st.present_stale_events << "); treating as comm fault";
            st.comm_fault = true;
            power_constrained_[c.id] = true;
            last_present_stale_counts_[c.id] = st.present_stale_events;
        }
        if (st.limit_stale_events > prev_limit_stale) {
            EVLOG_warning << "Connector " << c.id << " EVSE_LIMIT cadence stale events observed ("
                          << st.limit_stale_events << "); constraining power";
            power_constrained_[c.id] = true;
            last_limit_stale_counts_[c.id] = st.limit_stale_events;
        }
        static std::map<int, uint64_t> last_relay_conflict;
        const uint64_t prev_relay_conflict = last_relay_conflict[c.id];
        if (st.relay_conflict_count > prev_relay_conflict) {
            last_relay_conflict[c.id] = st.relay_conflict_count;
            EVLOG_error << "Connector " << c.id << " relay conflict/weld detected; halting charging";
            st.comm_fault = true;
            power_constrained_[c.id] = true;
        }
        const Slot* slot_for_conn = find_slot_for_gun(c.id);
        bool module_telem_valid = false;
        bool module_current_valid = false;
        double module_voltage_v = 0.0;
        double module_current_a = 0.0;
        double module_power_kw = 0.0;
        bool module_can_overload = false;
        double module_can_total_kbps = 0.0;
        bool module_health_valid = !module_controller_;
        if (module_controller_) {
            bool any_valid = false;
            bool any_health_valid = false;
            uint8_t healthy_mask = 0;
            uint8_t fault_mask = 0;
            std::array<double, 2> temps{{0.0, 0.0}};
            double v_sum = 0.0;
            int v_count = 0;
            int current_expected_count = 0;
            int current_valid_count = 0;
            double i_sum = 0.0;
            double p_sum = 0.0;

            const auto module_slots_it = connector_module_slots_.find(c.id);
            if (module_slots_it != connector_module_slots_.end()) {
                for (int idx = 0; idx < 2; ++idx) {
                    const int slot_id = module_slots_it->second[idx];
                    if (slot_id <= 0) continue;
                    const auto snap_it = module_snapshot_by_slot.find(slot_id);
                    const ModuleHealthSnapshot snap =
                        snap_it != module_snapshot_by_slot.end() ? snap_it->second : ModuleHealthSnapshot{};
                    if (!snap.valid) continue;
                    any_valid = true;
                    healthy_mask |= snap.healthy_mask;
                    fault_mask |= snap.fault_mask;
                    if (idx >= 0 && idx < static_cast<int>(temps.size())) {
                        temps[static_cast<std::size_t>(idx)] = snap.temperatures_c[static_cast<std::size_t>(idx)];
                    }
                    if (snap.health_valid) {
                        any_health_valid = true;
                    }
                    module_can_overload = module_can_overload || snap.can_overload_latched;
                    module_can_total_kbps = std::max(module_can_total_kbps, snap.can_total_kbps);
                    if (snap.telemetry_valid) {
                        module_telem_valid = true;
                        v_sum += snap.voltage_v;
                        v_count++;
                        current_expected_count++;
                        if (snap.current_valid) {
                            current_valid_count++;
                            i_sum += snap.current_a;
                            p_sum += snap.power_kw;
                        }
                    }
                }
            } else if (slot_for_conn) {
                auto snap_it = module_snapshot_by_slot.find(slot_for_conn->id);
                const ModuleHealthSnapshot snap =
                    snap_it != module_snapshot_by_slot.end() ? snap_it->second : ModuleHealthSnapshot{};
                if (snap.valid) {
                    any_valid = true;
                    healthy_mask = snap.healthy_mask;
                    fault_mask = snap.fault_mask;
                    for (std::size_t i = 0; i < snap.temperatures_c.size() && i < temps.size(); ++i) {
                        temps[i] = snap.temperatures_c[i];
                    }
                    any_health_valid = snap.health_valid;
                    module_can_overload = module_can_overload || snap.can_overload_latched;
                    module_can_total_kbps = std::max(module_can_total_kbps, snap.can_total_kbps);
                    if (snap.telemetry_valid) {
                        module_telem_valid = true;
                        v_sum = snap.voltage_v;
                        v_count = 1;
                        current_expected_count = 1;
                        if (snap.current_valid) {
                            current_valid_count = 1;
                            i_sum = snap.current_a;
                            p_sum = snap.power_kw;
                        }
                    }
                }
            }

            if (any_valid) {
                st.module_healthy_mask = healthy_mask;
                st.module_fault_mask = fault_mask;
                st.module_temp_c = temps;
                module_health_valid = any_health_valid;
            } else {
                module_health_valid = false;
            }
            if (module_telem_valid && v_count > 0) {
                module_voltage_v = v_sum / static_cast<double>(v_count);
                module_current_valid =
                    current_expected_count > 0 && current_valid_count == current_expected_count;
                if (module_current_valid) {
                    module_current_a = i_sum;
                    module_power_kw = p_sum > 0.0 ? p_sum : (module_voltage_v * module_current_a) / 1000.0;
                }
            }
        }
        const bool have_telemetry = st.last_telemetry.time_since_epoch().count() != 0;
        const bool critical_trip = st.estop || st.earth_fault;
        if ((have_telemetry || critical_trip) && safety_trip_needed(st)) {
            trip_global = true;
            if (global_reason.empty()) {
                if (st.estop) {
                    global_reason = "estop";
                } else if (st.earth_fault) {
                    global_reason = "earth";
                } else {
                    global_reason = "safety";
                }
            }
        }

        GunState g{};
        g.id = c.id;
        g.slot_id = slot_for_conn ? slot_for_conn->id : c.id;
        g.gc_id = slot_for_conn ? slot_for_conn->gc_id : "GC_" + std::to_string(c.id);
        g.gun_power_limit_kw = c.max_power_w > 0 ? c.max_power_w / 1000.0 : 0.0;
        g.gun_current_limit_a = c.max_current_a > 0 ? c.max_current_a : 0.0;
        g.max_voltage_v = c.max_voltage_v > 0.0 ? c.max_voltage_v : 0.0;
        g.min_voltage_v = c.min_voltage_v > 0.0 ? c.min_voltage_v : planner_cfg_.min_voltage_v_for_div;
        g.priority = 0;
        g.i_set_a = last_current_limit_a_[c.id];
        g.connector_temp_c = st.connector_temp_c;
        g.gc_welded = st.gc_welded;
        g.mc_welded = st.mc_welded;
        g.safety_ok = st.safety_ok && !st.estop && !st.earth_fault && !st.overcurrent_fault && !st.comm_fault &&
                      !st.cp_fault && !st.isolation_fault && !st.overtemp_fault;
        g.plugged_in = st.plugged_in;
        g.reserved = reserved_connectors_[c.id];

        const bool lock_required = c.require_lock;
        const bool lock_ok = !lock_required || st.lock_engaged;
        bool session_present = false;
        bool session_authorized = false;
        bool session_transaction_started = false;
        std::chrono::steady_clock::time_point session_tx_start{};
        std::string active_session_id;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            auto sit = sessions_.find(c.id);
            if (sit != sessions_.end()) {
                session_present = true;
                session_authorized = sit->second.authorized;
                session_transaction_started = sit->second.transaction_started;
                session_tx_start = sit->second.transaction_started_at;
                active_session_id = sit->second.session_id;
            }
        }
        auto reset_connector_runtime_state = [&](int connector_id) {
            gc_open_pending_.erase(connector_id);
            gc_open_request_time_.erase(connector_id);
            gc_open_timeout_exceeded_since_.erase(connector_id);
            gc_close_request_time_.erase(connector_id);
            power_delivery_stall_since_.erase(connector_id);
            power_request_lost_since_.erase(connector_id);
            last_power_request_active_.erase(connector_id);
            last_cp_request_drop_.erase(connector_id);
            last_cp_request_drop_reason_.erase(connector_id);
            current_underdelivery_since_.erase(connector_id);
            current_underdelivery_log_.erase(connector_id);
            precharge_arm_ready_since_.erase(connector_id);
            precharge_ramp_since_.erase(connector_id);
            precharge_voltage_stable_since_.erase(connector_id);
            precharge_transition_since_.erase(connector_id);
            precharge_overcurrent_since_.erase(connector_id);
            precharge_overshoot_since_.erase(connector_id);
            precharge_ramp_cmd_voltage_v_.erase(connector_id);
            precharge_target_power_kw_.erase(connector_id);
            module_missing_since_.erase(connector_id);
            stuck_output_voltage_since_.erase(connector_id);
            stuck_output_current_since_.erase(connector_id);

            const auto module_slots_it = connector_module_slots_.find(connector_id);
            if (module_slots_it != connector_module_slots_.end()) {
                for (const int slot_id : module_slots_it->second) {
                    if (slot_id <= 0) {
                        continue;
                    }
                    mc_open_pending_.erase(slot_id);
                    mc_open_request_time_.erase(slot_id);
                    const Slot* runtime_slot = find_slot(slots_, slot_id);
                    if (!runtime_slot) {
                        continue;
                    }
                    mc_switch_ready_since_.erase(runtime_slot->mc_id);
                    if (!runtime_slot->gc_id.empty()) {
                        gc_switch_ready_since_.erase(runtime_slot->gc_id);
                    }
                }
            } else if (slot_for_conn) {
                mc_open_pending_.erase(slot_for_conn->id);
                mc_open_request_time_.erase(slot_for_conn->id);
                mc_switch_ready_since_.erase(slot_for_conn->mc_id);
                if (!slot_for_conn->gc_id.empty()) {
                    gc_switch_ready_since_.erase(slot_for_conn->gc_id);
                }
            }
        };
        if (session_present && !active_session_id.empty()) {
            auto& last_session_id = last_plan_session_id_[c.id];
            if (last_session_id != active_session_id) {
                reset_connector_runtime_state(c.id);
                last_session_id = active_session_id;
            }
        } else if (last_plan_session_id_.erase(c.id) > 0) {
            reset_connector_runtime_state(c.id);
        }
        const bool disabled_by_csms = evse_disabled_.count(c.id) ? evse_disabled_[c.id] : false;
        const bool disabled_by_local = local_disabled.count(c.id) ? local_disabled.at(c.id) : false;
        const bool disabled_by_control = disabled_by_csms || disabled_by_local;
        const bool paused_by_csms = paused_evse_.count(c.id) ? paused_evse_[c.id] : false;
        const bool session_ready = session_present && session_authorized && !disabled_by_control && !paused_by_csms;
        const AuthorizationState auth_state = get_auth_state(c.id);
        const bool auth_pending = auth_state == AuthorizationState::Pending;
        bool post_stop_plugged = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const auto it = post_stop_plugged_.find(c.id);
            post_stop_plugged = it != post_stop_plugged_.end() && it->second;
        }
        const bool power_ready =
            session_ready && (st.relay_closed || power_delivery_requested(st, lock_required));
        const bool hlc_power_phase =
            st.hlc_power_ready || (st.hlc_stage >= HLC_MIN_POWER_STAGE && !st.hlc_charge_complete);
        // Allow precharge/warmup decisions even before the OCPP transaction is authorized so ISO15118 can progress.
        // Energy delivery is still gated by `power_ready`/GC closure later in the state machine.
        bool precharge_hint = lock_ok && st.plugged_in && !post_stop_plugged && !st.hlc_charge_complete &&
                              is_hlc_precharge_phase(st) && !disabled_by_control && !paused_by_csms;
        if (cfg_.require_auth_for_precharge) {
            precharge_hint = precharge_hint && session_ready;
        }
        // Post-precharge hold: once the EV has completed precharge (HLC power phase) but OCPP authorization is still
        // pending, keep modules allocated so we can hold voltage without forcing a 0V dip. Current delivery is forced
        // to 0A later in the command loop.
        const bool post_precharge_hold_candidate =
            !cfg_.require_auth_for_precharge && session_present && !session_authorized && auth_pending && lock_ok &&
            st.plugged_in && !post_stop_plugged && !st.hlc_charge_complete && hlc_power_phase &&
            !disabled_by_control && !paused_by_csms;
        if (session_present) {
            g.reserved = false;
            reserved_connectors_[c.id] = false;
        }

        // Do not feed back PLC-reported EVSE limits into planner caps.
        // Those limits are controller-published outputs and can transiently reflect
        // dynamic command throttling; re-applying them here creates a recursive clamp.
        if (profile_current_limit_a_.count(c.id) && profile_current_limit_a_[c.id] > 0.0) {
            g.gun_current_limit_a = g.gun_current_limit_a > 0.0
                                        ? std::min(g.gun_current_limit_a, profile_current_limit_a_[c.id])
                                        : profile_current_limit_a_[c.id];
        }
        if (profile_power_limit_kw_.count(c.id) && profile_power_limit_kw_[c.id] > 0.0) {
            g.gun_power_limit_kw = g.gun_power_limit_kw > 0.0
                                       ? std::min(g.gun_power_limit_kw, profile_power_limit_kw_[c.id])
                                       : profile_power_limit_kw_[c.id];
        }

        double measured_v = st.present_voltage_v ? st.present_voltage_v.value()
                                                 : (last_voltage_v_[c.id] > 50.0 ? last_voltage_v_[c.id]
                                                                                  : planner_cfg_.default_voltage_v);
        double measured_i =
            st.present_current_a ? st.present_current_a.value()
                                 : (last_power_w_[c.id] > 0 && measured_v > 0.0 ? last_power_w_[c.id] / measured_v
                                                                                : 0.0);
        double measured_power_kw =
            st.present_power_w ? st.present_power_w.value() / 1000.0
                               : (last_power_w_[c.id] > 0 ? last_power_w_[c.id] / 1000.0 : 0.0);
        const bool prefer_module_meter = (c.meter_source == "module");
        const bool missing_present = !st.present_voltage_v || !st.present_current_a;
        if (module_telem_valid && (prefer_module_meter || missing_present)) {
            measured_v = module_voltage_v;
            st.present_voltage_v = measured_v;
            if (module_current_valid) {
                measured_i = module_current_a;
                measured_power_kw = module_power_kw;
                st.present_current_a = measured_i;
                st.present_power_w = measured_power_kw * 1000.0;
            } else {
                // If module current readback blips briefly, keep the last fresh present-current sample
                // for one telemetry window instead of forcing an immediate 0A step.
                const bool present_fresh =
                    st.last_telemetry.time_since_epoch().count() != 0 &&
                    (now - st.last_telemetry) <= telemetry_timeout(cfg_);
                if (st.relay_closed && present_fresh && st.present_current_a && st.present_power_w) {
                    measured_i = std::max(0.0, st.present_current_a.value());
                    measured_power_kw = std::max(0.0, st.present_power_w.value() / 1000.0);
                } else {
                    measured_i = 0.0;
                    measured_power_kw = 0.0;
                    st.present_current_a = 0.0;
                    st.present_power_w = 0.0;
                }
            }
        } else if (!st.relay_closed) {
            // Do not keep publishing stale current/power values when the gun contactor is open.
            measured_i = 0.0;
            measured_power_kw = 0.0;
            st.present_current_a = 0.0;
            st.present_power_w = 0.0;
        }

        // In tie/islanding mode, publish aggregated island telemetry for the active gun so metering and safety
        // logic reflect the total current delivered across all contributing slots.
        bool island_telem_valid = false;
        bool island_telem_complete = false;
        bool island_current_valid = false;
        double island_voltage_v = 0.0;
        double island_current_a = 0.0;
        double island_power_kw = 0.0;
        if (tie_mode && slot_for_conn) {
            const auto island_it = telemetry_slot_to_island.find(slot_for_conn->id);
            if (island_it != telemetry_slot_to_island.end()) {
                const int island_id = island_it->second;
                const auto telem_it = telemetry_by_island.find(island_id);
                if (telem_it != telemetry_by_island.end()) {
                    island_telem_valid = telem_it->second.telemetry_valid;
                    island_telem_complete = telem_it->second.telemetry_complete;
                    island_current_valid = telem_it->second.current_valid;
                    island_voltage_v = telem_it->second.voltage_v;
                    island_current_a = telem_it->second.current_a;
                    island_power_kw = telem_it->second.power_kw;
                    if (st.relay_closed && island_telem_valid && island_current_valid) {
                        measured_v = island_voltage_v;
                        measured_i = island_current_a;
                        measured_power_kw = island_power_kw;
                        st.present_voltage_v = measured_v;
                        st.present_current_a = measured_i;
                        st.present_power_w = measured_power_kw * 1000.0;
                    }
                }
            }
        }
        if (measured_v > 0.0) {
            last_voltage_v_[c.id] = measured_v;
        }
        if (st.present_power_w) {
            last_power_w_[c.id] = st.present_power_w.value();
        } else if (module_current_valid) {
            last_power_w_[c.id] = measured_power_kw * 1000.0;
        }

        const auto snap_state_it = snapshots.find(c.id);
        const bool module_health_valid_snap =
            snap_state_it != snapshots.end() ? snap_state_it->second.module_health_valid : true;
        const bool module_unavailable_fault_snap =
            snap_state_it != snapshots.end() ? snap_state_it->second.module_unavailable_fault : false;
        // Update module health for both module slots owned by this connector.
        const auto module_slots_it = connector_module_slots_.find(c.id);
        if (module_slots_it != connector_module_slots_.end()) {
            for (int idx = 0; idx < 2; ++idx) {
                const int slot_id = module_slots_it->second[idx];
                if (slot_id <= 0) continue;
                const uint8_t bit = static_cast<uint8_t>(1U << idx);
                const bool healthy_bit = (st.module_healthy_mask & bit) != 0;
                const bool fault_bit = (st.module_fault_mask & bit) != 0;
                const double module_temp = (idx < static_cast<int>(st.module_temp_c.size()))
                                               ? st.module_temp_c[static_cast<std::size_t>(idx)]
                                               : 0.0;
                for (auto& mod : module_states_) {
                    if (mod.slot_id == slot_id) {
                        mod.temperature_c = module_temp;
                        if (module_health_valid_snap) {
                            mod.healthy = healthy_bit && !fault_bit;
                        } else if (module_unavailable_fault_snap) {
                            mod.healthy = false;
                        }
                        break;
                    }
                }
            }
        }
        int healthy_modules = 0;
        if (tie_mode && slot_for_conn && telemetry_slot_to_island.count(slot_for_conn->id)) {
            const int island_id = telemetry_slot_to_island.at(slot_for_conn->id);
            for (const auto& mod : module_states_) {
                if (!mod.healthy) continue;
                const auto it = telemetry_slot_to_island.find(mod.slot_id);
                if (it != telemetry_slot_to_island.end() && it->second == island_id) {
                    healthy_modules++;
                }
            }
        } else if (connector_module_slots_.count(c.id)) {
            const auto slot_ids = connector_module_slots_[c.id];
            for (const auto& mod : module_states_) {
                if (!mod.healthy) continue;
                if ((slot_ids[0] > 0 && mod.slot_id == slot_ids[0]) ||
                    (slot_ids[1] > 0 && mod.slot_id == slot_ids[1])) {
                    healthy_modules++;
                }
            }
        } else if (slot_for_conn) {
            for (const auto& mod : module_states_) {
                if (mod.healthy && mod.slot_id == slot_for_conn->id) {
                    healthy_modules++;
                }
            }
        }

        const bool modules_ok = healthy_modules > 0;

        const bool welded = st.gc_welded || st.mc_welded;
        const bool isolation_fault = st.isolation_fault;
        const bool comm_fault = st.comm_fault;
        const bool thermal_fault = st.overtemp_fault;
        const bool overcurrent_fault = st.overcurrent_fault;
        const bool meter_fault_active =
            st.meter_stale && (power_ready || precharge_hint || post_precharge_hold_candidate || st.relay_closed);

        // If we reach precharge/power stages but no healthy modules are available, fail safe so the PLC/HLC can
        // abort instead of looping indefinitely with 0 power.
        const bool need_modules = power_ready || precharge_hint || post_precharge_hold_candidate;
        bool module_unavailable_fault = false;
        if (need_modules && !modules_ok) {
            auto& ts = module_missing_since_[c.id];
            if (ts.time_since_epoch().count() == 0) {
                ts = now;
            } else {
                const auto timeout_ms = std::max({2000, cfg_.precharge_timeout_ms, cfg_.telemetry_timeout_ms});
                if ((now - ts) > std::chrono::milliseconds(timeout_ms)) {
                    constexpr auto kModuleRecoveryGrace = std::chrono::seconds(20);
                    const auto missing_age = now - ts;
                    if (missing_age >= (std::chrono::milliseconds(timeout_ms) + kModuleRecoveryGrace)) {
                        module_unavailable_fault = true;
                        EVLOG_error << "Connector " << c.id
                                    << " modules unavailable beyond recovery grace"
                                    << " missing_ms="
                                    << std::chrono::duration_cast<std::chrono::milliseconds>(missing_age).count()
                                    << " timeout_ms=" << timeout_ms;
                    } else {
                        static std::map<int, std::chrono::steady_clock::time_point> last_module_missing_log;
                        auto& last_log = last_module_missing_log[c.id];
                        if (last_log.time_since_epoch().count() == 0 ||
                            (now - last_log) > std::chrono::seconds(2)) {
                            EVLOG_warning << "Connector " << c.id
                                          << " modules temporarily unavailable; waiting for recovery"
                                          << " missing_ms="
                                          << std::chrono::duration_cast<std::chrono::milliseconds>(missing_age).count();
                            last_log = now;
                        }
                    }
                }
            }
        } else {
            module_missing_since_.erase(c.id);
        }

        // Telemetry-only sanity checks (no aux relay feedback):
        // If the output path is expected to be de-energized but voltage/current remains high for a sustained window,
        // treat it as a local fault (possible welded contactor, EV backfeed, or discharge failure).
        const bool present_fresh = st.last_telemetry.time_since_epoch().count() != 0 &&
                                   (now - st.last_telemetry) <= telemetry_timeout(cfg_);
        const bool recent_target_forced_fault =
            st.last_target_update.time_since_epoch().count() != 0 &&
            (now - st.last_target_update) <= std::chrono::milliseconds(std::max(2500, cfg_.telemetry_timeout_ms));
        const bool cp_confirms_off = st.cp_state == 'A' || st.cp_state == 'B';
        const double stuck_v_threshold =
            (cfg_.unlock_voltage_threshold_v > 0.0) ? cfg_.unlock_voltage_threshold_v : 60.0;
        const double stuck_i_threshold =
            (cfg_.gc_open_current_a > 0.0) ? cfg_.gc_open_current_a : 1.0;
        constexpr std::chrono::milliseconds kStuckVoltageTimeoutMs(12000);
        constexpr std::chrono::milliseconds kStuckCurrentTimeoutMs(2500);
        const bool module_flowing_now = module_current_valid && std::fabs(module_current_a) >= std::max(stuck_i_threshold, 1.0);
        const bool current_near_zero = !st.present_current_a || std::fabs(st.present_current_a.value()) < std::max(stuck_i_threshold, 1.5);
        const bool high_voltage_present =
            st.present_voltage_v && st.present_voltage_v.value() >= std::max(80.0, stuck_v_threshold * 0.5);
        const bool output_expected_off = !power_ready && !precharge_hint && !post_precharge_hold_candidate && !st.relay_closed &&
                                         cp_confirms_off && !recent_target_forced_fault && current_near_zero &&
                                         !module_flowing_now;
        bool forced_fault = false;
        std::string forced_fault_reason;
        if (output_expected_off && present_fresh && st.present_voltage_v &&
            st.present_voltage_v.value() >= stuck_v_threshold) {
            auto& ts = stuck_output_voltage_since_[c.id];
            if (ts.time_since_epoch().count() == 0) {
                ts = now;
            } else if ((now - ts) >= kStuckVoltageTimeoutMs) {
                forced_fault = true;
                forced_fault_reason = "StuckVoltage";
            }
        } else {
            stuck_output_voltage_since_.erase(c.id);
        }
        if (output_expected_off && present_fresh && high_voltage_present && st.present_current_a &&
            std::fabs(st.present_current_a.value()) >= stuck_i_threshold) {
            auto& ts = stuck_output_current_since_[c.id];
            if (ts.time_since_epoch().count() == 0) {
                ts = now;
            } else if (!forced_fault && (now - ts) >= kStuckCurrentTimeoutMs) {
                forced_fault = true;
                forced_fault_reason = "StuckCurrent";
            }
        } else {
            stuck_output_current_since_.erase(c.id);
        }

        const bool general_fault = !st.safety_ok || st.cp_fault || meter_fault_active || welded || isolation_fault ||
                                   thermal_fault || overcurrent_fault || comm_fault ||
                                   module_can_overload || forced_fault;
        uint8_t fault_bits = 0;
        if (general_fault) fault_bits |= 0x01;
        if (comm_fault) fault_bits |= 0x02;
        if (isolation_fault) fault_bits |= 0x04;
        if (thermal_fault) fault_bits |= 0x08;
        if (overcurrent_fault) fault_bits |= 0x10;
        if (welded) fault_bits |= 0x20;

        if (hardware_) {
            const bool output_enabled = st.relay_closed;
            const bool modules_online =
                st.plugged_in && !post_stop_plugged && !general_fault && module_telem_valid && modules_ok;
            const bool regulating = power_ready || precharge_hint || post_precharge_hold_candidate || modules_online;
            hardware_->publish_fault_state(c.id, fault_bits);
            hardware_->publish_evse_present(c.id, measured_v, measured_i, measured_power_kw, output_enabled,
                                            regulating);
        }

        const double target_voltage_for_calc = st.target_voltage_v
                                                   ? st.target_voltage_v.value()
                                                   : (measured_v > 0.0 ? measured_v : planner_cfg_.default_voltage_v);
        double ev_target_kw = 0.0;
        if ((power_ready || precharge_hint || post_precharge_hold_candidate) && st.target_current_a) {
            ev_target_kw = (target_voltage_for_calc * st.target_current_a.value()) / 1000.0;
            if (ev_target_kw > 0.0) {
                last_ev_target_power_kw_[c.id] = ev_target_kw;
            }
        } else if (last_ev_target_power_kw_.count(c.id)) {
            ev_target_kw = last_ev_target_power_kw_[c.id];
        }

        double req_kw = 0.0;
        if ((power_ready || precharge_hint || post_precharge_hold_candidate) && g.safety_ok &&
            !st.gc_welded && !st.mc_welded) {
            const double precharge_i_max = (cfg_.precharge_max_current_a > 0.0) ? cfg_.precharge_max_current_a : 2.0;
            const bool hlc_precharge_only = is_hlc_precharge_phase(st) && !hlc_power_phase;
            const bool have_target_timestamp = st.last_target_update.time_since_epoch().count() != 0;
            // Hold the last valid HLC target long enough to ride out short PLC/CP jitter bursts.
            constexpr auto kHlcPowerTargetHoldMs = std::chrono::milliseconds(12000);
            // Use EV-requested current/voltage whenever available (CurrentDemand / PreChargeReq), but avoid
            // allocating extra modules during the post-precharge authorization hold (no energy delivery).
            if (!post_precharge_hold_candidate && st.target_current_a) {
                const double i_req_raw = st.target_current_a.value();
                double i_req = std::max(0.0, i_req_raw);
                if (hlc_precharge_only && precharge_i_max > 0.0) {
                    i_req = std::min(i_req, precharge_i_max);
                }
                const double v_req = st.target_voltage_v ? st.target_voltage_v.value() : target_voltage_for_calc;
                if (v_req > 0.0 && i_req > 0.0) {
                    req_kw = (v_req * i_req) / 1000.0;
                }
            }

            // During HLC power phase, briefly hold the last valid EV target if target frames gap.
            // This avoids abrupt 51A->2A drops from short communication jitter.
            const bool hold_last_hlc_target =
                req_kw <= 0.0 && hlc_power_phase && !hlc_precharge_only && !post_precharge_hold_candidate &&
                have_target_timestamp &&
                (now - st.last_target_update) <= kHlcPowerTargetHoldMs && ev_target_kw > 0.0;
            if (hold_last_hlc_target) {
                req_kw = ev_target_kw;
            }

            // Keepalive current is only for precharge/auth-hold phases; do not use it for normal
            // HLC power delivery where stale targets should use bounded hold above.
            if (req_kw <= 0.0 && (precharge_hint || post_precharge_hold_candidate)) {
                double v_keep = g.ev_req_voltage_v;
                if (st.target_voltage_v && st.target_voltage_v.value() > 0.0) {
                    v_keep = st.target_voltage_v.value();
                }
                if (v_keep <= 0.0) {
                    v_keep = target_voltage_for_calc > 0.0 ? target_voltage_for_calc : planner_cfg_.default_voltage_v;
                }
                // Budget ~2A at the current target voltage (clamped) so precharge can progress without
                // over-allocating modules. Upper bound is limited so the planner keeps this to a single module.
                const double module_kw = planner_cfg_.module_power_kw > 0.0 ? planner_cfg_.module_power_kw : 30.0;
                const double keep_max_kw = std::min(2.0, std::max(0.1, module_kw * 0.95));
                const double keep_i = (precharge_i_max > 0.0) ? precharge_i_max : 2.0;
                req_kw = std::clamp((v_keep * keep_i) / 1000.0, 0.1, keep_max_kw);
            }
        }
        g.ev_req_power_kw = std::max(0.0, req_kw);
        g.ev_req_voltage_v = st.target_voltage_v ? st.target_voltage_v.value()
                                                 : (st.present_voltage_v ? st.present_voltage_v.value() : measured_v);
        if (g.ev_req_voltage_v <= 0.0) {
            g.ev_req_voltage_v = measured_v > 0.0 ? measured_v : planner_cfg_.default_voltage_v;
        }
        double max_voltage_v = std::numeric_limits<double>::max();
        if (c.max_voltage_v > 0.0) {
            max_voltage_v = c.max_voltage_v;
        }
        if (g.min_voltage_v > 0.0 && g.ev_req_voltage_v < g.min_voltage_v) {
            g.ev_req_voltage_v = g.min_voltage_v;
        }
        g.ev_req_voltage_v = std::min(g.ev_req_voltage_v, max_voltage_v);
        g.i_meas_a = measured_i;
        g.v_meas_v = measured_v;

        const bool blocked = !g.safety_ok || module_unavailable_fault || st.gc_welded || st.mc_welded || forced_fault;
        const bool post_precharge_hold = post_precharge_hold_candidate && !blocked;
        if (post_precharge_hold) {
            hold_guns_no_current.insert(c.id);
        }
        // Allow module allocation during ISO15118 precharge even if OCPP authorization is still pending.
        // Additionally, after precharge completes (HLC power phase) hold voltage with 0A until authorization arrives.
        g.ev_session_active = session_ready || (session_present && (precharge_hint || post_precharge_hold));
        const bool ready_for_power = (power_ready || precharge_hint || post_precharge_hold) && !blocked;

        if (blocked) {
            g.fsm_state = GunFsmState::Fault;
        } else if (st.relay_closed) {
            g.fsm_state = GunFsmState::Charging;
        } else if (ready_for_power) {
            g.fsm_state = GunFsmState::Ready;
        } else if (session_present) {
            g.fsm_state = GunFsmState::EvDetected;
        } else {
            g.fsm_state = GunFsmState::Idle;
        }
        // EVSE limit watchdog: if we have offered limits recently but PLC hasn't ACKed, constrain power.
        {
            const auto warn_threshold = std::max(std::chrono::milliseconds(500),
                                                 evse_limit_ack_timeout(cfg_) / 2);
            const bool ack_relevant = evse_limit_ack_watchdog_relevant(st, lock_required);
            const bool tx_started = session_present && session_transaction_started;
            const bool tx_start_known = session_tx_start.time_since_epoch().count() != 0;
            const bool ack_after_tx_start =
                tx_started && st.last_evse_limit_ack.time_since_epoch().count() > 0 &&
                (!tx_start_known || st.last_evse_limit_ack >= session_tx_start);
            bool ack_watchdog_constrained = false;
            if (ack_after_tx_start) {
                const auto age = now - st.last_evse_limit_ack;
                if (age > warn_threshold && ack_relevant && ready_for_power) {
                    ack_watchdog_constrained = true;
                    EVLOG_warning << "Connector " << c.id << " EVSE limit ACK stale ("
                                  << std::chrono::duration_cast<std::chrono::milliseconds>(age).count()
                                  << "ms)";
                }
            } else if (tx_started && tx_start_known && ack_relevant && ready_for_power) {
                const auto since_tx_start = now - session_tx_start;
                if (since_tx_start > warn_threshold) {
                    ack_watchdog_constrained = true;
                    EVLOG_warning << "Connector " << c.id << " EVSE limit ACK missing since transaction start ("
                                  << std::chrono::duration_cast<std::chrono::milliseconds>(since_tx_start).count()
                                  << "ms)";
                }
            }
            if (ack_watchdog_constrained) {
                // Diagnostic-only: do not throttle active delivery based only on ACK freshness.
                EVLOG_debug << "Connector " << c.id
                            << " ACK watchdog note (no power constraint applied)";
            }
        }
        guns.push_back(g);
        gun_lookup[g.id] = g;

        ConnSnapshot snap;
        snap.status = st;
        snap.measured_voltage_v = measured_v;
        snap.measured_power_kw = measured_power_kw;
        snap.measured_current_a = measured_i;
        snap.forced_fault = forced_fault;
        snap.forced_fault_reason = forced_fault_reason;
        snap.island_telem_valid = island_telem_valid;
        snap.island_telem_complete = island_telem_complete;
        snap.island_voltage_v = island_voltage_v;
        snap.island_current_a = island_current_a;
        snap.island_power_kw = island_power_kw;
        snap.module_telem_valid = module_telem_valid;
        snap.module_current_valid = module_current_valid;
        snap.module_voltage_v = module_voltage_v;
        snap.module_current_a = module_current_a;
        snap.module_power_kw = module_power_kw;
        snap.module_can_overload = module_can_overload;
        snap.module_can_total_kbps = module_can_total_kbps;
        snap.module_health_valid = module_health_valid;
        snap.module_unavailable_fault = module_unavailable_fault;
        snapshots[c.id] = snap;
    }

    // Avoid relay flip-flopping on noisy / intermittently-powered safety inputs by requiring the
    // global trip condition to be cleared continuously for a short window before unlatching.
    constexpr std::chrono::milliseconds kGlobalFaultClearDebounceMs(2000);
    if (trip_global) {
        global_fault_clear_since_ = std::chrono::steady_clock::time_point{};
    } else if (global_fault_latched_) {
        if (global_fault_clear_since_.time_since_epoch().count() == 0) {
            global_fault_clear_since_ = now;
        } else if ((now - global_fault_clear_since_) >= kGlobalFaultClearDebounceMs) {
            global_fault_latched_ = false;
            global_fault_reason_.clear();
            global_fault_clear_since_ = std::chrono::steady_clock::time_point{};
            EVLOG_info << "Global fault cleared";
        }
    } else {
        global_fault_clear_since_ = std::chrono::steady_clock::time_point{};
    }

    if (trip_global || global_fault_latched_) {
        const std::string fault_reason = global_reason.empty() ? "safety" : global_reason;
        ocpp::v16::Reason stop_reason = ocpp::v16::Reason::Other;
        if (fault_reason == "estop") {
            stop_reason = ocpp::v16::Reason::EmergencyStop;
        }
        enter_global_fault(fault_reason, stop_reason);
        apply_zero_power_plan();
        return;
    }

    power_manager_.update_modules(module_states_);
    power_manager_.update_guns(guns);
    const auto plan = power_manager_.compute_plan();

    // Update module enabled flags from MN commands
    for (auto& m : module_states_) {
        const auto it = plan.mn_commands.find(m.mn_id);
        if (it != plan.mn_commands.end()) {
            m.enabled = (it->second == ContactorState::Closed);
        } else {
            m.enabled = false;
        }
    }

    // Build dispatch lookup
    std::map<int, GunDispatch> gun_dispatch;
    for (const auto& d : plan.guns) {
        gun_dispatch[d.gun_id] = d;
    }
    {
        std::ostringstream os;
        os << "Planner summary";
        for (const auto& d : plan.guns) {
            os << " [g" << d.gun_id << " m=" << d.modules_assigned << " i_lim=" << d.current_limit_a
               << " p=" << d.p_budget_kw << " V=" << d.voltage_set_v << "]";
        }
        EVLOG_debug << os.str();
    }

    std::map<std::string, int> module_slot_index;
    std::map<std::string, bool> module_health_by_id;
    for (const auto& m : module_states_) {
        module_slot_index[m.id] = m.slot_index;
        module_health_by_id[m.id] = m.healthy;
    }
    std::map<int, SlotModuleSelection> slot_selections;
    for (const auto& slot : slots_) {
        slot_selections[slot.id] = compute_slot_module_selection(plan, slot, module_slot_index);
    }

    struct SlotCommandInfo {
        SlotModuleSelection selection;
        GunStatus status;
        GunState gun_state;
        bool disabled_by_csms{false};
        bool paused{false};
        bool local_fault{false};
        bool module_health_valid{true};
        bool module_unavailable_fault{false};
        bool module_can_overload{false};
        bool module_telem_valid{false};
        bool module_current_valid{false};
        double module_voltage_v{0.0};
        double module_current_a{0.0};
        double module_power_kw{0.0};
        double module_can_total_kbps{0.0};
        uint8_t slot_cfg_mask{0};
        bool modules_healthy{true};
        int modules_final{0};
        uint8_t mask_final{0};
        ContactorState desired_mc_state{ContactorState::Closed};
        ContactorState desired_gc_state{ContactorState::Open};
        double meas_current{0.0};
        double meas_voltage{0.0};
        std::string fault_reason;
    };

    std::map<int, SlotCommandInfo> slot_info;
    std::map<int, int> actual_modules_per_gun;
    std::map<int, bool> island_fault;
    std::map<int, std::string> island_fault_reason;
    std::map<int, int> precharge_home_slot_for_gun;
    std::map<int, uint8_t> precharge_home_mask_for_gun;
    std::map<int, uint8_t> precharge_home_relay_mask_for_gun;
    std::set<int> precharge_frozen_mc_slots;

    for (const auto& slot : slots_) {
        SlotCommandInfo info{};
        info.selection = slot_selections.count(slot.id) ? slot_selections[slot.id] : SlotModuleSelection{};
        const auto mc_it = plan.mc_commands.find(slot.mc_id);
        info.desired_mc_state = mc_it != plan.mc_commands.end() ? mc_it->second : ContactorState::Closed;
        if (slot.gun_id > 0 && !slot.gc_id.empty()) {
            const auto gc_it = plan.gc_commands.find(slot.gc_id);
            info.desired_gc_state = gc_it != plan.gc_commands.end() ? gc_it->second : ContactorState::Open;
        }

        const int owner_id =
            slot_owner_connector_.count(slot.id) ? slot_owner_connector_[slot.id] : slot.gun_id;
        const auto snap_it = snapshots.find(owner_id);
        info.status = snap_it != snapshots.end() ? snap_it->second.status : GunStatus{};
        info.meas_current = snap_it != snapshots.end() ? snap_it->second.measured_current_a : 0.0;
        info.meas_voltage = snap_it != snapshots.end()
                                ? snap_it->second.status.present_voltage_v.value_or(last_voltage_v_[owner_id])
                                : last_voltage_v_[owner_id];
        const bool slot_has_modules = !slot.modules.empty();
        if (snap_it != snapshots.end()) {
            info.module_health_valid = snap_it->second.module_health_valid;
            info.module_unavailable_fault = snap_it->second.module_unavailable_fault;
            info.module_can_overload = snap_it->second.module_can_overload;
            info.module_telem_valid = snap_it->second.module_telem_valid;
            info.module_current_valid = snap_it->second.module_current_valid;
            info.module_voltage_v = snap_it->second.module_voltage_v;
            info.module_current_a = snap_it->second.module_current_a;
            info.module_power_kw = snap_it->second.module_power_kw;
            info.module_can_total_kbps = snap_it->second.module_can_total_kbps;
        }
        if (module_controller_ && slot_has_modules) {
            const auto ms_it = module_snapshot_by_slot.find(slot.id);
            const ModuleHealthSnapshot ms =
                ms_it != module_snapshot_by_slot.end() ? ms_it->second : ModuleHealthSnapshot{};
            if (ms.valid) {
                info.module_health_valid = ms.health_valid;
                info.module_can_overload = ms.can_overload_latched;
                info.module_telem_valid = ms.telemetry_valid;
                info.module_current_valid = ms.current_valid;
                info.module_voltage_v = ms.voltage_v;
                info.module_current_a = ms.current_a;
                info.module_power_kw = ms.power_kw;
                info.module_can_total_kbps = ms.can_total_kbps;
            } else {
                info.module_health_valid = false;
                info.module_telem_valid = false;
                info.module_current_valid = false;
                info.module_voltage_v = 0.0;
                info.module_current_a = 0.0;
                info.module_power_kw = 0.0;
            }
        }
        info.gun_state = gun_lookup.count(owner_id) ? gun_lookup.at(owner_id) : GunState{};
        {
            const bool csms_disabled = evse_disabled_.count(owner_id) ? evse_disabled_[owner_id] : false;
            const bool local_disabled_owner = local_disabled.count(owner_id) ? local_disabled.at(owner_id) : false;
            // `disabled_by_csms` is used as a general "do not energize" gate in the planner. Include locally
            // disabled connectors here to prevent module/relay chatter after local faults.
            info.disabled_by_csms = csms_disabled || local_disabled_owner;
        }
        info.paused = paused_evse_[owner_id] || info.disabled_by_csms;

        int runtime_healthy_modules = 0;
        for (const auto& m : module_states_) {
            if (m.slot_id == slot.id && m.healthy) {
                runtime_healthy_modules++;
            }
        }
        info.modules_healthy = slot_has_modules && runtime_healthy_modules > 0;
        const int grace_ms = std::max(0, cfg_.module_health_grace_ms);
        bool module_health_ok = info.modules_healthy;
        if (info.modules_healthy) {
            last_module_health_ok_[slot.id] = now;
        }
        bool module_grace_active = false;
        const auto grace_it = last_module_health_ok_.find(slot.id);
        if (grace_it != last_module_health_ok_.end() &&
            (now - grace_it->second) <= std::chrono::milliseconds(grace_ms)) {
            module_grace_active = true;
        }
        // Module telemetry can dip during tie switching / relay transitions.
        // Honor a short grace window before declaring module loss.
        if ((!info.module_health_valid || !info.modules_healthy) && module_grace_active) {
            module_health_ok = true;
        } else if (!info.module_health_valid) {
            // Snapshot validity can briefly blip during bus churn. If runtime health still reports
            // healthy modules, keep the slot operable and let the debounced unavailable-fault path decide.
            module_health_ok = info.modules_healthy;
        }
        if (info.module_unavailable_fault) {
            module_health_ok = false;
        }
        if (!slot_has_modules) {
            module_health_ok = true;
        }

        if (info.status.gc_welded) info.fault_reason = "GCWelded";
        if (info.status.mc_welded) info.fault_reason = "MCWelded";
        if (info.status.estop && info.fault_reason.empty()) info.fault_reason = "EmergencyStop";
        if (info.status.earth_fault && info.fault_reason.empty()) info.fault_reason = "EarthFault";
        if (info.status.isolation_fault && info.fault_reason.empty()) info.fault_reason = "Isolation";
        if (info.status.overtemp_fault && info.fault_reason.empty()) info.fault_reason = "Overtemp";
        if (info.status.overcurrent_fault && info.fault_reason.empty()) info.fault_reason = "Overcurrent";
        if (info.status.cp_fault && info.fault_reason.empty()) info.fault_reason = "CPFault";
        if (!info.status.safety_ok && info.fault_reason.empty()) info.fault_reason = "SafetyTrip";
        if (info.status.comm_fault && info.fault_reason.empty()) info.fault_reason = "CommFault";
        if (info.module_can_overload && info.fault_reason.empty()) info.fault_reason = "ModuleCanOverload";
        const bool forced_fault = snap_it != snapshots.end() && snap_it->second.forced_fault;
        if (forced_fault && info.fault_reason.empty()) {
            info.fault_reason =
                !snap_it->second.forced_fault_reason.empty() ? snap_it->second.forced_fault_reason : "ForcedFault";
        }
        const bool modules_expected = info.selection.gun_id > 0 && info.selection.module_count > 0 && !info.disabled_by_csms;
        // Treat missing/invalid module health here as a soft degraded condition only.
        // Hard-stop is handled by the debounced module_unavailable_fault path from snapshots.
        const bool modules_degraded = slot_has_modules && modules_expected && !module_health_ok;
        (void)modules_degraded;
        if (info.module_unavailable_fault && info.fault_reason.empty()) info.fault_reason = "ModuleUnavailable";
        const bool hard_safety_fault =
            !info.status.safety_ok || info.status.estop || info.status.earth_fault || info.status.cp_fault;
        info.local_fault = hard_safety_fault || forced_fault || info.module_unavailable_fault || info.module_can_overload ||
                           info.status.gc_welded || info.status.mc_welded || info.status.isolation_fault ||
                           info.status.overtemp_fault || info.status.overcurrent_fault || info.status.comm_fault;

        // Persist module health decision for downstream command logic.
        info.modules_healthy = module_health_ok;

        if (info.selection.gun_id > 0 && info.selection.module_count > 0 && !info.local_fault &&
            !info.disabled_by_csms) {
            info.modules_final = info.selection.module_count;
            info.mask_final = info.selection.mask;
        } else {
            info.modules_final = 0;
            info.mask_final = 0;
        }

        uint8_t cfg_mask = 0u;
        for (const auto& module_id : slot.modules) {
            auto bit_it = module_slot_index.find(module_id);
            if (bit_it == module_slot_index.end()) {
                continue;
            }
            const int bit = bit_it->second;
            if (bit >= 0 && bit < 8) {
                cfg_mask |= static_cast<uint8_t>(1u << bit);
            }
        }
        info.slot_cfg_mask = cfg_mask;

        slot_info[slot.id] = info;
    }

    if (tie_mode) {
        for (const auto& c : cfg_.connectors) {
            const auto st_it = status_by_connector.find(c.id);
            if (st_it == status_by_connector.end()) {
                continue;
            }
            const GunStatus& st = st_it->second;
            const bool hlc_power_phase =
                st.hlc_power_ready || (st.hlc_stage >= HLC_MIN_POWER_STAGE && !st.hlc_charge_complete);
            const bool hlc_precharge_only = is_hlc_precharge_phase(st) && !hlc_power_phase;
            if (!hlc_precharge_only) {
                continue;
            }
            const Slot* home = find_slot_for_gun(c.id);
            if (!home) {
                continue;
            }
            const auto info_it = slot_info.find(home->id);
            if (info_it == slot_info.end()) {
                continue;
            }
            const auto& info = info_it->second;
            if (info.disabled_by_csms || info.local_fault) {
                continue;
            }
            // Keep bus topology stable during precharge to avoid MC relay chatter.
            precharge_frozen_mc_slots.insert(home->id);
            const Slot* ccw = find_slot(slots_, home->ccw_id);
            if (ccw) {
                precharge_frozen_mc_slots.insert(ccw->id);
            }
        }
    }

    std::map<int, int> island_gc_request_count;
    if (tie_mode && !telemetry_slot_to_island.empty()) {
        for (const auto& c : cfg_.connectors) {
            const Slot* home = find_slot_for_gun(c.id);
            if (!home) continue;
            const auto info_it = slot_info.find(home->id);
            if (info_it == slot_info.end()) continue;
            const auto& info = info_it->second;
            const auto snap_it = snapshots.find(c.id);
            bool wants_gc = (info.desired_gc_state == ContactorState::Closed);
            if (snap_it != snapshots.end() && snap_it->second.status.relay_closed) {
                wants_gc = true;
            }
            if (!wants_gc) continue;
            if (!(snap_it != snapshots.end() && snap_it->second.status.relay_closed)) {
                if (info.disabled_by_csms || info.local_fault) continue;
            }
            const auto isl_it = telemetry_slot_to_island.find(home->id);
            if (isl_it != telemetry_slot_to_island.end()) {
                island_gc_request_count[isl_it->second] += 1;
            }
        }
    }

    std::map<int, ContactorState> mc_state_cmd_by_slot;
    if (tie_mode) {
        for (const auto& slot : slots_) {
            auto info_it = slot_info.find(slot.id);
            if (info_it == slot_info.end()) {
                continue;
            }
            auto& info = info_it->second;
            ContactorState desired = info.desired_mc_state;
            if (info.disabled_by_csms || info.local_fault) {
                desired = ContactorState::Open;
            }

            const bool precharge_mc_frozen =
                precharge_frozen_mc_slots.count(slot.id) > 0 && !info.disabled_by_csms && !info.local_fault;
            if (precharge_mc_frozen) {
                // During precharge, force MC boundaries closed around the home segment so GC sequencing can proceed
                // without island open/close jitter.
                desired = ContactorState::Closed;
            }

            ContactorState prev = ContactorState::Open;
            const auto prev_it = last_mc_state_.find(slot.mc_id);
            if (prev_it != last_mc_state_.end()) {
                prev = prev_it->second;
            }

            ContactorState gated = prev;
            if (precharge_mc_frozen) {
                const auto held = enforce_hold(slot.mc_id, desired, last_mc_state_, mc_command_change_time_,
                                               planner_cfg_.min_mc_hold_ms, true);
                mc_state_cmd_by_slot[slot.id] = held;
                continue;
            }
            if (desired == prev) {
                mc_switch_ready_since_.erase(slot.mc_id);
                mc_open_pending_.erase(slot.id);
                mc_open_request_time_.erase(slot.id);
                gated = desired;
	            } else {
	                const Slot* cw = find_slot(slots_, slot.cw_id);
	                const int island_a =
	                    telemetry_slot_to_island.count(slot.id) ? telemetry_slot_to_island.at(slot.id) : 0;
	                const int island_b =
	                    cw && telemetry_slot_to_island.count(cw->id) ? telemetry_slot_to_island.at(cw->id) : 0;
                const auto isl_a_it = telemetry_by_island.find(island_a);
                const auto isl_b_it = telemetry_by_island.find(island_b);
                IslandTelemetryLite telem_a{};
                IslandTelemetryLite telem_b{};
                if (isl_a_it != telemetry_by_island.end()) {
                    telem_a.complete = isl_a_it->second.telemetry_complete;
                    telem_a.voltage_v = isl_a_it->second.voltage_v;
                    telem_a.current_a = isl_a_it->second.current_a;
                }
	                if (isl_b_it != telemetry_by_island.end()) {
	                    telem_b.complete = isl_b_it->second.telemetry_complete;
	                    telem_b.voltage_v = isl_b_it->second.voltage_v;
	                    telem_b.current_a = isl_b_it->second.current_a;
	                }
                // Some topologies include "pass-through" slots that have neither modules nor a gun.
                // Those islands have no telemetry source of their own, but we still model an MC contactor
                // for the segment (e.g., a bus tie / bypass). If we require telemetry on both sides, the
                // MC can deadlock open forever, blocking island formation and charging.
                //
                // Treat MC edges adjacent to telemetry-less islands as a single-sided switch: mirror the
                // known-side telemetry so we still enforce current gating and avoid dv checks against a
                // non-existent measurement point.
                auto island_has_telem = [&](int island_id) -> bool {
                    if (island_id == 0) {
                        return false;
                    }
                    const auto slots_it = telemetry_island_slots.find(island_id);
                    if (slots_it == telemetry_island_slots.end()) {
                        return false;
                    }
                    return std::any_of(slots_it->second.begin(), slots_it->second.end(), [&](int sid) {
                        const Slot* s = find_slot(slots_, sid);
                        return s && (!s->modules.empty() || s->gun_id > 0);
                    });
                };
                const bool island_a_has_telem = island_has_telem(island_a);
                const bool island_b_has_telem = island_has_telem(island_b);
                if (island_a != 0 && !island_a_has_telem && island_b_has_telem) {
                    telem_a = telem_b;
                }
                if (!cw || island_b == 0 || (!island_b_has_telem && island_a_has_telem)) {
                    telem_b = telem_a;
                }
	                bool merge_ok = true;
	                if (desired == ContactorState::Closed && prev == ContactorState::Open &&
	                    island_a > 0 && island_b > 0 && island_a != island_b) {
	                    const int gc_a =
	                        island_gc_request_count.count(island_a) ? island_gc_request_count.at(island_a) : 0;
                    const int gc_b =
                        island_gc_request_count.count(island_b) ? island_gc_request_count.at(island_b) : 0;
                    if (gc_a + gc_b > 1) {
                        merge_ok = false;
                    }
                }
                gated = gate_tie_switch(slot.mc_id, desired, prev, telem_a, telem_b,
                                        switch_i_thresh, max_dv_v, stable_ms,
                                        merge_ok, mc_switch_ready_since_, now);

                if (desired == ContactorState::Open && gated == ContactorState::Closed) {
                    mc_open_pending_[slot.id] = true;
                    auto& ts = mc_open_request_time_[slot.id];
                    if (ts.time_since_epoch().count() == 0) {
                        ts = now;
                    } else if ((now - ts) > MC_OPEN_TIMEOUT_MS) {
                        info.local_fault = true;
                        if (info.fault_reason.empty()) {
                            info.fault_reason = "MCOpenTimeout";
                        }
                        EVLOG_warning << "MC open timeout for slot " << slot.id << " (gun " << info.selection.gun_id
                                      << "); locking out connector";
                        // Fail-safe: force island cut OPEN once timeout expires.
                        gated = ContactorState::Open;
                        mc_switch_ready_since_.erase(slot.mc_id);
                    }
                } else {
                    mc_open_pending_.erase(slot.id);
                    mc_open_request_time_.erase(slot.id);
                }
            }

            const auto held = enforce_hold(slot.mc_id, gated, last_mc_state_, mc_command_change_time_,
                                           planner_cfg_.min_mc_hold_ms, true);
            mc_state_cmd_by_slot[slot.id] = held;
        }

    }

    std::set<int> switching_islands;
    if (tie_mode && !telemetry_slot_to_island.empty()) {
        for (const auto& slot : slots_) {
            const auto info_it = slot_info.find(slot.id);
            if (info_it == slot_info.end()) continue;
            ContactorState desired = info_it->second.desired_mc_state;
            auto gated_it = mc_state_cmd_by_slot.find(slot.id);
            ContactorState gated = gated_it != mc_state_cmd_by_slot.end() ? gated_it->second : desired;
            if (desired != gated) {
                const int island_a =
                    telemetry_slot_to_island.count(slot.id) ? telemetry_slot_to_island.at(slot.id) : 0;
                const int island_b =
                    slot.cw_id != 0 && telemetry_slot_to_island.count(slot.cw_id)
                        ? telemetry_slot_to_island.at(slot.cw_id)
                        : 0;
                if (island_a > 0) switching_islands.insert(island_a);
                if (island_b > 0) switching_islands.insert(island_b);
            }
        }
    }

    // Compute runtime islands from the commanded MC states (tie mode) so that module allocation
    // and safety checks track the actual bus topology during switching.
    std::map<int, int> slot_to_island;
    std::map<int, std::vector<int>> island_slots;
    {
        std::map<int, std::size_t> slot_index;
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            slot_index[slots_[i].id] = i;
        }
        std::vector<std::size_t> parent(slots_.size());
        for (std::size_t i = 0; i < parent.size(); ++i) {
            parent[i] = i;
        }
        auto find_root = [&](std::size_t i) -> std::size_t {
            while (parent[i] != i) {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            return i;
        };
        auto unite = [&](std::size_t a, std::size_t b) {
            const auto ra = find_root(a);
            const auto rb = find_root(b);
            if (ra != rb) {
                parent[rb] = ra;
            }
        };
        if (tie_mode) {
            for (std::size_t i = 0; i < slots_.size(); ++i) {
                const auto& s = slots_[i];
                if (s.cw_id == 0) continue;
                const auto cw_it = slot_index.find(s.cw_id);
                if (cw_it == slot_index.end()) continue;
                ContactorState mc_state = ContactorState::Closed;
                const auto mc_it = mc_state_cmd_by_slot.find(s.id);
                if (mc_it != mc_state_cmd_by_slot.end()) {
                    mc_state = mc_it->second;
                } else {
                    const auto info_it = slot_info.find(s.id);
                    if (info_it != slot_info.end()) {
                        mc_state = info_it->second.desired_mc_state;
                    }
                }
                if (mc_state == ContactorState::Closed) {
                    unite(i, cw_it->second);
                }
            }
        }
        std::map<std::size_t, int> root_to_island;
        int next_island = 1;
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            const auto root = find_root(i);
            auto it = root_to_island.find(root);
            if (it == root_to_island.end()) {
                it = root_to_island.emplace(root, next_island++).first;
            }
            const int island_id = it->second;
            slot_to_island[slots_[i].id] = island_id;
            island_slots[island_id].push_back(slots_[i].id);
        }
    }

    std::map<int, int> gun_home_slot;
    std::map<int, int> gun_home_island;
    for (const auto& c : cfg_.connectors) {
        const Slot* slot = find_slot_for_gun(c.id);
        if (!slot) continue;
        gun_home_slot[c.id] = slot->id;
        if (slot_to_island.count(slot->id)) {
            gun_home_island[c.id] = slot_to_island[slot->id];
        }
    }

    // Enforce one-gun-per-island: if topology changes are pending, islands can remain merged.
    // In that case, only allow one gun contactor to be closed in a given island.
    std::map<int, bool> gun_blocked_by_island_conflict;
    if (tie_mode) {
        std::map<int, std::vector<int>> island_guns_requesting_gc;
        for (const auto& c : cfg_.connectors) {
            if (!gun_home_slot.count(c.id) || !gun_home_island.count(c.id)) continue;
            const int home_slot_id = gun_home_slot[c.id];
            const int island_id = gun_home_island[c.id];
            const auto info_it = slot_info.find(home_slot_id);
            if (info_it == slot_info.end()) continue;
            bool wants_gc = info_it->second.desired_gc_state == ContactorState::Closed;
            if (!wants_gc) continue;
            island_guns_requesting_gc[island_id].push_back(c.id);
        }
        for (const auto& kv : island_guns_requesting_gc) {
            const auto& guns_req = kv.second;
            if (guns_req.size() <= 1) {
                continue;
            }
            int winner = 0;
            for (int gid : guns_req) {
                const auto snap_it = snapshots.find(gid);
                if (snap_it != snapshots.end() && snap_it->second.status.relay_closed) {
                    winner = gid;
                    break;
                }
            }
            if (winner == 0) {
                winner = *std::min_element(guns_req.begin(), guns_req.end());
            }
            for (int gid : guns_req) {
                if (gid != winner) {
                    gun_blocked_by_island_conflict[gid] = true;
                    power_constrained_[gid] = true;
                }
            }
        }
    }

    // If a slot's planned gun is not in the same runtime island, its modules are not actually connected.
    // Drop those allocations so we don't over-command current/power.
    for (auto& kv : slot_info) {
        const int slot_id = kv.first;
        auto& info = kv.second;
        const int planned_gun = info.selection.gun_id;
        if (planned_gun <= 0) {
            continue;
        }
        if (gun_blocked_by_island_conflict.count(planned_gun) && gun_blocked_by_island_conflict[planned_gun]) {
            info.modules_final = 0;
            info.mask_final = 0;
            continue;
        }
        if (!gun_home_island.count(planned_gun)) {
            info.modules_final = 0;
            info.mask_final = 0;
            continue;
        }
        const int slot_island = slot_to_island.count(slot_id) ? slot_to_island[slot_id] : -1;
        if (slot_island != gun_home_island[planned_gun]) {
            info.modules_final = 0;
            info.mask_final = 0;
        }
    }

    actual_modules_per_gun.clear();
    island_fault.clear();
    island_fault_reason.clear();
    // Fallback: if the planner dropped modules but the EV is requesting power, keep at least the
    // configured slot modules online so HLC does not stall with zero current.
    for (auto& kv : slot_info) {
        const int slot_id = kv.first;
        auto& info = kv.second;
        const int gid = info.selection.gun_id;
        if (gid <= 0) {
            continue;
        }
        const Slot* slot = find_slot(slots_, slot_id);
        if (!slot || slot->modules.empty()) {
            continue;
        }
        const auto g_it = gun_lookup.find(gid);
        const GunState gstate = (g_it != gun_lookup.end()) ? g_it->second : GunState{};
        const bool ev_requesting =
            (gstate.ev_req_power_kw > 0.1) || info.status.hlc_power_ready ||
            is_hlc_precharge_phase(info.status) || info.status.hlc_stage >= HLC_MIN_POWER_STAGE;
        if (info.modules_final == 0 && ev_requesting && !info.disabled_by_csms && !info.local_fault &&
            info.modules_healthy) {
            // If the slot is not electrically connected to the gun's runtime island yet (e.g. tie closure pending),
            // do not count it as an assigned module. The module command loop will still warm it up (0A) so
            // voltages can align and the tie can close without interrupting the active gun.
            if (gun_home_island.count(gid) && slot_to_island.count(slot_id) &&
                slot_to_island.at(slot_id) != gun_home_island.at(gid)) {
                continue;
            }
            uint8_t mask = 0u;
            int module_count = 0;
            for (const auto& module_id : slot->modules) {
                const auto bit_it = module_slot_index.find(module_id);
                if (bit_it == module_slot_index.end()) {
                    continue;
                }
                const auto health_it = module_health_by_id.find(module_id);
                if (health_it == module_health_by_id.end() || !health_it->second) {
                    continue;
                }
                const int bit = bit_it->second;
                if (bit < 0 || bit >= 8) {
                    continue;
                }
                const uint8_t bit_mask = static_cast<uint8_t>(1u << bit);
                if ((mask & bit_mask) == 0u) {
                    mask |= bit_mask;
                    module_count++;
                }
            }
            if (module_count > 0) {
                info.modules_final = module_count;
                info.mask_final = mask;
            }
        }
    }

    // Precharge lock: use exactly one healthy module from the connector's home slot and keep other slot modules
    // for that gun disabled until power phase starts. This avoids relay/mask jitter during HLC precharge.
    for (const auto& c : cfg_.connectors) {
        const auto st_it = status_by_connector.find(c.id);
        if (st_it == status_by_connector.end()) {
            continue;
        }
        const GunStatus& st = st_it->second;
        const bool hlc_power_phase =
            st.hlc_power_ready || (st.hlc_stage >= HLC_MIN_POWER_STAGE && !st.hlc_charge_complete);
        const bool hlc_precharge_only = is_hlc_precharge_phase(st) && !hlc_power_phase;
        if (!hlc_precharge_only) {
            continue;
        }

        const Slot* home = find_slot_for_gun(c.id);
        if (!home || home->modules.empty()) {
            continue;
        }
        const auto home_it = slot_info.find(home->id);
        if (home_it == slot_info.end()) {
            continue;
        }
        auto& home_info = home_it->second;
        if (home_info.disabled_by_csms || home_info.local_fault || !home_info.modules_healthy) {
            continue;
        }

        uint8_t precharge_mask = 0u;
        for (const auto& module_id : home->modules) {
            const auto bit_it = module_slot_index.find(module_id);
            if (bit_it == module_slot_index.end()) {
                continue;
            }
            const auto health_it = module_health_by_id.find(module_id);
            if (health_it == module_health_by_id.end() || !health_it->second) {
                continue;
            }
            const int bit = bit_it->second;
            if (bit < 0 || bit >= 8) {
                continue;
            }
            precharge_mask = static_cast<uint8_t>(1u << bit);
            break;
        }
        if (precharge_mask == 0u) {
            for (int bit = 0; bit < 8; ++bit) {
                const uint8_t bit_mask = static_cast<uint8_t>(1u << bit);
                if ((home_info.slot_cfg_mask & bit_mask) != 0u) {
                    precharge_mask = bit_mask;
                    break;
                }
            }
        }
        if (precharge_mask == 0u) {
            continue;
        }

        precharge_home_slot_for_gun[c.id] = home->id;
        precharge_home_mask_for_gun[c.id] = precharge_mask;
        uint8_t precharge_relay_mask = 0u;
        const auto relay_slots_it = connector_module_slots_.find(c.id);
        if (relay_slots_it != connector_module_slots_.end()) {
            const auto& relay_slots = relay_slots_it->second;
            if (relay_slots[0] == home->id) {
                precharge_relay_mask |= 0x01u;
            }
            if (relay_slots[1] == home->id) {
                precharge_relay_mask |= 0x02u;
            }
        }
        if (precharge_relay_mask != 0u) {
            precharge_home_relay_mask_for_gun[c.id] = precharge_relay_mask;
        }
        home_info.modules_final = 1;
        home_info.mask_final = precharge_mask;

        for (auto& slot_kv : slot_info) {
            if (slot_kv.first == home->id) {
                continue;
            }
            auto& info = slot_kv.second;
            if (info.selection.gun_id == c.id) {
                info.modules_final = 0;
                info.mask_final = 0u;
            }
        }

        static std::map<int, std::chrono::steady_clock::time_point> last_log;
        auto& ts = last_log[c.id];
        const bool allow_log =
            ts.time_since_epoch().count() == 0 || (now - ts) > std::chrono::seconds(1);
        if (allow_log) {
            EVLOG_info << "Precharge single-module lock connector=" << c.id
                       << " slot=" << home->id << " mask=0x" << std::hex
                       << static_cast<int>(precharge_mask) << std::dec;
            ts = now;
        }
    }

    for (const auto& kv : slot_info) {
        const auto& info = kv.second;
        const int gid = info.selection.gun_id;
        if (gid <= 0) continue;
        actual_modules_per_gun[gid] += info.modules_final;
        if (info.local_fault && !island_fault[gid]) {
            island_fault[gid] = true;
            island_fault_reason[gid] = info.fault_reason.empty() ? "IslandSlotFault" : info.fault_reason;
        }
    }

    std::map<int, GunDispatch> adjusted_dispatch = gun_dispatch;
    for (auto& kv : adjusted_dispatch) {
        auto& disp = kv.second;
        const int gid = kv.first;
        const int actual = actual_modules_per_gun.count(gid) ? actual_modules_per_gun[gid] : 0;
        disp.modules_assigned = actual;
        const auto g_it = gun_lookup.find(gid);
        const double gun_cap = g_it != gun_lookup.end() && g_it->second.gun_power_limit_kw > 0.0
                                   ? g_it->second.gun_power_limit_kw
                                   : disp.p_budget_kw;
        const double p_cap_modules = actual * planner_cfg_.module_power_kw;
        const double p_set = std::min({disp.p_budget_kw, p_cap_modules, gun_cap});
        const double v_target = disp.voltage_set_v > 0.0 ? disp.voltage_set_v : planner_cfg_.default_voltage_v;
        const double v_safe = std::max(planner_cfg_.min_voltage_v_for_div, v_target);
        double i_target = v_safe > 0.0 ? (p_set * 1000.0) / v_safe : 0.0;
        if (g_it != gun_lookup.end() && g_it->second.gun_current_limit_a > 0.0) {
            i_target = std::min(i_target, g_it->second.gun_current_limit_a);
        }
        disp.p_set_kw = p_set;
        disp.voltage_set_v = v_target;
        disp.current_limit_a = i_target;
        if (hold_guns_no_current.count(gid)) {
            // Authorization hold: keep modules enabled and voltage regulated, but do not allow any current/power.
            disp.p_set_kw = 0.0;
            disp.current_limit_a = 0.0;
        }
    }

    auto relay_closed_effective = [&](int connector, const GunStatus& st) {
        if (st.relay_closed) {
            return true;
        }
        if (cfg_.plc_relay_feedback) {
            return false;
        }
        const Slot* home = find_slot_for_gun(connector);
        if (!home) {
            return false;
        }
        const auto it = last_gc_state_.find(home->gc_id);
        return it != last_gc_state_.end() && it->second == ContactorState::Closed;
    };

    // CCS/DC precharge: enforce <=2A (configurable) and stage contactor closure at ~0V.
    // Before GC closes, arm modules at 0V + low current clamp; after closure, ramp toward EV target voltage.
    const double precharge_i_max = (cfg_.precharge_max_current_a > 0.0) ? cfg_.precharge_max_current_a : 2.0;
    if (precharge_i_max > 0.0) {
        const double precharge_arm_v_max =
            std::max(2.0, std::min(20.0, (cfg_.precharge_voltage_tolerance_v > 0.0)
                                             ? cfg_.precharge_voltage_tolerance_v
                                             : 20.0));
        for (auto& kv : adjusted_dispatch) {
            auto& disp = kv.second;
            const int gid = kv.first;
            const auto st_it = status_by_connector.find(gid);
            if (st_it == status_by_connector.end()) {
                continue;
            }
            const GunStatus& st = st_it->second;
            const bool hlc_power_phase =
                st.hlc_power_ready || (st.hlc_stage >= HLC_MIN_POWER_STAGE && !st.hlc_charge_complete);
            const bool hlc_precharge_only = is_hlc_precharge_phase(st) && !hlc_power_phase;
            if (!hlc_precharge_only) {
                precharge_ramp_cmd_voltage_v_.erase(gid);
                precharge_target_power_kw_.erase(gid);
                precharge_arm_ready_since_.erase(gid);
                precharge_ramp_since_.erase(gid);
                precharge_voltage_stable_since_.erase(gid);
                precharge_transition_since_.erase(gid);
                precharge_overcurrent_since_.erase(gid);
                precharge_overshoot_since_.erase(gid);
                continue;
            }
            const auto g_it = gun_lookup.find(gid);
            double precharge_current_cmd = precharge_i_max;
            if (g_it != gun_lookup.end() && g_it->second.gun_current_limit_a > 0.0) {
                precharge_current_cmd = std::min(precharge_current_cmd, g_it->second.gun_current_limit_a);
            }
            if (hold_guns_no_current.count(gid)) {
                precharge_current_cmd = 0.0;
            }
            disp.current_limit_a = precharge_current_cmd;
            double precharge_target_v = disp.voltage_set_v;
            if (g_it != gun_lookup.end() && g_it->second.ev_req_voltage_v > 0.0) {
                precharge_target_v = g_it->second.ev_req_voltage_v;
            }
            precharge_target_v = std::max(precharge_arm_v_max, precharge_target_v);
            double& precharge_target_p_kw = precharge_target_power_kw_[gid];
            if (precharge_current_cmd <= 0.0) {
                precharge_target_p_kw = 0.0;
            } else if (precharge_target_p_kw <= 0.0) {
                const double precharge_cap_kw = (precharge_target_v * precharge_current_cmd) / 1000.0;
                precharge_target_p_kw = std::max(disp.p_set_kw, precharge_cap_kw);
            }
            const bool gc_closed = relay_closed_effective(gid, st);
            if (!gc_closed) {
                // Arm at ~0V before closing GC.
                precharge_ramp_cmd_voltage_v_[gid] = 0.0;
                disp.voltage_set_v = 0.0;
                disp.p_set_kw = 0.0;
            } else {
                const double target_v = precharge_target_v;
                double& ramp_v = precharge_ramp_cmd_voltage_v_[gid];
                if (ramp_v < 0.0) {
                    ramp_v = 0.0;
                }
                const double ramp_step_v = std::max(10.0, target_v * 0.1);
                if (ramp_v + ramp_step_v < target_v) {
                    ramp_v += ramp_step_v;
                } else {
                    ramp_v = target_v;
                }
                disp.voltage_set_v = ramp_v;
                if (precharge_current_cmd <= 0.0) {
                    disp.p_set_kw = 0.0;
                } else {
                    const double v_safe = std::max(planner_cfg_.min_voltage_v_for_div, disp.voltage_set_v);
                    const double p_max = (v_safe * precharge_current_cmd) / 1000.0;
                    disp.p_set_kw = std::min(precharge_target_p_kw, p_max);
                }
            }
        }
    }
    for (const auto& c : cfg_.connectors) {
        if (!adjusted_dispatch.count(c.id)) {
            precharge_ramp_cmd_voltage_v_.erase(c.id);
            precharge_target_power_kw_.erase(c.id);
            precharge_arm_ready_since_.erase(c.id);
            precharge_ramp_since_.erase(c.id);
            precharge_voltage_stable_since_.erase(c.id);
            precharge_transition_since_.erase(c.id);
            precharge_overcurrent_since_.erase(c.id);
            precharge_overshoot_since_.erase(c.id);
        }
    }

    std::vector<const ConnectorConfig*> connector_order;
    connector_order.reserve(cfg_.connectors.size());
    for (const auto& c : cfg_.connectors) {
        connector_order.push_back(&c);
    }
    auto is_home_slot = [&](const ConnectorConfig* cptr) -> bool {
        const Slot* slot = find_slot_for_gun(cptr->id);
        if (!slot) return false;
        const auto it = slot_info.find(slot->id);
        if (it == slot_info.end()) return false;
        const int gun_for_slot = it->second.selection.gun_id;
        const bool in_island = it->second.selection.in_island && gun_for_slot > 0;
        return in_island && slot->gun_id == gun_for_slot;
    };
    std::stable_partition(connector_order.begin(), connector_order.end(), is_home_slot);

    // Per-gun gating derived from the home slot. Populated in the command loop (home slots are processed first).
    std::map<int, bool> gun_drive_modules;
    std::map<int, bool> gun_allow_energy;

    for (const auto* cptr : connector_order) {
        const auto& c = *cptr;
        const Slot* slot = find_slot_for_gun(c.id);
        if (!slot) continue;
        const auto info_it = slot_info.find(slot->id);
        SlotCommandInfo info{};
        if (info_it != slot_info.end()) {
            info = info_it->second;
        }
        const int gun_for_slot = info.selection.gun_id;
        const bool in_island = info.selection.in_island && gun_for_slot > 0;
        const bool is_home = in_island && slot->gun_id == gun_for_slot;

        const auto disp_it = adjusted_dispatch.find(gun_for_slot);
        GunDispatch dispatch{};
        if (disp_it != adjusted_dispatch.end()) {
            dispatch = disp_it->second;
        } else {
            dispatch.gun_id = gun_for_slot;
            dispatch.voltage_set_v = info.meas_voltage > 0.0 ? info.meas_voltage : planner_cfg_.default_voltage_v;
        }

        const auto snap_it = snapshots.find(c.id);
        const GunStatus status = snap_it != snapshots.end() ? snap_it->second.status : GunStatus{};
        const double meas_i = snap_it != snapshots.end() ? snap_it->second.measured_current_a : 0.0;
        const bool connector_prefers_module_meter = (c.meter_source == "module");
        const GunState g = gun_lookup.count(c.id) ? gun_lookup.at(c.id) : GunState{};
        const bool hold_no_current = hold_guns_no_current.count(c.id) > 0;
        bool local_fault = info.local_fault;
        if (is_home && island_fault.count(gun_for_slot) && island_fault[gun_for_slot] && !local_fault) {
            local_fault = true;
            info.fault_reason = island_fault_reason[gun_for_slot];
        }

        // MC (bus cut / island boundary) sequencing.
        //
        // Split/tie mode precomputes safe switching using module telemetry (see mc_state_cmd_by_slot).
        bool mc_closed_cmd = true;
        bool isolation_ready = true;
        const auto it = mc_state_cmd_by_slot.find(slot->id);
        const ContactorState actual = (it != mc_state_cmd_by_slot.end()) ? it->second : info.desired_mc_state;
        mc_closed_cmd = (actual == ContactorState::Closed);
        if (actual != info.desired_mc_state) {
            isolation_ready = false;
            if (is_home) {
                power_constrained_[c.id] = true;
            }
        }
        if (local_fault || dispatch.modules_assigned == 0 || info.disabled_by_csms) {
            isolation_ready = false;
            mc_closed_cmd = false;
        }
        const int current_island =
            telemetry_slot_to_island.count(slot->id) ? telemetry_slot_to_island.at(slot->id) : 0;
        if (current_island > 0 && switching_islands.count(current_island)) {
            bool block_isolation = true;
            if (is_home && gun_for_slot > 0) {
                const bool island_conflict =
                    gun_blocked_by_island_conflict.count(gun_for_slot) &&
                    gun_blocked_by_island_conflict.at(gun_for_slot);
                const int gc_requests =
                    island_gc_request_count.count(current_island) ? island_gc_request_count.at(current_island) : 0;
                const bool mc_settled = (actual == info.desired_mc_state);
                const bool home_module_available =
                    !slot->modules.empty() && info.modules_healthy && !info.disabled_by_csms && !local_fault;
                // If only one gun is active in the island, allow GC sequencing when either:
                // - this MC is already in its target state, or
                // - the home slot already has a healthy local module that can sustain precharge.
                // This prevents deadlock when non-critical tie boundaries are still settling.
                if (gc_requests <= 1 && !island_conflict && (mc_settled || home_module_available)) {
                    block_isolation = false;
                }
            }
            if (block_isolation) {
                isolation_ready = false;
                if (is_home) {
                    power_constrained_[c.id] = true;
                }
            }
        }

        // GC sequencing:
        // - During precharge, close only after modules are armed at ~0V with low current clamp.
        // - During power delivery, use ΔV/current gating for close transitions.
        // - During stop, avoid opening under load.
        const double gc_open_thresh = planner_cfg_.gc_open_current_a > 0.0 ? planner_cfg_.gc_open_current_a : 0.5;
        const bool hlc_power_phase =
            status.hlc_power_ready || (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
        const bool hlc_precharge_phase = is_hlc_precharge_phase(status);
        const bool hlc_precharge_only = hlc_precharge_phase && !hlc_power_phase;
        const bool cp_known = status.cp_state != 'U';
        const bool cp_power_ready = status.cp_state == 'C' || status.cp_state == 'D';
        const bool hlc_known = status.hlc_stage > 0 || hlc_precharge_phase || status.hlc_power_ready;
        const bool ev_requesting = cp_power_ready;
        constexpr double kEvCurrentRequestThresholdA = 0.1;
        const bool ev_current_req =
            status.target_current_a && status.target_current_a.value() > kEvCurrentRequestThresholdA;
        const bool ev_voltage_req = status.target_voltage_v && status.target_voltage_v.value() > 10.0;
        const bool ev_target_recent =
            status.last_target_update.time_since_epoch().count() != 0 &&
            (now - status.last_target_update) <= telemetry_timeout(cfg_);
        const bool ev_target_active = ev_target_recent && ev_current_req && ev_voltage_req;
        const bool gc_closed_effective = relay_closed_effective(c.id, status);
        const double precharge_i_limit = (cfg_.precharge_max_current_a > 0.0) ? cfg_.precharge_max_current_a : 2.0;
        const double precharge_arm_v_max =
            std::max(2.0, std::min(20.0, (cfg_.precharge_voltage_tolerance_v > 0.0)
                                             ? cfg_.precharge_voltage_tolerance_v
                                             : 20.0));
        const auto precharge_timeout =
            std::chrono::milliseconds(std::max(1000, cfg_.precharge_timeout_ms));
        const auto precharge_transition_timeout =
            std::chrono::milliseconds(std::max(30000, cfg_.precharge_timeout_ms * 20));

        const bool precharge_single_home_slot =
            is_home && precharge_home_slot_for_gun.count(c.id) &&
            precharge_home_slot_for_gun.at(c.id) == slot->id;
        if (hlc_precharge_only && precharge_single_home_slot && !local_fault && !info.disabled_by_csms) {
            // Precharge path should not be blocked by tie/island churn.
            mc_closed_cmd = true;
            isolation_ready = true;
        }

        constexpr double kFlowingCurrentA = 0.8;
        constexpr double kFlowingPowerW = 350.0;
        const double meas_i_abs = std::fabs(meas_i);
        const double present_i_abs =
            status.present_current_a ? std::fabs(status.present_current_a.value()) : 0.0;
        const double present_p_abs = status.present_power_w ? std::fabs(status.present_power_w.value()) : 0.0;
        const bool power_path_active_now =
            meas_i_abs >= kFlowingCurrentA || present_i_abs >= kFlowingCurrentA ||
            present_p_abs >= kFlowingPowerW;
        // If CP/PLC state is temporarily unknown but power is still clearly flowing, keep request active.
        const bool cp_unknown_with_flow = !cp_known && status.relay_closed && power_path_active_now;
        bool power_request_active =
            (ev_requesting && hlc_power_phase) || (!hlc_known && ev_target_active) ||
            (hlc_power_phase && ev_target_active) || cp_unknown_with_flow;
        std::string request_loss_reason;
        if (hlc_power_phase && !hlc_precharge_phase && !ev_requesting) {
            const bool explicit_unplug = cp_known && status.cp_state == 'A' && !status.plugged_in;
            if (explicit_unplug) {
                power_request_lost_since_.erase(c.id);
                power_request_active = false;
                request_loss_reason = "cp_a_unplugged";
            } else {
                auto& lost_since = power_request_lost_since_[c.id];
                if (lost_since.time_since_epoch().count() == 0) {
                    lost_since = now;
                }
                // Keep charging through short CP/telemetry jitter. Only declare request loss if
                // CP/targets/current have all been inactive for a sustained window.
                constexpr auto kPowerRequestLossDebounce = std::chrono::milliseconds(12000);
                constexpr auto kTargetFreshHold = std::chrono::milliseconds(12000);
                const bool target_recent_extended =
                    status.last_target_update.time_since_epoch().count() != 0 &&
                    (now - status.last_target_update) <= kTargetFreshHold;
                if ((now - lost_since) < kPowerRequestLossDebounce || target_recent_extended || power_path_active_now) {
                    power_request_active = true;
                } else {
                    power_request_active = false;
                    if (!cp_known) {
                        request_loss_reason = "cp_unknown_sustained";
                    } else if (status.cp_state == 'B') {
                        request_loss_reason = "cp_b_sustained";
                    } else {
                        request_loss_reason = "ev_not_requesting_sustained";
                    }
                }
            }
        } else {
            power_request_lost_since_.erase(c.id);
        }
        bool power_delivery_allowed = power_request_active;
        if (hlc_power_phase && !hlc_precharge_phase && cp_known && !cp_power_ready) {
            const auto lost_it = power_request_lost_since_.find(c.id);
            const auto cp_not_ready_ms =
                (lost_it != power_request_lost_since_.end() && lost_it->second.time_since_epoch().count() != 0)
                    ? std::chrono::duration_cast<std::chrono::milliseconds>(now - lost_it->second).count()
                    : 0;
            if (cp_not_ready_ms >= CP_NOT_READY_POWER_CUT_DELAY_MS.count()) {
                power_delivery_allowed = false;
                static std::map<int, std::chrono::steady_clock::time_point> last_cp_cut_log;
                auto& last_log = last_cp_cut_log[c.id];
                if (last_log.time_since_epoch().count() == 0 ||
                    (now - last_log) >= std::chrono::seconds(1)) {
                    EVLOG_warning << "Connector " << c.id
                                  << " CP not power-ready; clamping current to 0"
                                  << " cp=" << status.cp_state
                                  << " hlc_stage=" << static_cast<int>(status.hlc_stage)
                                  << " hold_ms=" << cp_not_ready_ms;
                    last_log = now;
                }
            }
        }
        const auto prev_request_it = last_power_request_active_.find(c.id);
        const bool prev_request_active =
            (prev_request_it != last_power_request_active_.end()) ? prev_request_it->second : power_request_active;
        if (prev_request_active != power_request_active) {
            const auto lost_it = power_request_lost_since_.find(c.id);
            const auto lost_ms = (lost_it != power_request_lost_since_.end() &&
                                  lost_it->second.time_since_epoch().count() != 0)
                                     ? std::chrono::duration_cast<std::chrono::milliseconds>(now - lost_it->second).count()
                                     : 0;
            EVLOG_info << "Connector " << c.id << " EV power request "
                       << (power_request_active ? "restored" : "lost")
                       << " cp=" << status.cp_state
                       << " hlc_stage=" << static_cast<int>(status.hlc_stage)
                       << " hlc_ready=" << (status.hlc_power_ready ? "1" : "0")
                       << " relay=" << (status.relay_closed ? "1" : "0")
                       << " meas_I=" << meas_i
                       << " present_I=" << status.present_current_a.value_or(0.0)
                       << " target_I=" << status.target_current_a.value_or(0.0)
                       << " target_recent=" << (ev_target_recent ? "1" : "0")
                       << " drop_age_ms=" << lost_ms
                       << " reason=" << (request_loss_reason.empty() ? "none" : request_loss_reason)
                       << " session_active=" << (g.ev_session_active ? "1" : "0");
        }
        last_power_request_active_[c.id] = power_request_active;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (power_request_active) {
                last_cp_request_drop_.erase(c.id);
                last_cp_request_drop_reason_.erase(c.id);
            } else if (!request_loss_reason.empty()) {
                last_cp_request_drop_[c.id] = now;
                last_cp_request_drop_reason_[c.id] = request_loss_reason;
            }
        }
        const bool gc_debounce_hold = is_home && hlc_power_phase && !hlc_precharge_phase && !ev_requesting &&
                                      power_delivery_allowed && gc_closed_effective;
        bool gc_close_requested = is_home &&
                                  (info.desired_gc_state == ContactorState::Closed || gc_debounce_hold) &&
                                  dispatch.modules_assigned > 0 && !info.disabled_by_csms && !local_fault;
        if (hlc_power_phase && !hlc_precharge_phase && !power_delivery_allowed) {
            gc_close_requested = false;
        }
        if (hlc_precharge_only && precharge_single_home_slot && !info.disabled_by_csms && !local_fault) {
            gc_close_requested = dispatch.modules_assigned > 0;
        }
        bool gc_closed_cmd = gc_close_requested;
        bool gc_close_blocked = false;

        if (gc_closed_cmd && is_home && !gc_closed_effective) {
            if (hlc_known && !(hlc_power_phase || hlc_precharge_phase) && !ev_current_req) {
                static std::map<std::string, std::chrono::steady_clock::time_point> last_hlc_log;
                auto& last_log = last_hlc_log[slot->gc_id];
                const bool allow_log = (last_log.time_since_epoch().count() == 0) ||
                                       ((now - last_log) >= std::chrono::milliseconds(500));
                if (allow_log) {
                    EVLOG_info << "GC close blocked by HLC stage"
                               << " connector=" << c.id
                               << " gc=" << slot->gc_id
                               << " hlc_stage=" << static_cast<int>(status.hlc_stage)
                               << " precharge=" << (hlc_precharge_phase ? "1" : "0");
                    last_log = now;
                }
                gc_closed_cmd = false;
                gc_close_blocked = true;
                power_constrained_[c.id] = true;
            }
            if (gc_closed_cmd && !isolation_ready) {
                static std::map<std::string, std::chrono::steady_clock::time_point> last_isolation_log;
                auto& last_log = last_isolation_log[slot->gc_id];
                const bool allow_log = (last_log.time_since_epoch().count() == 0) ||
                                       ((now - last_log) >= std::chrono::milliseconds(500));
                if (allow_log) {
                    EVLOG_info << "GC close blocked by isolation_ready=0"
                               << " connector=" << c.id
                               << " gc=" << slot->gc_id
                               << " mc_desired=" << (info.desired_mc_state == ContactorState::Closed ? "C" : "O")
                               << " mc_actual=" << (mc_closed_cmd ? "C" : "O")
                               << " switching_island=" << (switching_islands.count(current_island) ? "1" : "0")
                               << " island=" << current_island;
                    last_log = now;
                }
                gc_closed_cmd = false;
                gc_close_blocked = true;
                power_constrained_[c.id] = true;
            }
            if (gc_closed_cmd) {
                bool close_ready = false;
                bool telem_complete = false;
                double island_v = 0.0;
                double island_i = 0.0;
                double v_target = g.ev_req_voltage_v > 0.0 ? g.ev_req_voltage_v : info.meas_voltage;
                double dv = 0.0;
                double close_v = info.meas_voltage;
                const double close_i = meas_i;
                bool close_v_from_module = false;

                if (!gc_closed_effective && (hlc_precharge_phase || hlc_power_phase)) {
                    const bool prefer_module_meter = (c.meter_source == "module");
                    if (prefer_module_meter && info.module_telem_valid && info.module_voltage_v >= 0.0) {
                        close_v = info.module_voltage_v;
                        close_v_from_module = true;
                    }
                }

                if (hlc_precharge_only) {
                    const bool arm_voltage_ok = std::fabs(close_v) <= precharge_arm_v_max;
                    const bool arm_current_ok = std::fabs(close_i) < switch_i_thresh;
                    const bool arm_command_ok =
                        dispatch.voltage_set_v <= (precharge_arm_v_max + 1.0) &&
                        dispatch.current_limit_a <= (precharge_i_limit + 0.25);
                    const bool arm_ok = arm_voltage_ok && arm_current_ok && arm_command_ok;
                    if (arm_ok) {
                        auto& since = precharge_arm_ready_since_[c.id];
                        if (since.time_since_epoch().count() == 0) {
                            since = now;
                        }
                        if (stable_ms.count() == 0) {
                            close_ready = true;
                        } else {
                            close_ready = (now - since) >= stable_ms;
                        }
                    } else {
                        precharge_arm_ready_since_.erase(c.id);
                    }
                } else {
                    bool safe_close = false;
                    if (tie_mode && current_island > 0) {
                        const auto telem_it = telemetry_by_island.find(current_island);
                        if (telem_it != telemetry_by_island.end() && telem_it->second.telemetry_complete) {
                            telem_complete = true;
                            island_v = telem_it->second.voltage_v;
                            island_i = telem_it->second.current_a;
                            if (v_target > 0.0) {
                                dv = std::fabs(island_v - v_target);
                                if (dv <= gc_close_max_dv && std::fabs(island_i) < switch_i_thresh) {
                                    safe_close = true;
                                }
                            }
                        }
                    }
                    const bool have_target = v_target > 0.0;
                    const bool have_close_v = close_v > 0.0;
                    const double dv_fallback = (have_target && have_close_v) ? std::fabs(close_v - v_target) : 0.0;
                    const bool fallback_allowed = hlc_power_phase || (hlc_precharge_phase && close_v_from_module);
                    const bool fallback_ready =
                        !safe_close && fallback_allowed && have_target && have_close_v &&
                        dv_fallback <= gc_close_max_dv && std::fabs(close_i) < switch_i_thresh;

                    close_ready = safe_close || fallback_ready;
                    if (close_ready) {
                        if (stable_ms.count() == 0) {
                            gc_switch_ready_since_.erase(slot->gc_id);
                        } else {
                            auto& since = gc_switch_ready_since_[slot->gc_id];
                            if (since.time_since_epoch().count() == 0) {
                                since = now;
                            }
                            if ((now - since) >= stable_ms) {
                                gc_switch_ready_since_.erase(slot->gc_id);
                            } else {
                                close_ready = false;
                            }
                        }
                    } else {
                        gc_switch_ready_since_.erase(slot->gc_id);
                    }
                }

                if (!close_ready) {
                    gc_closed_cmd = false;
                    gc_close_blocked = true;
                    if (is_home) {
                        power_constrained_[c.id] = true;
                    }
                    const auto ready_it = gc_switch_ready_since_.find(slot->gc_id);
                    const double ready_ms =
                        (ready_it != gc_switch_ready_since_.end() && ready_it->second.time_since_epoch().count() != 0)
                            ? static_cast<double>(
                                  std::chrono::duration_cast<std::chrono::milliseconds>(now - ready_it->second).count())
                            : 0.0;
                    static std::map<std::string, std::chrono::steady_clock::time_point> last_gc_block_log;
                    auto& last_log = last_gc_block_log[slot->gc_id];
                    const bool allow_log = (last_log.time_since_epoch().count() == 0) ||
                                           ((now - last_log) >= std::chrono::milliseconds(500));
                    if (allow_log) {
                        EVLOG_info << "GC close gated"
                                   << " connector=" << c.id
                                   << " gc=" << slot->gc_id
                                   << " desired=" << (info.desired_gc_state == ContactorState::Closed ? "C" : "O")
                                   << " precharge_only=" << (hlc_precharge_only ? "1" : "0")
                                   << " isolation_ready=" << (isolation_ready ? "1" : "0")
                                   << " tie_mode=" << (tie_mode ? "1" : "0")
                                   << " island=" << current_island
                                   << " telem_complete=" << (telem_complete ? "1" : "0")
                                   << " V_island=" << island_v
                                   << " I_island=" << island_i
                                   << " V_meas=" << close_v
                                   << " V_meas_src=" << (close_v_from_module ? "module" : "present")
                                   << " V_target=" << v_target
                                   << " dv=" << dv
                                   << " dv_max=" << gc_close_max_dv
                                   << " i_max=" << switch_i_thresh
                                   << " stable_ms=" << stable_ms.count()
                                   << " ready_ms=" << ready_ms;
                        last_log = now;
                    }
                }
            }
        } else {
            if (!hlc_precharge_only) {
                precharge_arm_ready_since_.erase(c.id);
            }
            gc_switch_ready_since_.erase(slot->gc_id);
        }
        if (gc_close_requested && is_home && !gc_closed_effective && !local_fault && !info.disabled_by_csms &&
            (power_request_active || hlc_precharge_only)) {
            if (!gc_closed_cmd || gc_close_blocked) {
                auto& ts = gc_close_request_time_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else {
                    const auto timeout_ms = hlc_precharge_only
                                                ? precharge_timeout
                                                : std::chrono::milliseconds(
                                                      std::max(2000, std::min(cfg_.precharge_timeout_ms, 5000)));
                    if ((now - ts) > timeout_ms) {
                        local_fault = true;
                        const char* reason = hlc_precharge_only ? "PrechargeCloseTimeout" : "GCCloseTimeout";
                        if (info.fault_reason.empty()) {
                            info.fault_reason = reason;
                        }
                        EVLOG_warning << "GC close timeout for connector " << c.id
                                      << " reason=" << reason
                                      << " (hlc_stage=" << static_cast<int>(status.hlc_stage)
                                      << " V_meas=" << info.meas_voltage
                                      << " V_target=" << g.ev_req_voltage_v << ")";
                    }
                }
            } else {
                gc_close_request_time_.erase(c.id);
            }
        } else {
            gc_close_request_time_.erase(c.id);
        }
        bool current_valid = false;
        if (snap_it != snapshots.end()) {
            current_valid = tie_mode && status.relay_closed ? snap_it->second.island_telem_complete
                                                            : snap_it->second.module_current_valid;
        }
        const bool present_fresh = status.last_telemetry.time_since_epoch().count() != 0 &&
                                   (now - status.last_telemetry) <= telemetry_timeout(cfg_);
        if (!current_valid && present_fresh) {
            // Module-meter connectors must rely on fresh module current telemetry for
            // current-dependent faulting; avoid arming on fallback stale present current.
            if (!connector_prefers_module_meter) {
                current_valid = status.present_current_a.has_value();
            }
        }
        if (!local_fault && is_home && hlc_precharge_only && gc_close_requested &&
            (gc_closed_cmd || gc_closed_effective)) {
            auto& ramp_since = precharge_ramp_since_[c.id];
            if (ramp_since.time_since_epoch().count() == 0) {
                ramp_since = now;
            }
            const double target_v = status.target_voltage_v.value_or(dispatch.voltage_set_v);
            const bool have_target_v = target_v > 0.0;
            const double dv = have_target_v ? std::fabs(info.meas_voltage - target_v) : 0.0;
            const bool at_precharge_target = have_target_v && dv <= gc_close_max_dv;
            if (at_precharge_target) {
                auto& stable_since = precharge_voltage_stable_since_[c.id];
                if (stable_since.time_since_epoch().count() == 0) {
                    stable_since = now;
                }
                auto& transition_since = precharge_transition_since_[c.id];
                if (transition_since.time_since_epoch().count() == 0 &&
                    (now - stable_since) >= PRECHARGE_STABLE_HOLD_MS) {
                    transition_since = now;
                }
                if (transition_since.time_since_epoch().count() != 0 &&
                    (now - transition_since) > precharge_transition_timeout) {
                    local_fault = true;
                    if (info.fault_reason.empty()) {
                        info.fault_reason = "PrechargeTransitionTimeout";
                    }
                }
            } else {
                precharge_voltage_stable_since_.erase(c.id);
                precharge_transition_since_.erase(c.id);
            }

            if (!local_fault && have_target_v && info.meas_voltage > (target_v + gc_close_max_dv)) {
                auto& ts = precharge_overshoot_since_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else if ((now - ts) >= PRECHARGE_OVERSHOOT_HOLD_MS) {
                    local_fault = true;
                    if (info.fault_reason.empty()) {
                        info.fault_reason = "PrechargeVoltageOvershoot";
                    }
                }
            } else {
                precharge_overshoot_since_.erase(c.id);
            }

            if (!local_fault && std::fabs(meas_i) > (precharge_i_limit + 0.5)) {
                auto& ts = precharge_overcurrent_since_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else if ((now - ts) >= PRECHARGE_OVERCURRENT_HOLD_MS) {
                    local_fault = true;
                    if (info.fault_reason.empty()) {
                        info.fault_reason = "PrechargeOverCurrent";
                    }
                }
            } else {
                precharge_overcurrent_since_.erase(c.id);
            }

            if (!local_fault && have_target_v && !at_precharge_target &&
                (now - ramp_since) > precharge_timeout) {
                local_fault = true;
                if (info.fault_reason.empty()) {
                    info.fault_reason = "PrechargeTimeout";
                }
            }
        } else {
            precharge_ramp_since_.erase(c.id);
            precharge_voltage_stable_since_.erase(c.id);
            precharge_transition_since_.erase(c.id);
            precharge_overcurrent_since_.erase(c.id);
            precharge_overshoot_since_.erase(c.id);
            if (!hlc_precharge_only) {
                precharge_arm_ready_since_.erase(c.id);
            }
        }

        const double stall_current_req_thresh = 1.0;
        const bool stall_current_requested =
            status.target_current_a && status.target_current_a.value() >= stall_current_req_thresh;
        const bool stall_cp_power_ready = cp_power_ready;
        const bool evse_current_offered = dispatch.current_limit_a >= stall_current_req_thresh;
        const double stall_no_current_thresh = std::max(gc_open_thresh, 1.0);
        const bool stall_target_recent =
            status.last_target_update.time_since_epoch().count() != 0 &&
            (now - status.last_target_update) <= telemetry_timeout(cfg_);
        // Cross-signal validation: if module telemetry is available and still reporting non-trivial current,
        // do not arm a delivery-stall fault from a single path alone.
        const bool stall_module_low_confirmed =
            !info.module_current_valid || std::fabs(info.module_current_a) < stall_no_current_thresh;
        if (!local_fault && !hold_no_current && gc_close_requested && power_delivery_allowed && stall_cp_power_ready &&
            stall_current_requested && stall_target_recent && stall_module_low_confirmed && evse_current_offered &&
            current_valid && (gc_closed_cmd || gc_closed_effective)) {
            if (std::fabs(meas_i) < stall_no_current_thresh) {
                auto& ts = power_delivery_stall_since_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else {
                    const auto configured_timeout =
                        std::chrono::milliseconds(std::max(2000, cfg_.precharge_timeout_ms));
                    const auto timeout_ms = std::max(configured_timeout, POWER_DELIVERY_STALL_TIMEOUT_MS);
                    if ((now - ts) > timeout_ms) {
                        constexpr auto kStallRecoveryGrace = std::chrono::seconds(20);
                        const auto target_age_ms = status.last_target_update.time_since_epoch().count() != 0
                                                       ? std::chrono::duration_cast<std::chrono::milliseconds>(
                                                             now - status.last_target_update)
                                                             .count()
                                                       : -1;
                        const auto stall_age = now - ts;
                        static std::map<int, std::chrono::steady_clock::time_point> last_stall_log;
                        auto& last_log = last_stall_log[c.id];
                        if (last_log.time_since_epoch().count() == 0 ||
                            (now - last_log) >= std::chrono::seconds(2)) {
                            EVLOG_warning << "Power delivery stalled for connector " << c.id
                                          << " (I_meas=" << meas_i
                                          << "A I_offered=" << dispatch.current_limit_a
                                          << "A target_I=" << status.target_current_a.value_or(0.0)
                                          << "A module_I=" << info.module_current_a
                                          << "A module_telem_valid=" << (info.module_telem_valid ? "1" : "0")
                                          << " module_current_valid=" << (info.module_current_valid ? "1" : "0")
                                          << " target_age_ms=" << target_age_ms
                                          << " I_thresh=" << stall_no_current_thresh
                                          << "A stall_ms="
                                          << std::chrono::duration_cast<std::chrono::milliseconds>(stall_age).count()
                                          << ") -- waiting for telemetry/module recovery";
                            last_log = now;
                        }
                        if (stall_age >= (timeout_ms + kStallRecoveryGrace)) {
                            local_fault = true;
                            if (info.fault_reason.empty()) {
                                info.fault_reason = "PowerDeliveryStalled";
                            }
                            EVLOG_error << "Power delivery stall persisted beyond recovery grace on connector " << c.id
                                        << " stall_ms="
                                        << std::chrono::duration_cast<std::chrono::milliseconds>(stall_age).count()
                                        << " timeout_ms="
                                        << std::chrono::duration_cast<std::chrono::milliseconds>(timeout_ms).count();
                        }
                    }
                }
            } else {
                power_delivery_stall_since_.erase(c.id);
            }
        } else {
            power_delivery_stall_since_.erase(c.id);
        }
        if (!gc_closed_cmd && !local_fault && is_home) {
            bool current_valid = true;
            if (tie_mode && gc_closed_effective) {
                current_valid = snap_it != snapshots.end() && snap_it->second.island_telem_complete;
            }
            const bool safe_to_open = !gc_closed_effective || (current_valid && std::fabs(meas_i) < gc_open_thresh);
            if (!safe_to_open) {
                gc_closed_cmd = true;
                gc_open_pending_[c.id] = true;
                auto& ts = gc_open_request_time_[c.id];
                auto& timeout_since = gc_open_timeout_exceeded_since_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                    timeout_since = std::chrono::steady_clock::time_point{};
                } else {
                    // Allow extra settling time proportional to remaining current above the
                    // open threshold. This avoids false GCOpenTimeout trips on legitimate
                    // current decay tails while still faulting persistent non-decay.
                    const double i_abs = std::fabs(meas_i);
                    const double above_thresh_a = std::max(0.0, i_abs - gc_open_thresh);
                    const int dynamic_extra_ms = static_cast<int>(std::clamp(above_thresh_a * 40.0, 0.0, 4000.0));
                    const auto open_timeout = GC_OPEN_TIMEOUT_MS + std::chrono::milliseconds(dynamic_extra_ms);
                    if ((now - ts) > open_timeout) {
                        if (timeout_since.time_since_epoch().count() == 0) {
                            timeout_since = now;
                        } else if ((now - timeout_since) >= GC_OPEN_FAULT_CONFIRM_MS) {
                            local_fault = true;
                            info.fault_reason = info.fault_reason.empty() ? "GCOpenTimeout" : info.fault_reason;
                            EVLOG_warning << "GC open timeout for connector " << c.id
                                          << " while waiting for current to drop"
                                          << " (I_meas=" << meas_i
                                          << "A I_thresh=" << gc_open_thresh
                                          << "A timeout_ms="
                                          << std::chrono::duration_cast<std::chrono::milliseconds>(open_timeout).count()
                                          << " confirm_ms=" << GC_OPEN_FAULT_CONFIRM_MS.count()
                                          << ")";
                        }
                    } else {
                        timeout_since = std::chrono::steady_clock::time_point{};
                    }
                }
            } else {
                gc_open_pending_.erase(c.id);
                gc_open_request_time_.erase(c.id);
                gc_open_timeout_exceeded_since_.erase(c.id);
            }
        } else if (!is_home) {
            gc_open_pending_.erase(c.id);
            gc_open_request_time_.erase(c.id);
            gc_open_timeout_exceeded_since_.erase(c.id);
        } else {
            gc_open_pending_.erase(c.id);
            gc_open_request_time_.erase(c.id);
            gc_open_timeout_exceeded_since_.erase(c.id);
        }
        if (info.disabled_by_csms || local_fault) {
            gc_closed_cmd = false;
        }

        const auto enforced_mc = enforce_hold(slot->mc_id,
                                              mc_closed_cmd ? ContactorState::Closed : ContactorState::Open,
                                              last_mc_state_, mc_command_change_time_,
                                              planner_cfg_.min_mc_hold_ms, true);
        mc_closed_cmd = (enforced_mc == ContactorState::Closed);
        const auto enforced_gc = enforce_hold(slot->gc_id,
                                              gc_closed_cmd ? ContactorState::Closed : ContactorState::Open,
                                              last_gc_state_, gc_command_change_time_,
                                              planner_cfg_.min_gc_hold_ms, true);
        gc_closed_cmd = (enforced_gc == ContactorState::Closed);

        // Keep the per-slot MC command map in sync with the final (post-gating/post-hold) decision used below.
        // This avoids stale relay-mask decisions when tie safety logic changes MC state later in the loop.
        mc_state_cmd_by_slot[slot->id] = mc_closed_cmd ? ContactorState::Closed : ContactorState::Open;

        const bool module_health_ok = info.modules_healthy;
        const bool island_modules_allowed = in_island && !local_fault && !info.disabled_by_csms &&
                                            isolation_ready && dispatch.modules_assigned > 0 && module_health_ok;
        const bool slot_modules_allowed = island_modules_allowed && info.modules_final > 0;
        const int slot_module_cmd = slot_modules_allowed ? info.modules_final : 0;
        const int gc_module_count = is_home ? dispatch.modules_assigned : slot_module_cmd;
        if (is_home && gun_for_slot > 0) {
            gun_drive_modules[gun_for_slot] = island_modules_allowed;
            // Allow energy whenever modules are assigned and the gun contactor is commanded closed.
            // GC close itself is gated by ΔV/current stability (open-loop safety) before reaching this point.
            // When we're holding GC closed only to wait for current to decay (open guard / min-hold),
            // do not continue delivering current: force current to 0A until the contactor can open safely.
            gun_allow_energy[gun_for_slot] = island_modules_allowed && gc_closed_cmd && gc_close_requested;
        }
        const bool allow_energy_for_gun =
            gun_for_slot > 0 && gun_allow_energy.count(gun_for_slot) ? gun_allow_energy[gun_for_slot] : false;
        const bool allow_energy = slot_modules_allowed && allow_energy_for_gun;
        const bool allow_energy_home = is_home && allow_energy_for_gun;

        uint8_t relay_mask_cmd = 0u;
        bool module_slots_valid = false;
        const auto module_slots_it = connector_module_slots_.find(c.id);
        if (module_slots_it != connector_module_slots_.end()) {
            const auto& module_slots = module_slots_it->second;
            module_slots_valid = (module_slots[0] > 0 || module_slots[1] > 0);
            auto slot_closed = [&](int slot_id) -> bool {
                if (slot_id <= 0) {
                    return false;
                }
                if (slot_id == slot->id) {
                    return mc_closed_cmd;
                }
                const auto mc_it = mc_state_cmd_by_slot.find(slot_id);
                if (mc_it != mc_state_cmd_by_slot.end()) {
                    return mc_it->second == ContactorState::Closed;
                }
                const auto info2_it = slot_info.find(slot_id);
                return info2_it != slot_info.end() && info2_it->second.desired_mc_state == ContactorState::Closed;
            };
            if (slot_closed(module_slots[0])) {
                relay_mask_cmd |= 0x01u;
            }
            if (slot_closed(module_slots[1])) {
                relay_mask_cmd |= 0x02u;
            }
        }
        if (hlc_precharge_only && precharge_single_home_slot) {
            const auto precharge_relay_it = precharge_home_relay_mask_for_gun.find(c.id);
            if (precharge_relay_it != precharge_home_relay_mask_for_gun.end() &&
                precharge_relay_it->second != 0u) {
                relay_mask_cmd = precharge_relay_it->second;
            }
        }
        const bool connector_fault = !info.gun_state.safety_ok;
        if (info.disabled_by_csms || connector_fault) {
            relay_mask_cmd = 0u;
        }
        if (gc_module_count <= 0) {
            relay_mask_cmd = 0u;
        }

        PowerCommand cmd;
        cmd.connector = c.id;
        cmd.module_count = gc_module_count;
        cmd.module_mask = relay_mask_cmd;
        cmd.gc_closed = gc_closed_cmd;
        cmd.mc_closed = (!info.disabled_by_csms && !local_fault) ? mc_closed_cmd : false;
        cmd.voltage_set_v = dispatch.voltage_set_v;
        cmd.current_limit_a = allow_energy_home ? dispatch.current_limit_a : 0.0;
        cmd.power_kw = allow_energy_home ? dispatch.p_set_kw : 0.0;

        if (cmd.module_count > 0 && cmd.module_mask == 0 && !info.disabled_by_csms && !connector_fault && !local_fault) {
            if (!module_slots_valid && mc_closed_cmd) {
                uint8_t fallback_mask = info.slot_cfg_mask;
                if (fallback_mask == 0) {
                    const auto last_it = last_module_mask_cmd_.find(c.id);
                    if (last_it != last_module_mask_cmd_.end()) {
                        fallback_mask = last_it->second;
                    }
                }
                if (fallback_mask == 0) {
                    const int max_bits = std::numeric_limits<uint8_t>::digits;
                    const int capped = std::max(0, std::min(cmd.module_count, max_bits));
                    fallback_mask = capped > 0 ? static_cast<uint8_t>((1u << capped) - 1u) : 0u;
                }
                if (fallback_mask != 0) {
                    static std::map<int, std::chrono::steady_clock::time_point> last_log;
                    auto& last_ts = last_log[c.id];
                    const bool allow_log = last_ts.time_since_epoch().count() == 0 ||
                                           (now - last_ts) > std::chrono::seconds(1);
                    if (allow_log) {
                        EVLOG_warning << "Connector " << c.id
                                      << " module mask missing; using fallback mask 0x"
                                      << std::hex << static_cast<int>(fallback_mask) << std::dec
                                      << " for module_count=" << cmd.module_count;
                        last_ts = now;
                    }
                    cmd.module_mask = fallback_mask;
                }
            }
        }

        if (cmd.module_count <= 0) {
            cmd.module_mask = 0;
            cmd.mc_closed = false;
        }

        if (info.paused && g.ev_session_active && is_home) {
            cmd.current_limit_a = 0.0;
            cmd.power_kw = 0.0;
            if (info.disabled_by_csms) {
                cmd.gc_closed = false;
                cmd.mc_closed = (!local_fault && !info.disabled_by_csms) ? mc_closed_cmd : false;
                cmd.module_count = 0;
                cmd.module_mask = 0;
            } else {
                if (!allow_energy && isolation_ready && !local_fault) {
                    cmd.gc_closed = gc_closed_cmd;
                }
                cmd.mc_closed = (!info.disabled_by_csms && !local_fault) ? mc_closed_cmd : false;
            }
        }

        // Persist dynamic home-slot fault decisions (e.g., GC open timeout) back into `slot_info`
        // so module command generation sees the same final state.
        if (is_home) {
            const auto info_it = slot_info.find(slot->id);
            if (info_it != slot_info.end()) {
                info_it->second.local_fault = local_fault;
                info_it->second.fault_reason = info.fault_reason;
            }
        }

        last_current_limit_a_[c.id] = cmd.current_limit_a;
        if (is_home && gun_for_slot > 0) {
            last_requested_power_kw_[gun_for_slot] = dispatch.p_set_kw;
        }
        last_module_alloc_[c.id] = cmd.module_count;
        last_module_mask_cmd_[c.id] = cmd.module_mask;
        EvseLimits limits{};
        // Always advertise non-zero EVSE capability limits to the PLC (used for CPD/CurrentDemand),
        // even when not yet delivering energy. Avoid publishing 0A/0kW limits during idle/warmup,
        // which can stall ISO15118 progression on many EVs.
        const double cap_v = (c.max_voltage_v > 0.0) ? c.max_voltage_v : cfg_.default_voltage_v;
        if (cap_v > 0.0) {
            limits.max_voltage_v = cap_v;
        }
        if (g.gun_current_limit_a > 0.0) {
            limits.max_current_a = g.gun_current_limit_a;
        }
        if (g.gun_power_limit_kw > 0.0) {
            limits.max_power_kw = g.gun_power_limit_kw;
        }
        if (allow_energy_home) {
            // Advertise EVSE capability, not instantaneous dispatch target. Using cmd.current_limit_a/cmd.power_kw
            // here can trap the EV at precharge-level current after PowerDelivery start.
            const double module_cap_kw =
                (cmd.module_count > 0 && planner_cfg_.module_power_kw > 0.0)
                    ? (static_cast<double>(cmd.module_count) * planner_cfg_.module_power_kw)
                    : 0.0;
            if (module_cap_kw > 0.0) {
                if (limits.max_power_kw) {
                    limits.max_power_kw = std::min(limits.max_power_kw.value(), module_cap_kw);
                } else {
                    limits.max_power_kw = module_cap_kw;
                }
                const double v_for_cap = std::max(
                    planner_cfg_.min_voltage_v_for_div,
                    g.ev_req_voltage_v > 0.0
                        ? g.ev_req_voltage_v
                        : (cmd.voltage_set_v > 0.0 ? cmd.voltage_set_v : cfg_.default_voltage_v));
                if (v_for_cap > 0.0) {
                    const double module_cap_current_a = (module_cap_kw * 1000.0) / v_for_cap;
                    if (module_cap_current_a > 0.0) {
                        if (limits.max_current_a) {
                            limits.max_current_a = std::min(limits.max_current_a.value(), module_cap_current_a);
                        } else {
                            limits.max_current_a = module_cap_current_a;
                        }
                    }
                }
            }
        }
        if (allow_energy_home && limits.max_current_a && limits.max_current_a.value() > 0.0) {
            const auto target_timeout = telemetry_timeout(cfg_);
            const bool target_recent = status.last_target_update.time_since_epoch().count() != 0 &&
                                       (now - status.last_target_update) <= target_timeout;
            double measured_i_abs = 0.0;
            bool measured_i_valid = false;
            if (connector_prefers_module_meter) {
                if (info.module_current_valid) {
                    measured_i_abs = std::fabs(info.module_current_a);
                    measured_i_valid = true;
                }
            } else {
                if (status.present_current_a) {
                    measured_i_abs = std::fabs(status.present_current_a.value());
                    measured_i_valid = true;
                } else if (info.module_current_valid) {
                    measured_i_abs = std::fabs(info.module_current_a);
                    measured_i_valid = true;
                }
            }
            const double target_i = std::max(0.0, status.target_current_a.value_or(cmd.current_limit_a));
            const bool sustained_phase =
                cp_power_ready && power_request_active && status.relay_closed &&
                (status.hlc_power_ready || status.hlc_stage >= HLC_MIN_POWER_STAGE) &&
                !status.hlc_charge_complete;
            const bool underdelivering_now =
                sustained_phase && target_recent && measured_i_valid && target_i >= 2.0 &&
                (measured_i_abs + std::max(0.6, target_i * 0.12)) < target_i;
            auto& last_log = current_underdelivery_log_[c.id];
            if (underdelivering_now) {
                auto& since = current_underdelivery_since_[c.id];
                if (since.time_since_epoch().count() == 0) {
                    since = now;
                } else if ((now - since) >= std::chrono::milliseconds(2500)) {
                    if (last_log.time_since_epoch().count() == 0 ||
                        (now - last_log) >= std::chrono::seconds(1)) {
                        EVLOG_warning << "Connector " << c.id
                                      << " module underdelivery observed target_I=" << target_i
                                      << "A measured_I=" << measured_i_abs
                                      << "A offered_I=" << limits.max_current_a.value() << "A";
                        last_log = now;
                    }
                }
            } else {
                current_underdelivery_since_.erase(c.id);
            }
        } else {
            current_underdelivery_since_.erase(c.id);
            current_underdelivery_log_.erase(c.id);
        }
        hardware_->apply_power_command(cmd);
        hardware_->set_evse_limits(c.id, limits);
        last_mc_state_[slot->mc_id] = cmd.mc_closed ? ContactorState::Closed : ContactorState::Open;
        last_gc_state_[slot->gc_id] = cmd.gc_closed ? ContactorState::Closed : ContactorState::Open;

        if (is_home) {
            if (charge_point_) {
                charge_point_->on_max_current_offered(c.id, static_cast<std::int32_t>(std::round(cmd.current_limit_a)));
                charge_point_->on_max_power_offered(
                    c.id, static_cast<std::int32_t>(std::round(cmd.power_kw * 1000.0)));
            }

            const bool constrained = g.ev_session_active && !local_fault && !info.disabled_by_csms &&
                                     (((dispatch.modules_assigned == 0 && dispatch.p_budget_kw > 0.0) ||
                                       (cmd.module_count == 0 && dispatch.modules_assigned > 0) ||
                                       (dispatch.p_set_kw + 1e-3 < dispatch.p_budget_kw)));
            power_constrained_[c.id] = power_constrained_[c.id] || constrained;
        }

        auto local_fault_error_code = [&](const std::string& reason) {
            return errors::local_fault_error_code(reason);
        };

        std::string prev_local_fault;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            const auto it = last_local_fault_reason_.find(c.id);
            if (it != last_local_fault_reason_.end()) {
                prev_local_fault = it->second;
            }
        }

        if (local_fault && is_home && g.ev_session_active) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                stop_origin_hint_[c.id] = !info.fault_reason.empty() ? ("planner_" + info.fault_reason)
                                                                      : "planner_local_fault";
            }
            finish_transaction(c.id, ocpp::v16::Reason::PowerLoss, std::nullopt);
            mark_local_hw_disable(c.id, !info.fault_reason.empty() ? info.fault_reason : "LocalFault");
            hardware_->disable(c.id);
            const std::string reason = !info.fault_reason.empty() ? info.fault_reason : "LocalFault";
            if (!prev_local_fault.empty() && prev_local_fault != reason) {
                sync_ocpp_error(c.id, "local_fault_" + prev_local_fault,
                                local_fault_error_code(prev_local_fault), true, false,
                                "LocalFault", cfg_.vendor, prev_local_fault);
            }
            sync_ocpp_error(c.id, "local_fault_" + reason, local_fault_error_code(reason), true, true,
                            "LocalFault", cfg_.vendor, reason);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                connector_faulted_[c.id] = true;
                connector_state_[c.id] = ConnectorState::Faulted;
                last_local_fault_reason_[c.id] = reason;
            }
        } else if (!local_fault && !prev_local_fault.empty()) {
            // If the connector is still locally disabled (e.g., due to a sequencing timeout), keep the local fault
            // latched until the lockout is cleared (typically after unplug). This avoids oscillating error states.
            const bool lockout_active = local_disabled.count(c.id) ? local_disabled.at(c.id) : false;
            if (!lockout_active) {
                sync_ocpp_error(c.id, "local_fault_" + prev_local_fault,
                                local_fault_error_code(prev_local_fault), true, false,
                                "LocalFault", cfg_.vendor, prev_local_fault);
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    last_local_fault_reason_.erase(c.id);
                }
            }
        }

        EVLOG_debug << "Planner dispatch slot " << slot->id << " gun=" << gun_for_slot
                    << " modules=" << cmd.module_count << " mask=0x" << std::hex
                    << static_cast<int>(cmd.module_mask) << std::dec << " I_lim=" << cmd.current_limit_a
                    << "A V_set=" << cmd.voltage_set_v << " GC=" << (cmd.gc_closed ? "C" : "O")
                    << " MC=" << (cmd.mc_closed ? "C" : "O");
    }

    if (module_controller_) {
        for (const auto& slot : slots_) {
            if (slot.modules.empty()) {
                continue;
            }
            const auto info_it = slot_info.find(slot.id);
            if (info_it == slot_info.end()) {
                continue;
            }
            const auto& info = info_it->second;

            const int gun_id = info.selection.gun_id;
            const int owner_id =
                slot_owner_connector_.count(slot.id) ? slot_owner_connector_[slot.id] : slot.gun_id;

            GunDispatch dispatch{};
            const auto disp_it = adjusted_dispatch.find(gun_id);
            if (disp_it != adjusted_dispatch.end()) {
                dispatch = disp_it->second;
            } else {
                dispatch.gun_id = gun_id;
                dispatch.voltage_set_v = info.meas_voltage > 0.0 ? info.meas_voltage : planner_cfg_.default_voltage_v;
                dispatch.modules_assigned = 0;
            }

            bool post_stop_plugged = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                const auto it = post_stop_plugged_.find(owner_id);
                post_stop_plugged = it != post_stop_plugged_.end() && it->second;
            }

            const int warmup_owner_id = (info.selection.gun_id > 0) ? info.selection.gun_id : owner_id;
            GunStatus warmup_status = info.status;
            GunState warmup_state = info.gun_state;
            if (warmup_owner_id != owner_id) {
                const auto warm_snap_it = snapshots.find(warmup_owner_id);
                if (warm_snap_it != snapshots.end()) {
                    warmup_status = warm_snap_it->second.status;
                }
                const auto warm_state_it = gun_lookup.find(warmup_owner_id);
                if (warm_state_it != gun_lookup.end()) {
                    warmup_state = warm_state_it->second;
                }
            }
            bool warmup_post_stop_plugged = post_stop_plugged;
            if (warmup_owner_id != owner_id) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                const auto it = post_stop_plugged_.find(warmup_owner_id);
                warmup_post_stop_plugged = it != post_stop_plugged_.end() && it->second;
            }

            bool local_fault = info.local_fault;
            if (gun_id > 0 && island_fault.count(gun_id) && island_fault.at(gun_id)) {
                local_fault = true;
            }

            const bool in_island = info.selection.in_island && gun_id > 0;
            const bool module_health_ok = info.modules_healthy;
            const bool slot_modules_allowed =
                in_island && info.modules_final > 0 && !local_fault && !info.disabled_by_csms && module_health_ok &&
                dispatch.modules_assigned > 0 && !post_stop_plugged;

            const int slot_module_cmd = slot_modules_allowed ? info.modules_final : 0;
            const uint8_t slot_mask_cmd = slot_modules_allowed ? info.mask_final : 0u;
            const uint8_t slot_cfg_mask = info.slot_cfg_mask;

            const bool drive_modules_for_gun =
                gun_id > 0 && gun_drive_modules.count(gun_id) ? gun_drive_modules.at(gun_id) : false;
            const bool drive_modules = slot_modules_allowed && drive_modules_for_gun;
            const bool allow_energy_for_gun =
                gun_id > 0 && gun_allow_energy.count(gun_id) ? gun_allow_energy.at(gun_id) : false;

            const bool warmup_safe = warmup_status.plugged_in && !warmup_status.hlc_charge_complete &&
                                     !warmup_post_stop_plugged && !info.disabled_by_csms && !local_fault &&
                                     !warmup_status.cp_fault && warmup_state.safety_ok &&
                                     !warmup_status.gc_welded && !warmup_status.mc_welded;
            const bool cp_power_requesting = warmup_status.cp_state == 'C' || warmup_status.cp_state == 'D';
            // Keep warmup active during true precharge (CP can still be B there), but require CP power request for
            // post-precharge phases. This prevents stale HLC stage=power-delivery with CP=B from re-driving modules
            // after EV stop/unplug transitions.
            const bool warmup_phase =
                is_hlc_precharge_phase(warmup_status) ||
                (cp_power_requesting &&
                 (warmup_status.hlc_power_ready ||
                  (warmup_status.hlc_stage >= HLC_MIN_POWER_STAGE && !warmup_status.hlc_charge_complete) ||
                  warmup_status.relay_closed));
            bool warmup = warmup_safe && !drive_modules && warmup_phase;
            uint8_t warmup_mask = warmup ? slot_cfg_mask : 0u;
            const auto precharge_slot_it = precharge_home_slot_for_gun.find(warmup_owner_id);
            if (precharge_slot_it != precharge_home_slot_for_gun.end()) {
                const int precharge_home_slot = precharge_slot_it->second;
                if (precharge_home_slot != slot.id) {
                    warmup = false;
                    warmup_mask = 0u;
                } else {
                    const auto precharge_mask_it = precharge_home_mask_for_gun.find(warmup_owner_id);
                    if (precharge_mask_it != precharge_home_mask_for_gun.end() &&
                        precharge_mask_it->second != 0u) {
                        warmup_mask = precharge_mask_it->second;
                    }
                }
            }
            if (warmup && warmup_mask == 0u) {
                const int fallback_count =
                    dispatch.modules_assigned > 0 ? dispatch.modules_assigned : info.modules_final;
                if (fallback_count > 0) {
                    const int max_bits = std::numeric_limits<uint8_t>::digits;
                    const int capped = std::max(0, std::min(fallback_count, max_bits));
                    warmup_mask = capped > 0 ? static_cast<uint8_t>((1u << capped) - 1u) : 0u;
                }
                if (warmup_mask == 0u) {
                    const auto last_it = last_module_mask_cmd_.find(owner_id);
                    if (last_it != last_module_mask_cmd_.end()) {
                        warmup_mask = last_it->second;
                    }
                }
                if (warmup_mask != 0u) {
                    static std::map<int, std::chrono::steady_clock::time_point> last_log;
                    auto& last_ts = last_log[owner_id];
                    const bool allow_log = last_ts.time_since_epoch().count() == 0 ||
                                           (now - last_ts) > std::chrono::seconds(1);
                    if (allow_log) {
                        EVLOG_warning << "Connector " << owner_id
                                      << " warmup mask missing; using fallback mask 0x"
                                      << std::hex << static_cast<int>(warmup_mask) << std::dec;
                        last_ts = now;
                    }
                }
            }

            ModuleCommandRequest mreq;
            mreq.slot_id = slot.id;
            const double share = (dispatch.modules_assigned > 0)
                                     ? (static_cast<double>(slot_module_cmd) /
                                        static_cast<double>(dispatch.modules_assigned))
                                     : 0.0;
            const double commanded_current_limit_a = allow_energy_for_gun ? dispatch.current_limit_a : 0.0;
            const double commanded_power_kw = allow_energy_for_gun ? dispatch.p_set_kw : 0.0;
            bool enable = drive_modules;
            uint8_t mask = drive_modules ? slot_mask_cmd : 0u;
            double voltage_v = drive_modules ? dispatch.voltage_set_v : 0.0;
            double current_a = drive_modules ? (commanded_current_limit_a * share) : 0.0;
            double power_kw = drive_modules ? (commanded_power_kw * share) : 0.0;
            if (warmup && warmup_mask != 0u) {
                enable = true;
                mask = warmup_mask;
                voltage_v = dispatch.voltage_set_v > 0.0 ? dispatch.voltage_set_v
                                                        : (info.meas_voltage > 0.0 ? info.meas_voltage
                                                                                   : planner_cfg_.default_voltage_v);
                current_a = 0.0;
                power_kw = 0.0;
            }
            mreq.enable = enable;
            mreq.mask = mask;
            mreq.voltage_v = voltage_v;
            mreq.current_a = current_a;
            mreq.power_kw = power_kw;
            module_controller_->apply_command(mreq);
            last_module_command_by_slot_[slot.id] = mreq;
        }
    }
}

bool OcppAdapter::safety_trip_needed(const GunStatus& status) const {
    return !status.safety_ok || status.estop || status.earth_fault;
}

void OcppAdapter::mark_local_hw_disable(std::int32_t connector, const std::string& reason) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    local_hw_disable_[connector] = true;
    if (!reason.empty()) {
        local_hw_disable_reason_[connector] = reason;
    }
}

void OcppAdapter::maybe_reenable_local_hw(std::int32_t connector, const GunStatus& status, bool block_reenable,
                                          bool disabled_by_csms, bool paused) {
    if (global_fault_latched_.load()) {
        return;
    }
    if (block_reenable) {
        return;
    }
    bool should_enable = false;
    std::string reason;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto it = local_hw_disable_.find(connector);
        if (it == local_hw_disable_.end() || !it->second) {
            return;
        }
        if (disabled_by_csms || paused) {
            return;
        }
        const auto rit = local_hw_disable_reason_.find(connector);
        if (rit != local_hw_disable_reason_.end()) {
            reason = rit->second;
        }
        if (local_disable_latches_until_unplug(reason) && status.plugged_in) {
            return;
        }
        should_enable = true;
        it->second = false;
        local_hw_disable_reason_.erase(connector);
    }
    if (should_enable && hardware_) {
        if (!reason.empty()) {
            EVLOG_info << "Re-enabling EVSE hardware on connector " << connector << " after " << reason;
        } else {
            EVLOG_info << "Re-enabling EVSE hardware on connector " << connector;
        }
        hardware_->enable(connector);
    }
}

void OcppAdapter::enter_global_fault(const std::string& reason, ocpp::v16::Reason stop_reason) {
    if (global_fault_latched_.exchange(true)) {
        return;
    }
    global_fault_reason_ = reason;
    EVLOG_error << "Global fault latched: " << reason;
    const auto desc = errors::global_fault_descriptor(reason);
    for (const auto& c : cfg_.connectors) {
        finish_transaction(c.id, stop_reason, std::nullopt);
        hardware_->disable(c.id);
        sync_ocpp_error(c.id, reason, desc.ocpp_code, desc.is_fault, true, desc.info, cfg_.vendor,
                        desc.vendor_code);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            connector_faulted_[c.id] = true;
            connector_state_[c.id] = ConnectorState::Faulted;
        }
    }
}

void OcppAdapter::apply_zero_power_plan() {
    mc_open_pending_.clear();
    mc_open_request_time_.clear();
    mc_switch_ready_since_.clear();
    gc_open_pending_.clear();
    gc_open_request_time_.clear();
    gc_open_timeout_exceeded_since_.clear();
    gc_close_request_time_.clear();
    gc_switch_ready_since_.clear();
    power_delivery_stall_since_.clear();
    precharge_ramp_cmd_voltage_v_.clear();
    precharge_target_power_kw_.clear();
    precharge_arm_ready_since_.clear();
    precharge_ramp_since_.clear();
    precharge_voltage_stable_since_.clear();
    precharge_transition_since_.clear();
    precharge_overcurrent_since_.clear();
    precharge_overshoot_since_.clear();
    for (const auto& c : cfg_.connectors) {
        PowerCommand cmd;
        cmd.connector = c.id;
        cmd.module_count = 0;
        cmd.module_mask = 0;
        cmd.gc_closed = false;
        cmd.mc_closed = false;
        cmd.voltage_set_v = last_voltage_v_[c.id];
        cmd.current_limit_a = 0.0;
        cmd.power_kw = 0.0;
        last_current_limit_a_[c.id] = 0.0;
        last_module_alloc_[c.id] = 0;
        last_module_mask_cmd_[c.id] = 0;
        hardware_->apply_power_command(cmd);
        EvseLimits limits{};
        limits.max_voltage_v = cmd.voltage_set_v;
        limits.max_current_a = 0.0;
        limits.max_power_kw = 0.0;
        hardware_->set_evse_limits(c.id, limits);
        if (charge_point_) {
            charge_point_->on_max_current_offered(c.id, 0);
            charge_point_->on_max_power_offered(c.id, 0);
        }
    }
    if (module_controller_) {
        for (const auto& slot : slots_) {
            if (slot.modules.empty()) {
                continue;
            }
            ModuleCommandRequest mreq;
            mreq.slot_id = slot.id;
            mreq.enable = false;
            mreq.mask = 0;
            mreq.voltage_v = 0.0;
            mreq.current_a = 0.0;
            mreq.power_kw = 0.0;
            module_controller_->apply_command(mreq);
            last_module_command_by_slot_[slot.id] = mreq;
        }
    }
}

ocpp::v16::UnlockStatus OcppAdapter::unlock_connector(std::int32_t connector) {
    if (!hardware_) {
        return ocpp::v16::UnlockStatus::NotSupported;
    }

    if (connector == 0) {
        bool any_unlocked = false;
        bool any_failed = false;
        bool any_supported = false;
        for (const auto& c : cfg_.connectors) {
            const auto st = unlock_connector(c.id);
            if (st == ocpp::v16::UnlockStatus::Unlocked) {
                any_unlocked = true;
                any_supported = true;
            } else if (st == ocpp::v16::UnlockStatus::UnlockFailed) {
                any_failed = true;
                any_supported = true;
            }
        }
        if (any_failed) {
            return ocpp::v16::UnlockStatus::UnlockFailed;
        }
        if (any_unlocked) {
            return ocpp::v16::UnlockStatus::Unlocked;
        }
        return any_supported ? ocpp::v16::UnlockStatus::UnlockFailed
                             : ocpp::v16::UnlockStatus::NotSupported;
    }

    const auto cfg_it =
        std::find_if(cfg_.connectors.begin(), cfg_.connectors.end(),
                     [&](const ConnectorConfig& c) { return c.id == connector; });
    const bool lock_required = (cfg_it != cfg_.connectors.end()) ? cfg_it->require_lock : true;
    if (!lock_required) {
        return hardware_->unlock(connector);
    }

    const GunStatus status = hardware_->get_status(connector);
    const double unlock_v = (cfg_.unlock_voltage_threshold_v > 0.0) ? cfg_.unlock_voltage_threshold_v : 60.0;
    if (status.relay_closed) {
        EVLOG_warning << "Unlock blocked: relay_closed=1 connector=" << connector;
        return ocpp::v16::UnlockStatus::UnlockFailed;
    }
    if (!status.present_voltage_v) {
        EVLOG_warning << "Unlock blocked: missing voltage telemetry connector=" << connector;
        return ocpp::v16::UnlockStatus::UnlockFailed;
    }
    const double v = status.present_voltage_v.value();
    if (v >= unlock_v) {
        EVLOG_warning << "Unlock blocked: V_out too high connector=" << connector
                      << " V=" << v << "V threshold=" << unlock_v << "V";
        return ocpp::v16::UnlockStatus::UnlockFailed;
    }
    return hardware_->unlock(connector);
}

std::string OcppAdapter::make_session_id() const {
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<std::uint64_t> dist;

    std::stringstream ss;
    ss << cfg_.charge_point_id << "-" << std::hex << dist(gen);
    return ss.str();
}

void OcppAdapter::refresh_charging_profile_limits() {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (!charge_point_) {
        return;
    }

    profile_current_limit_a_.clear();
    profile_power_limit_kw_.clear();
    profile_next_refresh_.reset();

    const auto schedules =
        charge_point_->get_all_composite_charging_schedules(3600, ocpp::v16::ChargingRateUnit::A);
    const auto now_utc = ocpp::DateTime().to_time_point();
    const auto now_steady = std::chrono::steady_clock::now();

    for (const auto& kv : schedules) {
        const int connector_id = kv.first;
        const auto& sched = kv.second;
        if (sched.chargingSchedulePeriod.empty()) {
            continue;
        }
        const auto start_tp = sched.startSchedule ? sched.startSchedule->to_time_point() : now_utc;
        const auto elapsed = now_utc - start_tp;
        const bool in_future = elapsed < std::chrono::seconds::zero();

        // Track next refresh boundary for future periods or duration end.
        auto consider_next_refresh = [&](const std::chrono::seconds& delta_from_now) {
            if (delta_from_now <= std::chrono::seconds::zero()) {
                return;
            }
            const auto candidate =
                now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(delta_from_now);
            if (!profile_next_refresh_ || candidate < *profile_next_refresh_) {
                profile_next_refresh_ = candidate;
            }
        };

        if (in_future) {
            consider_next_refresh(std::chrono::duration_cast<std::chrono::seconds>(-elapsed));
            continue;
        }

        const auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
        if (sched.duration && elapsed_s >= *sched.duration) {
            continue;
        }

        const ocpp::v16::ChargingSchedulePeriod* active = nullptr;
        std::optional<int> next_start_s;
        for (const auto& period : sched.chargingSchedulePeriod) {
            if (period.startPeriod <= elapsed_s) {
                if (!active || period.startPeriod > active->startPeriod) {
                    active = &period;
                }
            } else {
                if (!next_start_s || period.startPeriod < *next_start_s) {
                    next_start_s = period.startPeriod;
                }
            }
        }

        if (next_start_s) {
            const auto next_tp = start_tp + std::chrono::seconds(*next_start_s);
            consider_next_refresh(std::chrono::duration_cast<std::chrono::seconds>(next_tp - now_utc));
        }
        if (sched.duration) {
            const auto end_tp = start_tp + std::chrono::seconds(*sched.duration);
            consider_next_refresh(std::chrono::duration_cast<std::chrono::seconds>(end_tp - now_utc));
        }

        if (!active) {
            continue;
        }

        if (sched.chargingRateUnit == ocpp::v16::ChargingRateUnit::A) {
            profile_current_limit_a_[connector_id] = active->limit;
        } else if (sched.chargingRateUnit == ocpp::v16::ChargingRateUnit::W) {
            profile_power_limit_kw_[connector_id] = active->limit / 1000.0;
        }
    }
}

void OcppAdapter::request_status_refresh(const std::string& reason) {
    std::lock_guard<std::mutex> lock(status_refresh_mutex_);
    pending_status_refresh_.store(true);
    pending_status_refresh_reason_ = reason;
}

void OcppAdapter::maybe_refresh_status_notifications(const std::chrono::steady_clock::time_point& now) {
    if (!charge_point_ || !csms_connected_.load() || !boot_accepted_.load()) {
        return;
    }
    std::string reason;
    bool do_refresh = false;
    {
        std::lock_guard<std::mutex> lock(status_refresh_mutex_);
        const bool pending = pending_status_refresh_.load();
        const bool have_last = last_status_refresh_.time_since_epoch().count() != 0;
        const bool min_gap_ok = !have_last || (now - last_status_refresh_) >= STATUS_REFRESH_MIN_GAP;
        if (pending && min_gap_ok) {
            do_refresh = true;
            reason = pending_status_refresh_reason_;
            pending_status_refresh_.store(false);
            pending_status_refresh_reason_.clear();
            last_status_refresh_ = now;
        }
    }
    if (!do_refresh) {
        return;
    }
    EVLOG_info << "Triggering StatusNotification refresh (reason=" << (reason.empty() ? "unknown" : reason) << ")";
    trigger_status_notifications_if_supported(*charge_point_);
}

void OcppAdapter::expire_reservations(const std::chrono::steady_clock::time_point& now) {
    std::vector<int> expired_connectors;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        for (auto it = reservation_id_by_connector_.begin(); it != reservation_id_by_connector_.end();) {
            const int connector = it->first;
            const int reservation_id = it->second;
            auto exp_it = reservation_expiry_.find(reservation_id);
            if (exp_it == reservation_expiry_.end()) {
                ++it;
                continue;
            }
            if (exp_it->second <= now) {
                reserved_connectors_[connector] = false;
                reservation_required_tag_.erase(connector);
                reservation_parent_tag_.erase(connector);
                reservation_lookup_.erase(reservation_id);
                reservation_expiry_.erase(exp_it);
                expired_connectors.push_back(connector);
                it = reservation_id_by_connector_.erase(it);
            } else {
                ++it;
            }
        }
    }
    if (charge_point_) {
        for (const int connector : expired_connectors) {
            EVLOG_info << "Reservation expired on connector " << connector;
            charge_point_->on_reservation_end(connector);
        }
    }
}

void OcppAdapter::process_post_stop_state(std::int32_t connector, const GunStatus& status,
                                          const std::chrono::steady_clock::time_point& now,
                                          bool* post_stop_plugged, bool* pending_session_stop,
                                          std::optional<std::string>* pending_session_stop_id) {
    if (!post_stop_plugged || !pending_session_stop || !pending_session_stop_id) {
        return;
    }
    const bool vehicle_present = infer_vehicle_present(status);

    std::lock_guard<std::mutex> lock(state_mutex_);
    bool hold_expired = false;
    if (!vehicle_present) {
        post_stop_plugged_[connector] = false;
        post_stop_time_[connector] = std::chrono::steady_clock::time_point{};
    }
    if (post_stop_plugged_[connector]) {
        const auto it_time = post_stop_time_.find(connector);
        hold_expired =
            it_time != post_stop_time_.end() && it_time->second.time_since_epoch().count() != 0 &&
            (now - it_time->second) >= POST_STOP_HOLD_MS;
    }

    auto pending_it = pending_session_stop_.find(connector);
    bool pending_expired = false;
    auto pending_since_it = pending_session_stop_since_.find(connector);
    if (pending_since_it != pending_session_stop_since_.end() &&
        pending_since_it->second.time_since_epoch().count() != 0 &&
        (now - pending_since_it->second) >= POST_STOP_MAX_HOLD_MS) {
        pending_expired = true;
    }
    if (pending_it != pending_session_stop_.end() && (!vehicle_present || hold_expired || pending_expired)) {
        *pending_session_stop_id = pending_it->second;
        pending_session_stop_.erase(pending_it);
        pending_session_stop_since_.erase(connector);
        if (pending_expired) {
            EVLOG_warning << "Pending session stop exceeded max hold on connector " << connector
                          << "; releasing session stop";
        }
    }
    *pending_session_stop = pending_session_stop_.count(connector) != 0;

    if (post_stop_plugged_[connector]) {
        const auto it_fault = connector_faulted_.find(connector);
        const bool fault_latched = (it_fault != connector_faulted_.end()) && it_fault->second;
        if (!fault_latched && !(*pending_session_stop) && hold_expired) {
            post_stop_plugged_[connector] = false;
            post_stop_time_[connector] = std::chrono::steady_clock::time_point{};
            EVLOG_info << "Clearing post-stop hold on connector " << connector;
        }
    }

    *post_stop_plugged = post_stop_plugged_[connector];
}

void OcppAdapter::update_connector_state(std::int32_t connector, GunStatus status, bool has_session,
                                         bool tx_started, bool authorized, bool fault_active, bool disabled,
                                         bool post_stop_plugged, bool seamless_retry_active,
                                         bool suppress_available_event) {
    const auto now = std::chrono::steady_clock::now();
    ConnectorState next = ConnectorState::Available;
    if (seamless_retry_active) {
        status.plugged_in = true;
        if (status.cp_state == 'U') {
            status.cp_state = 'B';
        }
    }
    if (!has_session) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        charging_request_lost_since_.erase(connector);
    }
    const bool cp_known = status.cp_state != 'U';
    const bool vehicle_present = infer_vehicle_present(status, has_session);
    const bool finishing_hint = post_stop_plugged || status.hlc_charge_complete;
    const bool hlc_precharge_phase = is_hlc_precharge_phase(status);
    bool paused = false;
    bool constrained = false;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        paused = paused_evse_[connector];
        constrained = power_constrained_[connector];
    }
    const bool meter_fault_active = status.meter_stale && ((has_session && tx_started) || status.relay_closed);
    if (fault_active || meter_fault_active) {
        next = ConnectorState::Faulted;
    } else if (disabled) {
        next = has_session ? ConnectorState::SuspendedEVSE : ConnectorState::Unavailable;
    } else if (has_session) {
        if (!tx_started) {
            next = ConnectorState::Preparing;
        } else if (!vehicle_present || status.hlc_charge_complete) {
            next = ConnectorState::Finishing;
        } else if (paused || constrained) {
            // Avoid false SuspendedEVSE when energy is still flowing (e.g., load-sharing throttling).
            if (status.relay_closed) {
                const bool hlc_not_power_phase = status.hlc_stage > 0 && status.hlc_stage < HLC_MIN_POWER_STAGE;
                // Relay can be closed during precharge/transition. Do not advertise Charging until
                // the HLC power phase is actually active.
                next = (hlc_precharge_phase || hlc_not_power_phase) ? ConnectorState::Preparing
                                                                    : ConnectorState::Charging;
            } else if (hlc_precharge_phase) {
                next = ConnectorState::Preparing;
            } else {
                constexpr double kMinEvRequestCurrentA = 0.5;
                bool ev_requesting = false;
                const bool hlc_req =
                    status.hlc_power_ready ||
                    (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
                if (cp_known) {
                    ev_requesting = (status.cp_state == 'C' || status.cp_state == 'D');
                } else {
                    ev_requesting = hlc_req ||
                        (hlc_req && status.target_current_a &&
                         status.target_current_a.value() > kMinEvRequestCurrentA);
                }
                const bool no_power_stage_yet = status.hlc_stage == 0 && !status.hlc_power_ready;
                next = no_power_stage_yet ? ConnectorState::Preparing
                                          : (ev_requesting ? ConnectorState::SuspendedEVSE
                                                           : ConnectorState::SuspendedEV);
            }
        } else if (!status.relay_closed) {
            constexpr double kMinEvRequestCurrentA = 0.5;
            bool ev_requesting = false;
            const bool hlc_req =
                status.hlc_power_ready ||
                (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
            if (cp_known) {
                ev_requesting = (status.cp_state == 'C' || status.cp_state == 'D');
            } else {
                ev_requesting = hlc_req ||
                    (hlc_req && status.target_current_a &&
                     status.target_current_a.value() > kMinEvRequestCurrentA);
            }
            if (hlc_precharge_phase) {
                next = ConnectorState::Preparing;
            } else {
                const bool no_power_stage_yet = status.hlc_stage == 0 && !status.hlc_power_ready;
                next = no_power_stage_yet ? ConnectorState::Preparing
                                          : (ev_requesting ? ConnectorState::SuspendedEVSE
                                                           : ConnectorState::SuspendedEV);
            }
        } else {
            constexpr double kMinEvRequestCurrentA = 0.5;
            const bool hlc_req =
                status.hlc_power_ready ||
                (status.hlc_stage >= HLC_MIN_POWER_STAGE && !status.hlc_charge_complete);
            bool ev_requesting = false;
            if (cp_known) {
                ev_requesting = (status.cp_state == 'C' || status.cp_state == 'D');
            } else {
                ev_requesting = hlc_req ||
                    (hlc_req && status.target_current_a &&
                     status.target_current_a.value() > kMinEvRequestCurrentA);
            }
            bool request_lost = false;
            if (hlc_req && cp_known && !ev_requesting && !hlc_precharge_phase) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                auto& since = charging_request_lost_since_[connector];
                if (since.time_since_epoch().count() == 0) {
                    since = now;
                }
                // Keep status in Charging through short CP=B/U jitter to avoid unnecessary
                // Charging<->SuspendedEV churn while still reflecting EVReady drops promptly.
                constexpr auto kChargingStateCpDropDebounce = std::chrono::milliseconds(700);
                request_lost = (now - since) >= kChargingStateCpDropDebounce;
            } else {
                std::lock_guard<std::mutex> lock(state_mutex_);
                charging_request_lost_since_.erase(connector);
            }
            const bool hlc_not_power_phase = status.hlc_stage > 0 && status.hlc_stage < HLC_MIN_POWER_STAGE;
            if (hlc_precharge_phase || hlc_not_power_phase) {
                next = ConnectorState::Preparing;
            } else {
                next = request_lost ? ConnectorState::SuspendedEV : ConnectorState::Charging;
            }
        }
    } else if (finishing_hint && vehicle_present) {
        next = ConnectorState::Finishing;
    } else if (vehicle_present) {
        next = ConnectorState::Preparing;
    }

    ConnectorState current;
    uint64_t event_seq = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current = connector_state_[connector];
        if (next == current) {
            return;
        }
        event_seq = ++status_event_seq_[connector];
    }
    EVLOG_info << "Connector " << connector << " state " << connector_state_name(current) << " -> "
               << connector_state_name(next) << " seq=" << event_seq
               << " cp=" << status.cp_state
               << " plugged=" << (status.plugged_in ? "1" : "0")
               << " relay=" << (status.relay_closed ? "1" : "0")
               << " auth=" << (status.authorization_granted ? "1" : "0")
               << " hlc_stage=" << static_cast<int>(status.hlc_stage)
               << " hlc_ready=" << (status.hlc_power_ready ? "1" : "0")
               << " fault=" << (fault_active ? "1" : "0")
               << " meter_stale=" << (status.meter_stale ? "1" : "0");

    if (!charge_point_) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        connector_state_[connector] = next;
        return;
    }

    switch (next) {
    case ConnectorState::Faulted:
        break;
    case ConnectorState::Preparing:
        // libocpp v0.31.x transitions to Preparing on on_session_started(). Older forks exposed an
        // explicit on_usage_initiated() helper; avoid hard dependency on that non-upstream API.
        break;
    case ConnectorState::SuspendedEV:
        charge_point_->on_suspend_charging_ev(connector);
        break;
    case ConnectorState::SuspendedEVSE:
        charge_point_->on_suspend_charging_evse(connector);
        break;
    case ConnectorState::Finishing:
        // libocpp transitions to Finishing when on_transaction_stopped() is called with a reason that
        // requires user action (i.e. not EVDisconnected). Older forks exposed on_user_action_required().
        break;
    case ConnectorState::Charging:
        charge_point_->on_resume_charging(connector);
        break;
    case ConnectorState::Available:
        if (!suppress_available_event) {
            charge_point_->on_enabled(connector);
        }
        break;
    case ConnectorState::Unavailable:
        charge_point_->on_disabled(connector);
        break;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    connector_state_[connector] = next;
}

std::chrono::steady_clock::time_point OcppAdapter::to_steady(std::chrono::system_clock::time_point t_sys) const {
    const auto now_sys = std::chrono::system_clock::now();
    const auto now_steady = std::chrono::steady_clock::now();
    return now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(t_sys - now_sys);
}

std::chrono::system_clock::time_point OcppAdapter::to_system(std::chrono::steady_clock::time_point t_steady) const {
    const auto now_sys = std::chrono::system_clock::now();
    const auto now_steady = std::chrono::steady_clock::now();
    return now_sys + std::chrono::duration_cast<std::chrono::system_clock::duration>(t_steady - now_steady);
}

int OcppAdapter::meter_interval_seconds_for_connector(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    auto it = connector_meter_intervals_.find(connector);
    if (it != connector_meter_intervals_.end() && it->second > 0) {
        return it->second;
    }
    return std::max(1, cfg_.meter_sample_interval_s);
}

std::string OcppAdapter::token_source_to_string(AuthTokenSource src) {
    switch (src) {
    case AuthTokenSource::RFID: return "rfid";
    case AuthTokenSource::Autocharge: return "autocharge";
    case AuthTokenSource::RemoteStart: return "remotestart";
    default: return "unknown";
    }
}

std::string OcppAdapter::auth_state_to_string(AuthorizationState state) {
    switch (state) {
    case AuthorizationState::Unknown: return "unknown";
    case AuthorizationState::Pending: return "pending";
    case AuthorizationState::Granted: return "granted";
    case AuthorizationState::Denied: return "denied";
    default: return "unknown";
    }
}

AuthTokenSource OcppAdapter::token_source_from_string(const std::string& s) {
    if (s == "rfid") return AuthTokenSource::RFID;
    if (s == "autocharge") return AuthTokenSource::Autocharge;
    if (s == "remotestart") return AuthTokenSource::RemoteStart;
    return AuthTokenSource::RFID;
}

void OcppAdapter::set_auth_state(std::int32_t connector, AuthorizationState state) {
    AuthorizationState prev = AuthorizationState::Unknown;
    {
        std::lock_guard<std::mutex> lock(auth_mutex_);
        prev = auth_state_cache_[connector];
        if (prev == state) {
            return;
        }
        auth_state_cache_[connector] = state;
        if (state == AuthorizationState::Denied) {
            auth_denied_since_[connector] = std::chrono::steady_clock::now();
            last_denied_token_.erase(connector);
        } else {
            auth_denied_since_.erase(connector);
            last_denied_token_.erase(connector);
        }
    }
    EVLOG_info << "Auth state connector " << connector << ": " << auth_state_to_string(prev)
               << " -> " << auth_state_to_string(state);
    hardware_->set_authorization_state(connector, state);
}

AuthorizationState OcppAdapter::get_auth_state(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(auth_mutex_);
    const auto it = auth_state_cache_.find(connector);
    if (it != auth_state_cache_.end()) {
        return it->second;
    }
    return AuthorizationState::Unknown;
}

std::optional<std::chrono::steady_clock::time_point> OcppAdapter::get_auth_denied_since(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(auth_mutex_);
    const auto it = auth_denied_since_.find(connector);
    if (it != auth_denied_since_.end()) {
        return it->second;
    }
    return std::nullopt;
}

void OcppAdapter::note_auth_denied(std::int32_t connector, const std::string& id_token) {
    if (id_token.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(auth_mutex_);
    auth_denied_since_[connector] = std::chrono::steady_clock::now();
    last_denied_token_[connector] = id_token;
}

bool OcppAdapter::should_bypass_token_dedup(std::int32_t connector, const AuthToken& token) {
    if (token.source != AuthTokenSource::RFID) {
        return false;
    }
    std::lock_guard<std::mutex> lock(auth_mutex_);
    const auto auth_it = auth_state_cache_.find(connector);
    if (auth_it == auth_state_cache_.end() || auth_it->second != AuthorizationState::Denied) {
        return false;
    }
    const auto denied_it = last_denied_token_.find(connector);
    if (denied_it == last_denied_token_.end()) {
        return false;
    }
    const auto trimmed = clamp_id_token(token.id_token);
    return trimmed == denied_it->second;
}

bool OcppAdapter::token_matches_reservation(std::int32_t connector, const std::string& token,
                                            const std::optional<std::string>& parent_token) {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const auto trimmed = clamp_id_token(token);
    const auto req_it = reservation_required_tag_.find(connector);
    if (req_it == reservation_required_tag_.end()) {
        return true;
    }
    if (trimmed == req_it->second) {
        return true;
    }
    const auto parent_it = reservation_parent_tag_.find(connector);
    if (parent_it != reservation_parent_tag_.end() && parent_it->second) {
        if (trimmed == parent_it->second.value()) {
            return true;
        }
        if (parent_token && *parent_token == parent_it->second.value()) {
            return true;
        }
    }
    return false;
}

ocpp::v16::DataTransferResponse
OcppAdapter::handle_data_transfer_request(const ocpp::v16::DataTransferRequest& request) {
    ocpp::v16::DataTransferResponse resp;
    const std::string vendor = request.vendorId.get();
    const std::string message_id = request.messageId ? request.messageId->get() : "";

    auto known_vendor = [](const std::string& v) {
        static const std::set<std::string> vendors = {"iso15118",
                                                      "PnC",
                                                      "pnc",
                                                      "org.openchargealliance.iso15118",
                                                      "org.openchargealliance.ocpp",
                                                      "org.openchargealliance.iso15118pnc",
                                                      "org.openchargealliance.pnc"};
        return vendors.find(v) != vendors.end();
    };

    if (!known_vendor(vendor)) {
        resp.status = ocpp::v16::DataTransferStatus::UnknownVendorId;
        resp.data = std::string("Unsupported vendor: ") + vendor;
        EVLOG_warning << "DataTransfer request from CSMS rejected: unknown vendorId=" << vendor
                      << " messageId=" << message_id;
        return resp;
    }

    if (message_id == "ping" || message_id == "health" || message_id == "status") {
        resp.status = ocpp::v16::DataTransferStatus::Accepted;
        resp.data = "ok";
        return resp;
    }
    if (message_id == "ClearLocalAuthCache" || message_id == "ClearLocalCache") {
        clear_local_auth_cache();
        resp.status = ocpp::v16::DataTransferStatus::Accepted;
        resp.data = "localAuthCacheCleared";
        return resp;
    }
    if (message_id == "ClearPendingTokens") {
        clear_pending_tokens();
        resp.status = ocpp::v16::DataTransferStatus::Accepted;
        resp.data = "pendingTokensCleared";
        return resp;
    }

    // Known vendor but no specific handler: respond cleanly with UnknownMessageId to avoid silent drop.
    resp.status = ocpp::v16::DataTransferStatus::UnknownMessageId;
    resp.data = std::string("No handler for vendor=") + vendor +
        (message_id.empty() ? "" : (" messageId=" + message_id));
    EVLOG_warning << "DataTransfer request for vendor=" << vendor << " messageId=" << message_id
                  << " is not implemented; responding UnknownMessageId";
    return resp;
}

void OcppAdapter::handle_configuration_key_change(const ocpp::v16::KeyValue& key_value) {
    const std::string key = key_value.key.get();
    const std::string value = key_value.value ? key_value.value->get() : "";
    EVLOG_info << "Configuration key changed by CSMS: " << key << "=" << value;

    auto parse_int = [&](int fallback) {
        try {
            return std::stoi(value);
        } catch (...) {
            return fallback;
        }
    };
    if (key == "MeterValueSampleInterval") {
        const int interval = std::max(1, parse_int(cfg_.meter_sample_interval_s));
        std::lock_guard<std::mutex> lock(plan_mutex_);
        for (const auto& c : cfg_.connectors) {
            connector_meter_intervals_[c.id] = interval;
        }
    } else if (key == "MinimumStatusDuration") {
        cfg_.minimum_status_duration_s = parse_int(cfg_.minimum_status_duration_s);
    } else if (key == "ConnectionTimeOut") {
        hardware_->set_connection_timeout(parse_int(0));
    } else if (key == "AutochargeEnabled") {
        if (key_value.value) {
            const bool enabled = ocpp::conversions::string_to_bool(value);
            set_autocharge_enabled(enabled, "ChangeConfiguration");
        } else {
            EVLOG_warning << "Ignoring AutochargeEnabled change: missing value";
        }
    }
}

void OcppAdapter::persist_pending_tokens() {
    std::map<std::int32_t, std::deque<PendingToken>> snapshot;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        snapshot = pending_tokens_;
    }
    persist_pending_tokens_snapshot(snapshot);
}

void OcppAdapter::persist_pending_tokens_snapshot(
    const std::map<std::int32_t, std::deque<PendingToken>>& snapshot) {
    if (pending_token_store_.empty()) {
        return;
    }
    if (cfg_.auth_wait_timeout_s <= 0) {
        // When authorization timeout is disabled, avoid persisting pending tokens across restarts.
        std::error_code ec;
        std::filesystem::remove(pending_token_store_, ec);
        return;
    }
    try {
        nlohmann::json root;
        root["tokens"] = nlohmann::json::array();
        for (const auto& kv : snapshot) {
            for (const auto& pending : kv.second) {
                nlohmann::json entry;
                entry["connector"] = kv.first;
                entry["idToken"] = pending.token.id_token;
                entry["source"] = token_source_to_string(pending.token.source);
                entry["connectorHint"] = pending.token.connector_hint;
                entry["prevalidated"] = pending.token.prevalidated;
                entry["deferUntilOnline"] = pending.defer_until_online;
                entry["receivedAt"] =
                    std::chrono::duration_cast<std::chrono::seconds>(
                        to_system(pending.token.received_at).time_since_epoch()).count();
                if (pending.expires_at == std::chrono::steady_clock::time_point::max()) {
                    // Sentinel for "never expires" (authorizationSeconds==0).
                    entry["expiresAt"] = 0;
                } else {
                    entry["expiresAt"] =
                        std::chrono::duration_cast<std::chrono::seconds>(
                            to_system(pending.expires_at).time_since_epoch()).count();
                }
                root["tokens"].push_back(entry);
            }
        }

        std::error_code ec;
        std::filesystem::create_directories(pending_token_store_.parent_path(), ec);
        const auto tmp_path = pending_token_store_.string() + ".tmp";
        {
            std::ofstream out(tmp_path, std::ios::trunc);
            if (!out) {
                EVLOG_warning << "Failed to open pending token store for write: " << tmp_path;
                return;
            }
            out << root.dump(2) << "\n";
        }
        std::filesystem::rename(tmp_path, pending_token_store_, ec);
        if (ec) {
            std::error_code rm_ec;
            std::filesystem::remove(pending_token_store_, rm_ec);
            ec.clear();
            std::filesystem::rename(tmp_path, pending_token_store_, ec);
        }
        if (ec) {
            EVLOG_warning << "Failed to persist pending tokens: " << ec.message();
        }
    } catch (const std::exception& e) {
        EVLOG_warning << "Failed to persist pending tokens: " << e.what();
    }
}

void OcppAdapter::load_pending_tokens_from_disk() {
    if (pending_token_store_.empty()) return;
    if (!std::filesystem::exists(pending_token_store_)) return;
    if (cfg_.auth_wait_timeout_s <= 0) {
        std::error_code ec;
        std::filesystem::remove(pending_token_store_, ec);
        return;
    }
    try {
        std::ifstream in(pending_token_store_);
        if (!in) return;
        nlohmann::json root;
        in >> root;
        if (!root.contains("tokens") || !root["tokens"].is_array()) return;
        const auto now_sys = std::chrono::system_clock::now();
        const auto now_steady = std::chrono::steady_clock::now();
        const bool auth_timeout_enabled = cfg_.auth_wait_timeout_s > 0;
        const auto ttl =
            auth_timeout_enabled ? std::chrono::seconds(std::max(1, cfg_.auth_wait_timeout_s)) : std::chrono::seconds(0);
        for (const auto& entry : root["tokens"]) {
            try {
                const int connector = entry.value("connector", 0);
                const std::string id = entry.value("idToken", "");
                if (connector <= 0 || id.empty()) continue;
                const std::string src_str = entry.value("source", "rfid");
                AuthToken token;
                token.id_token = id;
                token.source = token_source_from_string(src_str);
                token.connector_hint = entry.value("connectorHint", 0);
                token.prevalidated = entry.value("prevalidated", false);
                const auto recv_epoch = std::chrono::seconds(entry.value("receivedAt", 0LL));
                const auto exp_epoch = std::chrono::seconds(entry.value("expiresAt", 0LL));
                const auto recv_sys = recv_epoch.count() > 0 ? std::chrono::system_clock::time_point(recv_epoch) : now_sys;
                const auto recv_delta = recv_sys - now_sys;
                token.received_at =
                    now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(recv_delta);
                PendingToken pending;
                pending.token = token;
                pending.defer_until_online = entry.value("deferUntilOnline", false);
                if (exp_epoch.count() > 0) {
                    const auto exp_sys = std::chrono::system_clock::time_point(exp_epoch);
                    if (exp_sys <= now_sys) continue;
                    const auto exp_delta = exp_sys - now_sys;
                    pending.expires_at =
                        now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(exp_delta);
                } else {
                    // If stored expiry is missing (or sentinel), recompute via config.
                    pending.expires_at = auth_timeout_enabled ? (token.received_at + ttl)
                                                              : std::chrono::steady_clock::time_point::max();
                }
                pending_tokens_[connector].push_back(pending);
            } catch (...) {
                continue;
            }
        }
    } catch (const std::exception& e) {
        EVLOG_warning << "Failed to load pending tokens: " << e.what();
    }
}

} // namespace charger
