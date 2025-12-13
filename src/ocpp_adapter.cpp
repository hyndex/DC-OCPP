// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <thread>
#include <cmath>
#include <limits>
#include <type_traits>
#include <cstddef>
#include <nlohmann/json.hpp>

#include <everest/logging.hpp>
#include <ocpp/common/evse_security_impl.hpp>
#include <ocpp/v16/ocpp_enums.hpp>

namespace charger {

namespace fs = std::filesystem;

namespace {

constexpr uint8_t HLC_MIN_POWER_STAGE = 4;
constexpr std::chrono::milliseconds MC_OPEN_TIMEOUT_MS(2000);
constexpr std::chrono::milliseconds GC_OPEN_TIMEOUT_MS(2000);

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

SlotModuleSelection compute_slot_module_selection(const Plan& plan, const Slot& slot) {
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
                sel.mask |= static_cast<uint8_t>(1U << idx);
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
    const bool targets_present =
        status.target_current_a.has_value() || status.target_voltage_v.has_value();
    const bool measured_present =
        status.present_voltage_v.has_value() || status.present_current_a.has_value() ||
        status.present_power_w.has_value();
    bool hlc_ready = status.hlc_power_ready;
    if (!hlc_ready && status.hlc_stage >= HLC_MIN_POWER_STAGE && status.hlc_cable_check_ok &&
        !status.hlc_charge_complete) {
        hlc_ready = true;
    }
    if (!hlc_ready && status.hlc_precharge_active && (targets_present || measured_present)) {
        hlc_ready = true;
    }
    return cp_ready && (hlc_ready || targets_present || measured_present);
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
    pending_token_store_ = cfg_.database_dir / "pending_tokens.json";
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
    }
    load_pending_tokens_from_disk();
    initialize_slots();
}

OcppAdapter::~OcppAdapter() {
    stop();
}

void OcppAdapter::prepare_security_files() const {
    auto touch = [](const fs::path& path) {
        if (!path.empty() && !fs::exists(path)) {
            std::ofstream out(path);
            out << "";
        }
    };

    touch(cfg_.security.csms_ca_bundle);
    touch(cfg_.security.mo_ca_bundle);
    touch(cfg_.security.v2g_ca_bundle);
}

const Slot* OcppAdapter::find_slot_for_gun(int gun_id) const {
    return charger::find_slot_for_gun(slots_, gun_id);
}

void OcppAdapter::initialize_slots() {
    if (slots_initialized_) return;

    if (cfg_.allow_cross_slot_islands && hardware_ && !hardware_->supports_cross_slot_islands()) {
        EVLOG_warning << "Cross-slot islands requested in config but hardware does not support it; disabling.";
        cfg_.allow_cross_slot_islands = false;
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
    planner_cfg_ = pcfg;
    power_manager_ = PowerManager(planner_cfg_);

    std::vector<Slot> slots;
    if (!cfg_.slots.empty()) {
        for (const auto& sm : cfg_.slots) {
            Slot s;
            s.id = sm.id;
            s.gun_id = sm.gun_id;
            s.gc_id = sm.gc_id.empty() ? "GC_" + std::to_string(sm.id) : sm.gc_id;
            s.mc_id = sm.mc_id.empty() ? "MC_" + std::to_string(sm.id) : sm.mc_id;
            s.cw_id = sm.cw_id;
            s.ccw_id = sm.ccw_id;
            for (const auto& m : sm.modules) {
                s.modules.push_back(m.id);
                ModuleState ms;
                ms.id = m.id;
                ms.slot_id = sm.id;
                ms.mn_id = m.mn_id.empty() ? "MN_" + std::to_string(sm.id) + "_0" : m.mn_id;
                module_states_.push_back(ms);
            }
            slots.push_back(s);
        }
    } else {
        for (std::size_t i = 0; i < cfg_.connectors.size(); ++i) {
            const auto& c = cfg_.connectors[i];
            Slot s;
            s.id = c.id;
            s.gun_id = c.id;
            s.gc_id = "GC_" + std::to_string(c.id);
            s.mc_id = "MC_" + std::to_string(c.id);
            s.cw_id = cfg_.connectors[(i + 1) % cfg_.connectors.size()].id;
            s.ccw_id = cfg_.connectors[(i + cfg_.connectors.size() - 1) % cfg_.connectors.size()].id;
            s.modules.push_back("M" + std::to_string(c.id) + "_0");
            s.modules.push_back("M" + std::to_string(c.id) + "_1");
            slots.push_back(s);

            ModuleState m0;
            m0.id = s.modules[0];
            m0.slot_id = s.id;
            m0.mn_id = "MN_" + std::to_string(s.id) + "_0";
            module_states_.push_back(m0);

            ModuleState m1;
            m1.id = s.modules[1];
            m1.slot_id = s.id;
            m1.mn_id = "MN_" + std::to_string(s.id) + "_1";
            module_states_.push_back(m1);
        }
    }

    slots_ = slots;
    power_manager_.set_slots(slots);
    slots_initialized_ = true;
}

bool OcppAdapter::start() {
    prepare_security_files();
    Everest::Logging::init(cfg_.logging_config.string(), "dc-ocpp");

    const auto config_str = load_and_patch_ocpp_config(cfg_);

    ocpp::SecurityConfiguration security_cfg{};
    security_cfg.csms_ca_bundle = cfg_.security.csms_ca_bundle;
    security_cfg.mf_ca_bundle = cfg_.security.mo_ca_bundle;
    security_cfg.mo_ca_bundle = cfg_.security.mo_ca_bundle;
    security_cfg.v2g_ca_bundle = cfg_.security.v2g_ca_bundle;
    security_cfg.csms_leaf_cert_directory = cfg_.security.client_cert_dir;
    security_cfg.csms_leaf_key_directory = cfg_.security.client_key_dir;
    security_cfg.secc_leaf_cert_directory = cfg_.security.secc_cert_dir;
    security_cfg.secc_leaf_key_directory = cfg_.security.secc_key_dir;

    charge_point_ = std::make_unique<ocpp::v16::ChargePoint>(config_str, cfg_.share_path, cfg_.user_config,
                                                             cfg_.database_dir, cfg_.sql_migrations,
                                                             cfg_.message_log_path, nullptr, security_cfg);

    register_callbacks();

    std::map<int, ocpp::v16::ChargePointStatus> connector_status_map;
    connector_status_map.emplace(0, ocpp::v16::ChargePointStatus::Available);
    for (const auto& connector : cfg_.connectors) {
        connector_status_map.emplace(connector.id, ocpp::v16::ChargePointStatus::Available);
    }

    if (!charge_point_->start(connector_status_map, ocpp::v16::BootReasonEnum::PowerUp)) {
        EVLOG_error << "Failed to start charge point";
        return false;
    }

    refresh_charging_profile_limits();

    running_ = true;
    start_metering_threads();
    planner_thread_running_ = true;
    planner_thread_ = std::thread([this]() {
        while (running_ && planner_thread_running_) {
            try {
                apply_power_plan();
            } catch (const std::exception& e) {
                EVLOG_warning << "Planner thread error: " << e.what();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    return true;
}

void OcppAdapter::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    planner_thread_running_ = false;
    if (planner_thread_.joinable()) {
        planner_thread_.join();
    }
    for (auto& thread : meter_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    meter_threads_.clear();
    if (charge_point_) {
        charge_point_->stop();
    }
}

void OcppAdapter::register_callbacks() {
    charge_point_->register_enable_evse_callback([this](std::int32_t connector) {
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            evse_disabled_[connector] = false;
        }
        const bool ok = hardware_->enable(connector);
        if (ok) {
            charge_point_->on_enabled(connector);
        } else {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            evse_disabled_[connector] = true;
        }
        return ok;
    });

    charge_point_->register_disable_evse_callback([this](std::int32_t connector) {
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            evse_disabled_[connector] = true;
        }
        bool had_session = false;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            had_session = sessions_.count(connector) > 0;
        }
        if (had_session) {
            finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
        }
        const bool ok = hardware_->disable(connector);
        if (ok) {
            charge_point_->on_disabled(connector);
        }
        return ok;
    });

    charge_point_->register_pause_charging_callback([this](std::int32_t connector) {
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            paused_evse_[connector] = true;
        }
        charge_point_->on_suspend_charging_evse(connector);
        return true;
    });

    charge_point_->register_resume_charging_callback([this](std::int32_t connector) {
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            paused_evse_[connector] = false;
        }
        charge_point_->on_resume_charging(connector);
        return true;
    });

    charge_point_->register_stop_transaction_callback([this](std::int32_t connector, ocpp::v16::Reason reason) {
        const bool ok = hardware_->stop_transaction(connector, reason);
        if (ok) {
            finish_transaction(connector, reason);
        }
        return ok;
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
            if (token.connector_hint > 0) {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                reserved_connectors_[token.connector_hint] = false;
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

    charge_point_->register_remote_start_acceptance_callback(
        [this](const std::string& id_token, const std::vector<std::int32_t>& referenced) {
            (void)id_token;
            return evaluate_remote_start_acceptance(id_token, referenced);
        });

    charge_point_->register_reserve_now_callback(
        [this](std::int32_t reservation_id, std::int32_t connector, ocpp::DateTime expiryDate,
               ocpp::CiString<20> idTag, std::optional<ocpp::CiString<20>> parent_id) {
            const auto status = hardware_->reserve(reservation_id, connector, expiryDate, idTag.get(),
                                                   parent_id ? std::optional<std::string>(parent_id->get()) : std::nullopt);
            if (status == ocpp::v16::ReservationStatus::Accepted) {
                charge_point_->on_reservation_start(connector);
                std::lock_guard<std::mutex> lock(plan_mutex_);
                reserved_connectors_[connector] = true;
                reservation_lookup_[reservation_id] = connector;
            }
            return status;
        });

    charge_point_->register_cancel_reservation_callback([this](std::int32_t reservation_id) {
        const bool ok = hardware_->cancel_reservation(reservation_id);
        if (ok) {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            auto it = reservation_lookup_.find(reservation_id);
            if (it != reservation_lookup_.end()) {
                reserved_connectors_[it->second] = false;
                reservation_lookup_.erase(it);
            }
        }
        return ok;
    });

    charge_point_->register_unlock_connector_callback(
        [this](std::int32_t connector) { return hardware_->unlock(connector); });

    charge_point_->register_upload_diagnostics_callback(
        [this](const ocpp::v16::GetDiagnosticsRequest& request) {
            charge_point_->on_log_status_notification(-1, "Uploading");
            auto resp = hardware_->upload_diagnostics(request);
            charge_point_->on_log_status_notification(-1, resp.status == ocpp::v16::LogStatusEnumType::Accepted
                                                                ? "Uploaded"
                                                                : "UploadFailed");
            return resp;
        });

    charge_point_->register_upload_logs_callback(
        [this](const ocpp::v16::GetLogRequest& request) {
            const int req_id = request.requestId;
            charge_point_->on_log_status_notification(req_id, "Uploading");
            auto resp = hardware_->upload_logs(request);
            charge_point_->on_log_status_notification(req_id, resp.status == ocpp::v16::LogStatusEnumType::Accepted
                                                                  ? "Uploaded"
                                                                  : "UploadFailed");
            return resp;
        });

    charge_point_->register_update_firmware_callback(
        [this](const ocpp::v16::UpdateFirmwareRequest msg) {
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Downloading);
            hardware_->update_firmware(msg);
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installing);
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installed);
        });

    charge_point_->register_signed_update_firmware_callback(
        [this](const ocpp::v16::SignedUpdateFirmwareRequest msg) {
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Downloading);
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installing);
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installed);
            return hardware_->update_firmware_signed(msg);
        });

    charge_point_->register_set_connection_timeout_callback(
        [this](std::int32_t connection_timeout) { hardware_->set_connection_timeout(connection_timeout); });

    charge_point_->register_is_reset_allowed_callback(
        [this](const ocpp::v16::ResetType& reset_type) { return hardware_->is_reset_allowed(reset_type); });

    charge_point_->register_reset_callback(
        [this](const ocpp::v16::ResetType& reset_type) { hardware_->reset(reset_type); });

    charge_point_->register_connection_state_changed_callback([](bool is_connected) {
        EVLOG_info << "CSMS websocket state changed: " << (is_connected ? "connected" : "disconnected");
    });

    charge_point_->register_signal_set_charging_profiles_callback([this]() { refresh_charging_profile_limits(); });
}

void OcppAdapter::start_metering_threads() {
    for (const auto& connector : cfg_.connectors) {
        const auto interval = connector.meter_sample_interval_s > 0 ? connector.meter_sample_interval_s
                                                                     : cfg_.meter_sample_interval_s;
        meter_threads_.emplace_back([this, connector_id = connector.id, interval]() {
            metering_loop(connector_id, interval);
        });
    }
}

void OcppAdapter::metering_loop(std::int32_t connector, int interval_s) {
    const auto meter_period = std::chrono::seconds(std::max(1, interval_s));
    const auto control_tick = std::chrono::seconds(1);
    auto next_meter_push = std::chrono::steady_clock::now();

    while (running_) {
        try {
            const auto loop_start = std::chrono::steady_clock::now();
            const bool push_meter_now = loop_start >= next_meter_push;
            const auto now = loop_start;
            const auto auth_wait_timeout = std::chrono::seconds(std::max(1, cfg_.auth_wait_timeout_s));
            const auto power_request_timeout = std::chrono::seconds(std::max(1, cfg_.power_request_timeout_s));
            auto hw_tokens = hardware_->poll_auth_tokens();
            if (!hw_tokens.empty()) {
                ingest_auth_tokens(hw_tokens, now);
            }
            auto status = hardware_->get_status(connector);
            record_presence_state(connector, status.plugged_in, now);
            auto measurement = hardware_->sample_meter(connector);
            bool constrained = false;
            bool paused = false;
            bool disabled = false;
            {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                constrained = power_constrained_[connector];
                paused = paused_evse_[connector];
                disabled = evse_disabled_[connector];
            }

            ActiveSession session{};
            bool had_session = false;
            bool pending_changed = false;
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
                const bool blocked_for_session = !status.safety_ok || status.estop || status.earth_fault ||
                    status.comm_fault || status.cp_fault || status.hlc_charge_complete;
                if (!disabled && status.plugged_in && it == sessions_.end() && !blocked_for_session) {
                    ActiveSession s{};
                    s.session_id = make_session_id();
                    s.meter_start_wh = measurement.power_meter.energy_Wh_import.total;
                    s.connected_at = now;
                    s.ev_connected = true;
                    if (auto pending = pop_next_pending_token(connector, now)) {
                        try_authorize_with_token(connector, s, *pending);
                    } else {
                        set_auth_state(connector, AuthorizationState::Pending);
                    }
                    sessions_[connector] = s;
                    const auto reason = s.authorized ? ocpp::SessionStartedReason::Authorized
                                                     : ocpp::SessionStartedReason::EVConnected;
                    charge_point_->on_session_started(connector, s.session_id, reason, std::nullopt);
                    it = sessions_.find(connector);
                } else if (it != sessions_.end()) {
                    it->second.ev_connected = status.plugged_in;
                    if (!it->second.authorized) {
                        if (auto pending = pop_next_pending_token(connector, now)) {
                            try_authorize_with_token(connector, it->second, *pending);
                        } else {
                            set_auth_state(connector, AuthorizationState::Pending);
                        }
                    }
                }

                if (it != sessions_.end()) {
                    session = it->second;
                    had_session = true;
                }
                if (!status.plugged_in && it == sessions_.end()) {
                    set_auth_state(connector, AuthorizationState::Unknown);
                }
                if (pending_changed) {
                    persist_pending_tokens_locked();
                }
            }
            bool fault = false;
            const auto cfg_it = std::find_if(cfg_.connectors.begin(), cfg_.connectors.end(),
                                             [&](const ConnectorConfig& c) { return c.id == connector; });
            const bool lock_required = cfg_it != cfg_.connectors.end() ? cfg_it->require_lock : true;
            if (status.cp_fault) {
                ocpp::v16::ErrorInfo err("cp_fault", ocpp::v16::ChargePointErrorCode::OtherError, true);
                report_fault(connector, err);
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (had_session) {
                    finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                    hardware_->disable(connector);
                    had_session = false;
                } else if (!already_faulted) {
                    hardware_->disable(connector);
                }
                fault = true;
            }
            if (!status.safety_ok || status.estop || status.earth_fault || status.comm_fault) {
                ocpp::v16::ErrorInfo err("safety", ocpp::v16::ChargePointErrorCode::GroundFailure, true);
                ocpp::v16::Reason stop_reason = ocpp::v16::Reason::EmergencyStop;
                if (status.estop) {
                    err = ocpp::v16::ErrorInfo("estop", ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
                    stop_reason = ocpp::v16::Reason::EmergencyStop;
                } else if (status.comm_fault) {
                    err = ocpp::v16::ErrorInfo("comm", ocpp::v16::ChargePointErrorCode::InternalError, true);
                    stop_reason = ocpp::v16::Reason::PowerLoss;
                } else if (status.earth_fault) {
                    err = ocpp::v16::ErrorInfo("earth", ocpp::v16::ChargePointErrorCode::GroundFailure, true);
                    stop_reason = ocpp::v16::Reason::Other;
                }
                report_fault(connector, err);
                bool already_faulted = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    already_faulted = connector_faulted_[connector];
                    connector_faulted_[connector] = true;
                }
                if (!already_faulted) {
                    finish_transaction(connector, stop_reason, std::nullopt);
                    hardware_->disable(connector);
                    had_session = false;
                }
                fault = true;
            }
            if (!fault && status.isolation_fault) {
                ocpp::v16::ErrorInfo err("isolation_fault",
                                         ocpp::v16::ChargePointErrorCode::GroundFailure, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }
            if (!fault && status.overtemp_fault) {
                ocpp::v16::ErrorInfo err("overtemp",
                                         ocpp::v16::ChargePointErrorCode::HighTemperature, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }
            if (!fault && status.overcurrent_fault) {
                ocpp::v16::ErrorInfo err("overcurrent",
                                         ocpp::v16::ChargePointErrorCode::OverCurrentFailure, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }
            if (!fault && status.gc_welded) {
                ocpp::v16::ErrorInfo err("gc_welded",
                                         ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }
            if (!fault && status.mc_welded) {
                ocpp::v16::ErrorInfo err("mc_welded",
                                         ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }
            const uint8_t usable_modules =
                static_cast<uint8_t>(status.module_healthy_mask &
                                     static_cast<uint8_t>(~status.module_fault_mask));
            if (!fault && usable_modules == 0) {
                ocpp::v16::ErrorInfo err("module_fault",
                                         ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            } else if (!fault && status.module_fault_mask != 0) {
                ocpp::v16::ErrorInfo err("module_degraded",
                                         ocpp::v16::ChargePointErrorCode::OtherError, false);
                report_fault(connector, err);
            }
            // Lock fault: if lock disengaged while plug-in/session active, treat as fault
            if (!fault && lock_required && !status.lock_engaged && (status.plugged_in || had_session)) {
                ocpp::v16::ErrorInfo err("lock_fault",
                                         ocpp::v16::ChargePointErrorCode::ConnectorLockFailure, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }
            if (!fault && status.cp_state == 'D') {
                ocpp::v16::ErrorInfo err("ventilation_required",
                                         ocpp::v16::ChargePointErrorCode::OtherError, true);
                report_fault(connector, err);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
                fault = true;
            }

            if (push_meter_now) {
                push_meter_values(connector, measurement);
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
            if (status.meter_stale) {
                ocpp::v16::ErrorInfo err("meter_stale", ocpp::v16::ChargePointErrorCode::PowerMeterFailure, false);
                report_fault(connector, err);
                fault = true;
            } else if (!fault) {
                charge_point_->on_all_errors_cleared(connector);
                std::lock_guard<std::mutex> lock(state_mutex_);
                connector_faulted_[connector] = false;
            }

            const bool power_ready = had_session && session.authorized &&
                (status.relay_closed || power_delivery_requested(status, lock_required)) && !disabled;
            if (had_session && status.cp_state != 'U' && !status.plugged_in) {
                finish_transaction(connector, ocpp::v16::Reason::EVDisconnected, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
            } else if (had_session && !session.authorized && session.connected_at.time_since_epoch().count() &&
                       (now - session.connected_at) > auth_wait_timeout) {
                EVLOG_warning << "Session on connector " << connector
                              << " timed out waiting for authorization, stopping session";
                set_auth_state(connector, AuthorizationState::Denied);
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
            } else if (had_session && session.authorized && !power_ready && session.authorized_at &&
                       !constrained && !paused && (now - *session.authorized_at) > power_request_timeout) {
                EVLOG_warning << "Session on connector " << connector
                              << " timed out waiting for EV power request after authorization";
                finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
            } else if (had_session && power_ready && !session.transaction_started) {
                // Start OCPP transaction aligned to power delivery readiness
                std::lock_guard<std::mutex> lock(session_mutex_);
                auto it = sessions_.find(connector);
                if (it != sessions_.end() && !it->second.transaction_started && it->second.authorized &&
                    it->second.id_token.has_value()) {
                    charge_point_->on_transaction_started(connector, it->second.session_id, it->second.id_token.value(),
                                                          it->second.meter_start_wh, std::nullopt, ocpp::DateTime(),
                                                          std::nullopt);
                    it->second.transaction_started = true;
                    it->second.power_requested_at = now;
                    session = it->second;
                }
            }

            update_connector_state(connector, status, had_session, session.transaction_started, session.authorized,
                                   fault, disabled);
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
                                     std::optional<ocpp::CiString<20>> id_tag_end) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    const auto it = sessions_.find(connector);
    if (it == sessions_.end()) {
        return;
    }

    const bool can_start = it->second.authorized && it->second.id_token.has_value();
    if (!it->second.transaction_started && can_start) {
        charge_point_->on_transaction_started(connector, it->second.session_id, it->second.id_token.value(),
                                               it->second.meter_start_wh, std::nullopt, ocpp::DateTime(),
                                               std::nullopt);
        it->second.transaction_started = true;
    }

    const auto session_id = it->second.session_id;
    if (it->second.transaction_started) {
        const auto measurement = hardware_->sample_meter(connector);
        const auto energy_wh = measurement.power_meter.energy_Wh_import.total;
        charge_point_->on_transaction_stopped(connector, session_id, reason, ocpp::DateTime(),
                                              static_cast<float>(energy_wh), id_tag_end, std::nullopt);
    }
    pending_tokens_.erase(connector);
    persist_pending_tokens_locked();
    set_auth_state(connector, AuthorizationState::Unknown);
    charge_point_->on_session_stopped(connector, session_id);
    sessions_.erase(it);
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
    if (charge_point_) {
        charge_point_->on_all_errors_cleared(connector);
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
    const auto ttl = std::chrono::seconds(std::max(1, cfg_.auth_wait_timeout_s));
    std::lock_guard<std::mutex> lock(session_mutex_);
    for (auto token : tokens) {
        PendingToken pending;
        pending.token = token;
        if (pending.token.received_at.time_since_epoch().count() == 0) {
            pending.token.received_at = now;
        }
        pending.expires_at = pending.token.received_at + ttl;
        const int target = select_connector_for_token(pending.token);
        pending_tokens_[target].push_back(std::move(pending));
    }
    persist_pending_tokens_locked();
}

int OcppAdapter::select_connector_for_token(const AuthToken& token) const {
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
OcppAdapter::pop_next_pending_token(std::int32_t connector, const std::chrono::steady_clock::time_point& now) {
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
    for (auto qit = queue.begin(); qit != queue.end();) {
        if (qit->expires_at <= now) {
            qit = queue.erase(qit);
        } else {
            ++qit;
        }
    }
    if (queue.empty()) {
        return std::nullopt;
    }
    std::size_t best_idx = 0;
    int best_prio = priority(queue.front().token.source);
    for (std::size_t idx = 1; idx < queue.size(); ++idx) {
        const int prio = priority(queue[idx].token.source);
        if (prio < best_prio ||
            (prio == best_prio && queue[idx].token.received_at < queue[best_idx].token.received_at)) {
            best_prio = prio;
            best_idx = idx;
        }
    }
    PendingToken selected = queue[best_idx];
    queue.erase(queue.begin() + static_cast<std::ptrdiff_t>(best_idx));
    persist_pending_tokens_locked();
    return selected;
}

AuthorizationState OcppAdapter::try_authorize_with_token(std::int32_t connector, ActiveSession& session,
                                                         const PendingToken& pending) {
    if (session.authorized) {
        return AuthorizationState::Granted;
    }
    const auto trimmed = clamp_id_token(pending.token.id_token);
    bool accepted = false;
    if (pending.token.prevalidated) {
        accepted = true;
    } else if (charge_point_) {
        const auto info = charge_point_->authorize_id_token(ocpp::CiString<20>(trimmed));
        accepted = (info.id_tag_info.status == ocpp::v16::AuthorizationStatus::Accepted);
    }
    if (accepted) {
        session.authorized = true;
        session.id_token = trimmed;
        session.authorized_at = std::chrono::steady_clock::now();
        session.token_source = pending.token.source;
        set_auth_state(connector, AuthorizationState::Granted);
        persist_pending_tokens_locked();
        return AuthorizationState::Granted;
    } else {
        const bool keep_waiting = (pending.token.source == AuthTokenSource::Autocharge);
        const auto state = keep_waiting ? AuthorizationState::Pending : AuthorizationState::Denied;
        set_auth_state(connector, state);
        persist_pending_tokens_locked();
        return state;
    }
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
    struct ConnSnapshot {
        GunStatus status;
        double measured_power_kw{0.0};
        double measured_current_a{0.0};
    };
    std::map<int, ConnSnapshot> snapshots;
    bool trip_global = false;
    std::string global_reason;
    auto derate_linear = [](double value, double temp_c, double start_c, double trip_c) {
        if (value <= 0.0) return 0.0;
        if (trip_c <= start_c) return value;
        if (temp_c >= trip_c) return 0.0;
        if (temp_c <= start_c) return value;
        const double scale = 1.0 - ((temp_c - start_c) / (trip_c - start_c));
        return value * std::max(0.0, scale);
    };
    for (const auto& c : cfg_.connectors) {
        power_constrained_[c.id] = false;
    }

    for (const auto& c : cfg_.connectors) {
        GunStatus st = hardware_->get_status(c.id);
        const Slot* slot_for_conn = find_slot_for_gun(c.id);
        if (safety_trip_needed(st)) {
            trip_global = true;
            if (global_reason.empty()) {
                if (st.estop) {
                    global_reason = "estop";
                } else if (st.earth_fault) {
                    global_reason = "earth_fault";
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
        g.safety_ok = st.safety_ok && !st.estop && !st.earth_fault && !st.isolation_fault &&
                      !st.overtemp_fault && !st.overcurrent_fault && !st.comm_fault;
        g.plugged_in = st.plugged_in;
        g.reserved = reserved_connectors_[c.id];
        if (planner_cfg_.connector_derate_trip_c > 0.0 &&
            st.connector_temp_c >= planner_cfg_.connector_derate_trip_c) {
            g.safety_ok = false;
        }

        const bool lock_required = c.require_lock;
        bool session_present = false;
        bool session_authorized = false;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            auto sit = sessions_.find(c.id);
            if (sit != sessions_.end()) {
                session_present = true;
                session_authorized = sit->second.authorized;
            }
        }
        const bool disabled_by_csms = evse_disabled_.count(c.id) ? evse_disabled_[c.id] : false;
        const bool paused_by_csms = paused_evse_.count(c.id) ? paused_evse_[c.id] : false;
        const bool session_ready = session_present && session_authorized && !disabled_by_csms && !paused_by_csms;
        const bool power_ready = session_ready &&
            (st.relay_closed || power_delivery_requested(st, lock_required));
        if (session_present) {
            g.reserved = false;
            reserved_connectors_[c.id] = false;
        }

        if (st.evse_max_current_a) {
            g.gun_current_limit_a = g.gun_current_limit_a > 0.0
                                        ? std::min(g.gun_current_limit_a, st.evse_max_current_a.value())
                                        : st.evse_max_current_a.value();
        }
        if (st.evse_max_power_kw) {
            g.gun_power_limit_kw = g.gun_power_limit_kw > 0.0
                                       ? std::min(g.gun_power_limit_kw, st.evse_max_power_kw.value())
                                       : st.evse_max_power_kw.value();
        }
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
        if (measured_v > 0.0) {
            last_voltage_v_[c.id] = measured_v;
        }
        double measured_i =
            st.present_current_a ? st.present_current_a.value()
                                 : (last_power_w_[c.id] > 0 && measured_v > 0.0 ? last_power_w_[c.id] / measured_v
                                                                                : 0.0);
        double measured_power_kw =
            st.present_power_w ? st.present_power_w.value() / 1000.0
                               : (last_power_w_[c.id] > 0 ? last_power_w_[c.id] / 1000.0 : 0.0);
        if (st.present_power_w) {
            last_power_w_[c.id] = st.present_power_w.value();
        }

        const uint8_t healthy_mask = st.module_healthy_mask;
        const uint8_t fault_mask = st.module_fault_mask;
        const uint8_t usable_mask = static_cast<uint8_t>(healthy_mask & static_cast<uint8_t>(~fault_mask));
        const int healthy_modules = popcount(usable_mask);
        const bool modules_ok = healthy_modules > 0;

        double req_kw = 0.0;
        if (power_ready && g.safety_ok && modules_ok && !st.gc_welded && !st.mc_welded) {
            if (st.target_voltage_v && st.target_current_a) {
                req_kw = (st.target_voltage_v.value() * st.target_current_a.value()) / 1000.0;
            } else if (st.target_current_a) {
                req_kw = (measured_v > 0.0 ? measured_v : 800.0) * st.target_current_a.value() / 1000.0;
            } else if (st.evse_max_power_kw) {
                req_kw = st.evse_max_power_kw.value();
            } else if (last_requested_power_kw_[c.id] > 0.0) {
                req_kw = last_requested_power_kw_[c.id];
            } else if (g.gun_power_limit_kw > 0.0) {
                req_kw = g.gun_power_limit_kw;
            }
            if (req_kw <= 0.0) {
                req_kw = g.gun_power_limit_kw;
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
        if (st.evse_max_voltage_v) {
            max_voltage_v = std::min(max_voltage_v, st.evse_max_voltage_v.value());
        }
        if (g.min_voltage_v > 0.0 && g.ev_req_voltage_v < g.min_voltage_v) {
            g.ev_req_voltage_v = g.min_voltage_v;
        }
        g.ev_req_voltage_v = std::min(g.ev_req_voltage_v, max_voltage_v);
        g.i_meas_a = measured_i;

        // Update module health by slot
        if (slot_for_conn) {
            for (std::size_t idx = 0; idx < slot_for_conn->modules.size(); ++idx) {
                const auto& module_id = slot_for_conn->modules[idx];
                for (auto& mod : module_states_) {
                    if (mod.id == module_id) {
                        const uint8_t bit = static_cast<uint8_t>(1U << idx);
                        const bool healthy_bit = (st.module_healthy_mask & bit) != 0;
                        const bool fault_bit = (st.module_fault_mask & bit) != 0;
                        const double module_temp = st.module_temp_c[idx];
                        mod.temperature_c = module_temp;
                        bool healthy = healthy_bit && g.safety_ok && !fault_bit;
                        if (planner_cfg_.module_derate_trip_c > 0.0 &&
                            module_temp >= planner_cfg_.module_derate_trip_c) {
                            healthy = false;
                        }
                        mod.healthy = healthy;
                        break;
                    }
                }
            }
        }
        int runtime_healthy_modules = healthy_modules;
        if (slot_for_conn) {
            runtime_healthy_modules = 0;
            for (const auto& mod : module_states_) {
                if (mod.slot_id == slot_for_conn->id && mod.healthy) {
                    runtime_healthy_modules++;
                }
            }
        }
        const bool blocked = !g.safety_ok || runtime_healthy_modules <= 0 || st.gc_welded || st.mc_welded;
        g.ev_session_active = session_ready;
        const bool ready_for_power = power_ready && !blocked;

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
        if (st.last_evse_limit_ack.time_since_epoch().count() > 0) {
            const auto age = now - st.last_evse_limit_ack;
            if (age > std::chrono::milliseconds(1000) && ready_for_power) {
                power_constrained_[c.id] = true;
                EVLOG_warning << "Connector " << c.id << " EVSE limit ACK stale (" << age.count()
                              << "ms); constraining power";
            }
        }
        guns.push_back(g);
        gun_lookup[g.id] = g;

        snapshots[c.id] = ConnSnapshot{st, measured_power_kw, measured_i};
        if (g.connector_temp_c > 0.0) {
            g.gun_current_limit_a = derate_linear(g.gun_current_limit_a, g.connector_temp_c,
                                                  planner_cfg_.connector_derate_start_c,
                                                  planner_cfg_.connector_derate_trip_c);
            g.gun_power_limit_kw = derate_linear(g.gun_power_limit_kw, g.connector_temp_c,
                                                 planner_cfg_.connector_derate_start_c,
                                                 planner_cfg_.connector_derate_trip_c);
            if (g.gun_current_limit_a <= 0.0) {
                g.safety_ok = false;
            }
        }
    }

    if (!trip_global && global_fault_latched_) {
        global_fault_latched_ = false;
        global_fault_reason_.clear();
        EVLOG_info << "Global fault cleared";
    }

    if (trip_global || global_fault_latched_) {
        enter_global_fault(global_reason.empty() ? "safety" : global_reason, ocpp::v16::Reason::EmergencyStop);
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

    std::map<int, SlotModuleSelection> slot_selections;
    for (const auto& c : cfg_.connectors) {
        const Slot* slot = find_slot_for_gun(c.id);
        if (!slot) continue;
        auto sel = compute_slot_module_selection(plan, *slot);
        slot_selections[slot->id] = sel;
    }

    struct SlotCommandInfo {
        SlotModuleSelection selection;
        GunStatus status;
        GunState gun_state;
        bool disabled_by_csms{false};
        bool paused{false};
        bool local_fault{false};
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

    for (const auto& c : cfg_.connectors) {
        const Slot* slot = find_slot_for_gun(c.id);
        if (!slot) continue;

        SlotCommandInfo info{};
        info.selection = slot_selections.count(slot->id) ? slot_selections[slot->id] : SlotModuleSelection{};
        const auto mc_it = plan.mc_commands.find(slot->mc_id);
        info.desired_mc_state = mc_it != plan.mc_commands.end() ? mc_it->second : ContactorState::Closed;
        const auto gc_it = plan.gc_commands.find(slot->gc_id);
        info.desired_gc_state = gc_it != plan.gc_commands.end() ? gc_it->second : ContactorState::Open;
        const auto snap_it = snapshots.find(c.id);
        info.status = snap_it != snapshots.end() ? snap_it->second.status : GunStatus{};
        info.meas_current = snap_it != snapshots.end() ? snap_it->second.measured_current_a : 0.0;
        info.meas_voltage = snap_it != snapshots.end() ? snap_it->second.status.present_voltage_v.value_or(last_voltage_v_[c.id])
                                                       : last_voltage_v_[c.id];
        info.gun_state = gun_lookup.count(c.id) ? gun_lookup.at(c.id) : GunState{};
        info.disabled_by_csms = evse_disabled_.count(c.id) ? evse_disabled_[c.id] : false;
        info.paused = paused_evse_[c.id] || info.disabled_by_csms;

        int runtime_healthy_modules = 0;
        for (const auto& m : module_states_) {
            if (m.slot_id == slot->id && m.healthy) {
                runtime_healthy_modules++;
            }
        }

        if (info.status.gc_welded) info.fault_reason = "GCWelded";
        if (info.status.mc_welded) info.fault_reason = "MCWelded";
        if (info.status.isolation_fault && info.fault_reason.empty()) info.fault_reason = "Isolation";
        if (info.status.overtemp_fault && info.fault_reason.empty()) info.fault_reason = "Overtemp";
        if (info.status.overcurrent_fault && info.fault_reason.empty()) info.fault_reason = "Overcurrent";
        if (info.status.comm_fault && info.fault_reason.empty()) info.fault_reason = "CommFault";
        info.local_fault = !info.gun_state.safety_ok || runtime_healthy_modules <= 0 || info.status.gc_welded ||
                           info.status.mc_welded || info.status.isolation_fault || info.status.overtemp_fault ||
                           info.status.overcurrent_fault || info.status.comm_fault;

        if (info.selection.gun_id > 0 && info.selection.module_count > 0 && !info.local_fault &&
            !info.disabled_by_csms && info.desired_mc_state == ContactorState::Closed) {
            info.modules_final = info.selection.module_count;
            info.mask_final = info.selection.mask;
        } else {
            info.modules_final = 0;
            info.mask_final = 0;
        }

        slot_info[slot->id] = info;
        if (info.selection.gun_id > 0) {
            actual_modules_per_gun[info.selection.gun_id] += info.modules_final;
            if (info.local_fault && !island_fault[info.selection.gun_id]) {
                island_fault[info.selection.gun_id] = true;
                island_fault_reason[info.selection.gun_id] =
                    info.fault_reason.empty() ? "IslandSlotFault" : info.fault_reason;
            }
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
    }

    for (const auto& c : cfg_.connectors) {
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
        const GunState g = gun_lookup.count(c.id) ? gun_lookup.at(c.id) : GunState{};
        bool local_fault = info.local_fault;
        if (is_home && island_fault.count(gun_for_slot) && island_fault[gun_for_slot] && !local_fault) {
            local_fault = true;
            info.fault_reason = island_fault_reason[gun_for_slot];
        }

        // MC sequencing: honor open-on-charge islands but gate opens on low current with a short timeout.
        const double mc_open_thresh = planner_cfg_.mc_open_current_a > 0.0 ? planner_cfg_.mc_open_current_a : 0.5;
        bool mc_closed_cmd = (info.desired_mc_state == ContactorState::Closed);
        bool isolation_ready = true;
        const bool mc_change_requested = last_mc_state_.count(slot->mc_id) &&
            info.desired_mc_state != last_mc_state_[slot->mc_id];
        if (mc_change_requested &&
            ((status.relay_closed || std::fabs(meas_i) >= mc_open_thresh))) {
            mc_closed_cmd = (last_mc_state_[slot->mc_id] == ContactorState::Closed);
            power_constrained_[c.id] = true;
            EVLOG_debug << "Deferring MC change for slot " << slot->id << " while charging is active";
            isolation_ready = false;
        } else if (info.desired_mc_state == ContactorState::Open && !local_fault) {
            const bool already_isolated =
                (last_mc_state_[slot->mc_id] == ContactorState::Open) &&
                (!mc_open_pending_.count(slot->id) || !mc_open_pending_[slot->id]);
            if (!already_isolated) {
                const bool safe_to_open = (std::fabs(meas_i) < mc_open_thresh) || !status.relay_closed;
                if (!safe_to_open) {
                    mc_closed_cmd = true;
                    mc_open_pending_[slot->id] = true;
                    auto& ts = mc_open_request_time_[slot->id];
                    if (ts.time_since_epoch().count() == 0) {
                        ts = now;
                    } else if ((now - ts) > MC_OPEN_TIMEOUT_MS) {
                        local_fault = true;
                        info.fault_reason = info.fault_reason.empty() ? "MCOpenTimeout" : info.fault_reason;
                        EVLOG_warning << "MC open timeout for slot " << slot->id << " (gun " << gun_for_slot
                                      << "); locking out connector";
                    }
                    isolation_ready = false;
                } else {
                    mc_closed_cmd = false;
                    mc_open_pending_.erase(slot->id);
                    mc_open_request_time_.erase(slot->id);
                }
            } else {
                mc_closed_cmd = false;
            }
        } else {
            mc_open_pending_.erase(slot->id);
            mc_open_request_time_.erase(slot->id);
            mc_closed_cmd = (info.desired_mc_state == ContactorState::Closed);
        }
        if (local_fault) {
            isolation_ready = false;
            mc_open_pending_.erase(slot->id);
            mc_open_request_time_.erase(slot->id);
            mc_closed_cmd = false;
        }

        // GC: avoid opening under load unless forced; rely on PLC to ramp to zero.
        const double gc_open_thresh = planner_cfg_.gc_open_current_a > 0.0 ? planner_cfg_.gc_open_current_a : 0.5;
        bool gc_closed_cmd = is_home && (info.desired_gc_state == ContactorState::Closed) &&
                             dispatch.modules_assigned > 0 && !info.disabled_by_csms && !local_fault;
        if (!gc_closed_cmd && !local_fault && is_home) {
            const bool safe_to_open = std::fabs(meas_i) < gc_open_thresh || !status.relay_closed;
            if (!safe_to_open) {
                gc_closed_cmd = true;
                gc_open_pending_[c.id] = true;
                auto& ts = gc_open_request_time_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else if ((now - ts) > GC_OPEN_TIMEOUT_MS) {
                    local_fault = true;
                    info.fault_reason = info.fault_reason.empty() ? "GCOpenTimeout" : info.fault_reason;
                    EVLOG_warning << "GC open timeout for connector " << c.id
                                  << " while waiting for current to drop";
                }
            } else {
                gc_open_pending_.erase(c.id);
                gc_open_request_time_.erase(c.id);
            }
        } else if (!is_home) {
            gc_open_pending_.erase(c.id);
            gc_open_request_time_.erase(c.id);
        } else {
            gc_open_pending_.erase(c.id);
            gc_open_request_time_.erase(c.id);
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

        // Precharge/voltage match before closing GC (home slot only).
        bool precharge_ok = true;
        if (is_home) {
            const double v_target = dispatch.voltage_set_v;
            double v_meas = status.present_voltage_v ? status.present_voltage_v.value() : last_voltage_v_[c.id];
            if (v_meas <= 0.0) v_meas = last_voltage_v_[c.id];
            if (v_meas <= 0.0) v_meas = planner_cfg_.default_voltage_v;
            const bool precharge_needed = dispatch.modules_assigned > 0 && gc_closed_cmd &&
                                          !info.disabled_by_csms && !local_fault;
            if (precharge_needed) {
                const double dv = std::fabs(v_meas - v_target);
                precharge_ok = dv <= cfg_.precharge_voltage_tolerance_v;
                if (!precharge_ok) {
                    auto& ts = precharge_start_[c.id];
                    if (ts.time_since_epoch().count() == 0) {
                        ts = now;
                    } else if ((now - ts) > std::chrono::milliseconds(cfg_.precharge_timeout_ms)) {
                        local_fault = true;
                        info.fault_reason = info.fault_reason.empty() ? "PrechargeTimeout" : info.fault_reason;
                        EVLOG_warning << "Precharge timeout for connector " << c.id;
                    }
                } else {
                    precharge_start_.erase(c.id);
                }
            } else {
                precharge_start_.erase(c.id);
            }
            if (!precharge_ok) {
                gc_closed_cmd = false;
            }
            if (local_fault) {
                precharge_start_.erase(c.id);
            }
        }

        const bool modules_allowed = in_island && !local_fault && !info.disabled_by_csms &&
                                     isolation_ready && dispatch.modules_assigned > 0 && mc_closed_cmd;
        const int slot_module_cmd = modules_allowed ? info.modules_final : 0;
        const uint8_t slot_mask_cmd = modules_allowed ? info.mask_final : 0;
        const int gc_module_count = is_home ? dispatch.modules_assigned : slot_module_cmd;
        const bool allow_energy = modules_allowed && gc_closed_cmd && isolation_ready && precharge_ok && is_home;

        PowerCommand cmd;
        cmd.connector = c.id;
        cmd.module_count = gc_module_count;
        cmd.module_mask = slot_mask_cmd;
        cmd.gc_closed = gc_closed_cmd;
        cmd.mc_closed = (!info.disabled_by_csms && !local_fault) ? mc_closed_cmd : false;
        cmd.voltage_set_v = dispatch.voltage_set_v;
        cmd.current_limit_a = allow_energy ? dispatch.current_limit_a : 0.0;
        cmd.power_kw = allow_energy ? dispatch.p_set_kw : 0.0;

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
                if (cmd.module_mask == 0 && slot_mask_cmd != 0) {
                    cmd.module_mask = slot_mask_cmd;
                } else if (cmd.module_mask == 0 && last_module_mask_cmd_[c.id] != 0) {
                    cmd.module_mask = last_module_mask_cmd_[c.id];
                }
            }
        }

        last_current_limit_a_[c.id] = cmd.current_limit_a;
        last_requested_power_kw_[c.id] = dispatch.p_set_kw;
        last_module_alloc_[c.id] = cmd.module_count;
        last_module_mask_cmd_[c.id] = cmd.module_mask;
        EvseLimits limits{};
        if (dispatch.voltage_set_v > 0.0) {
            limits.max_voltage_v = dispatch.voltage_set_v;
        }
        limits.max_current_a = cmd.current_limit_a;
        limits.max_power_kw = cmd.power_kw;
        hardware_->apply_power_command(cmd);
        hardware_->set_evse_limits(c.id, limits);
        last_mc_state_[slot->mc_id] = cmd.mc_closed ? ContactorState::Closed : ContactorState::Open;
        last_gc_state_[slot->gc_id] = cmd.gc_closed ? ContactorState::Closed : ContactorState::Open;

        if (is_home) {
            charge_point_->on_max_current_offered(c.id, static_cast<std::int32_t>(std::round(cmd.current_limit_a)));
            charge_point_->on_max_power_offered(
                c.id, static_cast<std::int32_t>(std::round(cmd.power_kw * 1000.0)));

            const bool constrained = g.ev_session_active && !local_fault && !info.disabled_by_csms &&
                                     (((dispatch.modules_assigned == 0 && dispatch.p_budget_kw > 0.0) ||
                                       (cmd.module_count == 0 && dispatch.modules_assigned > 0) ||
                                       (dispatch.p_set_kw + 1e-3 < dispatch.p_budget_kw)));
            power_constrained_[c.id] = constrained;
        } else {
            power_constrained_[c.id] = false;
        }

        if (local_fault && is_home && g.ev_session_active) {
            finish_transaction(c.id, ocpp::v16::Reason::PowerLoss, std::nullopt);
            hardware_->disable(c.id);
            const std::string reason = !info.fault_reason.empty() ? info.fault_reason : "LocalFault";
            if (charge_point_) {
                ocpp::v16::ErrorInfo err(reason, ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
                charge_point_->on_error(c.id, err);
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                connector_faulted_[c.id] = true;
                connector_state_[c.id] = ConnectorState::Faulted;
            }
        }

        EVLOG_debug << "Planner dispatch slot " << slot->id << " gun=" << gun_for_slot
                    << " modules=" << cmd.module_count << " mask=0x" << std::hex
                    << static_cast<int>(cmd.module_mask) << std::dec << " I_lim=" << cmd.current_limit_a
                    << "A V_set=" << cmd.voltage_set_v << " GC=" << (cmd.gc_closed ? "C" : "O")
                    << " MC=" << (cmd.mc_closed ? "C" : "O");
    }
}

bool OcppAdapter::safety_trip_needed(const GunStatus& status) const {
    return !status.safety_ok || status.estop || status.earth_fault;
}

void OcppAdapter::enter_global_fault(const std::string& reason, ocpp::v16::Reason stop_reason) {
    if (global_fault_latched_.exchange(true)) {
        return;
    }
    global_fault_reason_ = reason;
    EVLOG_error << "Global fault latched: " << reason;
    for (const auto& c : cfg_.connectors) {
        finish_transaction(c.id, stop_reason, std::nullopt);
        hardware_->disable(c.id);
        if (charge_point_) {
            ocpp::v16::ErrorInfo err(reason, ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
            charge_point_->on_error(c.id, err);
        }
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
    gc_open_pending_.clear();
    gc_open_request_time_.clear();
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

    // Pull composite schedules for the next hour; stack returns schedules per connector.
    const auto schedules =
        charge_point_->get_all_composite_charging_schedules(3600, ocpp::v16::ChargingRateUnit::A);

    for (const auto& kv : schedules) {
        const int connector_id = kv.first;
        const auto& sched = kv.second;
        if (sched.chargingSchedulePeriod.empty()) {
            continue;
        }
        auto period = *std::min_element(sched.chargingSchedulePeriod.begin(), sched.chargingSchedulePeriod.end(),
                                        [](const auto& a, const auto& b) { return a.startPeriod < b.startPeriod; });
        if (sched.chargingRateUnit == ocpp::v16::ChargingRateUnit::A) {
            profile_current_limit_a_[connector_id] = period.limit;
        } else if (sched.chargingRateUnit == ocpp::v16::ChargingRateUnit::W) {
            profile_power_limit_kw_[connector_id] = period.limit / 1000.0;
        }
    }
}

void OcppAdapter::update_connector_state(std::int32_t connector, const GunStatus& status, bool has_session,
                                         bool tx_started, bool authorized, bool fault_active, bool disabled) {
    ConnectorState next = ConnectorState::Available;
    const bool cp_known = status.cp_state != 'U';
    bool paused = false;
    bool constrained = false;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        paused = paused_evse_[connector];
        constrained = power_constrained_[connector];
    }
    if (fault_active || status.meter_stale) {
        next = ConnectorState::Faulted;
    } else if (disabled) {
        next = ConnectorState::SuspendedEVSE;
    } else if (has_session) {
        if (!tx_started) {
            next = ConnectorState::Preparing;
        } else if (cp_known && !status.plugged_in) {
            next = ConnectorState::Finishing;
        } else if (paused || constrained) {
            next = ConnectorState::SuspendedEVSE;
        } else if (!status.relay_closed) {
            const bool ev_requesting = cp_known ? (status.cp_state == 'C' || status.cp_state == 'D') : authorized;
            next = ev_requesting ? ConnectorState::SuspendedEVSE : ConnectorState::SuspendedEV;
        } else {
            next = ConnectorState::Charging;
        }
    } else if (status.plugged_in || (cp_known && status.cp_state != 'U')) {
        next = ConnectorState::Preparing;
    }

    ConnectorState current;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current = connector_state_[connector];
    }
    if (next == current) {
        return;
    }

    switch (next) {
    case ConnectorState::Faulted:
        charge_point_->on_disabled(connector);
        break;
    case ConnectorState::Preparing:
        charge_point_->on_all_errors_cleared(connector);
        break;
    case ConnectorState::SuspendedEV:
        charge_point_->on_suspend_charging_ev(connector);
        break;
    case ConnectorState::SuspendedEVSE:
        charge_point_->on_suspend_charging_evse(connector);
        break;
    case ConnectorState::Finishing:
        charge_point_->on_suspend_charging_ev(connector);
        break;
    case ConnectorState::Charging:
        charge_point_->on_resume_charging(connector);
        break;
    case ConnectorState::Available:
        charge_point_->on_all_errors_cleared(connector);
        charge_point_->on_enabled(connector);
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

std::string OcppAdapter::token_source_to_string(AuthTokenSource src) {
    switch (src) {
    case AuthTokenSource::RFID: return "rfid";
    case AuthTokenSource::Autocharge: return "autocharge";
    case AuthTokenSource::RemoteStart: return "remotestart";
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
    }
    hardware_->set_authorization_state(connector, state);
}

ocpp::v16::RemoteStartStopStatus
OcppAdapter::evaluate_remote_start_acceptance(const std::string& /*id_token*/,
                                              const std::vector<std::int32_t>& referenced_connectors) {
    std::vector<int> candidates;
    if (!referenced_connectors.empty()) {
        for (auto c : referenced_connectors) {
            if (c > 0) candidates.push_back(static_cast<int>(c));
        }
    }
    if (candidates.empty()) {
        for (const auto& cfgc : cfg_.connectors) {
            candidates.push_back(cfgc.id);
        }
    }
    auto is_disabled_or_reserved = [&](int cid) {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        const bool disabled = evse_disabled_.count(cid) ? evse_disabled_[cid] : false;
        const bool reserved = reserved_connectors_.count(cid) ? reserved_connectors_[cid] : false;
        return disabled || reserved;
    };
    auto is_faulted = [&](int cid) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return connector_faulted_.count(cid) ? connector_faulted_[cid] : false;
    };

    for (int cid : candidates) {
        if (is_disabled_or_reserved(cid) || is_faulted(cid)) {
            continue;
        }
        const auto cfg_it = std::find_if(cfg_.connectors.begin(), cfg_.connectors.end(),
                                         [&](const ConnectorConfig& c) { return c.id == cid; });
        const bool lock_required = cfg_it != cfg_.connectors.end() ? cfg_it->require_lock : true;
        const auto st = hardware_->get_status(cid);
        const bool healthy_modules = (static_cast<uint8_t>(st.module_healthy_mask &
                                                           static_cast<uint8_t>(~st.module_fault_mask)) != 0);
        const bool safe = st.safety_ok && !st.estop && !st.earth_fault && !st.comm_fault && !st.cp_fault &&
            !st.hlc_charge_complete && !st.isolation_fault && !st.overtemp_fault && !st.overcurrent_fault &&
            !st.meter_stale && !st.gc_welded && !st.mc_welded && healthy_modules;
        const bool lock_ok = !lock_required || st.lock_engaged;
        if (st.plugged_in && safe && lock_ok) {
            return ocpp::v16::RemoteStartStopStatus::Accepted;
        }
    }
    return ocpp::v16::RemoteStartStopStatus::Rejected;
}

void OcppAdapter::persist_pending_tokens() {
    std::lock_guard<std::mutex> lock(session_mutex_);
    persist_pending_tokens_locked();
}

void OcppAdapter::persist_pending_tokens_locked() {
    try {
        nlohmann::json root;
        root["tokens"] = nlohmann::json::array();
        const auto now_sys = std::chrono::system_clock::now();
        for (const auto& kv : pending_tokens_) {
            for (const auto& pending : kv.second) {
                nlohmann::json entry;
                entry["connector"] = kv.first;
                entry["idToken"] = pending.token.id_token;
                entry["source"] = token_source_to_string(pending.token.source);
                entry["connectorHint"] = pending.token.connector_hint;
                entry["prevalidated"] = pending.token.prevalidated;
                entry["receivedAt"] =
                    std::chrono::duration_cast<std::chrono::seconds>(to_system(pending.token.received_at).time_since_epoch()).count();
                entry["expiresAt"] =
                    std::chrono::duration_cast<std::chrono::seconds>(to_system(pending.expires_at).time_since_epoch()).count();
                root["tokens"].push_back(entry);
            }
        }
        if (!pending_token_store_.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(pending_token_store_.parent_path(), ec);
            std::ofstream out(pending_token_store_);
            if (out) {
                out << root.dump(2);
            }
        }
    } catch (const std::exception& e) {
        EVLOG_warning << "Failed to persist pending tokens: " << e.what();
    }
}

void OcppAdapter::load_pending_tokens_from_disk() {
    if (pending_token_store_.empty()) return;
    if (!std::filesystem::exists(pending_token_store_)) return;
    try {
        std::ifstream in(pending_token_store_);
        if (!in) return;
        nlohmann::json root;
        in >> root;
        if (!root.contains("tokens") || !root["tokens"].is_array()) return;
        const auto now_sys = std::chrono::system_clock::now();
        const auto now_steady = std::chrono::steady_clock::now();
        const auto ttl = std::chrono::seconds(std::max(1, cfg_.auth_wait_timeout_s));
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
                const auto recv_sys = std::chrono::system_clock::time_point(recv_epoch);
                const auto exp_sys = std::chrono::system_clock::time_point(exp_epoch);
                if (exp_sys <= now_sys) continue;
                const auto recv_delta = recv_sys - now_sys;
                const auto exp_delta = exp_sys - now_sys;
                token.received_at = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(recv_delta);
                PendingToken pending;
                pending.token = token;
                // If stored expiry is missing, recompute via TTL from receive time
                pending.expires_at = exp_epoch.count() > 0
                    ? now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(exp_delta)
                    : token.received_at + ttl;
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
