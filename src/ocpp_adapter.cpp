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

#include <everest/logging.hpp>
#include <ocpp/common/evse_security_impl.hpp>

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

uint8_t compute_module_mask_for_gun(const Plan& plan, const Slot& slot, int gun_id) {
    uint8_t mask = 0;
    const auto it_island =
        std::find_if(plan.islands.begin(), plan.islands.end(),
                     [&](const IslandState& island) { return island.gun_id && island.gun_id.value() == gun_id; });
    if (it_island == plan.islands.end()) {
        return mask;
    }
    for (std::size_t idx = 0; idx < slot.modules.size(); ++idx) {
        const auto& module_id = slot.modules[idx];
        if (std::find(it_island->module_ids.begin(), it_island->module_ids.end(), module_id) !=
            it_island->module_ids.end()) {
            mask |= static_cast<uint8_t>(1U << idx);
        }
    }
    return mask;
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
    for (const auto& c : cfg_.connectors) {
        connector_faulted_[c.id] = false;
        connector_state_[c.id] = ConnectorState::Available;
        evse_disabled_[c.id] = false;
        reserved_connectors_[c.id] = false;
        power_constrained_[c.id] = false;
        paused_evse_[c.id] = false;
    }
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
            const auto target =
                referenced_connectors.empty() ? cfg_.connectors.front().id : referenced_connectors.front();
            begin_transaction(target, id_token, ocpp::SessionStartedReason::Authorized);
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
    while (running_) {
        try {
            auto status = hardware_->get_status(connector);
            bool had_session = has_active_session(connector);
            std::chrono::steady_clock::time_point session_start{};
            bool tx_started = false;
            std::string session_id_copy;
            std::string id_tag_copy;
            double meter_start_copy = 0.0;
            {
                std::lock_guard<std::mutex> lock(session_mutex_);
                auto it = sessions_.find(connector);
                if (it != sessions_.end()) {
                    session_start = it->second.started_at;
                    tx_started = it->second.transaction_started;
                    session_id_copy = it->second.session_id;
                    id_tag_copy = it->second.id_token;
                    meter_start_copy = it->second.meter_start_wh;
                }
            }
            bool constrained = false;
            bool paused = false;
            {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                constrained = power_constrained_[connector];
                paused = paused_evse_[connector];
            }
            bool fault = false;
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
            const auto cfg_it = std::find_if(cfg_.connectors.begin(), cfg_.connectors.end(),
                                             [&](const ConnectorConfig& c) { return c.id == connector; });
            const bool lock_required = cfg_it != cfg_.connectors.end() ? cfg_it->require_lock : true;
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

            auto measurement = hardware_->sample_meter(connector);
            push_meter_values(connector, measurement);
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

            const bool power_ready = status.relay_closed || power_delivery_requested(status, lock_required);
            if (had_session && status.cp_state != 'U' && !status.plugged_in) {
                finish_transaction(connector, ocpp::v16::Reason::EVDisconnected, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
            } else if (had_session && !power_ready && session_start.time_since_epoch().count()) {
                const auto now = std::chrono::steady_clock::now();
                const auto age = std::chrono::duration_cast<std::chrono::seconds>(now - session_start).count();
                if (!constrained && !paused && age > 60) {
                    EVLOG_warning << "Session on connector " << connector
                                  << " timed out waiting for EV power request, stopping session";
                    finish_transaction(connector, ocpp::v16::Reason::Other, std::nullopt);
                    hardware_->disable(connector);
                    had_session = false;
                }
            } else if (had_session && power_ready && !tx_started) {
                // Start OCPP transaction aligned to power delivery readiness
                std::lock_guard<std::mutex> lock(session_mutex_);
                auto it = sessions_.find(connector);
                if (it != sessions_.end() && !it->second.transaction_started) {
                    charge_point_->on_transaction_started(connector, it->second.session_id, it->second.id_token,
                                                           it->second.meter_start_wh, std::nullopt, ocpp::DateTime(),
                                                           std::nullopt);
                    it->second.transaction_started = true;
                }
            }

            update_connector_state(connector, status, had_session, fault);
        } catch (const std::exception& e) {
            EVLOG_warning << "Metering loop error on connector " << connector << ": " << e.what();
        }
        std::this_thread::sleep_for(std::chrono::seconds(interval_s));
    }
}

bool OcppAdapter::begin_transaction(std::int32_t connector, const std::string& id_token,
                                    ocpp::SessionStartedReason reason) {
    bool disabled = false;
    {
        std::lock_guard<std::mutex> plan_lock(plan_mutex_);
        disabled = evse_disabled_[connector];
        reserved_connectors_[connector] = false;
        for (auto it = reservation_lookup_.begin(); it != reservation_lookup_.end();) {
            if (it->second == connector) {
                it = reservation_lookup_.erase(it);
            } else {
                ++it;
            }
        }
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (sessions_.find(connector) != sessions_.end()) {
        EVLOG_warning << "Connector " << connector << " already has an active session";
        return false;
    }
    if (disabled) {
        EVLOG_warning << "Connector " << connector << " is disabled by CSMS; rejecting session start";
        return false;
    }

    // Safety gate before starting
    auto status = hardware_->get_status(connector);
    if (!status.safety_ok || status.estop || status.earth_fault || status.comm_fault || status.cp_fault) {
        EVLOG_warning << "Cannot start session on connector " << connector << " due to safety/comm/cp faults";
        return false;
    }

    const auto cfg_it = std::find_if(cfg_.connectors.begin(), cfg_.connectors.end(),
                                     [&](const ConnectorConfig& c) { return c.id == connector; });
    const bool lock_required = cfg_it != cfg_.connectors.end() ? cfg_it->require_lock : true;
    if (lock_required && !status.lock_engaged) {
        EVLOG_warning << "Cannot start session on connector " << connector << " because connector lock is open";
        return false;
    }

    const bool cp_known = status.cp_state != 'U';
    if (cp_known && !status.plugged_in) {
        EVLOG_warning << "Cannot start session on connector " << connector << " because EV is unplugged";
        return false;
    }
    if (status.hlc_charge_complete) {
        EVLOG_warning << "Cannot start session on connector " << connector << " because HLC reports charge complete";
        return false;
    }

    const auto measurement = hardware_->sample_meter(connector);
    const auto meter_start = measurement.power_meter.energy_Wh_import.total;
    const auto session_id = make_session_id();

    charge_point_->on_session_started(connector, session_id, reason, std::nullopt);

    sessions_[connector] = ActiveSession{session_id, id_token, meter_start, std::chrono::steady_clock::now(), false};
    return true;
}

void OcppAdapter::finish_transaction(std::int32_t connector, ocpp::v16::Reason reason,
                                     std::optional<ocpp::CiString<20>> id_tag_end) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    const auto it = sessions_.find(connector);
    if (it == sessions_.end()) {
        return;
    }

    if (!it->second.transaction_started) {
        charge_point_->on_transaction_started(connector, it->second.session_id, it->second.id_token,
                                               it->second.meter_start_wh, std::nullopt, ocpp::DateTime(),
                                               std::nullopt);
        it->second.transaction_started = true;
    }

    const auto measurement = hardware_->sample_meter(connector);
    const auto energy_wh = measurement.power_meter.energy_Wh_import.total;
    const auto session_id = it->second.session_id;
    charge_point_->on_transaction_stopped(connector, session_id, reason, ocpp::DateTime(), static_cast<float>(energy_wh),
                                          id_tag_end, std::nullopt);
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

bool OcppAdapter::has_active_session(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    return sessions_.find(connector) != sessions_.end();
}

void OcppAdapter::apply_power_plan() {
    std::lock_guard<std::mutex> plan_lock(plan_mutex_);
    const auto now = std::chrono::steady_clock::now();
    std::map<int, std::string> fault_reasons;
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
        const bool session_active = has_active_session(c.id);
        const bool disabled_by_csms = evse_disabled_.count(c.id) ? evse_disabled_[c.id] : false;
        const bool power_ready = session_active &&
            (st.relay_closed || power_delivery_requested(st, lock_required)) && !disabled_by_csms;
        if (session_active) {
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
        g.ev_session_active = session_active;
        const bool ready_for_power = power_ready && !blocked;

        if (blocked) {
            g.fsm_state = GunFsmState::Fault;
        } else if (st.relay_closed) {
            g.fsm_state = GunFsmState::Charging;
        } else if (ready_for_power) {
            g.fsm_state = GunFsmState::Ready;
        } else if (session_active) {
            g.fsm_state = GunFsmState::EvDetected;
        } else {
            g.fsm_state = GunFsmState::Idle;
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

    for (const auto& c : cfg_.connectors) {
        const Slot* slot = find_slot_for_gun(c.id);
        if (!slot) continue;

        const auto gc_it = plan.gc_commands.find(slot->gc_id);
        const auto mc_it = plan.mc_commands.find(slot->mc_id);
        const ContactorState gc_state =
            gc_it != plan.gc_commands.end() ? gc_it->second : ContactorState::Open;
        const ContactorState mc_state =
            mc_it != plan.mc_commands.end() ? mc_it->second : ContactorState::Closed;
        const bool disabled_by_csms = evse_disabled_.count(c.id) ? evse_disabled_[c.id] : false;

        const auto disp_it = gun_dispatch.find(c.id);
        GunDispatch dispatch{};
        if (disp_it != gun_dispatch.end()) {
            dispatch = disp_it->second;
        } else {
            dispatch.gun_id = c.id;
            dispatch.modules_assigned = 0;
            dispatch.p_budget_kw = 0.0;
            dispatch.p_set_kw = 0.0;
            dispatch.current_limit_a = 0.0;
            dispatch.voltage_set_v = last_voltage_v_[c.id] > 0.0 ? last_voltage_v_[c.id] : 800.0;
        }

        const auto snap_it = snapshots.find(c.id);
        const GunStatus status = snap_it != snapshots.end() ? snap_it->second.status : GunStatus{};
        const double meas_i = snap_it != snapshots.end() ? snap_it->second.measured_current_a : 0.0;
        const GunState g = gun_lookup.count(c.id) ? gun_lookup.at(c.id) : GunState{};
        ContactorState planned_mc_state = mc_state;
        // Prevent MC state changes while connector is actively relaying energy or current above threshold.
        const bool mc_change_requested = last_mc_state_.count(slot->mc_id) &&
            mc_state != last_mc_state_[slot->mc_id];
        const double mc_lock_thresh = planner_cfg_.mc_open_current_a > 0.0 ? planner_cfg_.mc_open_current_a : 0.5;
        if (mc_change_requested &&
            ((status.relay_closed || std::fabs(meas_i) >= mc_lock_thresh))) {
            planned_mc_state = last_mc_state_[slot->mc_id];
            power_constrained_[c.id] = true;
            EVLOG_debug << "Deferring MC change for slot " << slot->id << " while charging is active";
        }

        int runtime_healthy_modules = 0;
        const auto island_it =
            std::find_if(plan.islands.begin(), plan.islands.end(),
                         [&](const IslandState& island) { return island.gun_id && island.gun_id.value() == c.id; });
        if (island_it != plan.islands.end()) {
            for (const auto& mod_id : island_it->module_ids) {
                const auto mit = std::find_if(module_states_.begin(), module_states_.end(),
                                              [&](const ModuleState& m) { return m.id == mod_id; });
                if (mit != module_states_.end() && mit->healthy) {
                    runtime_healthy_modules++;
                }
            }
        }
        if (runtime_healthy_modules == 0) {
            for (const auto& mod : module_states_) {
                if (mod.slot_id == slot->id && mod.healthy) {
                    runtime_healthy_modules++;
                }
            }
        }

        bool local_fault = !g.safety_ok || runtime_healthy_modules <= 0 || status.gc_welded ||
                           status.mc_welded || status.isolation_fault || status.overtemp_fault ||
                           status.overcurrent_fault || status.comm_fault;
        if (status.gc_welded) fault_reasons[c.id] = "GCWelded";
        if (status.mc_welded) fault_reasons[c.id] = "MCWelded";
        bool evse_paused = paused_evse_[c.id] || disabled_by_csms;

        // MC sequencing: honor open-on-charge islands but gate opens on low current with a short timeout.
        const double mc_open_thresh = planner_cfg_.mc_open_current_a > 0.0 ? planner_cfg_.mc_open_current_a : 0.5;
        bool mc_closed_cmd = (mc_state == ContactorState::Closed);
        bool isolation_ready = true;
        if (mc_state == ContactorState::Open && !local_fault) {
            const bool already_isolated =
                (last_mc_state_[slot->mc_id] == ContactorState::Open) &&
                (!mc_open_pending_.count(slot->id) || !mc_open_pending_[slot->id]);
            if (already_isolated) {
                mc_closed_cmd = false;
            } else {
                const bool safe_to_open = (std::fabs(meas_i) < mc_open_thresh) || !status.relay_closed;
                if (safe_to_open) {
                    mc_closed_cmd = false;
                    mc_open_pending_.erase(slot->id);
                    mc_open_request_time_.erase(slot->id);
                } else {
                    mc_closed_cmd = true;
                    mc_open_pending_[slot->id] = true;
                    auto& ts = mc_open_request_time_[slot->id];
                    if (ts.time_since_epoch().count() == 0) {
                        ts = now;
                    } else if ((now - ts) > MC_OPEN_TIMEOUT_MS) {
                        local_fault = true;
                        fault_reasons[c.id] = "MCOpenTimeout";
                        EVLOG_warning << "MC open timeout for slot " << slot->id << " (gun " << c.id
                                      << "); locking out connector";
                    }
                    isolation_ready = false;
                }
            }
        } else {
            mc_open_pending_.erase(slot->id);
            mc_open_request_time_.erase(slot->id);
            mc_closed_cmd = (mc_state == ContactorState::Closed);
        }
        if (local_fault) {
            isolation_ready = false;
            mc_open_pending_.erase(slot->id);
            mc_open_request_time_.erase(slot->id);
            mc_closed_cmd = false;
        }

        // GC: avoid opening under load unless forced; rely on PLC to ramp to zero.
        const double gc_open_thresh = planner_cfg_.gc_open_current_a > 0.0 ? planner_cfg_.gc_open_current_a : 0.5;
        bool gc_closed_cmd = (gc_state == ContactorState::Closed);
        if (!gc_closed_cmd && !local_fault) {
            const bool safe_to_open = std::fabs(meas_i) < gc_open_thresh || !status.relay_closed;
            if (!safe_to_open) {
                gc_closed_cmd = true;
                gc_open_pending_[c.id] = true;
                auto& ts = gc_open_request_time_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else if ((now - ts) > GC_OPEN_TIMEOUT_MS) {
                    local_fault = true;
                    EVLOG_warning << "GC open timeout for connector " << c.id
                                  << " while waiting for current to drop";
                }
            } else {
                gc_open_pending_.erase(c.id);
                gc_open_request_time_.erase(c.id);
            }
        } else {
            gc_open_pending_.erase(c.id);
            gc_open_request_time_.erase(c.id);
        }
        if (disabled_by_csms || local_fault) {
            gc_closed_cmd = false;
        }

        mc_closed_cmd = (planned_mc_state == ContactorState::Closed);
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

        // Precharge/voltage match before closing GC.
        const double v_target = dispatch.voltage_set_v;
        double v_meas = status.present_voltage_v ? status.present_voltage_v.value() : last_voltage_v_[c.id];
        if (v_meas <= 0.0) v_meas = last_voltage_v_[c.id];
        if (v_meas <= 0.0) v_meas = planner_cfg_.default_voltage_v;
        bool precharge_ok = true;
        const bool precharge_needed = dispatch.modules_assigned > 0 && gc_state == ContactorState::Closed &&
                                      !disabled_by_csms && !local_fault;
        if (precharge_needed) {
            const double dv = std::fabs(v_meas - v_target);
            precharge_ok = dv <= cfg_.precharge_voltage_tolerance_v;
            if (!precharge_ok) {
                auto& ts = precharge_start_[c.id];
                if (ts.time_since_epoch().count() == 0) {
                    ts = now;
                } else if ((now - ts) > std::chrono::milliseconds(cfg_.precharge_timeout_ms)) {
                    local_fault = true;
                    precharge_ok = false;
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

        // Allow modules to energize the island even if GC stays open for precharge.
        const bool modules_allowed = (!local_fault) && !disabled_by_csms && mc_closed_cmd &&
                                     dispatch.modules_assigned > 0;
        const bool allow_energy = modules_allowed && gc_closed_cmd && isolation_ready && precharge_ok;
        const int effective_modules = modules_allowed ? dispatch.modules_assigned : 0;
        uint8_t module_mask = modules_allowed ? compute_module_mask_for_gun(plan, *slot, c.id) : 0;

        PowerCommand cmd;
        cmd.connector = c.id;
        cmd.module_count = effective_modules;
        cmd.module_mask = module_mask;
        cmd.gc_closed = gc_closed_cmd;
        cmd.mc_closed = (!disabled_by_csms && !local_fault) ? mc_closed_cmd : false;
        cmd.voltage_set_v = dispatch.voltage_set_v;
        cmd.current_limit_a = allow_energy ? dispatch.current_limit_a : 0.0;
        cmd.power_kw = allow_energy ? dispatch.p_set_kw : 0.0;

        if (evse_paused && g.ev_session_active) {
            // Keep contactors closed but clamp current to zero for a graceful pause.
            cmd.current_limit_a = 0.0;
            cmd.power_kw = 0.0;
            if (disabled_by_csms) {
                cmd.gc_closed = false;
                cmd.mc_closed = (!local_fault && !disabled_by_csms) ? mc_closed_cmd : false;
                cmd.module_count = 0;
                cmd.module_mask = 0;
            } else {
                if (!allow_energy && isolation_ready && !local_fault) {
                    cmd.gc_closed = gc_closed_cmd;
                }
                cmd.mc_closed = (!disabled_by_csms && !local_fault) ? mc_closed_cmd : false;
                if (cmd.module_count <= 0 && isolation_ready && dispatch.modules_assigned > 0 && !local_fault) {
                    cmd.module_count = dispatch.modules_assigned;
                    cmd.module_mask = compute_module_mask_for_gun(plan, *slot, c.id);
                }
                if (cmd.module_mask == 0 && cmd.module_count > 0) {
                    cmd.module_mask = compute_module_mask_for_gun(plan, *slot, c.id);
                }
                if (cmd.module_mask == 0 && last_module_mask_cmd_[c.id] != 0) {
                    cmd.module_mask = last_module_mask_cmd_[c.id];
                }
            }
        }

        last_current_limit_a_[c.id] = cmd.current_limit_a;
        last_requested_power_kw_[c.id] = dispatch.p_set_kw;
        last_module_alloc_[c.id] = cmd.module_count;
        last_module_mask_cmd_[c.id] = cmd.module_mask;
        hardware_->apply_power_command(cmd);
        last_mc_state_[slot->mc_id] = cmd.mc_closed ? ContactorState::Closed : ContactorState::Open;
        last_gc_state_[slot->gc_id] = cmd.gc_closed ? ContactorState::Closed : ContactorState::Open;

        // Inform OCPP of offered limits
        charge_point_->on_max_current_offered(c.id, static_cast<std::int32_t>(std::round(cmd.current_limit_a)));
        charge_point_->on_max_power_offered(
            c.id, static_cast<std::int32_t>(std::round(cmd.power_kw * 1000.0)));

        const bool constrained = g.ev_session_active && !local_fault && !disabled_by_csms &&
                                 (((dispatch.modules_assigned == 0 && dispatch.p_budget_kw > 0.0) ||
                                   (cmd.module_count == 0 && dispatch.modules_assigned > 0) ||
                                   (dispatch.p_set_kw + 1e-3 < dispatch.p_budget_kw)));
        power_constrained_[c.id] = constrained;

        if (local_fault && g.ev_session_active) {
            finish_transaction(c.id, ocpp::v16::Reason::PowerLoss, std::nullopt);
            hardware_->disable(c.id);
            fault_reasons[c.id] = fault_reasons.count(c.id) ? fault_reasons[c.id] : "LocalFault";
            if (charge_point_) {
                ocpp::v16::ErrorInfo err(fault_reasons[c.id],
                                         ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true);
                charge_point_->on_error(c.id, err);
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                connector_faulted_[c.id] = true;
                connector_state_[c.id] = ConnectorState::Faulted;
            }
        }

        EVLOG_debug << "Planner dispatch gun " << c.id << " slots=" << (slot ? slot->id : -1)
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
                                         bool fault_active) {
    ConnectorState next = ConnectorState::Available;
    const bool cp_known = status.cp_state != 'U';
    bool paused = false;
    bool constrained = false;
    bool disabled = false;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        paused = paused_evse_[connector];
        constrained = power_constrained_[connector];
        disabled = evse_disabled_[connector];
    }
    if (fault_active || status.meter_stale) {
        next = ConnectorState::Faulted;
    } else if (has_session) {
        if (cp_known && !status.plugged_in) {
            next = ConnectorState::SuspendedEV;
        } else if (paused || constrained || disabled) {
            next = ConnectorState::SuspendedEVSE;
        } else if (!status.relay_closed) {
            const bool ev_requesting = cp_known ? (status.cp_state == 'C' || status.cp_state == 'D') : true;
            next = ev_requesting ? ConnectorState::SuspendedEVSE : ConnectorState::SuspendedEV;
        } else {
            next = ConnectorState::Charging;
        }
    } else if (disabled) {
        next = ConnectorState::SuspendedEVSE;
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
    case ConnectorState::SuspendedEV:
        charge_point_->on_suspend_charging_ev(connector);
        break;
    case ConnectorState::SuspendedEVSE:
        charge_point_->on_suspend_charging_evse(connector);
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

} // namespace charger
