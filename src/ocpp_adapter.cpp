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

#include <everest/logging.hpp>
#include <ocpp/common/evse_security_impl.hpp>

namespace charger {

namespace fs = std::filesystem;

namespace {

const Slot* find_slot(const std::vector<Slot>& slots, int id) {
    auto it = std::find_if(slots.begin(), slots.end(), [&](const Slot& s) { return s.id == id; });
    return it == slots.end() ? nullptr : &(*it);
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

} // namespace

OcppAdapter::OcppAdapter(ChargerConfig cfg, std::shared_ptr<HardwareInterface> hardware) :
    cfg_(std::move(cfg)),
    hardware_(std::move(hardware)),
    power_manager_(PlannerConfig{}) {
    for (const auto& c : cfg_.connectors) {
        connector_faulted_[c.id] = false;
        connector_state_[c.id] = ConnectorState::Available;
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

void OcppAdapter::initialize_slots() {
    if (slots_initialized_) return;

    PlannerConfig pcfg;
    if (!cfg_.connectors.empty()) {
        pcfg.module_power_kw = cfg_.connectors.front().max_power_w > 0 ? cfg_.connectors.front().max_power_w / 2000.0
                                                                       : 30.0;
    }
    pcfg.grid_limit_kw = 1000.0;
    power_manager_ = PowerManager(pcfg);

    std::vector<Slot> slots;
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

    running_ = true;
    start_metering_threads();
    return true;
}

void OcppAdapter::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
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
        const bool ok = hardware_->enable(connector);
        if (ok) {
            charge_point_->on_enabled(connector);
        }
        return ok;
    });

    charge_point_->register_disable_evse_callback([this](std::int32_t connector) {
        const bool ok = hardware_->disable(connector);
        if (ok) {
            charge_point_->on_disabled(connector);
        }
        return ok;
    });

    charge_point_->register_pause_charging_callback([this](std::int32_t connector) {
        const bool ok = hardware_->pause_charging(connector);
        if (ok) {
            charge_point_->on_suspend_charging_evse(connector);
        }
        return ok;
    });

    charge_point_->register_resume_charging_callback([this](std::int32_t connector) {
        const bool ok = hardware_->resume_charging(connector);
        if (ok) {
            charge_point_->on_resume_charging(connector);
        }
        return ok;
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
            }
            return status;
        });

    charge_point_->register_cancel_reservation_callback([this](std::int32_t reservation_id) {
        return hardware_->cancel_reservation(reservation_id);
    });

    charge_point_->register_unlock_connector_callback(
        [this](std::int32_t connector) { return hardware_->unlock(connector); });

    charge_point_->register_upload_diagnostics_callback(
        [this](const ocpp::v16::GetDiagnosticsRequest& request) {
            auto resp = hardware_->upload_diagnostics(request);
            charge_point_->on_log_status_notification(-1, "Uploading");
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                charge_point_->on_log_status_notification(-1, "Uploaded");
            }).detach();
            return resp;
        });

    charge_point_->register_upload_logs_callback(
        [this](const ocpp::v16::GetLogRequest& request) {
            auto resp = hardware_->upload_logs(request);
            const int req_id = request.requestId;
            charge_point_->on_log_status_notification(req_id, "Uploading");
            std::thread([this, req_id]() {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                charge_point_->on_log_status_notification(req_id, "Uploaded");
            }).detach();
            return resp;
        });

    charge_point_->register_update_firmware_callback(
        [this](const ocpp::v16::UpdateFirmwareRequest msg) {
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Downloading);
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installing);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installed);
            }).detach();
            hardware_->update_firmware(msg);
        });

    charge_point_->register_signed_update_firmware_callback(
        [this](const ocpp::v16::SignedUpdateFirmwareRequest msg) {
            charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Downloading);
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installing);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                charge_point_->on_firmware_update_status_notification(-1, ocpp::FirmwareStatusNotification::Installed);
            }).detach();
            return hardware_->update_firmware_signed(msg);
        });

    charge_point_->register_set_connection_timeout_callback(
        [this](std::int32_t connection_timeout) { hardware_->set_connection_timeout(connection_timeout); });

    charge_point_->register_is_reset_allowed_callback(
        [this](const ocpp::v16::ResetType& reset_type) { return hardware_->is_reset_allowed(reset_type); });

    charge_point_->register_reset_callback(
        [this](const ocpp::v16::ResetType& reset_type) { hardware_->reset(reset_type); });

    charge_point_->register_signal_set_charging_profiles_callback([]() {
        EVLOG_info << "Charging profile update received (apply limits in hardware if needed)";
    });

    charge_point_->register_connection_state_changed_callback([](bool is_connected) {
        EVLOG_info << "CSMS websocket state changed: " << (is_connected ? "connected" : "disconnected");
    });
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

            auto measurement = hardware_->sample_meter(connector);
            push_meter_values(connector, measurement);
            {
                std::lock_guard<std::mutex> plan_lock(plan_mutex_);
                if (measurement.power_meter.voltage_V && measurement.power_meter.voltage_V->DC.has_value()) {
                    last_voltage_v_[connector] = measurement.power_meter.voltage_V->DC.value();
                }
                if (measurement.power_meter.power_W && measurement.power_meter.power_W->total.has_value()) {
                    last_power_w_[connector] = measurement.power_meter.power_W->total.value();
                } else if (measurement.power_meter.current_A && measurement.power_meter.current_A->DC.has_value() &&
                           measurement.power_meter.voltage_V && measurement.power_meter.voltage_V->DC.has_value()) {
                    const double v = measurement.power_meter.voltage_V->DC.value();
                    const double i = measurement.power_meter.current_A->DC.value();
                    last_power_w_[connector] = v * i;
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

            if (had_session && status.cp_state != 'U' && !status.plugged_in) {
                finish_transaction(connector, ocpp::v16::Reason::EVDisconnected, std::nullopt);
                hardware_->disable(connector);
                had_session = false;
            }

            update_connector_state(connector, status, had_session, fault);
            apply_power_plan();
        } catch (const std::exception& e) {
            EVLOG_warning << "Metering loop error on connector " << connector << ": " << e.what();
        }
        std::this_thread::sleep_for(std::chrono::seconds(interval_s));
    }
}

bool OcppAdapter::begin_transaction(std::int32_t connector, const std::string& id_token,
                                    ocpp::SessionStartedReason reason) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (sessions_.find(connector) != sessions_.end()) {
        EVLOG_warning << "Connector " << connector << " already has an active session";
        return false;
    }

    // Safety gate before starting
    auto status = hardware_->get_status(connector);
    if (!status.safety_ok || status.estop || status.earth_fault || status.comm_fault || status.cp_fault) {
        EVLOG_warning << "Cannot start session on connector " << connector << " due to safety/comm/cp faults";
        return false;
    }

    const bool cp_known = status.cp_state != 'U';
    if (cp_known && !status.plugged_in) {
        EVLOG_warning << "Cannot start session on connector " << connector << " because EV is unplugged";
        return false;
    }

    const auto measurement = hardware_->sample_meter(connector);
    const auto meter_start = measurement.power_meter.energy_Wh_import.total;
    const auto session_id = make_session_id();

    charge_point_->on_session_started(connector, session_id, reason, std::nullopt);
    charge_point_->on_transaction_started(connector, session_id, id_token, meter_start, std::nullopt,
                                          ocpp::DateTime(), std::nullopt);

    sessions_[connector] = ActiveSession{session_id, id_token, meter_start};
    hardware_->resume_charging(connector);
    return true;
}

void OcppAdapter::finish_transaction(std::int32_t connector, ocpp::v16::Reason reason,
                                     std::optional<ocpp::CiString<20>> id_tag_end) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    const auto it = sessions_.find(connector);
    if (it == sessions_.end()) {
        return;
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
        charge_point_->on_meter_values(connector, measurement);
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
    std::vector<GunState> guns;
    guns.reserve(cfg_.connectors.size());
    struct ConnSnapshot {
        GunStatus status;
        double measured_power_kw{0.0};
        double measured_current_a{0.0};
    };
    std::map<int, ConnSnapshot> snapshots;

    for (const auto& c : cfg_.connectors) {
        GunState g{};
        g.id = c.id;
        g.slot_id = c.id;
        g.gc_id = "GC_" + std::to_string(c.id);
        g.gun_power_limit_kw = c.max_power_w > 0 ? c.max_power_w / 1000.0 : 0.0;
        g.gun_current_limit_a = c.max_current_a > 0 ? c.max_current_a : 0.0;
        g.priority = 0;
        g.ev_session_active = has_active_session(c.id);
        g.i_set_a = last_current_limit_a_[c.id];
        auto st = hardware_->get_status(c.id);
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
        const double measured_v = st.present_voltage_v ? st.present_voltage_v.value()
                                                       : (last_voltage_v_[c.id] > 50.0 ? last_voltage_v_[c.id]
                                                                                        : 800.0);
        if (measured_v > 0.0) {
            last_voltage_v_[c.id] = measured_v;
        }
        const double measured_i =
            st.present_current_a ? st.present_current_a.value()
                                 : (last_power_w_[c.id] > 0 && measured_v > 0.0 ? last_power_w_[c.id] / measured_v
                                                                                : 0.0);
        const double measured_power_kw =
            st.present_power_w ? st.present_power_w.value() / 1000.0
                               : (last_power_w_[c.id] > 0 ? last_power_w_[c.id] / 1000.0 : 0.0);
        if (st.present_power_w) {
            last_power_w_[c.id] = st.present_power_w.value();
        }

        if (g.ev_session_active) {
            double req_kw = 0.0;
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
            g.ev_req_power_kw = std::min(req_kw, g.gun_power_limit_kw > 0.0 ? g.gun_power_limit_kw : req_kw);
        } else {
            g.ev_req_power_kw = 0.0;
        }
        g.ev_req_voltage_v = st.target_voltage_v ? st.target_voltage_v.value()
                                                 : (st.present_voltage_v ? st.present_voltage_v.value() : measured_v);
        if (g.ev_req_voltage_v <= 0.0) {
            g.ev_req_voltage_v = measured_v > 0.0 ? measured_v : 800.0;
        }
        g.connector_temp_c = st.connector_temp_c;
        if (st.relay_closed) {
            g.fsm_state = GunFsmState::Charging;
        } else if (g.ev_session_active) {
            g.fsm_state = GunFsmState::Ready;
        } else {
            g.fsm_state = GunFsmState::Idle;
        }
        guns.push_back(g);

        // Update module health by slot
        if (const Slot* slot = find_slot(slots_, c.id)) {
            for (std::size_t idx = 0; idx < slot->modules.size(); ++idx) {
                const auto& module_id = slot->modules[idx];
                for (auto& mod : module_states_) {
                    if (mod.id == module_id) {
                        const uint8_t bit = static_cast<uint8_t>(1U << idx);
                        const bool healthy_bit = (st.module_healthy_mask & bit) != 0;
                        const bool fault_bit = (st.module_fault_mask & bit) != 0;
                        mod.healthy = healthy_bit && !st.comm_fault && st.safety_ok && !st.estop && !st.earth_fault &&
                                      !st.isolation_fault && !st.overtemp_fault && !st.overcurrent_fault &&
                                      !fault_bit;
                        break;
                    }
                }
            }
        }
        snapshots[c.id] = ConnSnapshot{st, measured_power_kw, measured_i};
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

    for (const auto& c : cfg_.connectors) {
        const Slot* slot = find_slot(slots_, c.id);
        if (!slot) continue;

        const auto gc_it = plan.gc_commands.find(slot->gc_id);
        const auto mc_it = plan.mc_commands.find(slot->mc_id);
        const ContactorState gc_state =
            gc_it != plan.gc_commands.end() ? gc_it->second : ContactorState::Open;
        const ContactorState mc_state =
            mc_it != plan.mc_commands.end() ? mc_it->second : ContactorState::Closed;

        const auto disp_it = gun_dispatch.find(c.id);
        GunDispatch dispatch{};
        if (disp_it != gun_dispatch.end()) {
            dispatch = disp_it->second;
        } else {
            dispatch.gun_id = c.id;
            dispatch.modules_assigned = 0;
            dispatch.p_budget_kw = 0.0;
            dispatch.current_limit_a = 0.0;
            dispatch.voltage_set_v = last_voltage_v_[c.id] > 0.0 ? last_voltage_v_[c.id] : 800.0;
        }

        // MC open sequencing: keep MC closed until current nearly zero
        bool mc_closed_cmd = (mc_state == ContactorState::Closed);
        const auto snap_it = snapshots.find(c.id);
        const double meas_i = snap_it != snapshots.end() ? snap_it->second.measured_current_a : 0.0;
        const bool mc_wants_open = (mc_state == ContactorState::Open);
        if (mc_wants_open) {
            const bool safe_to_open = std::fabs(meas_i) < 1.0 && dispatch.modules_assigned == 0;
            if (!safe_to_open) {
                mc_open_pending_[slot->id] = true;
                mc_closed_cmd = true;
            } else {
                mc_open_pending_.erase(slot->id);
                mc_closed_cmd = false;
            }
        } else {
            mc_open_pending_.erase(slot->id);
        }

        const int effective_modules = (gc_state == ContactorState::Closed && mc_closed_cmd)
                                          ? dispatch.modules_assigned
                                          : 0;
        const uint8_t module_mask = compute_module_mask_for_gun(plan, *slot, c.id);

        PowerCommand cmd;
        cmd.connector = c.id;
        cmd.module_count = effective_modules;
        cmd.module_mask = module_mask;
        cmd.gc_closed = (gc_state == ContactorState::Closed);
        cmd.mc_closed = mc_closed_cmd;
        cmd.voltage_set_v = dispatch.voltage_set_v;
        cmd.current_limit_a = dispatch.current_limit_a;
        cmd.power_kw = dispatch.p_budget_kw;
        last_current_limit_a_[c.id] = cmd.current_limit_a;
        last_requested_power_kw_[c.id] = dispatch.p_budget_kw;
        last_module_alloc_[c.id] = effective_modules;
        hardware_->apply_power_command(cmd);

        if (dispatch.modules_assigned == 0 && has_active_session(c.id)) {
            finish_transaction(c.id, ocpp::v16::Reason::PowerLoss, std::nullopt);
            hardware_->disable(c.id);
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

void OcppAdapter::update_connector_state(std::int32_t connector, const GunStatus& status, bool has_session,
                                         bool fault_active) {
    ConnectorState next = ConnectorState::Available;
    const bool cp_known = status.cp_state != 'U';
    if (fault_active || status.meter_stale) {
        next = ConnectorState::Faulted;
    } else if (has_session) {
        if (cp_known && !status.plugged_in) {
            next = ConnectorState::SuspendedEV;
        } else if (!status.relay_closed) {
            const bool ev_requesting = cp_known ? (status.cp_state == 'C' || status.cp_state == 'D') : true;
            next = ev_requesting ? ConnectorState::SuspendedEVSE : ConnectorState::SuspendedEV;
        } else {
            next = ConnectorState::Charging;
        }
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
