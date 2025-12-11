// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"
#include "power_manager.hpp"

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <mutex>

#include <everest/logging.hpp>
#include <ocpp/v16/charge_point.hpp>

namespace charger {

enum class ConnectorState { Available, Charging, SuspendedEV, SuspendedEVSE, Faulted };

class OcppAdapter {
public:
    OcppAdapter(ChargerConfig cfg, std::shared_ptr<HardwareInterface> hardware);
    ~OcppAdapter();

    bool start();
    void stop();

    bool begin_transaction(std::int32_t connector, const std::string& id_token,
                           ocpp::SessionStartedReason reason = ocpp::SessionStartedReason::Authorized);
    void finish_transaction(std::int32_t connector, ocpp::v16::Reason reason,
                            std::optional<ocpp::CiString<20>> id_tag_end = std::nullopt);

    void push_meter_values(std::int32_t connector, const ocpp::Measurement& measurement);
    void report_fault(std::int32_t connector, const ocpp::v16::ErrorInfo& info);
    void clear_faults(std::int32_t connector);

private:
    struct ActiveSession {
        std::string session_id;
        std::string id_token;
        double meter_start_wh{0.0};
        std::chrono::steady_clock::time_point started_at;
        bool transaction_started{false};
    };

    ChargerConfig cfg_;
    std::shared_ptr<HardwareInterface> hardware_;
    std::unique_ptr<ocpp::v16::ChargePoint> charge_point_;
    PowerManager power_manager_;
    PlannerConfig planner_cfg_{};
    std::vector<Slot> slots_;

    std::atomic<bool> running_{false};
    std::map<std::int32_t, ActiveSession> sessions_;
    std::map<std::int32_t, bool> connector_faulted_;
    std::map<std::int32_t, ConnectorState> connector_state_;
    std::vector<std::thread> meter_threads_;
    std::thread planner_thread_;
    std::atomic<bool> planner_thread_running_{false};
    std::mutex session_mutex_;
    std::mutex state_mutex_;
    std::mutex plan_mutex_;
    std::mutex meter_mutex_;
    std::vector<ModuleState> module_states_;
    bool slots_initialized_{false};
    std::map<int, int> last_module_alloc_;
    std::map<int, double> last_voltage_v_;
    std::map<int, double> last_power_w_;
    std::map<int, double> last_current_limit_a_;
    std::map<int, double> last_requested_power_kw_;
    std::map<std::string, ContactorState> last_gc_state_;
    std::map<std::string, ContactorState> last_mc_state_;
    std::map<int, bool> mc_open_pending_;
    std::map<int, bool> paused_evse_;
    std::map<int, uint8_t> last_module_mask_cmd_;
    std::map<int, double> profile_current_limit_a_;
    std::map<int, double> profile_power_limit_kw_;
    std::map<int, double> last_energy_wh_;
    std::atomic<bool> global_fault_latched_{false};
    std::string global_fault_reason_;

    void register_callbacks();
    void start_metering_threads();
    void metering_loop(std::int32_t connector, int interval_s);
    std::string make_session_id() const;
    void prepare_security_files() const;
    void update_connector_state(std::int32_t connector, const GunStatus& status, bool has_session, bool fault_active);
    bool has_active_session(std::int32_t connector);
    void initialize_slots();
    void apply_power_plan();
    void refresh_charging_profile_limits();
    void enter_global_fault(const std::string& reason, ocpp::v16::Reason stop_reason);
    void apply_zero_power_plan();
    bool safety_trip_needed(const GunStatus& status) const;
};

} // namespace charger
