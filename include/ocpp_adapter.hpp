// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"
#include "power_manager.hpp"
#include "power_module_controller.hpp"

#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <mutex>
#include <filesystem>

#include <everest/logging.hpp>
#include <ocpp/v16/charge_point.hpp>

namespace charger {

enum class ConnectorState { Available, Preparing, Charging, SuspendedEV, SuspendedEVSE, Finishing, Faulted };

class OcppAdapter {
public:
    OcppAdapter(ChargerConfig cfg, std::shared_ptr<HardwareInterface> hardware);
    ~OcppAdapter();

    bool start();
    void stop();

    bool begin_transaction(std::int32_t connector, const std::string& id_token, bool prevalidated = false,
                           ocpp::SessionStartedReason reason = ocpp::SessionStartedReason::Authorized);
    void finish_transaction(std::int32_t connector, ocpp::v16::Reason reason,
                            std::optional<ocpp::CiString<20>> id_tag_end = std::nullopt);

    void push_meter_values(std::int32_t connector, const ocpp::Measurement& measurement);
    void report_fault(std::int32_t connector, const ocpp::v16::ErrorInfo& info);
    void clear_faults(std::int32_t connector);

private:
    struct ActiveSession {
        std::string session_id;
        std::optional<std::string> id_token;
        double meter_start_wh{0.0};
        std::chrono::steady_clock::time_point connected_at;
        std::optional<std::chrono::steady_clock::time_point> authorized_at;
        std::optional<std::chrono::steady_clock::time_point> power_requested_at;
        std::chrono::steady_clock::time_point pending_started;
        std::chrono::steady_clock::time_point last_seen_plugged;
        bool transaction_started{false};
        bool authorized{false};
        bool ev_connected{false};
        AuthTokenSource token_source{AuthTokenSource::RFID};
    };

    struct PendingToken {
        AuthToken token;
        std::chrono::steady_clock::time_point expires_at;
    };

    ChargerConfig cfg_;
    std::shared_ptr<HardwareInterface> hardware_;
    std::unique_ptr<ocpp::v16::ChargePoint> charge_point_;
    PlannerConfig planner_cfg_{};
    PowerManager power_manager_;
    std::vector<Slot> slots_;
    std::unique_ptr<PowerModuleController> module_controller_;

    std::atomic<bool> running_{false};
    std::filesystem::path pending_token_store_;
    std::filesystem::path local_auth_cache_store_;
    std::map<std::int32_t, ActiveSession> sessions_;
    std::map<std::int32_t, std::deque<PendingToken>> pending_tokens_;
    std::map<std::int32_t, std::chrono::steady_clock::time_point> plug_event_time_;
    std::map<std::int32_t, bool> plugged_in_state_;
    std::map<std::int32_t, bool> connector_faulted_;
    std::map<std::int32_t, ConnectorState> connector_state_;
    std::map<std::int32_t, bool> post_stop_plugged_;
    std::map<std::int32_t, std::chrono::steady_clock::time_point> post_stop_time_;
    std::vector<std::thread> meter_threads_;
    std::thread planner_thread_;
    std::atomic<bool> planner_thread_running_{false};
    std::map<int, bool> evse_disabled_;
    std::map<int, bool> reserved_connectors_;
    std::map<int, int> reservation_lookup_;
    std::map<int, std::string> reservation_required_tag_;
    std::map<int, std::optional<std::string>> reservation_parent_tag_;
    std::map<int, bool> power_constrained_;
    std::mutex session_mutex_;
    std::mutex state_mutex_;
    std::mutex plan_mutex_;
    std::mutex meter_mutex_;
    std::mutex auth_mutex_;
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
    std::map<int, std::chrono::steady_clock::time_point> mc_open_request_time_;
    std::map<int, bool> gc_open_pending_;
    std::map<int, std::chrono::steady_clock::time_point> gc_open_request_time_;
    std::map<std::string, std::chrono::steady_clock::time_point> mc_command_change_time_;
    std::map<std::string, std::chrono::steady_clock::time_point> gc_command_change_time_;
    std::map<int, bool> paused_evse_;
    std::map<int, uint8_t> last_module_mask_cmd_;
    std::map<int, double> profile_current_limit_a_;
    std::map<int, double> profile_power_limit_kw_;
    std::map<int, double> last_energy_wh_;
    std::map<int, double> last_meter_sent_wh_;
    std::map<int, std::chrono::steady_clock::time_point> last_meter_sent_time_;
    std::map<int, std::chrono::steady_clock::time_point> cp_fault_since_;
    std::map<int, uint64_t> last_present_stale_counts_;
    std::map<int, uint64_t> last_limit_stale_counts_;
    std::map<std::string, std::chrono::steady_clock::time_point> local_auth_cache_;
    std::map<std::string, std::chrono::steady_clock::time_point> recent_token_cache_;
    std::atomic<bool> global_fault_latched_{false};
    std::string global_fault_reason_;
    std::map<int, std::chrono::steady_clock::time_point> precharge_start_;
    std::map<int, AuthorizationState> auth_state_cache_;
    std::map<int, int> telemetry_mismatch_count_;
    bool simulation_mode_{false};
    bool force_comm_fault_{false};
    std::optional<std::chrono::steady_clock::time_point> profile_next_refresh_;
    std::map<int, int> connector_meter_intervals_;

    void register_callbacks();
    void start_metering_threads();
    void metering_loop(std::int32_t connector);
    std::string make_session_id() const;
    void prepare_security_files() const;
    void seed_default_evse_limits();
    const Slot* find_slot_for_gun(int gun_id) const;
    void update_connector_state(std::int32_t connector, GunStatus status, bool has_session, bool tx_started,
                                bool authorized, bool fault_active, bool disabled, bool post_stop_plugged,
                                bool seamless_retry_active);
    bool has_active_session(std::int32_t connector);
    void initialize_slots();
    void apply_power_plan();
    void refresh_charging_profile_limits();
    void enter_global_fault(const std::string& reason, ocpp::v16::Reason stop_reason);
    void apply_zero_power_plan();
    bool safety_trip_needed(const GunStatus& status) const;
    void record_presence_state(std::int32_t connector, bool plugged_in,
                               const std::chrono::steady_clock::time_point& now);
    void ingest_auth_tokens(const std::vector<AuthToken>& tokens,
                            const std::chrono::steady_clock::time_point& now);
    int select_connector_for_token(const AuthToken& token) const;
    std::optional<PendingToken> pop_next_pending_token(std::int32_t connector,
                                                       const std::chrono::steady_clock::time_point& now,
                                                       const std::optional<std::string>& required_token = std::nullopt,
                                                       const std::optional<std::string>& parent_token = std::nullopt);
    AuthorizationState try_authorize_with_token(std::int32_t connector, ActiveSession& session, const PendingToken& pending);
    bool authorize_from_cache(const std::string& token);
    void update_local_auth_cache(const std::string& token);
    std::string clamp_id_token(const std::string& raw) const;
    void persist_pending_tokens();
    void persist_pending_tokens_locked();
    void persist_local_auth_cache_locked();
    void load_pending_tokens_from_disk();
    void load_local_auth_cache_from_disk();
    std::chrono::steady_clock::time_point to_steady(std::chrono::system_clock::time_point t_sys) const;
    std::chrono::system_clock::time_point to_system(std::chrono::steady_clock::time_point t_steady) const;
    static std::string token_source_to_string(AuthTokenSource src);
    static AuthTokenSource token_source_from_string(const std::string& s);
    void set_auth_state(std::int32_t connector, AuthorizationState state);
    ocpp::v16::DataTransferResponse
    handle_data_transfer_request(const ocpp::v16::DataTransferRequest& request);
    void handle_configuration_key_change(const ocpp::v16::KeyValue& key_value);
    bool token_matches_reservation(std::int32_t connector, const std::string& token,
                                   const std::optional<std::string>& parent_token);
    int meter_interval_seconds_for_connector(std::int32_t connector);
};

} // namespace charger
