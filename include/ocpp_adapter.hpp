// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"
#include "power_manager.hpp"
#include "power_module_controller.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <filesystem>

#include <everest/logging.hpp>
#include <ocpp/v16/charge_point.hpp>

namespace charger {

enum class ConnectorState { Available, Preparing, Charging, SuspendedEV, SuspendedEVSE, Finishing, Unavailable, Faulted };

class OcppAdapter {
public:
    OcppAdapter(ChargerConfig cfg, std::shared_ptr<HardwareInterface> hardware);
    ~OcppAdapter();

    bool start();
    void stop();

    bool begin_transaction(std::int32_t connector, const std::string& id_token, bool prevalidated = false,
                           ocpp::SessionStartedReason reason = ocpp::SessionStartedReason::Authorized);
    void finish_transaction(std::int32_t connector, ocpp::v16::Reason reason,
                            std::optional<ocpp::CiString<20>> id_tag_end = std::nullopt,
                            bool defer_session_stop = false);

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
        bool defer_until_online{false};
    };

    struct AuthRequest {
        std::int32_t connector{0};
        std::string session_id;
        PendingToken pending;
        std::chrono::steady_clock::time_point enqueued_at;
    };

    struct AuthResult {
        std::int32_t connector{0};
        std::string session_id;
        PendingToken pending;
        bool accepted{false};
    };

    struct HlcControlState {
        bool digital_enabled{false};
        bool pnc_blocked{false};       // autocharge/PnC blocked due to rejection/timeout
        bool pnc_blocked_sent{false};  // last effective block sent to PLC
        std::optional<std::string> blocked_identity{};
        std::chrono::steady_clock::time_point block_expires{};
        std::chrono::steady_clock::time_point auth_pending_since{};
        std::optional<std::string> last_autocharge_id{};
    };

    struct HlcControlOutcome {
        bool digital_update{false};
        bool pnc_block_update{false};
        bool desired_digital{false};
        bool desired_pnc_blocked{false};
        bool force_auth_denied{false};
        bool auth_timeout_triggered{false};
    };

#ifdef CHARGER_UNIT_TESTS
public:
    struct TestHook {
        using ActiveSession = OcppAdapter::ActiveSession;
        using PendingToken = OcppAdapter::PendingToken;

        static void record_presence_state(OcppAdapter& adapter, std::int32_t connector, bool plugged_in,
                                          const std::chrono::steady_clock::time_point& now) {
            adapter.record_presence_state(connector, plugged_in, now);
        }

        static void ingest_auth_tokens(OcppAdapter& adapter, const std::vector<AuthToken>& tokens,
                                       const std::chrono::steady_clock::time_point& now) {
            adapter.ingest_auth_tokens(tokens, now);
        }

        static std::optional<PendingToken>
        pop_next_pending_token(OcppAdapter& adapter, std::int32_t connector, const std::chrono::steady_clock::time_point& now,
                               const std::optional<std::string>& required_token = std::nullopt,
                               const std::optional<std::string>& parent_token = std::nullopt,
                               bool* pending_changed = nullptr) {
            return adapter.pop_next_pending_token(connector, now, required_token, parent_token, pending_changed);
        }

        static AuthorizationState try_authorize_with_token(OcppAdapter& adapter, std::int32_t connector,
                                                           ActiveSession& session, const PendingToken& pending) {
            return adapter.try_authorize_with_token(connector, session, pending);
        }

        static void persist_pending_tokens(OcppAdapter& adapter) { adapter.persist_pending_tokens(); }

        static void set_auth_state(OcppAdapter& adapter, std::int32_t connector, AuthorizationState state) {
            adapter.set_auth_state(connector, state);
        }

        static void apply_power_plan(OcppAdapter& adapter) { adapter.apply_power_plan(); }

        static HlcControlOutcome
        apply_hlc_control(OcppAdapter& adapter, std::int32_t connector, const GunStatus& status, bool had_session,
                          const ActiveSession& session, bool post_stop_plugged,
                          const std::optional<std::string>& autocharge_reject_id, bool force_auth_denied,
                          const std::chrono::steady_clock::time_point& now) {
            return adapter.apply_hlc_control(connector, status, had_session, session, post_stop_plugged,
                                             autocharge_reject_id, force_auth_denied, now);
        }

        static void set_autocharge_enabled(OcppAdapter& adapter, bool enabled) {
            adapter.autocharge_enabled_.store(enabled);
        }

        static void set_csms_connected(OcppAdapter& adapter, bool connected) {
            adapter.csms_connected_.store(connected);
        }

        static void update_connector_state(OcppAdapter& adapter, std::int32_t connector, const GunStatus& status,
                                           bool has_session, bool tx_started, bool authorized, bool fault_active,
                                           bool disabled, bool post_stop_plugged, bool seamless_retry_active,
                                           bool suppress_available_event) {
            adapter.update_connector_state(connector, status, has_session, tx_started, authorized, fault_active,
                                           disabled, post_stop_plugged, seamless_retry_active,
                                           suppress_available_event);
        }

        static ConnectorState connector_state(OcppAdapter& adapter, std::int32_t connector) {
            std::lock_guard<std::mutex> lock(adapter.state_mutex_);
            const auto it = adapter.connector_state_.find(connector);
            if (it == adapter.connector_state_.end()) {
                return ConnectorState::Available;
            }
            return it->second;
        }

        static void process_post_stop_state(OcppAdapter& adapter, std::int32_t connector, const GunStatus& status,
                                            const std::chrono::steady_clock::time_point& now,
                                            bool* post_stop_plugged, bool* pending_session_stop,
                                            std::optional<std::string>* pending_session_stop_id) {
            adapter.process_post_stop_state(connector, status, now, post_stop_plugged, pending_session_stop,
                                            pending_session_stop_id);
        }

        static std::mutex& session_mutex(OcppAdapter& adapter) { return adapter.session_mutex_; }
        static std::map<std::int32_t, ActiveSession>& sessions(OcppAdapter& adapter) { return adapter.sessions_; }
        static std::map<std::int32_t, bool>& plugged_in_state(OcppAdapter& adapter) { return adapter.plugged_in_state_; }
        static std::map<int, bool>& power_constrained(OcppAdapter& adapter) { return adapter.power_constrained_; }
        static std::map<std::int32_t, bool>& post_stop_plugged(OcppAdapter& adapter) { return adapter.post_stop_plugged_; }
        static std::map<std::int32_t, std::chrono::steady_clock::time_point>& post_stop_time(OcppAdapter& adapter) {
            return adapter.post_stop_time_;
        }
        static std::map<std::int32_t, std::string>& pending_session_stop(OcppAdapter& adapter) {
            return adapter.pending_session_stop_;
        }
        static std::map<std::int32_t, bool>& connector_faulted(OcppAdapter& adapter) {
            return adapter.connector_faulted_;
        }
    };

private:
#endif

    ChargerConfig cfg_;
    std::shared_ptr<HardwareInterface> hardware_;
    std::unique_ptr<ocpp::v16::ChargePoint> charge_point_;
    PlannerConfig planner_cfg_{};
    PowerManager power_manager_;
    std::vector<Slot> slots_;
    std::map<int, std::array<int, 2>> connector_module_slots_;
    std::map<int, int> slot_owner_connector_;
    std::unique_ptr<PowerModuleController> module_controller_;

    std::atomic<bool> running_{false};
    std::filesystem::path pending_token_store_;
    std::map<std::int32_t, ActiveSession> sessions_;
    std::map<std::int32_t, std::deque<PendingToken>> pending_tokens_;
    std::map<std::int32_t, HlcControlState> hlc_control_;
    std::map<std::int32_t, std::chrono::steady_clock::time_point> plug_event_time_;
    std::map<std::int32_t, bool> plugged_in_state_;
    std::map<std::int32_t, bool> connector_faulted_;
    std::map<std::int32_t, std::set<std::string>> active_ocpp_errors_;
    std::map<std::int32_t, ConnectorState> connector_state_;
    std::map<std::int32_t, bool> post_stop_plugged_;
    std::map<std::int32_t, std::chrono::steady_clock::time_point> post_stop_time_;
    std::map<std::int32_t, std::string> pending_session_stop_;
    std::map<std::int32_t, std::chrono::steady_clock::time_point> pending_session_stop_since_;
    std::vector<std::thread> meter_threads_;
    std::thread planner_thread_;
    std::atomic<bool> planner_thread_running_{false};
    std::thread csms_reconnect_thread_;
    std::atomic<bool> csms_reconnect_thread_running_{false};
    std::thread auth_thread_;
    std::atomic<bool> auth_thread_running_{false};
    std::atomic<bool> csms_connected_{false};
    std::map<int, bool> evse_disabled_;
    std::map<int, bool> reserved_connectors_;
    std::map<int, int> reservation_lookup_;
    std::map<int, int> reservation_id_by_connector_;
    std::map<int, std::chrono::steady_clock::time_point> reservation_expiry_;
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
    std::map<int, double> last_ev_target_power_kw_;
    std::map<std::string, ContactorState> last_gc_state_;
    std::map<std::string, ContactorState> last_mc_state_;
    std::map<int, bool> mc_open_pending_;
    std::map<int, std::chrono::steady_clock::time_point> mc_open_request_time_;
    std::map<std::string, std::chrono::steady_clock::time_point> mc_switch_ready_since_;
    std::map<std::string, std::chrono::steady_clock::time_point> gc_switch_ready_since_;
    std::map<int, bool> gc_open_pending_;
    std::map<int, std::chrono::steady_clock::time_point> gc_open_request_time_;
    std::map<int, std::chrono::steady_clock::time_point> gc_close_request_time_;
    std::map<int, std::chrono::steady_clock::time_point> power_delivery_stall_since_;
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
    std::map<int, uint64_t> limit_ack_stale_events_;
    std::map<int, uint64_t> telemetry_timeout_events_;
    std::map<int, std::map<std::string, std::chrono::steady_clock::time_point>> recent_token_cache_;
    std::deque<AuthRequest> auth_queue_;
    std::map<int, AuthResult> auth_results_;
    std::map<int, std::string> auth_in_flight_;
    std::mutex auth_queue_mutex_;
    std::condition_variable auth_queue_cv_;
    struct RfidTapLatch {
        std::string token;
        std::chrono::steady_clock::time_point last_seen{};
        bool consumed{false};
    };
    std::map<std::int32_t, RfidTapLatch> rfid_tap_latch_;
    std::atomic<bool> global_fault_latched_{false};
    std::string global_fault_reason_;
    std::chrono::steady_clock::time_point global_fault_clear_since_{};
    std::map<int, std::chrono::steady_clock::time_point> module_missing_since_;
    std::map<int, std::chrono::steady_clock::time_point> last_module_health_ok_;
    std::map<int, std::string> last_local_fault_reason_;
    std::map<int, AuthorizationState> auth_state_cache_;
    std::map<int, std::chrono::steady_clock::time_point> auth_denied_since_;
    std::map<int, std::string> last_denied_token_;
    std::atomic<bool> autocharge_enabled_{true};
    std::atomic<bool> boot_accepted_{false};
    std::atomic<bool> pending_status_refresh_{false};
    std::chrono::steady_clock::time_point last_status_refresh_{};
    std::chrono::steady_clock::time_point last_status_refresh_log_{};
    std::string pending_status_refresh_reason_{};
    std::mutex status_refresh_mutex_;
    std::map<int, uint64_t> status_event_seq_;
    std::chrono::steady_clock::time_point last_autocharge_drop_log_{};
    std::chrono::steady_clock::time_point last_autocharge_block_log_{};
    std::chrono::steady_clock::time_point last_reservation_expiry_check_{};
    std::map<int, int> telemetry_mismatch_count_;
    std::mutex telemetry_mutex_;
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
                                bool seamless_retry_active, bool suppress_available_event);
    void process_post_stop_state(std::int32_t connector, const GunStatus& status,
                                 const std::chrono::steady_clock::time_point& now, bool* post_stop_plugged,
                                 bool* pending_session_stop, std::optional<std::string>* pending_session_stop_id);
    bool has_active_session(std::int32_t connector);
    void initialize_slots();
    void apply_power_plan();
    void refresh_charging_profile_limits();
    void request_status_refresh(const std::string& reason);
    void maybe_refresh_status_notifications(const std::chrono::steady_clock::time_point& now);
    void enter_global_fault(const std::string& reason, ocpp::v16::Reason stop_reason);
    void apply_zero_power_plan();
    bool safety_trip_needed(const GunStatus& status) const;
    void record_presence_state(std::int32_t connector, bool plugged_in,
                               const std::chrono::steady_clock::time_point& now);
    void ingest_auth_tokens(const std::vector<AuthToken>& tokens,
                            const std::chrono::steady_clock::time_point& now);
    void set_autocharge_enabled(bool enabled, const std::string& source);
    bool clear_pending_autocharge_tokens_locked();
    bool clear_pending_autocharge_tokens_for_connector_locked(std::int32_t connector);
    HlcControlOutcome apply_hlc_control(std::int32_t connector, const GunStatus& status, bool had_session,
                                        const ActiveSession& session, bool post_stop_plugged,
                                        const std::optional<std::string>& autocharge_reject_id,
                                        bool force_auth_denied,
                                        const std::chrono::steady_clock::time_point& now);
    int select_connector_for_token(const AuthToken& token) const;
    std::optional<PendingToken> pop_next_pending_token(std::int32_t connector,
                                                       const std::chrono::steady_clock::time_point& now,
                                                       const std::optional<std::string>& required_token = std::nullopt,
                                                       const std::optional<std::string>& parent_token = std::nullopt,
                                                       bool* pending_changed = nullptr);
    AuthorizationState try_authorize_with_token(std::int32_t connector, ActiveSession& session, const PendingToken& pending);
    AuthorizationState authorize_token_for_session(std::int32_t connector, const std::string& session_id,
                                                   const PendingToken& pending);
    AuthorizationState apply_authorization_result(std::int32_t connector, const std::string& session_id,
                                                  const PendingToken& pending, bool accepted);
    void enqueue_auth_request(std::int32_t connector, const std::string& session_id, const PendingToken& pending);
    std::optional<AuthResult> pop_auth_result(std::int32_t connector, const std::string& session_id);
    void clear_auth_queue_for_connector(std::int32_t connector);
    void auth_loop();
    void clear_local_auth_cache();
    void clear_pending_tokens();
    void clear_auth_queue();
    void handle_clear_cache();
    std::string clamp_id_token(const std::string& raw) const;
    void persist_pending_tokens();
    void persist_pending_tokens_snapshot(const std::map<std::int32_t, std::deque<PendingToken>>& snapshot);
    void load_pending_tokens_from_disk();
    void expire_reservations(const std::chrono::steady_clock::time_point& now);
    std::chrono::steady_clock::time_point to_steady(std::chrono::system_clock::time_point t_sys) const;
    std::chrono::system_clock::time_point to_system(std::chrono::steady_clock::time_point t_steady) const;
    void sync_ocpp_error(std::int32_t connector, const std::string& uuid, ocpp::v16::ChargePointErrorCode error_code,
                         bool is_fault, bool active, const std::optional<std::string>& info = std::nullopt,
                         const std::optional<std::string>& vendor_id = std::nullopt,
                         const std::optional<std::string>& vendor_error_code = std::nullopt);
    static std::string token_source_to_string(AuthTokenSource src);
    static std::string auth_state_to_string(AuthorizationState state);
    static AuthTokenSource token_source_from_string(const std::string& s);
    void set_auth_state(std::int32_t connector, AuthorizationState state);
    AuthorizationState get_auth_state(std::int32_t connector);
    std::optional<std::chrono::steady_clock::time_point> get_auth_denied_since(std::int32_t connector);
    void note_auth_denied(std::int32_t connector, const std::string& id_token);
    bool should_bypass_token_dedup(std::int32_t connector, const AuthToken& token);
    ocpp::v16::DataTransferResponse
    handle_data_transfer_request(const ocpp::v16::DataTransferRequest& request);
    void handle_configuration_key_change(const ocpp::v16::KeyValue& key_value);
    bool token_matches_reservation(std::int32_t connector, const std::string& token,
                                   const std::optional<std::string>& parent_token);
    int meter_interval_seconds_for_connector(std::int32_t connector);
};

} // namespace charger
