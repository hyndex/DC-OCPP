// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"
#include "can_contract.hpp"

#include <atomic>
#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <memory>

namespace charger {

/// \brief SocketCAN-based PLC backend using the CAN-TEST/Basic contract.
class PlcCanHardware : public HardwareInterface {
public:
    explicit PlcCanHardware(const ChargerConfig& cfg);
    ~PlcCanHardware() override;

    bool ok() const { return init_ok_; }

    bool enable(std::int32_t connector) override;
    bool disable(std::int32_t connector) override;
    bool pause_charging(std::int32_t connector) override;
    bool resume_charging(std::int32_t connector) override;
    bool stop_transaction(std::int32_t connector, ocpp::v16::Reason reason) override;
    ocpp::v16::UnlockStatus unlock(std::int32_t connector) override;
    ocpp::v16::ReservationStatus reserve(std::int32_t reservation_id, std::int32_t connector, ocpp::DateTime expiry,
                                         const std::string& id_tag,
                                         const std::optional<std::string>& parent_id) override;
    bool cancel_reservation(std::int32_t reservation_id) override;
    ocpp::v16::GetLogResponse upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) override;
    ocpp::v16::GetLogResponse upload_logs(const ocpp::v16::GetLogRequest& request) override;
    bool update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) override;
    ocpp::v16::UpdateFirmwareStatusEnumType
    update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) override;
    void set_connection_timeout(std::int32_t seconds) override;
    bool is_reset_allowed(const ocpp::v16::ResetType& reset_type) override;
    void reset(const ocpp::v16::ResetType& reset_type) override;
    void on_remote_start_token(const std::string& id_token, const std::vector<std::int32_t>& referenced_connectors,
                               bool prevalidated) override;
    ocpp::Measurement sample_meter(std::int32_t connector) override;
    GunStatus get_status(std::int32_t connector) override;
    void set_authorization_state(std::int32_t connector, bool authorized) override;
    void set_authorization_state(std::int32_t connector, AuthorizationState state) override;
    void set_digital_comm_enabled(std::int32_t connector, bool enabled) override;
    void set_pnc_blocked(std::int32_t connector, bool blocked) override;
    void apply_power_command(const PowerCommand& cmd) override;
    void apply_power_allocation(std::int32_t connector, int modules) override;
    void set_evse_limits(std::int32_t connector, const EvseLimits& limits) override;
    void publish_evse_present(std::int32_t connector, double voltage_v, double current_a, double power_kw,
                              bool output_enabled, bool regulating) override;
    void publish_fault_state(std::int32_t connector, uint8_t fault_bits) override;
    void set_no_energy_mode(std::int32_t connector, EvseNoEnergyMode mode) override;
    void clear_faults(std::int32_t connector) override;
    std::vector<AuthToken> poll_auth_tokens() override;
    bool supports_cross_slot_islands() const override;

private:
    enum class TxPriority {
        Critical = 0,
        Control = 1,
        Heartbeat = 2,
        Telemetry = 3,
        Debug = 4
    };

    enum class AutochargeIdSource {
        Evmac,
        Evccid,
        Emaid
    };

    struct IdentityAssembly {
        uint8_t len{0};
        uint8_t seg_cnt{0};
        std::vector<uint8_t> data;
        std::vector<bool> received;
        std::chrono::steady_clock::time_point updated{};

        void reset() {
            len = 0;
            seg_cnt = 0;
            data.clear();
            received.clear();
            updated = std::chrono::steady_clock::time_point{};
        }
    };

    struct RfidAssembly {
        uint8_t uid_len{0};
        uint8_t seg_cnt{0};
        uint8_t event_id{0};
        std::vector<uint8_t> data;
        std::vector<bool> received;
        std::chrono::steady_clock::time_point updated{};

        void reset() {
            uid_len = 0;
            seg_cnt = 0;
            event_id = 0;
            data.clear();
            received.clear();
            updated = std::chrono::steady_clock::time_point{};
        }
    };

    struct PlcState {
        ConnectorConfig cfg;
        int connector_id{0};
        int plc_id{0};
        std::string iface;
        std::atomic<uint8_t> seq{0};
        AuthorizationState desired_auth_state{AuthorizationState::Unknown};
        bool authorized{false};
        bool auth_pending{false};
        bool hlc_enabled{false};
        bool pnc_blocked{false};
        EvseNoEnergyMode no_energy_mode{EvseNoEnergyMode::None};
        bool sys_enable{false};
        bool output_enabled{false};
        bool regulating{false};
        uint8_t fault_bits{0};
        double present_voltage_v{0.0};
        double present_current_a{0.0};
        double present_power_kw{0.0};
        double ev_target_voltage_v{0.0};
        double ev_target_current_a{0.0};
        double ev_target_cache_voltage_v{0.0};
        double ev_target_cache_current_a{0.0};
        double energy_kwh{0.0};
        double freq_hz{0.0};
        can_contract::RelayStatus last_relay{};
        can_contract::SafetyStatus last_safety{};
        can_contract::MeterReading last_meter{};
        uint32_t evse_limit_ack_count{0};
        std::chrono::steady_clock::time_point last_evse_limit_ack{};
        std::chrono::steady_clock::time_point last_energy_update{};
        std::chrono::steady_clock::time_point last_status_rx{};
        std::chrono::steady_clock::time_point last_relay_rx{};
        std::chrono::steady_clock::time_point last_safety_rx{};
        std::chrono::steady_clock::time_point last_meter_rx{};
        std::chrono::steady_clock::time_point last_limits_tx{};
        std::chrono::steady_clock::time_point last_present_tx{};
        std::array<uint8_t, 8> last_limits_payload{};
        bool last_limits_payload_valid{false};
        std::array<uint8_t, 8> last_present_payload{};
        bool last_present_payload_valid{false};
        std::chrono::steady_clock::time_point last_ev_targets_rx{};
        std::chrono::steady_clock::time_point last_ev_target_cache_rx{};
        std::chrono::steady_clock::time_point last_relay_tx{};
        std::chrono::steady_clock::time_point last_evse_present_update{};
        std::chrono::steady_clock::time_point last_cp_rx{};
        std::chrono::steady_clock::time_point last_chargeinfo_rx{};
        std::chrono::steady_clock::time_point last_session_rx{};
        std::chrono::steady_clock::time_point last_boot_rx{};
        EvseLimits limits{};
        uint8_t relay_cmd_mask{0};
        uint8_t relay_enable_mask{0};
        bool relay_force_off{false};
        // Desired relay state (planner / EVSE control). Applied state is held in the fields above and is
        // rate-limited/debounced in update_fast_tx() to avoid relay chatter and excess CAN traffic.
        bool desired_sys_enable{true};
        uint8_t desired_relay_cmd_mask{0};
        uint8_t desired_relay_enable_mask{0};
        bool desired_relay_force_off{false};
        bool relay_state_dirty{true};
        bool relay_tx_urgent{false};
        std::array<std::chrono::steady_clock::time_point, 3> relay_last_change{};
        bool relay_clear_faults{false};
        std::array<std::chrono::steady_clock::time_point, 3> relay_mismatch_since{};
        uint64_t relay_conflict_count{0};
        // CP state from the CP voltage sampler (may be noisy on some harnesses).
        char cp_state_raw{'U'};
        uint8_t cp_duty_raw{0};
        // CP state reported by the session tracker (preferred for readiness / fault decisions).
        char cp_state_session{'U'};
        uint8_t cp_duty_session{0};
        bool plugged_raw{false};
        bool plugged_in{false};
        std::chrono::steady_clock::time_point plugged_raw_changed{};
        bool cp_fault_raw{false};
        bool cp_fault{false};
        std::chrono::steady_clock::time_point cp_fault_raw_changed{};
        uint8_t hlc_stage{0};
        bool hlc_charge_complete{false};
        bool hlc_precharge_active{false};
        bool hlc_cable_checked{false};
        bool hlc_auth_granted{false};
        bool hlc_auth_pending{false};
        std::chrono::steady_clock::time_point last_hlc_active{};
        bool lock_engaged{false};
        bool lock_engaged_valid{false};
        bool lock_command{true};
        bool lock_command_set{false};
        uint8_t boot_feature_flags{0};
        bool meter_available{true};
        uint64_t tlm_v3_crc_fail_count{0};
        std::chrono::steady_clock::time_point last_tlm_crc_warn{};
        // v3 safety bits are latched in telemetry between frames; require consecutive asserted frames
        // before treating critical trips as real to avoid one-frame false positives.
        uint8_t estop_active_streak{0};
        uint8_t earth_fault_streak{0};
        // Debounce safety-related trips to avoid flapping on single-frame glitches.
        std::chrono::steady_clock::time_point safety_trip_since{};
        std::chrono::steady_clock::time_point estop_trip_since{};
        std::chrono::steady_clock::time_point earth_trip_since{};
        std::chrono::steady_clock::time_point comm_trip_since{};
        IdentityAssembly evccid;
        IdentityAssembly evemaid0;
        IdentityAssembly evemaid1;
        IdentityAssembly evmac;
        std::vector<uint8_t> emaid0_cache;
        std::vector<uint8_t> emaid1_cache;
        std::chrono::steady_clock::time_point emaid0_rx{};
        std::chrono::steady_clock::time_point emaid1_rx{};
        RfidAssembly rfid;
        uint64_t tx_failures{0};
        uint64_t tx_errors_recent{0};
        std::chrono::steady_clock::time_point last_tx_warn{};
        std::chrono::steady_clock::time_point last_tx_ok{};
        std::chrono::steady_clock::time_point tx_quiet_until{};
        std::chrono::steady_clock::time_point last_tx_quiet_log{};
        uint32_t tx_failure_streak{0};

        // PLC CAN stats (low-overhead counters used for periodic bandwidth/backpressure logs).
        uint64_t tx_fast_ok{0};
        uint64_t tx_fast_fail{0};
        uint64_t tx_slow_ok{0};
        uint64_t tx_slow_fail{0};
        uint64_t tx_errno_enobufs{0};
        uint64_t tx_errno_eagain{0};
        uint64_t tx_errno_other{0};
        std::array<uint64_t, 4> tx_backpressure_level_hits{};
        uint64_t rx_tlm_v3{0};
    };

    ChargerConfig cfg_;
    bool use_crc_{true};
    int telemetry_timeout_ms_{2000};
    AutochargeIdSource autocharge_source_{AutochargeIdSource::Evmac};
    std::map<std::int32_t, PlcState> connectors_;
    std::map<std::string, int> sockets_;
    std::mutex sockets_mutex_;
    std::thread rx_thread_;
    std::thread tx_thread_;
    std::atomic<bool> running_{false};
    bool init_ok_{false};
    int connection_timeout_s_{0};
    int tx_limits_base_ms_{500};
    int tx_present_base_ms_{100};
    int tx_present_change_active_ms_{60};
    int tx_present_change_idle_ms_{250};
    int tx_present_idle_ms_{500};
    std::atomic<int> backpressure_level_{0};
    std::atomic<uint64_t> backpressure_until_ms_{0};
    std::atomic<uint64_t> last_backpressure_log_ms_{0};
    std::chrono::steady_clock::time_point started_at_{};
    mutable std::mutex state_mutex_;
    std::mutex token_mutex_;
    std::vector<AuthToken> pending_tokens_;

    struct IfaceStatsSnapshot {
        std::chrono::steady_clock::time_point last_log{};
        uint64_t tx_fast{0};
        uint64_t tx_slow{0};
        uint64_t rx_tlm_v3{0};
        uint64_t tx_errno_enobufs{0};
        uint64_t tx_errno_eagain{0};
        uint64_t tx_errno_other{0};
        std::array<uint64_t, 4> tx_backpressure_level_hits{};
    };
    std::map<std::string, IfaceStatsSnapshot> iface_stats_;
    std::chrono::steady_clock::time_point last_can_stats_check_{};

    struct TxRetryState {
        std::chrono::steady_clock::time_point last_err_log{};
        std::chrono::steady_clock::time_point last_reopen{};
        int enobufs_count{0};
    };
    std::mutex tx_retry_mutex_;
    std::map<std::string, TxRetryState> tx_retry_state_;

    void collect_can_stats_lines(std::chrono::steady_clock::time_point now, std::vector<std::string>& out);

    bool open_socket_for_iface(const std::string& iface);
    bool send_frame_raw(const std::string& iface,
                        int plc_id,
                        uint32_t can_id,
                        const std::array<uint8_t, 8>& data,
                        TxPriority pri,
                        int* out_errno = nullptr);
    void rx_loop();
    void tx_loop();
    void handle_frame(const std::string& iface, uint32_t can_id, const uint8_t data[8]);
    PlcState* find_state_by_plc(uint8_t plc_id, const std::string* iface = nullptr);
    std::int32_t connector_from_plc(uint8_t plc_id) const;
    void update_fast_tx(PlcState& st,
                        std::chrono::steady_clock::time_point now,
                        int backoff_factor,
                        std::unique_lock<std::mutex>& state_lock);
    void update_slow_tx(PlcState& st,
                        std::chrono::steady_clock::time_point now,
                        int backoff_factor,
                        std::unique_lock<std::mutex>& state_lock);
    bool set_relay_command(PlcState& st, bool gun_on, uint8_t module_mask, bool force_off);
    void set_lock_command(PlcState& st, bool lock);
    void emit_autocharge_token(PlcState& st, const std::string& id_token,
                               std::chrono::steady_clock::time_point now);
    void maybe_emit_emaid(PlcState& st, std::chrono::steady_clock::time_point now);
    static bool assemble_identity_segment(IdentityAssembly& asmbl,
                                          const can_contract::IdentitySegment& seg,
                                          std::chrono::steady_clock::time_point now,
                                          std::vector<uint8_t>& out);
    static bool assemble_rfid_segment(RfidAssembly& asmbl,
                                      const can_contract::RfidEventSegment& seg,
                                      std::chrono::steady_clock::time_point now,
                                      std::vector<uint8_t>& out);
    static uint16_t clamp_to_0p5(double v);
    static uint16_t clamp_to_0p5k(double kw);
    static uint16_t clamp_to_0p1_current(double a);
    static uint16_t clamp_to_0p2_current(double a);
    int compute_interval_ms(int base_ms, int min_ms, int max_ms, int backoff_factor) const;
    int backpressure_factor(uint64_t now_ms);
    void note_tx_backpressure(bool severe);
};

} // namespace charger
