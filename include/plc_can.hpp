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
    void clear_faults(std::int32_t connector) override;
    std::vector<AuthToken> poll_auth_tokens() override;
    bool supports_cross_slot_islands() const override;

private:
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
        std::chrono::steady_clock::time_point last_auth_tx{};
        bool authorized{false};
        bool auth_pending{false};
        bool hlc_enabled{false};
        bool pnc_blocked{false};
        bool hlc_enabled_sent{false};
        bool pnc_blocked_sent{false};
        bool sys_enable{false};
        bool output_enabled{false};
        bool regulating{false};
        uint8_t fault_bits{0};
        double present_voltage_v{0.0};
        double present_current_a{0.0};
        double present_power_kw{0.0};
        double ev_target_voltage_v{0.0};
        double ev_target_current_a{0.0};
        double ev_present_voltage_v{0.0};
        double ev_present_current_a{0.0};
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
        std::chrono::steady_clock::time_point last_ev_targets_rx{};
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
        bool lock_engaged{false};
        bool lock_engaged_valid{false};
        bool lock_command{true};
        bool lock_command_set{false};
        uint8_t boot_feature_flags{0};
        bool meter_available{true};
        bool protocol_ok{false};
        bool protocol_verified{false};
        bool protocol_sent{false};
        std::chrono::steady_clock::time_point last_protocol_tx{};
        std::chrono::steady_clock::time_point last_protocol_ack{};
        std::chrono::steady_clock::time_point last_protocol_warn{};
        std::chrono::steady_clock::time_point last_hlc_enable_tx{};
        std::chrono::steady_clock::time_point last_pnc_block_tx{};
        // Track CAN-level protocol health.
        uint64_t relay_status_crc_fail_count{0};
        uint64_t safety_status_crc_fail_count{0};
        uint64_t config_ack_crc_fail_count{0};
        std::chrono::steady_clock::time_point last_relay_crc_warn{};
        std::chrono::steady_clock::time_point last_safety_crc_warn{};
        std::chrono::steady_clock::time_point last_config_ack_crc_warn{};
        // Debounce safety-related trips to avoid flapping on single-frame glitches.
        std::chrono::steady_clock::time_point safety_trip_since{};
        std::chrono::steady_clock::time_point estop_trip_since{};
        std::chrono::steady_clock::time_point earth_trip_since{};
        IdentityAssembly evccid;
        IdentityAssembly evemaid0;
        IdentityAssembly evemaid1;
        IdentityAssembly evmac;
        std::vector<uint8_t> emaid0_cache;
        std::vector<uint8_t> emaid1_cache;
        std::chrono::steady_clock::time_point emaid0_rx{};
        std::chrono::steady_clock::time_point emaid1_rx{};
        RfidAssembly rfid;
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
    int tx_limits_ms_{500};
    int tx_present_ms_{100};
    std::chrono::steady_clock::time_point started_at_{};
    mutable std::mutex state_mutex_;
    std::mutex token_mutex_;
    std::vector<AuthToken> pending_tokens_;


    bool open_socket_for_iface(const std::string& iface);
    bool send_frame(const PlcState& st, uint32_t can_id, const std::array<uint8_t, 8>& data);
    void rx_loop();
    void tx_loop();
    void handle_frame(uint32_t can_id, const uint8_t data[8]);
    PlcState* find_state_by_plc(uint8_t plc_id);
    std::int32_t connector_from_plc(uint8_t plc_id) const;
    void update_limits_tx(PlcState& st, std::chrono::steady_clock::time_point now);
    void update_present_tx(PlcState& st, std::chrono::steady_clock::time_point now);
    void update_relay_tx(PlcState& st, std::chrono::steady_clock::time_point now);
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
    static uint16_t clamp_to_0p1(double v);
    static uint16_t clamp_to_0p1k(double kw);
    static uint16_t clamp_to_0p1_current(double a);
};

} // namespace charger
