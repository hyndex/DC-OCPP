// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"

#include <atomic>
#include <algorithm>
#include <cstdint>
#include <chrono>
#include <cstddef>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <vector>

#ifdef __linux__
#include <linux/can.h>
#else
constexpr uint32_t CAN_EFF_FLAG = 0x80000000U;
constexpr uint32_t CAN_EFF_MASK = 0x1FFFFFFFU;
#endif

namespace charger {

constexpr uint32_t PLC_TX_MASK = 0xFFFFFFF0;

struct PlcFilterSpec {
    uint32_t id{0};
    uint32_t mask{0};
};

inline uint8_t plc_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

inline PlcFilterSpec make_plc_rx_filter(uint32_t base, int plc_id) {
    PlcFilterSpec spec{};
    const uint32_t base_mask = (PLC_TX_MASK | 0x0F | CAN_EFF_FLAG) & (CAN_EFF_MASK | CAN_EFF_FLAG);
    spec.mask = base_mask;
    spec.id = (base & PLC_TX_MASK) | static_cast<uint32_t>(plc_id & 0x0F) | CAN_EFF_FLAG;
    return spec;
}

struct PlcSafetyStatus {
    bool safety_ok{false};
    bool estop{false};
    bool earth_fault{false};
    bool comm_fault{false};
    bool remote_force_off{false};
};

struct PlcMeterSnapshot {
    double voltage_v{0.0};
    double current_a{0.0};
    double power_w{0.0};
    double energy_Wh{0.0};
    double freq_hz{0.0};
    bool ok{false};
    bool stale{true};
};

struct PlcCpStatus {
    char state_char{'U'};
    double duty_pct{0.0};
    uint16_t mv_peak{0};
    uint16_t mv_min{0};
    uint16_t mv_robust{0};
    bool valid{false};
    std::chrono::steady_clock::time_point last_rx{};
};

struct PlcStatus {
    bool relay_closed{false};
    bool sys_enabled{false};
    uint8_t last_cmd_seq{0};
    uint8_t relay_state_mask{0};
    uint8_t last_fault_reason{0};
    uint8_t hlc_stage{0};
    bool hlc_charge_complete{false};
    bool hlc_precharge_active{false};
    bool hlc_cable_check_ok{false};
    bool hlc_power_ready{false};
    PlcSafetyStatus safety;
    PlcSafetyStatus pending_safety;
    std::chrono::steady_clock::time_point pending_safety_since{};
    PlcMeterSnapshot meter;
    PlcCpStatus cp;
    double target_voltage_v{0.0};
    double target_current_a{0.0};
    double present_voltage_v{0.0};
    double present_current_a{0.0};
    double present_power_w{0.0};
    double max_voltage_v{0.0};
    double max_current_a{0.0};
    double max_power_kw{0.0};
    uint32_t limit_ack_count{0};
    std::chrono::steady_clock::time_point last_limit_ack{};
    std::chrono::steady_clock::time_point last_meter_rx{};
    std::chrono::steady_clock::time_point last_relay_status{};
    std::chrono::steady_clock::time_point last_safety_status{};
    std::chrono::steady_clock::time_point last_any_rx{};
    uint32_t session_epoch{0};
    bool auth_pending_flag{false};
    uint32_t rx_count{0};
    uint32_t tx_count{0};
    uint32_t uptime_s{0};
    uint8_t fw_major{0};
    uint8_t fw_minor{0};
    uint8_t fw_patch{0};
    uint8_t feature_flags{0};
    uint8_t error_code{0};
};

/// \brief PLC/CAN-backed implementation of HardwareInterface using the DBC in Ref/Basic/docs/CAN_DBC.dbc.
class PlcHardware : public HardwareInterface {
public:
    explicit PlcHardware(const ChargerConfig& cfg, bool open_can = true);
    ~PlcHardware() override;

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
    void apply_power_allocation(std::int32_t connector, int modules) override;

    ocpp::v16::GetLogResponse upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) override;
    ocpp::v16::GetLogResponse upload_logs(const ocpp::v16::GetLogRequest& request) override;
    void update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) override;
    ocpp::v16::UpdateFirmwareStatusEnumType
    update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) override;

    void set_connection_timeout(std::int32_t seconds) override;
    bool is_reset_allowed(const ocpp::v16::ResetType& reset_type) override;
    void reset(const ocpp::v16::ResetType& reset_type) override;

    void on_remote_start_token(const std::string& id_token,
                               const std::vector<std::int32_t>& referenced_connectors, bool prevalidated) override;

    ocpp::Measurement sample_meter(std::int32_t connector) override;
    GunStatus get_status(std::int32_t connector) override;
    void set_authorization_state(std::int32_t connector, bool authorized) override;
    void set_authorization_state(std::int32_t connector, AuthorizationState state) override;
    void apply_power_command(const PowerCommand& cmd) override;
    std::vector<AuthToken> poll_auth_tokens() override;
    bool supports_cross_slot_islands() const override;
    void set_evse_limits(std::int32_t connector, const EvseLimits& limits) override;
    void publish_evse_present(std::int32_t connector, double voltage_v, double current_a,
                              double power_kw, bool output_enabled, bool regulating) override;
    void publish_fault_state(std::int32_t connector, uint8_t fault_bits) override;
    /// \brief Inject a CAN frame directly (bypasses socket RX) for replay/testing.
    void ingest_can_frame(uint32_t can_id, const uint8_t* data, size_t len);
    void ingest_can_frame(uint32_t can_id, const std::vector<uint8_t>& data) {
        ingest_can_frame(can_id, data.data(), data.size());
    }

private:
    struct SegmentBuffer {
        uint8_t total_len{0};
        uint8_t expected_segments{0};
        std::vector<uint8_t> data;
        std::vector<bool> received;
        std::chrono::steady_clock::time_point last_rx{};
        void reset(uint8_t len, uint8_t segments, const std::chrono::steady_clock::time_point& now) {
            total_len = len;
            expected_segments = std::max<uint8_t>(static_cast<uint8_t>(1), segments);
            data.assign(total_len, 0);
            received.assign(expected_segments, false);
            last_rx = now;
        }
        bool complete() const {
            return !data.empty() && expected_segments > 0 &&
                   received.size() >= expected_segments &&
                   std::all_of(received.begin(), received.begin() + expected_segments, [](bool v) { return v; });
        }
    };

    struct Node {
        ConnectorConfig cfg;
        PlcStatus status;
        uint8_t cmd_seq{0};
        uint8_t expected_cmd_seq{0};
        bool awaiting_ack{false};
        std::chrono::steady_clock::time_point last_cmd_sent{};
        int retry_count{0};
        bool last_cmd_close{false};
        bool last_force_all_off{false};
        bool mc_closed_cmd{true};
        bool gc_closed_cmd{false};
        uint8_t module_mask{0x00}; // bit0: gun, bit1: module1, bit2: module2
        double energy_fallback_Wh{0.0};
        std::chrono::steady_clock::time_point last_energy_update{};
        bool meter_fallback_active{false};
        bool lock_engaged{true};
        bool lock_feedback_engaged{true};
        bool crc_mode_mismatch{false};
        bool crc_mode_mismatch_logged{false};
        bool crc_fault_logged{false};
        bool bus_off_logged{false};
        bool authorization_granted{false};
        uint8_t gcmc_cmd_seq{0};
        double last_limit_voltage_v{0.0};
        double last_limit_current_a{0.0};
        double last_limit_power_kw{0.0};
        uint64_t limit_tx_count{0};
        uint64_t limit_tx_fail{0};
        AuthorizationState authorization_state{AuthorizationState::Unknown};
        std::chrono::steady_clock::time_point last_auth_push{};
        uint64_t auth_push_count{0};
        uint8_t fault_bits{0};
        std::chrono::steady_clock::time_point last_fault_update{};
        std::chrono::steady_clock::time_point last_evse_present_tx{};
        std::chrono::steady_clock::time_point last_evse_limits_tx{};
        uint64_t present_stale_events{0};
        uint64_t limit_stale_events{0};
        uint64_t relay_conflict_count{0};
        std::map<uint8_t, SegmentBuffer> rfid_events;
        SegmentBuffer evccid;
        SegmentBuffer emaid0;
        SegmentBuffer emaid1;
        SegmentBuffer evmac;
    };

    std::map<std::int32_t, Node> nodes_; // keyed by connector id; one node per CAN iface (enforced in config)
    std::map<int, std::int32_t> plc_to_connector_;
    std::string iface_;
    bool use_crc8_{false};
    bool module_relays_enabled_{true};
    bool gun_relay_owned_by_plc_{true};
    int present_warn_ms_{1000};
    int limits_warn_ms_{1500};
    bool require_https_uploads_{true};
    std::size_t upload_max_bytes_{100 * 1024 * 1024};
    int upload_connect_timeout_s_{10};
    int upload_transfer_timeout_s_{60};
    bool upload_allow_file_targets_{true};
    int sock_{-1};
    std::atomic<bool> restart_requested_{false};
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
    std::mutex mtx_;
    std::vector<AuthToken> auth_events_;
    std::set<uint32_t> unknown_can_ids_;
    bool module_relays_disabled_logged_{false};
    bool cadence_warn_logged_{false};

    bool open_socket();
    void restart_can();
    void rx_loop();
    void handle_frame(uint32_t can_id, const uint8_t* data, size_t len);
    void handle_relay_status(Node& node, const uint8_t* data, size_t len);
    void handle_safety_status(Node& node, const uint8_t* data, size_t len);
    void handle_meter(Node& node, const uint8_t* data, size_t len);
    void handle_cp_levels(Node& node, const uint8_t* data, size_t len);
    void handle_charging_session(Node& node, const uint8_t* data, size_t len);
    void handle_evac_control(Node& node, const uint8_t* data, size_t len);
    void handle_evdc_targets(Node& node, const uint8_t* data, size_t len);
    void handle_evdc_limits(Node& node, const uint8_t* data, size_t len);
    void handle_ev_status_display(Node& node, const uint8_t* data, size_t len);
    void handle_charge_info(Node& node, const uint8_t* data, size_t len);
    void handle_gcmc_status(Node& node, const uint8_t* data, size_t len);
    void handle_hw_status(Node& node, const uint8_t* data, size_t len);
    void handle_debug_info(Node& node, const uint8_t* data, size_t len);
    void handle_boot_config(Node& node, const uint8_t* data, size_t len);
    void handle_rtev_log(Node& node, const uint8_t* data, size_t len);
    void handle_rtt_log(Node& node, const uint8_t* data, size_t len);
    void handle_software_info(Node& node, const uint8_t* data, size_t len);
    void handle_error_codes(Node& node, const uint8_t* data, size_t len);
    void handle_hw_config(Node& node, const uint8_t* data, size_t len);
    void handle_rfid_event(Node& node, const uint8_t* data, size_t len);
    void handle_identity_segment(Node& node, SegmentBuffer& buffer, AuthTokenSource source, const uint8_t* data,
                                 size_t len);
    void prune_segment_buffers(Node& node, const std::chrono::steady_clock::time_point& now);
    void send_evse_limits(Node& node, const EvseLimits& limits);
    void send_evse_present(Node& node, double voltage_v, double current_a, double power_kw,
                           bool output_enabled, bool regulating);
    void refresh_authorization_locked(Node& node, const std::chrono::steady_clock::time_point& now, bool force);
    void update_fault_bits(Node& node, uint8_t fault_bits, const std::chrono::steady_clock::time_point& now);
    bool send_config_command(Node& node, uint8_t param_id, uint32_t value);
#ifdef __linux__
    void handle_error_frame(const struct can_frame& frame);
#endif

    bool send_relay_command(Node& node, bool close, bool force_all_off = false);
    bool send_gcmc_command(Node& node, uint8_t seq, bool close, bool force_all_off);
    bool send_frame(uint32_t can_id, const uint8_t* data, size_t len);
    Node* find_node(std::int32_t connector);
    Node* find_node_by_plc(int plc_id);
    void update_cp_status(Node& node, char cp_state_char, uint8_t duty_pct, uint16_t mv_peak = 0,
                          uint16_t mv_min = 0, bool has_mv = false, uint16_t mv_robust = 0,
                          bool has_robust = false);
    void update_hlc_state(Node& node, uint8_t stage, uint8_t flags);
    bool derive_lock_input(const Node& node, uint8_t sw_mask_byte, bool relay_status_frame) const;
    bool derive_hlc_power_ready(const PlcStatus& status, bool authorized) const;
};

} // namespace charger
