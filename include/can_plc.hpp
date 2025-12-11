// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"

#include <atomic>
#include <cstdint>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <thread>

#ifdef __linux__
#include <linux/can.h>
#endif

namespace charger {

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
    bool valid{false};
    std::chrono::steady_clock::time_point last_rx{};
};

struct PlcStatus {
    bool relay_closed{false};
    bool sys_enabled{false};
    uint8_t last_cmd_seq{0};
    uint8_t relay_state_mask{0};
    uint8_t last_fault_reason{0};
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
    std::chrono::steady_clock::time_point last_meter_rx{};
    std::chrono::steady_clock::time_point last_relay_status{};
    std::chrono::steady_clock::time_point last_safety_status{};
    std::chrono::steady_clock::time_point last_any_rx{};
};

/// \brief PLC/CAN-backed implementation of HardwareInterface using the DBC in Ref/Basic/docs/CAN_DBC.dbc.
class PlcHardware : public HardwareInterface {
public:
    explicit PlcHardware(const ChargerConfig& cfg);
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
    void apply_power_command(const PowerCommand& cmd) override;

private:
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
        uint8_t module_mask{0x01}; // bit0: gun, bit1: module1, bit2: module2
    };

    std::map<std::int32_t, Node> nodes_; // keyed by connector id; one node per CAN iface (enforced in config)
    std::map<int, std::int32_t> plc_to_connector_;
    std::string iface_;
    int sock_{-1};
    std::atomic<bool> running_{false};
    std::thread rx_thread_;
    std::mutex mtx_;

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
#ifdef __linux__
    void handle_error_frame(const struct can_frame& frame);
#endif

    bool send_relay_command(Node& node, bool close, bool force_all_off = false);
    bool send_frame(uint32_t can_id, const uint8_t* data, size_t len);
    Node* find_node(std::int32_t connector);
    Node* find_node_by_plc(int plc_id);
    void update_cp_status(Node& node, char cp_state_char, uint8_t duty_pct, uint16_t mv_peak = 0,
                          uint16_t mv_min = 0, bool has_mv = false);
};

} // namespace charger
