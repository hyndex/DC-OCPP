// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"
#include "can_contract.hpp"

#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <memory>

namespace charger {

class SimulatedHardware;

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
    struct PlcState {
        ConnectorConfig cfg;
        int plc_id{0};
        std::string iface;
        std::atomic<uint8_t> seq{0};
        bool authorized{false};
        bool sys_enable{false};
        bool output_enabled{false};
        bool regulating{false};
        uint8_t fault_bits{0};
        double present_voltage_v{0.0};
        double present_current_a{0.0};
        double present_power_kw{0.0};
        double energy_kwh{0.0};
        double freq_hz{0.0};
        can_contract::RelayStatus last_relay{};
        can_contract::SafetyStatus last_safety{};
        can_contract::MeterReading last_meter{};
        std::chrono::steady_clock::time_point last_status_rx{};
        std::chrono::steady_clock::time_point last_meter_rx{};
        std::chrono::steady_clock::time_point last_limits_tx{};
        std::chrono::steady_clock::time_point last_present_tx{};
        EvseLimits limits{};
    };

    ChargerConfig cfg_;
    bool use_crc_{true};
    int telemetry_timeout_ms_{2000};
    std::map<std::int32_t, PlcState> connectors_;
    std::map<std::string, int> sockets_;
    std::thread rx_thread_;
    std::thread tx_thread_;
    std::atomic<bool> running_{false};
    bool init_ok_{false};
    int connection_timeout_s_{0};

    // Delegate uploads/firmware handling to the simulation backend to reuse tested code-paths.
    std::unique_ptr<SimulatedHardware> diag_helper_;

    bool open_socket_for_iface(const std::string& iface);
    bool send_frame(const PlcState& st, uint32_t can_id, const std::array<uint8_t, 8>& data);
    void rx_loop();
    void tx_loop();
    void handle_frame(uint32_t can_id, const uint8_t data[8]);
    PlcState* find_state_by_plc(uint8_t plc_id);
    std::int32_t connector_from_plc(uint8_t plc_id) const;
    void update_limits_tx(PlcState& st, std::chrono::steady_clock::time_point now);
    void update_present_tx(PlcState& st, std::chrono::steady_clock::time_point now);
    static uint16_t clamp_to_0p1(double v);
    static uint16_t clamp_to_0p1k(double kw);
    static uint16_t clamp_to_0p1_current(double a);
};

} // namespace charger
