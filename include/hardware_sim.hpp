// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"

#include <chrono>
#include <map>
#include <mutex>

namespace charger {

/// \brief Simple in-process hardware stub so OCPP flows can be exercised without real EVSE hardware.
class SimulatedHardware : public HardwareInterface {
public:
    explicit SimulatedHardware(const std::vector<ConnectorConfig>& connectors);
    ~SimulatedHardware() override = default;

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
    void update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) override;
    ocpp::v16::UpdateFirmwareStatusEnumType
    update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) override;
    void set_connection_timeout(std::int32_t seconds) override;
    bool is_reset_allowed(const ocpp::v16::ResetType& reset_type) override;
    void reset(const ocpp::v16::ResetType& reset_type) override;
    void on_remote_start_token(const std::string& id_token, const std::vector<std::int32_t>& referenced_connectors,
                               bool prevalidated) override;
    ocpp::Measurement sample_meter(std::int32_t connector) override;
    GunStatus get_status(std::int32_t connector) override;
    void apply_power_allocation(std::int32_t connector, int modules) override;

private:
    struct ConnectorState {
        ConnectorConfig config;
        bool enabled{true};
        bool charging{false};
        bool reserved{false};
        std::optional<std::int32_t> reservation_id;
        double target_power_w{0.0};
        double energy_Wh{0.0};
        std::chrono::steady_clock::time_point last_update;
    };

    std::mutex mutex_;
    std::map<std::int32_t, ConnectorState> connectors_;

    ConnectorState& get_state(std::int32_t connector);
    void update_energy(ConnectorState& state);
};

} // namespace charger
