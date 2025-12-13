// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <array>

namespace charger {

/// \brief Simple in-process hardware stub so OCPP flows can be exercised without real EVSE hardware.
class SimulatedHardware : public HardwareInterface {
public:
    struct FaultOverride {
        bool estop{false};
        bool earth_fault{false};
        bool isolation_fault{false};
        bool overtemp_fault{false};
        bool overcurrent_fault{false};
        bool comm_fault{false};
        bool gc_welded{false};
        bool mc_welded{false};
        bool paused{false};
        bool disabled{false};
        std::optional<uint8_t> healthy_mask;
        std::optional<uint8_t> fault_mask;
        std::optional<double> connector_temp_c;
        std::array<double, 2> module_temp_c{{0.0, 0.0}};
    };

    explicit SimulatedHardware(const ChargerConfig& cfg);
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
    void set_authorization_state(std::int32_t connector, bool authorized) override;
    void apply_power_command(const PowerCommand& cmd) override;
    void apply_power_allocation(std::int32_t connector, int modules) override;
    std::vector<AuthToken> poll_auth_tokens() override;
    bool supports_cross_slot_islands() const override;

    // Simulation controls for tests/harnesses
    void set_fault_override(std::int32_t connector, const FaultOverride& fault);
    void clear_fault_override(std::int32_t connector);
    void set_paused(std::int32_t connector, bool paused);
    void set_disabled(std::int32_t connector, bool disabled);
    void set_plugged_in(std::int32_t connector, bool plugged, bool lock_engaged = true);
    void set_ev_power_request(std::int32_t connector, bool request);
    void inject_auth_token(const AuthToken& token);

private:
    struct ConnectorState {
        ConnectorConfig config;
        bool enabled{true};
        bool charging{false};
        bool reserved{false};
        std::optional<std::int32_t> reservation_id;
        double target_power_w{0.0};
        double energy_Wh{0.0};
        bool plugged_in{true};
        bool request_power{false};
        bool lock_engaged{true};
        bool authorized{false};
        std::chrono::steady_clock::time_point last_update;
    };

    std::mutex mutex_;
    std::map<std::int32_t, ConnectorState> connectors_;
    bool require_https_uploads_{true};
    std::size_t upload_max_bytes_{100 * 1024 * 1024};
    int upload_connect_timeout_s_{10};
    int upload_transfer_timeout_s_{60};
    bool upload_allow_file_targets_{true};
    std::map<std::int32_t, FaultOverride> fault_overrides_;
    std::vector<AuthToken> auth_events_;

    ConnectorState& get_state(std::int32_t connector);
    void update_energy(ConnectorState& state);
};

} // namespace charger
