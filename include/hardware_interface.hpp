// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <optional>
#include <cstdint>
#include <array>
#include <string>
#include <vector>

#include <ocpp/common/types.hpp>
#include <ocpp/v16/messages/GetDiagnostics.hpp>
#include <ocpp/v16/messages/GetLog.hpp>
#include <ocpp/v16/messages/SignedUpdateFirmware.hpp>
#include <ocpp/v16/messages/UpdateFirmware.hpp>

namespace charger {

enum class AuthTokenSource { RFID, Autocharge, RemoteStart };

enum class AuthorizationState {
    Unknown,
    Pending,
    Granted,
    Denied
};

struct AuthToken {
    std::string id_token;
    AuthTokenSource source{AuthTokenSource::RFID};
    int connector_hint{0}; // 0 => no preference
    bool prevalidated{false};
    std::chrono::steady_clock::time_point received_at{std::chrono::steady_clock::now()};
};

struct GunStatus {
    bool safety_ok{true};
    bool estop{false};
    bool earth_fault{false};
    bool comm_fault{false};
    bool isolation_fault{false};
    bool overtemp_fault{false};
    bool overcurrent_fault{false};
    bool relay_closed{false};
    bool meter_stale{false};
    bool plugged_in{false};
    bool cp_fault{false};
    bool lock_engaged{true};
    bool authorization_granted{false};
    char cp_state{'U'};
    uint8_t hlc_stage{0};
    bool hlc_cable_check_ok{false};
    bool hlc_precharge_active{false};
    bool hlc_charge_complete{false};
    bool hlc_power_ready{false};
    double pilot_duty_pct{0.0};
    std::optional<double> target_voltage_v;
    std::optional<double> target_current_a;
    std::optional<double> present_voltage_v;
    std::optional<double> present_current_a;
    std::optional<double> present_power_w;
    std::optional<double> evse_max_voltage_v;
    std::optional<double> evse_max_current_a;
    std::optional<double> evse_max_power_kw;
    uint8_t module_healthy_mask{0x00}; // bit0=module0, bit1=module1, etc. Slot-local ordering.
    uint8_t module_fault_mask{0x00};   // bitmask mirroring module_healthy_mask for detected faults/welds
    bool gc_welded{false};
    bool mc_welded{false};
    double connector_temp_c{0.0};
    std::array<double, 2> module_temp_c{{0.0, 0.0}};
    uint32_t evse_limit_ack_count{0};
    std::chrono::steady_clock::time_point last_evse_limit_ack{};
};

/// \brief Planner dispatch toward hardware (per connector).
struct PowerCommand {
    std::int32_t connector{0};
    int module_count{0};
    uint8_t module_mask{0}; // bit0 -> slot module[0], bit1 -> slot module[1], etc.
    bool gc_closed{false};
    bool mc_closed{false};
    double voltage_set_v{0.0};
    double current_limit_a{0.0};
    double power_kw{0.0};
};

struct EvseLimits {
    std::optional<double> max_voltage_v;
    std::optional<double> max_current_a;
    std::optional<double> max_power_kw;
};

/// \brief Abstract interface your EVSE controller should implement.
class HardwareInterface {
public:
    virtual ~HardwareInterface() = default;

    virtual bool enable(std::int32_t connector) = 0;
    virtual bool disable(std::int32_t connector) = 0;
    virtual bool pause_charging(std::int32_t connector) = 0;
    virtual bool resume_charging(std::int32_t connector) = 0;
    virtual bool stop_transaction(std::int32_t connector, ocpp::v16::Reason reason) = 0;
    virtual ocpp::v16::UnlockStatus unlock(std::int32_t connector) = 0;
    virtual ocpp::v16::ReservationStatus reserve(std::int32_t reservation_id, std::int32_t connector,
                                                 ocpp::DateTime expiry, const std::string& id_tag,
                                                 const std::optional<std::string>& parent_id) = 0;
    virtual bool cancel_reservation(std::int32_t reservation_id) = 0;

    virtual ocpp::v16::GetLogResponse upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) = 0;
    virtual ocpp::v16::GetLogResponse upload_logs(const ocpp::v16::GetLogRequest& request) = 0;
    virtual void update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) = 0;
    virtual ocpp::v16::UpdateFirmwareStatusEnumType
    update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) = 0;

    virtual void set_connection_timeout(std::int32_t seconds) = 0;
    virtual bool is_reset_allowed(const ocpp::v16::ResetType& reset_type) = 0;
    virtual void reset(const ocpp::v16::ResetType& reset_type) = 0;

    virtual void on_remote_start_token(const std::string& id_token,
                                       const std::vector<std::int32_t>& referenced_connectors, bool prevalidated) = 0;

    /// \brief Return the latest meter sample for the connector.
    virtual ocpp::Measurement sample_meter(std::int32_t connector) = 0;

    /// \brief Return latest safety/comm/meter state for the connector.
    virtual GunStatus get_status(std::int32_t connector) = 0;

    /// \brief Notify hardware/PLC that authorization has been granted or revoked for the connector.
    /// Implementations that do not need this can keep the default no-op.
    virtual void set_authorization_state(std::int32_t connector, bool authorized) {
        set_authorization_state(connector, authorized ? AuthorizationState::Granted : AuthorizationState::Denied);
    }

    /// \brief Extended authorization state hook that supports Pending/Denied semantics.
    /// Default implementation falls back to the legacy bool-based hook.
    virtual void set_authorization_state(std::int32_t connector, AuthorizationState state) {
        set_authorization_state(connector, state == AuthorizationState::Granted);
    }

    /// \brief Apply computed power allocation (modules, contactors, setpoints) for the connector.
    /// Default fallbacks map to the older apply_power_allocation + enable/disable calls.
    virtual void apply_power_command(const PowerCommand& cmd) {
        if (cmd.module_count <= 0 || !cmd.gc_closed) {
            apply_power_allocation(cmd.connector, 0);
            disable(cmd.connector);
            return;
        }
        apply_power_allocation(cmd.connector, cmd.module_count);
        enable(cmd.connector);
    }

    /// \brief Apply computed power allocation (module count / enable) for the connector.
    /// Implementations may no-op if unsupported.
    virtual void apply_power_allocation(std::int32_t connector, int modules) { (void)connector; (void)modules; }

    /// \brief Publish EVSE-side limits (what the EV should be told via PLC/ISO15118).
    /// Default is a no-op for hardware that does not support dynamic limit injection.
    virtual void set_evse_limits(std::int32_t connector, const EvseLimits& limits) {
        (void)connector;
        (void)limits;
    }

    /// \brief Drain any auth tokens (RFID/Autocharge/etc.) detected by the hardware since the last poll.
    /// Default implementation returns an empty list.
    virtual std::vector<AuthToken> poll_auth_tokens() { return {}; }

    /// \brief Capability flag: can this hardware switch modules across slots/islands.
    virtual bool supports_cross_slot_islands() const { return false; }
};

} // namespace charger
