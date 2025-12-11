// SPDX-License-Identifier: Apache-2.0
#include "hardware_sim.hpp"

#include <cmath>
#include <everest/logging.hpp>
#include <ocpp/v16/ocpp_enums.hpp>

namespace charger {

SimulatedHardware::SimulatedHardware(const std::vector<ConnectorConfig>& connectors) {
    const auto now = std::chrono::steady_clock::now();
    for (const auto& cfg : connectors) {
        ConnectorState state{};
        state.config = cfg;
        state.enabled = true;
        state.charging = false;
        state.reserved = false;
        state.target_power_w = cfg.max_power_w;
        state.energy_Wh = 0.0;
        state.last_update = now;
        connectors_.emplace(cfg.id, state);
    }
}

SimulatedHardware::ConnectorState& SimulatedHardware::get_state(std::int32_t connector) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) {
        throw std::runtime_error("Unknown connector id " + std::to_string(connector));
    }
    return it->second;
}

void SimulatedHardware::update_energy(ConnectorState& state) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_update).count() / 1000.0;
    if (elapsed > 0 && state.charging) {
        const auto energy_added_Wh = (state.target_power_w * elapsed) / 3600.0;
        state.energy_Wh += energy_added_Wh;
    }
    state.last_update = now;
}

bool SimulatedHardware::enable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    state.enabled = true;
    return true;
}

bool SimulatedHardware::disable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    state.enabled = false;
    state.charging = false;
    return true;
}

bool SimulatedHardware::pause_charging(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    update_energy(state);
    state.charging = false;
    return true;
}

bool SimulatedHardware::resume_charging(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    update_energy(state);
    state.charging = state.enabled;
    return state.enabled;
}

bool SimulatedHardware::stop_transaction(std::int32_t connector, ocpp::v16::Reason /*reason*/) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    update_energy(state);
    state.charging = false;
    state.reserved = false;
    state.reservation_id.reset();
    return true;
}

ocpp::v16::UnlockStatus SimulatedHardware::unlock(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    (void)get_state(connector);
    return ocpp::v16::UnlockStatus::Unlocked;
}

ocpp::v16::ReservationStatus SimulatedHardware::reserve(std::int32_t reservation_id, std::int32_t connector,
                                                        ocpp::DateTime /*expiry*/, const std::string& /*id_tag*/,
                                                        const std::optional<std::string>& /*parent_id*/) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    if (state.reserved) {
        return ocpp::v16::ReservationStatus::Rejected;
    }
    state.reserved = true;
    state.reservation_id = reservation_id;
    return ocpp::v16::ReservationStatus::Accepted;
}

bool SimulatedHardware::cancel_reservation(std::int32_t reservation_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [id, state] : connectors_) {
        if (state.reservation_id && state.reservation_id.value() == reservation_id) {
            state.reservation_id.reset();
            state.reserved = false;
            return true;
        }
    }
    return false;
}

ocpp::v16::GetLogResponse SimulatedHardware::upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) {
    ocpp::v16::GetLogResponse response;
    response.status = ocpp::v16::LogStatusEnumType::Accepted;
    response.filename.emplace(request.location);
    return response;
}

ocpp::v16::GetLogResponse SimulatedHardware::upload_logs(const ocpp::v16::GetLogRequest& request) {
    ocpp::v16::GetLogResponse response;
    response.status = ocpp::v16::LogStatusEnumType::Accepted;
    response.filename.emplace(request.log.remoteLocation);
    return response;
}

void SimulatedHardware::update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) {
    EVLOG_info << "Simulated firmware update from " << request.location << " scheduled at "
               << request.retrieveDate.to_rfc3339();
}

ocpp::v16::UpdateFirmwareStatusEnumType
SimulatedHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) {
    EVLOG_info << "Simulated signed firmware update from " << request.firmware.location;
    return ocpp::v16::UpdateFirmwareStatusEnumType::Accepted;
}

void SimulatedHardware::set_connection_timeout(std::int32_t seconds) {
    EVLOG_debug << "Connection timeout set to " << seconds << " seconds";
}

bool SimulatedHardware::is_reset_allowed(const ocpp::v16::ResetType& /*reset_type*/) {
    return true;
}

void SimulatedHardware::reset(const ocpp::v16::ResetType& reset_type) {
    EVLOG_info << "Simulated reset of type " << ocpp::v16::conversions::reset_type_to_string(reset_type);
}

void SimulatedHardware::on_remote_start_token(const std::string& id_token,
                                              const std::vector<std::int32_t>& referenced_connectors,
                                              bool prevalidated) {
    EVLOG_info << "Remote start token received: " << id_token << " (prevalidated=" << std::boolalpha << prevalidated
               << ") for connectors " << referenced_connectors.size();
}

ocpp::Measurement SimulatedHardware::sample_meter(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    update_energy(state);

    ocpp::Measurement measurement{};
    measurement.power_meter.timestamp = ocpp::DateTime().to_rfc3339();
    measurement.power_meter.energy_Wh_import.total = state.energy_Wh;
    measurement.power_meter.power_W.emplace();
    measurement.power_meter.power_W->total = static_cast<float>(state.charging ? state.target_power_w : 0.0);
    measurement.power_meter.current_A.emplace();
    measurement.power_meter.current_A->DC = state.charging ? std::optional<float>(static_cast<float>(state.target_power_w / 400.0)) : std::nullopt;
    measurement.power_meter.voltage_V.emplace();
    measurement.power_meter.voltage_V->DC = 400.0F;
    return measurement;
}

GunStatus SimulatedHardware::get_status(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    GunStatus st{};
    st.safety_ok = true;
    st.relay_closed = state.enabled && state.charging;
    st.meter_stale = false;
    const bool ev_connected = state.charging;
    st.plugged_in = ev_connected;
    st.cp_fault = false;
    st.cp_state = ev_connected ? 'C' : 'U';
    st.pilot_duty_pct = ev_connected ? 100.0 : 0.0;
    return st;
}

void SimulatedHardware::apply_power_allocation(std::int32_t connector, int modules) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    state.enabled = modules > 0;
    if (!state.enabled) {
        state.charging = false;
    }
}

} // namespace charger
