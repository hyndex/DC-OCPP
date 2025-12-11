// SPDX-License-Identifier: Apache-2.0
#include "hardware_sim.hpp"

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <everest/logging.hpp>
#include <ocpp/v16/ocpp_enums.hpp>

namespace {

std::string bundle_logs(const std::string& prefix) {
    const auto ts = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    const std::string dir = "logs";
    std::filesystem::create_directories(dir);
    const std::string fname = dir + "/" + prefix + "_" + std::to_string(ts) + ".log";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    out << "Log bundle generated at " << ts << "\n";
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().string() == fname) continue;
        out << "\n==== " << entry.path().filename().string() << " ====\n";
        std::ifstream in(entry.path());
        out << in.rdbuf();
    }
    return fname;
}

bool upload_file_to_target(const std::string& source, const std::string& target) {
    if (target.empty()) return false;
    if (target.rfind("http://", 0) == 0 || target.rfind("https://", 0) == 0) {
        const std::string cmd = "curl -sSf -T \"" + source + "\" \"" + target + "\"";
        return std::system(cmd.c_str()) == 0;
    }
    std::string path = target;
    const std::string prefix = "file://";
    if (path.rfind(prefix, 0) == 0) {
        path = path.substr(prefix.size());
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    std::filesystem::copy_file(source, path, std::filesystem::copy_options::overwrite_existing, ec);
    return !ec;
}

} // namespace

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
    if (!state.lock_engaged && state.config.require_lock) {
        return false;
    }
    state.enabled = true;
    return true;
}

bool SimulatedHardware::disable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    state.enabled = false;
    state.charging = false;
    state.lock_engaged = true;
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
    auto& state = get_state(connector);
    state.lock_engaged = false;
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
    const std::string fname = bundle_logs("diagnostics");
    {
        std::ofstream out(fname, std::ios::app);
        out << "Diagnostics request location: " << request.location << "\n";
    }
    bool upload_ok = true;
    if (!request.location.empty()) {
        upload_ok = upload_file_to_target(fname, request.location);
        if (!upload_ok) {
            EVLOG_warning << "Diagnostics upload to " << request.location << " failed";
        }
    }
    if (!upload_ok) {
        response.status = ocpp::v16::LogStatusEnumType::Rejected;
    }
    response.filename.emplace(fname);
    return response;
}

ocpp::v16::GetLogResponse SimulatedHardware::upload_logs(const ocpp::v16::GetLogRequest& request) {
    ocpp::v16::GetLogResponse response;
    response.status = ocpp::v16::LogStatusEnumType::Accepted;
    const std::string fname = bundle_logs("logs");
    {
        std::ofstream out(fname, std::ios::app);
        out << "Log upload target: " << request.log.remoteLocation << "\n";
    }
    bool upload_ok = true;
    if (!request.log.remoteLocation.empty()) {
        upload_ok = upload_file_to_target(fname, request.log.remoteLocation);
        if (!upload_ok) {
            EVLOG_warning << "Log upload to " << request.log.remoteLocation << " failed";
        }
    }
    if (!upload_ok) {
        response.status = ocpp::v16::LogStatusEnumType::Rejected;
    }
    response.filename.emplace(fname);
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
    const double scale = state.config.meter_scale;
    measurement.power_meter.energy_Wh_import.total =
        state.energy_Wh * scale + state.config.meter_offset_wh;
    measurement.power_meter.power_W.emplace();
    measurement.power_meter.power_W->total =
        static_cast<float>((state.charging ? state.target_power_w : 0.0) * scale);
    measurement.power_meter.current_A.emplace();
    measurement.power_meter.current_A->DC =
        state.charging ? std::optional<float>(static_cast<float>((state.target_power_w / 400.0) * scale))
                       : std::nullopt;
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
    st.hlc_stage = st.relay_closed ? 5 : (ev_connected ? 4 : 0);
    st.hlc_cable_check_ok = ev_connected;
    st.hlc_precharge_active = false;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = st.relay_closed || ev_connected;
    st.lock_engaged = state.lock_engaged || !state.config.require_lock;
    st.module_temp_c[0] = 40.0;
    st.module_temp_c[1] = 40.0;
    return st;
}

void SimulatedHardware::apply_power_command(const PowerCommand& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(cmd.connector);
    update_energy(state);
    const bool enable = cmd.module_count > 0 && cmd.gc_closed && cmd.mc_closed;
    state.enabled = enable;
    state.charging = enable;
    double target_power_w = cmd.power_kw * 1000.0;
    if (cmd.current_limit_a > 0.0 && cmd.voltage_set_v > 0.0) {
        const double current_limited = cmd.current_limit_a * cmd.voltage_set_v;
        if (target_power_w <= 0.0) {
            target_power_w = current_limited;
        } else {
            target_power_w = std::min(target_power_w, current_limited);
        }
    }
    if (target_power_w <= 0.0 && enable) {
        target_power_w = state.target_power_w > 0.0 ? state.target_power_w : state.config.max_power_w;
    }
    if (state.config.max_power_w > 0.0 && target_power_w > 0.0) {
        target_power_w = std::min(target_power_w, state.config.max_power_w);
    }
    if (!enable) {
        target_power_w = 0.0;
    }
    state.target_power_w = std::max(0.0, target_power_w);
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
