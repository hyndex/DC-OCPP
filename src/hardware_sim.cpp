// SPDX-License-Identifier: Apache-2.0
#include "hardware_sim.hpp"

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <mutex>
#include <utility>
#include <curl/curl.h>
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

bool is_http_url(const std::string& target) {
    return target.rfind("http://", 0) == 0 || target.rfind("https://", 0) == 0;
}

bool upload_via_curl(const std::string& source, const std::string& url, bool require_https,
                     int connect_timeout_s, int transfer_timeout_s) {
    if (url.empty()) return false;
    if (require_https && url.rfind("https://", 0) != 0) {
        EVLOG_warning << "Rejecting non-HTTPS upload target " << url;
        return false;
    }
    static std::once_flag curl_once;
    std::call_once(curl_once, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    std::FILE* file = std::fopen(source.c_str(), "rb");
    if (!file) {
        curl_easy_cleanup(curl);
        return false;
    }
    const auto size = std::filesystem::file_size(source);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_READDATA, file);
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, static_cast<curl_off_t>(size));
    curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s > 0 ? connect_timeout_s : 10);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, transfer_timeout_s > 0 ? transfer_timeout_s : 60);

    const CURLcode res = curl_easy_perform(curl);
    std::fclose(file);
    curl_easy_cleanup(curl);
    return res == CURLE_OK;
}

bool upload_file_to_target(const std::string& source,
                           const std::string& target,
                           bool require_https,
                           std::size_t max_bytes,
                           int connect_timeout_s,
                           int transfer_timeout_s,
                           bool allow_file_targets) {
    if (target.empty()) return false;

    std::error_code ec;
    const auto source_size = std::filesystem::file_size(source, ec);
    if (ec || source_size > max_bytes) {
        EVLOG_warning << "Rejecting upload: size exceeds limit or unreadable (" << source << ")";
        return false;
    }

    if (is_http_url(target)) {
        return upload_via_curl(source, target, require_https, connect_timeout_s, transfer_timeout_s);
    }
    if (!allow_file_targets) {
        EVLOG_warning << "File uploads disabled; rejecting target " << target;
        return false;
    }
    std::string path = target;
    const std::string prefix = "file://";
    if (path.rfind(prefix, 0) == 0) {
        path = path.substr(prefix.size());
    }
    const auto canonical_parent = std::filesystem::weakly_canonical(std::filesystem::path(path).parent_path(), ec);
    if (ec) {
        EVLOG_warning << "Rejecting upload: invalid path " << path;
        return false;
    }
    std::filesystem::create_directories(canonical_parent, ec);
    const auto dest_path = std::filesystem::weakly_canonical(path, ec);
    if (ec) {
        EVLOG_warning << "Rejecting upload: invalid destination " << path;
        return false;
    }
    std::filesystem::copy_file(source, dest_path, std::filesystem::copy_options::overwrite_existing, ec);
    return !ec;
}

bool download_via_curl(const std::string& url, const std::string& destination, bool require_https,
                       int connect_timeout_s, int transfer_timeout_s) {
    if (url.empty()) return false;
    if (require_https && url.rfind("https://", 0) != 0) {
        EVLOG_warning << "Rejecting non-HTTPS firmware URL " << url;
        return false;
    }
    static std::once_flag curl_once;
    std::call_once(curl_once, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });
    CURL* curl = curl_easy_init();
    if (!curl) return false;

    std::FILE* file = std::fopen(destination.c_str(), "wb");
    if (!file) {
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, nullptr);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, file);
    curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout_s > 0 ? connect_timeout_s : 10);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, transfer_timeout_s > 0 ? transfer_timeout_s : 60);

    const CURLcode res = curl_easy_perform(curl);
    std::fclose(file);
    curl_easy_cleanup(curl);
    return res == CURLE_OK;
}

bool fetch_firmware(const std::string& location,
                    std::size_t max_bytes,
                    bool require_https,
                    int connect_timeout_s,
                    int transfer_timeout_s,
                    bool allow_file_targets) {
    if (location.empty()) return false;
    std::error_code ec;
    const std::string dest = "data/firmware/download.bin";
    std::filesystem::create_directories(std::filesystem::path(dest).parent_path(), ec);
    if (is_http_url(location)) {
        if (!download_via_curl(location, dest, require_https, connect_timeout_s, transfer_timeout_s)) {
            return false;
        }
    } else {
        if (!allow_file_targets) return false;
        std::string src = location;
        const std::string prefix = "file://";
        if (src.rfind(prefix, 0) == 0) {
            src = src.substr(prefix.size());
        }
        std::filesystem::copy_file(src, dest, std::filesystem::copy_options::overwrite_existing, ec);
        if (ec) return false;
    }
    const auto size = std::filesystem::file_size(dest, ec);
    if (ec || size > max_bytes) {
        EVLOG_warning << "Firmware download failed size check";
        std::filesystem::remove(dest, ec);
        return false;
    }
    return true;
}

} // namespace

namespace charger {

SimulatedHardware::SimulatedHardware(const ChargerConfig& cfg) {
    require_https_uploads_ = cfg.require_https_uploads;
    upload_max_bytes_ = cfg.upload_max_bytes;
    upload_connect_timeout_s_ = cfg.upload_connect_timeout_s;
    upload_transfer_timeout_s_ = cfg.upload_transfer_timeout_s;
    upload_allow_file_targets_ = cfg.upload_allow_file_targets;
    const auto now = std::chrono::steady_clock::now();
    for (const auto& conn_cfg : cfg.connectors) {
        ConnectorState state{};
        state.config = conn_cfg;
        state.enabled = true;
        state.charging = false;
        state.reserved = false;
        state.target_power_w = conn_cfg.max_power_w;
        state.energy_Wh = 0.0;
        state.plugged_in = false;
        state.request_power = false;
        state.authorized = false;
        state.auth_state = AuthorizationState::Unknown;
        state.last_update = now;
        connectors_.emplace(conn_cfg.id, state);
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
    state.request_power = false;
    state.authorized = false;
    state.auth_state = AuthorizationState::Denied;
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
    state.request_power = false;
    state.reserved = false;
    state.reservation_id.reset();
    state.authorized = false;
    state.auth_state = AuthorizationState::Unknown;
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
        upload_ok = upload_file_to_target(fname, request.location, require_https_uploads_,
                                          upload_max_bytes_, upload_connect_timeout_s_,
                                          upload_transfer_timeout_s_, upload_allow_file_targets_);
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
    if (!request.log.remoteLocation.get().empty()) {
        upload_ok = upload_file_to_target(fname, request.log.remoteLocation, require_https_uploads_,
                                          upload_max_bytes_, upload_connect_timeout_s_,
                                          upload_transfer_timeout_s_, upload_allow_file_targets_);
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
    const bool ok = fetch_firmware(request.location, upload_max_bytes_, require_https_uploads_,
                                   upload_connect_timeout_s_, upload_transfer_timeout_s_, upload_allow_file_targets_);
    if (!ok) {
        EVLOG_warning << "Simulated firmware download failed for " << request.location;
    } else {
        EVLOG_info << "Simulated firmware downloaded from " << request.location << " scheduled at "
                   << request.retrieveDate.to_rfc3339();
    }
}

ocpp::v16::UpdateFirmwareStatusEnumType
SimulatedHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) {
    const bool ok = fetch_firmware(request.firmware.location, upload_max_bytes_, require_https_uploads_,
                                   upload_connect_timeout_s_, upload_transfer_timeout_s_, upload_allow_file_targets_);
    return ok ? ocpp::v16::UpdateFirmwareStatusEnumType::Accepted
              : ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
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
    const auto fault_it = fault_overrides_.find(connector);
    const FaultOverride* fault = fault_it != fault_overrides_.end() ? &fault_it->second : nullptr;
    GunStatus st{};
    st.safety_ok = true;
    st.relay_closed = state.enabled && state.charging;
    st.meter_stale = false;
    const bool ev_connected = state.plugged_in;
    const bool ev_requests_power = state.request_power || state.charging;
    st.plugged_in = ev_connected;
    st.cp_fault = false;
    st.cp_state = ev_connected ? (ev_requests_power ? 'C' : 'B') : 'U';
    st.pilot_duty_pct = ev_connected ? 100.0 : 0.0;
    st.hlc_stage = st.relay_closed ? 5 : (ev_connected ? 4 : 0);
    st.hlc_cable_check_ok = ev_connected;
    st.hlc_precharge_active = false;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = state.auth_state == AuthorizationState::Granted && (st.relay_closed || ev_requests_power);
    st.lock_engaged = state.lock_engaged || !state.config.require_lock;
    st.authorization_granted = state.auth_state == AuthorizationState::Granted;
    st.module_healthy_mask = fault && fault->healthy_mask ? *fault->healthy_mask : 0x03;
    st.module_fault_mask = fault && fault->fault_mask ? *fault->fault_mask : 0x00;
    st.gc_welded = fault ? fault->gc_welded : false;
    st.mc_welded = fault ? fault->mc_welded : false;
    const double present_voltage = 400.0;
    const double present_current = state.charging ? state.target_power_w / present_voltage : 0.0;
    const double present_power_w = state.charging ? state.target_power_w : 0.0;
    st.present_voltage_v = present_voltage;
    st.present_current_a = present_current;
    st.present_power_w = present_power_w;
    st.target_voltage_v = present_voltage;
    st.target_current_a = present_current;
    const auto apply_limit = [](std::optional<double> primary, double fallback) -> std::optional<double> {
        if (primary && *primary > 0.0) return primary;
        if (fallback > 0.0) return std::optional<double>(fallback);
        return std::nullopt;
    };
    st.evse_max_power_kw = apply_limit(state.evse_limits.max_power_kw, state.config.max_power_w > 0.0
                                                                    ? state.config.max_power_w / 1000.0
                                                                    : 0.0);
    st.evse_max_voltage_v = apply_limit(state.evse_limits.max_voltage_v, state.config.max_voltage_v);
    st.evse_max_current_a = apply_limit(state.evse_limits.max_current_a, state.config.max_current_a);
    st.module_temp_c = fault ? fault->module_temp_c : std::array<double, 2>{{40.0, 40.0}};
    st.connector_temp_c = fault && fault->connector_temp_c ? *fault->connector_temp_c : 40.0;
    st.last_telemetry = std::chrono::steady_clock::now();
    st.estop = fault ? fault->estop : false;
    st.earth_fault = fault ? fault->earth_fault : false;
    st.isolation_fault = fault ? fault->isolation_fault : false;
    st.overtemp_fault = fault ? fault->overtemp_fault : false;
    st.overcurrent_fault = fault ? fault->overcurrent_fault : false;
    st.comm_fault = fault ? fault->comm_fault : false;
    if (fault && fault->disabled) {
        st.safety_ok = false;
    }
    if (st.estop || st.earth_fault || st.isolation_fault || st.overtemp_fault || st.overcurrent_fault ||
        st.comm_fault) {
        st.safety_ok = false;
    }
    return st;
}

void SimulatedHardware::apply_power_command(const PowerCommand& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(cmd.connector);
    const auto fault_it = fault_overrides_.find(cmd.connector);
    const FaultOverride* fault = fault_it != fault_overrides_.end() ? &fault_it->second : nullptr;
    update_energy(state);
    const bool disabled = fault && fault->disabled;
    const bool paused = fault && fault->paused;
    const bool enable = cmd.module_count > 0 && cmd.gc_closed && !disabled &&
        state.auth_state == AuthorizationState::Granted;
    state.enabled = enable;
    state.charging = enable && !paused;
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
    if (!enable || paused) {
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

void SimulatedHardware::set_fault_override(std::int32_t connector, const FaultOverride& fault) {
    std::lock_guard<std::mutex> lock(mutex_);
    fault_overrides_[connector] = fault;
}

void SimulatedHardware::clear_fault_override(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mutex_);
    fault_overrides_.erase(connector);
}

void SimulatedHardware::set_paused(std::int32_t connector, bool paused) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& fo = fault_overrides_[connector];
    fo.paused = paused;
}

void SimulatedHardware::set_disabled(std::int32_t connector, bool disabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& fo = fault_overrides_[connector];
    fo.disabled = disabled;
}

void SimulatedHardware::set_authorization_state(std::int32_t connector, bool authorized) {
    set_authorization_state(connector, authorized ? AuthorizationState::Granted : AuthorizationState::Denied);
}

void SimulatedHardware::set_authorization_state(std::int32_t connector, AuthorizationState state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& st = get_state(connector);
    st.auth_state = state;
    st.authorized = (state == AuthorizationState::Granted);
    if (state != AuthorizationState::Granted) {
        st.charging = false;
        st.request_power = false;
    }
}

void SimulatedHardware::set_plugged_in(std::int32_t connector, bool plugged, bool lock_engaged) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    update_energy(state);
    state.plugged_in = plugged;
    state.lock_engaged = lock_engaged || !state.config.require_lock;
    if (!plugged) {
        state.request_power = false;
        state.charging = false;
        state.enabled = false;
        state.target_power_w = 0.0;
        state.authorized = false;
        state.auth_state = AuthorizationState::Unknown;
    }
}

void SimulatedHardware::set_ev_power_request(std::int32_t connector, bool request) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    state.request_power = request;
}

void SimulatedHardware::set_evse_limits(std::int32_t connector, const EvseLimits& limits) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = get_state(connector);
    state.evse_limits = limits;
}

std::vector<AuthToken> SimulatedHardware::poll_auth_tokens() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<AuthToken> tokens;
    tokens.swap(auth_events_);
    return tokens;
}

void SimulatedHardware::inject_auth_token(const AuthToken& token) {
    std::lock_guard<std::mutex> lock(mutex_);
    AuthToken t = token;
    t.received_at = std::chrono::steady_clock::now();
    auth_events_.push_back(std::move(t));
}

bool SimulatedHardware::supports_cross_slot_islands() const {
    return false;
}

} // namespace charger
