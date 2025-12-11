// SPDX-License-Identifier: Apache-2.0
#include "can_plc.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <cstdlib>
#include <cerrno>
#include <filesystem>
#include <fstream>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <everest/logging.hpp>

namespace charger {

namespace {
constexpr uint32_t RELAY_STATUS_BASE = 0x160;
constexpr uint32_t SAFETY_STATUS_BASE = 0x190;
constexpr uint32_t ENERGY_METER_BASE = 0x170;
constexpr uint32_t CONFIG_ACK_BASE = 0x1A0;
constexpr uint32_t CP_LEVELS_BASE = 0x430;
constexpr uint32_t CHARGING_SESSION_BASE = 0x410;
constexpr uint32_t EVAC_CTRL_BASE = 0x250;
constexpr uint32_t EVDC_MAX_LIMITS_BASE = 0x200;
constexpr uint32_t EVDC_TARGETS_BASE = 0x210;
constexpr uint32_t EVDC_ENERGY_LIMITS_BASE = 0x230;
constexpr uint32_t EV_STATUS_DISPLAY_BASE = 0x220;
constexpr uint32_t CHARGE_INFO_BASE = 0x100;
constexpr uint32_t PLC_TX_MASK = 0xFFFFFFF0;
constexpr std::chrono::milliseconds RELAY_TIMEOUT_MS(300);
constexpr std::chrono::milliseconds METER_TIMEOUT_MS(2000);
constexpr std::chrono::milliseconds SAFETY_DEBOUNCE_MS(50);
constexpr std::chrono::milliseconds CP_TIMEOUT_MS(1000);
constexpr std::chrono::milliseconds BUS_IDLE_TIMEOUT_MS(1000);
constexpr uint8_t HLC_POWER_DELIVERY_STAGE = 4;

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t le_u16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

int16_t le_i16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

uint32_t le_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

bool cp_state_plugged(char state_char) {
    switch (state_char) {
    case 'B':
    case 'C':
    case 'D':
        return true;
    default:
        return false;
    }
}

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
    if (target.empty()) {
        return false;
    }
    if (target.rfind("http://", 0) == 0 || target.rfind("https://", 0) == 0) {
        std::string cmd = "curl -sSf -T \"" + source + "\" \"" + target + "\"";
        const int rc = std::system(cmd.c_str());
        return rc == 0;
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

PlcHardware::PlcHardware(const ChargerConfig& cfg) {
    iface_ = cfg.can_interface.empty() ? "can0" : cfg.can_interface;
    for (const auto& c : cfg.connectors) {
        if (plc_to_connector_.count(c.plc_id)) {
            throw std::runtime_error("Duplicate plc_id " + std::to_string(c.plc_id) + " in PLC driver init");
        }
        Node node{};
        node.cfg = c;
        node.lock_engaged = !c.require_lock;
        node.lock_feedback_engaged = !c.require_lock;
        nodes_.emplace(c.id, node);
        plc_to_connector_[c.plc_id] = c.id;
    }

#ifdef __linux__
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        throw std::runtime_error("Failed to open CAN socket");
    }
    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface_.c_str());
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        close(sock_);
        throw std::runtime_error("CAN interface not found: " + iface_);
    }

    sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to bind CAN socket");
    }

    const can_err_mask_t err_mask = CAN_ERR_BUSOFF | CAN_ERR_RESTARTED | CAN_ERR_CRTL;
    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to set CAN error filter");
    }

    // Install RX filters for each PLC TX base with specific plc_id nibble
    std::vector<can_filter> filters;
    for (const auto& kv : plc_to_connector_) {
        const int plc_id = kv.first & 0x0F;
        can_filter f{};
        f.can_id = (RELAY_STATUS_BASE & PLC_TX_MASK) | plc_id;
        f.can_mask = PLC_TX_MASK | 0x0F; // match base and low nibble
        filters.push_back(f);
        f.can_id = (SAFETY_STATUS_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (ENERGY_METER_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (CONFIG_ACK_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (CP_LEVELS_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (CHARGING_SESSION_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (EVAC_CTRL_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (EVDC_MAX_LIMITS_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (EVDC_TARGETS_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (EVDC_ENERGY_LIMITS_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (EV_STATUS_DISPLAY_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
        f.can_id = (CHARGE_INFO_BASE & PLC_TX_MASK) | plc_id;
        filters.push_back(f);
    }
    if (!filters.empty()) {
        if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                       sizeof(can_filter) * filters.size()) < 0) {
            close(sock_);
            throw std::runtime_error("Failed to set CAN filters");
        }
    }

    running_ = true;
    rx_thread_ = std::thread([this]() { rx_loop(); });
#else
    throw std::runtime_error("CAN PLC driver requires Linux socketcan support");
#endif
}

PlcHardware::~PlcHardware() {
    running_ = false;
#ifdef __linux__
    if (sock_ >= 0) {
        shutdown(sock_, SHUT_RDWR);
    }
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (sock_ >= 0) {
        close(sock_);
    }
#endif
}

PlcHardware::Node* PlcHardware::find_node(std::int32_t connector) {
    auto it = nodes_.find(connector);
    if (it == nodes_.end()) {
        return nullptr;
    }
    return &it->second;
}

PlcHardware::Node* PlcHardware::find_node_by_plc(int plc_id) {
    auto it = plc_to_connector_.find(plc_id);
    if (it == plc_to_connector_.end()) return nullptr;
    return find_node(it->second);
}

bool PlcHardware::enable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(connector);
    if (!node) return false;
    if (node->cfg.require_lock && !node->lock_engaged) {
        return false;
    }
    return send_relay_command(*node, true, false);
}

bool PlcHardware::disable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(connector);
    if (!node) return false;
    node->lock_engaged = true; // auto-lock when disabling
    return send_relay_command(*node, false, true);
}

bool PlcHardware::pause_charging(std::int32_t connector) {
    return disable(connector);
}

bool PlcHardware::resume_charging(std::int32_t connector) {
    return enable(connector);
}

bool PlcHardware::stop_transaction(std::int32_t connector, ocpp::v16::Reason /*reason*/) {
    return disable(connector);
}

void PlcHardware::apply_power_command(const PowerCommand& cmd) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(cmd.connector);
    if (!node) return;
    node->status.target_voltage_v = cmd.voltage_set_v;
    node->status.target_current_a = cmd.current_limit_a;
    uint8_t mask = 0x00;
    // module_mask bit0 -> module0, map to RLY2 (bit1)
    if (cmd.module_mask & 0x01) {
        mask |= 0x02;
    }
    if (cmd.module_mask & 0x02) {
        mask |= 0x04;
    }
    if (cmd.module_count > 0 && cmd.gc_closed && cmd.mc_closed) {
        mask |= 0x01;
    }
    node->module_mask = mask;
    const bool close = (mask & 0x01) != 0;
    send_relay_command(*node, close, !close);
}

void PlcHardware::apply_power_allocation(std::int32_t connector, int modules) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto* node = find_node(connector);
    if (!node) return;
    uint8_t mask = 0x00;
    if (modules >= 1) {
        mask |= 0x02; // module 1
    }
    if (modules >= 2) {
        mask |= 0x04; // module 2
    }
    // keep gun relay in sync with module availability
    if (modules > 0) {
        mask |= 0x01;
    }
    node->module_mask = mask;
    const bool close = modules > 0;
    send_relay_command(*node, close, !close);
}

ocpp::v16::UnlockStatus PlcHardware::unlock(std::int32_t /*connector*/) {
    std::lock_guard<std::mutex> lock(mtx_);
    // For hardware integration, drive lock actuator open here. Default to unlocked state.
    for (auto& kv : nodes_) {
        kv.second.lock_engaged = false;
    }
    return ocpp::v16::UnlockStatus::Unlocked;
}

ocpp::v16::ReservationStatus PlcHardware::reserve(std::int32_t /*reservation_id*/, std::int32_t /*connector*/,
                                                  ocpp::DateTime /*expiry*/, const std::string& /*id_tag*/,
                                                  const std::optional<std::string>& /*parent_id*/) {
    return ocpp::v16::ReservationStatus::Accepted;
}

bool PlcHardware::cancel_reservation(std::int32_t /*reservation_id*/) {
    return true;
}

ocpp::v16::GetLogResponse PlcHardware::upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) {
    ocpp::v16::GetLogResponse resp;
    resp.status = ocpp::v16::LogStatusEnumType::Accepted;
    const std::string fname = bundle_logs("diagnostics");
    bool upload_ok = true;
    if (!request.location.empty()) {
        upload_ok = upload_file_to_target(fname, request.location);
        if (!upload_ok) {
            EVLOG_warning << "Diagnostics upload to " << request.location << " failed";
        }
    }
    if (!upload_ok) {
        resp.status = ocpp::v16::LogStatusEnumType::Rejected;
    }
    resp.filename.emplace(fname);
    return resp;
}

ocpp::v16::GetLogResponse PlcHardware::upload_logs(const ocpp::v16::GetLogRequest& request) {
    ocpp::v16::GetLogResponse resp;
    resp.status = ocpp::v16::LogStatusEnumType::Accepted;
    const std::string fname = bundle_logs("logs");
    bool upload_ok = true;
    if (!request.log.remoteLocation.empty()) {
        upload_ok = upload_file_to_target(fname, request.log.remoteLocation);
        if (!upload_ok) {
            EVLOG_warning << "Log upload to " << request.log.remoteLocation << " failed";
        }
    }
    if (!upload_ok) {
        resp.status = ocpp::v16::LogStatusEnumType::Rejected;
    }
    resp.filename.emplace(fname);
    return resp;
}

void PlcHardware::update_firmware(const ocpp::v16::UpdateFirmwareRequest& /*request*/) {
    // Firmware update for PLC would be handled externally.
}

ocpp::v16::UpdateFirmwareStatusEnumType
PlcHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& /*request*/) {
    return ocpp::v16::UpdateFirmwareStatusEnumType::Accepted;
}

void PlcHardware::set_connection_timeout(std::int32_t /*seconds*/) {
}

bool PlcHardware::is_reset_allowed(const ocpp::v16::ResetType& /*reset_type*/) {
    return true;
}

void PlcHardware::reset(const ocpp::v16::ResetType& /*reset_type*/) {
}

void PlcHardware::on_remote_start_token(const std::string& /*id_token*/,
                                        const std::vector<std::int32_t>& /*referenced_connectors*/,
                                        bool /*prevalidated*/) {
}

ocpp::Measurement PlcHardware::sample_meter(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    ocpp::Measurement m{};
    auto* node = find_node(connector);
    if (!node) {
        return m;
    }

    const auto now = std::chrono::steady_clock::now();
    auto& meter = node->status.meter;
    const bool has_rx = node->status.last_meter_rx.time_since_epoch().count() != 0;
    meter.stale = has_rx &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(now - node->status.last_meter_rx) > METER_TIMEOUT_MS);

    const bool prefer_shunt = (node->cfg.meter_source == "shunt");
    // Prefer PLC-provided voltage if present; otherwise reuse last known DC voltage.
    if (meter.voltage_v <= 0.0 && node->status.present_voltage_v > 0.0) {
        meter.voltage_v = node->status.present_voltage_v;
    }
    if (prefer_shunt) {
        // For shunt-mode, rely on present V/I from EVDC telemetry rather than PLC meter counters.
        const double v = node->status.present_voltage_v > 0.0 ? node->status.present_voltage_v : meter.voltage_v;
        const double i = node->status.present_current_a;
        meter.voltage_v = v;
        meter.current_a = i;
        meter.power_w = v * i;
        meter.stale = false;
    }

    const bool meter_ok = !prefer_shunt && meter.ok && !meter.stale;
    double energy_wh = node->energy_fallback_Wh;
    const double power_for_energy_raw = meter_ok ? meter.power_w
        : (prefer_shunt ? meter.power_w : node->status.present_power_w);
    if (meter_ok) {
        node->meter_fallback_active = false;
        energy_wh = meter.energy_Wh;
        node->energy_fallback_Wh = energy_wh;
        node->last_energy_update = now;
    } else {
        node->meter_fallback_active = true;
        if (node->last_energy_update.time_since_epoch().count() == 0) {
            node->last_energy_update = now;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - node->last_energy_update).count() /
            1000.0;
        const double power_w = power_for_energy_raw * node->cfg.meter_scale;
        if (elapsed > 0) {
            const double delta = (power_w * elapsed) / 3600.0;
            node->energy_fallback_Wh = std::max(0.0, node->energy_fallback_Wh + delta);
        }
        node->last_energy_update = now;
        energy_wh = node->energy_fallback_Wh;
    }

    // Apply calibration scaling
    double power_raw = power_for_energy_raw;
    if (!meter_ok && !prefer_shunt && node->status.present_power_w > 0.0) {
        power_raw = node->status.present_power_w;
    }
    const double power_scaled = power_raw * node->cfg.meter_scale;
    const double current_scaled = meter.current_a * node->cfg.meter_scale;
    const double energy_scaled = energy_wh * node->cfg.meter_scale + node->cfg.meter_offset_wh;
    node->status.present_power_w = power_scaled;
    node->status.present_current_a = current_scaled;

    m.power_meter.timestamp = ocpp::DateTime().to_rfc3339();
    m.power_meter.energy_Wh_import.total = energy_scaled;
    m.power_meter.voltage_V.emplace();
    m.power_meter.voltage_V->DC = static_cast<float>(meter.voltage_v);
    m.power_meter.current_A.emplace();
    m.power_meter.current_A->DC = static_cast<float>(current_scaled);
    m.power_meter.power_W.emplace();
    m.power_meter.power_W->total = static_cast<float>(power_scaled);
    return m;
}

GunStatus PlcHardware::get_status(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(mtx_);
    GunStatus st{};
    auto* node = find_node(connector);
    if (!node) return st;
    const auto now = std::chrono::steady_clock::now();
    // Timeout detections
    if (node->status.last_relay_status.time_since_epoch().count() &&
        (now - node->status.last_relay_status) > RELAY_TIMEOUT_MS) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
    }
    if (node->status.last_safety_status.time_since_epoch().count() &&
        (now - node->status.last_safety_status) > RELAY_TIMEOUT_MS) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
    }
    if (node->awaiting_ack && (now - node->last_cmd_sent) > RELAY_TIMEOUT_MS) {
        if (node->retry_count < 3) {
            // retry last command
            send_relay_command(*node, node->last_cmd_close, node->last_force_all_off);
            node->retry_count++;
        } else {
            node->status.safety.comm_fault = true;
            node->status.safety.safety_ok = false;
        }
    }

    if (node->status.last_any_rx.time_since_epoch().count() &&
        (now - node->status.last_any_rx) > BUS_IDLE_TIMEOUT_MS) {
        node->status.safety.comm_fault = true;
        node->status.safety.safety_ok = false;
    }

    if (node->status.pending_safety_since.time_since_epoch().count()) {
        if ((now - node->status.pending_safety_since) >= SAFETY_DEBOUNCE_MS) {
            node->status.safety = node->status.pending_safety;
            node->status.pending_safety_since = {};
        }
    }

    const bool cp_seen = node->status.cp.last_rx.time_since_epoch().count() != 0;
    const bool cp_stale = cp_seen &&
        (std::chrono::duration_cast<std::chrono::milliseconds>(now - node->status.cp.last_rx) > CP_TIMEOUT_MS);
    const bool plugged = (cp_seen && !cp_stale) ? cp_state_plugged(node->status.cp.state_char) : false;

    st.safety_ok = node->status.safety.safety_ok;
    st.estop = node->status.safety.estop;
    st.earth_fault = node->status.safety.earth_fault;
    st.comm_fault = node->status.safety.comm_fault;
    st.relay_closed = node->status.relay_closed;
    st.meter_stale = node->status.meter.stale && !node->meter_fallback_active;
    st.plugged_in = plugged;
    st.cp_fault = cp_stale;
    st.cp_state = node->status.cp.state_char;
    st.hlc_stage = node->status.hlc_stage;
    st.hlc_cable_check_ok = node->status.hlc_cable_check_ok;
    st.hlc_precharge_active = node->status.hlc_precharge_active;
    st.hlc_charge_complete = node->status.hlc_charge_complete;
    st.hlc_power_ready = node->status.hlc_power_ready;
    st.pilot_duty_pct = node->status.cp.duty_pct;
    st.present_voltage_v = node->status.present_voltage_v > 0.0 ? std::optional<double>(node->status.present_voltage_v) : std::nullopt;
    st.present_current_a = std::optional<double>(node->status.present_current_a);
    st.present_power_w = node->status.present_power_w > 0.0 ? std::optional<double>(node->status.present_power_w) : std::nullopt;
    st.target_voltage_v = node->status.target_voltage_v > 0.0 ? std::optional<double>(node->status.target_voltage_v) : std::nullopt;
    st.target_current_a = node->status.target_current_a > 0.0 ? std::optional<double>(node->status.target_current_a) : std::nullopt;
    st.evse_max_voltage_v = node->status.max_voltage_v > 0.0 ? std::optional<double>(node->status.max_voltage_v) : std::nullopt;
    st.evse_max_current_a = node->status.max_current_a > 0.0 ? std::optional<double>(node->status.max_current_a) : std::nullopt;
    st.evse_max_power_kw = node->status.max_power_kw > 0.0 ? std::optional<double>(node->status.max_power_kw) : std::nullopt;
    st.lock_engaged = !node->cfg.require_lock || (node->lock_engaged && node->lock_feedback_engaged);
    if (st.comm_fault && node->cfg.require_lock) {
        st.lock_engaged = false;
    }
    // If PLC did not publish capabilities, fall back to configured hardware limits.
    if (!st.evse_max_voltage_v && node->cfg.max_voltage_v > 0.0) {
        st.evse_max_voltage_v = node->cfg.max_voltage_v;
    }
    if (!st.evse_max_current_a && node->cfg.max_current_a > 0.0) {
        st.evse_max_current_a = node->cfg.max_current_a;
    }
    if (!st.evse_max_power_kw && node->cfg.max_power_w > 0.0) {
        st.evse_max_power_kw = node->cfg.max_power_w / 1000.0;
    }
    uint8_t healthy_mask = 0x03; // two modules default healthy
    uint8_t fault_mask = 0x00;
    const uint8_t commanded_mask = node->module_mask & 0x06;
    const uint8_t actual_mask = node->status.relay_state_mask & 0x06;
    const bool gun_cmd = (node->module_mask & 0x01) != 0;
    const bool gun_actual = (node->status.relay_state_mask & 0x01) != 0;
    st.gc_welded = (!gun_cmd && gun_actual);
    st.mc_welded = false;
    if (st.comm_fault || !st.safety_ok || node->status.safety.remote_force_off) {
        healthy_mask = 0x00;
    } else {
        for (int idx = 0; idx < 2; ++idx) {
            const uint8_t bit = static_cast<uint8_t>(1U << (idx + 1));
            const bool commanded_on = (commanded_mask & bit) != 0;
            const bool actual_on = (actual_mask & bit) != 0;
            if (commanded_on && !actual_on) {
                healthy_mask &= static_cast<uint8_t>(~(1U << idx));
                fault_mask |= static_cast<uint8_t>(1U << idx);
            } else if (!commanded_on && actual_on) {
                st.mc_welded = true;
                healthy_mask &= static_cast<uint8_t>(~(1U << idx));
                fault_mask |= static_cast<uint8_t>(1U << idx);
            }
        }
    }
    st.module_healthy_mask = healthy_mask;
    st.module_fault_mask = fault_mask;
    return st;
}

bool PlcHardware::send_relay_command(Node& node, bool close, bool force_all_off) {
#ifdef __linux__
    uint8_t data[8] = {};
    data[0] = 0;
    if (close) {
        data[0] |= (node.module_mask & 0x07); // RLY1/2/3_CMD bits
    }
    data[0] |= (1 << 3);                 // SYS_ENABLE
    if (force_all_off) data[0] |= (1 << 4);
    data[1] = node.cmd_seq++;
    // Enable mask for relays present
    data[2] = node.module_mask & 0x07; // RLY1/2/3_ENABLE
    // CMD_MODE (steady)
    uint32_t can_id = 0x0300 | ((0x4 << 4) | (node.cfg.plc_id & 0x0F));
    const auto ok = send_frame(can_id | CAN_EFF_FLAG, data, sizeof(data));
    if (ok) {
        node.expected_cmd_seq = data[1];
        node.awaiting_ack = true;
        node.last_cmd_sent = std::chrono::steady_clock::now();
        node.retry_count = 0;
        node.last_cmd_close = close;
        node.last_force_all_off = force_all_off;
    }
    return ok;
#else
    (void)node;
    (void)close;
    (void)force_all_off;
    return false;
#endif
}

bool PlcHardware::send_frame(uint32_t can_id, const uint8_t* data, size_t len) {
#ifdef __linux__
    if (sock_ < 0) return false;
    struct can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = static_cast<uint8_t>(len);
    std::memcpy(frame.data, data, std::min<size_t>(len, sizeof(frame.data)));
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto written = write(sock_, &frame, sizeof(frame));
        if (written == sizeof(frame)) {
            return true;
        }
        if (errno == ENOBUFS || errno == EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5 * (attempt + 1)));
            continue;
        }
        EVLOG_warning << "CAN write failed (id=0x" << std::hex << can_id << std::dec << "): "
                      << std::strerror(errno);
        break;
    }
    return false;
#else
    (void)can_id;
    (void)data;
    (void)len;
    return false;
#endif
}

void PlcHardware::rx_loop() {
#ifdef __linux__
    while (running_) {
        struct can_frame frame {};
        const auto nbytes = read(sock_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            continue;
        }
        if (frame.can_id & CAN_ERR_FLAG) {
            handle_error_frame(frame);
            continue;
        }
        handle_frame(frame.can_id & CAN_EFF_MASK, frame.data, frame.can_dlc);
    }
#endif
}

void PlcHardware::handle_frame(uint32_t can_id, const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (nodes_.empty()) return;

    const int plc_id = static_cast<int>(can_id & 0x0F);
    auto* node = find_node_by_plc(plc_id);
    if (!node) return;
    node->status.last_any_rx = std::chrono::steady_clock::now();

    // CRC check when DLC==8 and CRC byte non-zero
    if (len == 8 && data[7] != 0) {
        if (crc8(data, 7) != data[7]) {
            node->status.safety.comm_fault = true;
            return;
        }
    }

    if ((can_id & PLC_TX_MASK) == (RELAY_STATUS_BASE & PLC_TX_MASK)) {
        handle_relay_status(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (SAFETY_STATUS_BASE & PLC_TX_MASK)) {
        handle_safety_status(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (ENERGY_METER_BASE & PLC_TX_MASK)) {
        handle_meter(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CP_LEVELS_BASE & PLC_TX_MASK)) {
        handle_cp_levels(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CHARGING_SESSION_BASE & PLC_TX_MASK)) {
        handle_charging_session(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVAC_CTRL_BASE & PLC_TX_MASK)) {
        handle_evac_control(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVDC_TARGETS_BASE & PLC_TX_MASK)) {
        handle_evdc_targets(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EVDC_MAX_LIMITS_BASE & PLC_TX_MASK) ||
               (can_id & PLC_TX_MASK) == (EVDC_ENERGY_LIMITS_BASE & PLC_TX_MASK)) {
        handle_evdc_limits(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (EV_STATUS_DISPLAY_BASE & PLC_TX_MASK)) {
        handle_ev_status_display(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CHARGE_INFO_BASE & PLC_TX_MASK)) {
        handle_charge_info(*node, data, len);
    } else if ((can_id & PLC_TX_MASK) == (CONFIG_ACK_BASE & PLC_TX_MASK)) {
        if (len >= 6) {
            const uint8_t param_id = data[0];
            const uint8_t status = data[1];
            const uint32_t value = le_u32(&data[2]);
            EVLOG_info << "PLC " << node->cfg.plc_id << " ConfigAck param=" << static_cast<int>(param_id)
                       << " status=" << static_cast<int>(status) << " value=0x" << std::hex << value << std::dec;
        }
    }
}

void PlcHardware::handle_relay_status(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    node.status.relay_state_mask = data[0] & 0x07;
    node.status.relay_closed = (data[0] & 0x01) != 0;
    PlcSafetyStatus s = node.status.safety;
    s.safety_ok = (data[2] & 0x10) != 0;    // bit20
    s.earth_fault = (data[2] & 0x20) != 0;  // bit21
    s.estop = (data[2] & 0x80) != 0;        // bit23
    node.status.last_cmd_seq = data[1];
    s.comm_fault = (data[0] & 0x80) != 0;   // bit7
    s.remote_force_off = false;
    const uint8_t fault_reason = data[3];
    node.status.last_fault_reason = fault_reason;
    if (fault_reason != 0) {
        s.safety_ok = false;
        if (fault_reason == 6 || fault_reason == 5 || fault_reason == 8) {
            s.comm_fault = true; // CAN_BUS_OFF, RELAYCTRL_TIMEOUT, CRC_FAIL
        }
        if (fault_reason == 7) {
            s.remote_force_off = true;
        }
        if (fault_reason == 10 || fault_reason == 12) {
            s.earth_fault = true;
        }
    }
    const bool lock_input = derive_lock_input(node, data[2], true);
    node.lock_feedback_engaged = lock_input;
    node.lock_engaged = node.cfg.require_lock ? lock_input : true;
    // Update pending safety to debounce
    node.status.pending_safety = s;
    node.status.pending_safety_since = std::chrono::steady_clock::now();
    if (node.awaiting_ack && node.expected_cmd_seq == node.status.last_cmd_seq) {
        node.awaiting_ack = false;
        node.retry_count = 0;
    }
    node.status.last_relay_status = std::chrono::steady_clock::now();
}

void PlcHardware::handle_safety_status(Node& node, const uint8_t* data, size_t len) {
    if (len < 2) return;
    PlcSafetyStatus s = node.status.safety;
    s.safety_ok = (data[0] & 0x10) != 0;
    s.earth_fault = (data[0] & 0x20) != 0;
    s.estop = (data[0] & 0x08) != 0;
    s.comm_fault = false; // explicit status frame indicates comm alive
    const bool lock_input = derive_lock_input(node, data[0], false);
    node.lock_feedback_engaged = lock_input;
    node.lock_engaged = node.cfg.require_lock ? lock_input : true;
    node.status.pending_safety = s;
    node.status.pending_safety_since = std::chrono::steady_clock::now();
    node.status.last_safety_status = std::chrono::steady_clock::now();
}

void PlcHardware::handle_meter(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    const uint8_t mux = data[0];
    auto& meter = node.status.meter;
    const bool meter_ok = (data[1] & 0x01) != 0;
    const bool comm_error = (data[1] & 0x02) != 0;
    const bool data_stale = (data[1] & 0x04) != 0;
    const bool overrange = (data[1] & 0x08) != 0;
    if (mux == 0) {
        const uint16_t v = le_u16(&data[2]);
        const int16_t i = le_i16(&data[4]);
        const int16_t p = le_i16(&data[6]);
        meter.voltage_v = v * 0.1;
        meter.current_a = i * 0.01;
        meter.power_w = p * 10.0;
        meter.ok = meter_ok && !overrange;
        node.status.present_voltage_v = meter.voltage_v;
        node.status.present_current_a = meter.current_a;
        node.status.present_power_w = meter.power_w;
    } else if (mux == 1) {
        const uint32_t e = le_u32(&data[2]);
        const uint16_t f = le_u16(&data[6]);
        meter.energy_Wh = static_cast<double>(e) * 100.0; // 0.1 kWh -> Wh
        meter.freq_hz = f * 0.01;
        meter.ok = meter_ok && !overrange;
    }
    meter.stale = data_stale;
    node.status.safety.comm_fault = comm_error;
    node.status.last_meter_rx = std::chrono::steady_clock::now();
    if (comm_error) {
        node.status.safety.safety_ok = false;
    }
}

void PlcHardware::handle_cp_levels(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    const char cp_state_char = static_cast<char>(data[2]);
    const uint8_t duty = data[3];
    const uint16_t mv_peak = le_u16(&data[4]);
    const uint16_t mv_min = le_u16(&data[6]);
    update_cp_status(node, cp_state_char, duty, mv_peak, mv_min, true);
}

void PlcHardware::handle_charging_session(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    const char cp_state_char = static_cast<char>(data[4]);
    const uint8_t duty = data[5];
    update_cp_status(node, cp_state_char, duty, 0, 0, false);
    uint8_t stage = node.status.hlc_stage;
    if (len >= 7) {
        stage = data[6];
    }
    uint8_t flags = 0;
    if (node.status.hlc_charge_complete) flags |= 0x01;
    if (node.status.hlc_precharge_active) flags |= 0x02;
    if (node.status.hlc_cable_check_ok) flags |= 0x04;
    update_hlc_state(node, stage, flags);
}

void PlcHardware::handle_evac_control(Node& node, const uint8_t* data, size_t len) {
    if (len < 2) return;
    const uint8_t duty = data[0];
    const char cp_state_char = static_cast<char>(data[1]);
    uint16_t tgt_v_0p1 = 0;
    uint16_t tgt_i_0p1 = 0;
    uint16_t pres_i_0p1 = 0;
    if (len >= 6) {
        tgt_v_0p1 = le_u16(&data[2]);
        tgt_i_0p1 = le_u16(&data[4]);
    }
    if (len >= 8) {
        pres_i_0p1 = le_u16(&data[6]);
    }
    node.status.target_voltage_v = tgt_v_0p1 * 0.1;
    node.status.target_current_a = tgt_i_0p1 * 0.1;
    node.status.present_current_a = pres_i_0p1 * 0.1;
    update_cp_status(node, cp_state_char, duty, 0, 0, false);
}

void PlcHardware::handle_evdc_targets(Node& node, const uint8_t* data, size_t len) {
    if (len < 8) return;
    const uint16_t tgt_v = le_u16(&data[0]);
    const uint16_t tgt_i = le_u16(&data[2]);
    const uint16_t pres_v = le_u16(&data[4]);
    const uint16_t pres_i = le_u16(&data[6]);
    node.status.target_voltage_v = tgt_v * 0.1;
    node.status.target_current_a = tgt_i * 0.1;
    node.status.present_voltage_v = pres_v * 0.1;
    node.status.present_current_a = pres_i * 0.1;
    node.status.present_power_w = node.status.present_voltage_v * node.status.present_current_a;
}

void PlcHardware::handle_evdc_limits(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    const uint16_t max_v = le_u16(&data[0]);
    const uint16_t max_i = le_u16(&data[2]);
    const uint16_t max_p = le_u16(&data[4]);
    node.status.max_voltage_v = max_v * 0.1;
    node.status.max_current_a = max_i * 0.1;
    node.status.max_power_kw = max_p * 0.1;
}

void PlcHardware::handle_ev_status_display(Node& node, const uint8_t* data, size_t len) {
    if (len < 6) return;
    const uint16_t pres_v = le_u16(&data[0]);
    const uint16_t pres_i = le_u16(&data[2]);
    const uint8_t stage = data[4];
    const uint8_t duty = data[5];
    const char cp_state_char = (len >= 7) ? static_cast<char>(data[6]) : node.status.cp.state_char;
    node.status.present_voltage_v = pres_v * 0.1;
    node.status.present_current_a = pres_i * 0.1;
    node.status.present_power_w = node.status.present_voltage_v * node.status.present_current_a;
    update_cp_status(node, cp_state_char, duty, 0, 0, false);
    uint8_t flags = 0;
    if (node.status.hlc_charge_complete) flags |= 0x01;
    if (node.status.hlc_precharge_active) flags |= 0x02;
    if (node.status.hlc_cable_check_ok) flags |= 0x04;
    update_hlc_state(node, stage, flags);
}

void PlcHardware::handle_charge_info(Node& node, const uint8_t* data, size_t len) {
    if (len < 2) return;
    const uint8_t stage = data[0];
    const uint8_t flags = data[1];
    update_hlc_state(node, stage, flags);
}

bool PlcHardware::derive_lock_input(const Node& node, uint8_t sw_mask_byte, bool relay_status_frame) const {
    const int idx = node.cfg.lock_input_switch;
    if (idx < 1 || idx > 4) return true;
    (void)relay_status_frame;
    const int bit = (idx == 4) ? 6 : (idx - 1);
    return (sw_mask_byte & (1 << bit)) != 0;
}

bool PlcHardware::derive_hlc_power_ready(const PlcStatus& status) const {
    if (status.hlc_charge_complete) {
        return false;
    }
    if (status.hlc_stage >= HLC_POWER_DELIVERY_STAGE && (status.hlc_cable_check_ok || !status.hlc_precharge_active)) {
        return true;
    }
    return false;
}

void PlcHardware::update_hlc_state(Node& node, uint8_t stage, uint8_t flags) {
    node.status.hlc_stage = stage;
    node.status.hlc_charge_complete = (flags & 0x01) != 0;
    node.status.hlc_precharge_active = (flags & 0x02) != 0;
    node.status.hlc_cable_check_ok = (flags & 0x04) != 0;
    node.status.hlc_power_ready = derive_hlc_power_ready(node.status);
}

void PlcHardware::update_cp_status(Node& node, char cp_state_char, uint8_t duty_pct, uint16_t mv_peak,
                                   uint16_t mv_min, bool has_mv) {
    auto& cp = node.status.cp;
    if (cp_state_char == 0) {
        cp_state_char = 'U';
    }
    cp.state_char = cp_state_char;
    cp.duty_pct = static_cast<double>(duty_pct);
    if (has_mv) {
        cp.mv_peak = mv_peak;
        cp.mv_min = mv_min;
    }
    cp.valid = true;
    cp.last_rx = std::chrono::steady_clock::now();
}

#ifdef __linux__
void PlcHardware::handle_error_frame(const struct can_frame& frame) {
    const uint32_t err = frame.can_id & CAN_ERR_MASK;
    const bool bus_off = (err & CAN_ERR_BUSOFF) != 0;
    const bool restarted = (err & CAN_ERR_RESTARTED) != 0;
    const bool ctrl_err = (err & CAN_ERR_CRTL) != 0;

    if (bus_off || ctrl_err) {
        EVLOG_error << "CAN error frame received (bus_off=" << bus_off << ", ctrl=" << ctrl_err << ")";
        for (auto& kv : nodes_) {
            kv.second.status.safety.comm_fault = true;
            kv.second.status.safety.safety_ok = false;
            kv.second.status.relay_closed = false;
        }
    }
    if (restarted) {
        EVLOG_info << "CAN controller restarted, clearing comm faults";
        for (auto& kv : nodes_) {
            kv.second.status.safety.comm_fault = false;
        }
    }
}
#endif

} // namespace charger
