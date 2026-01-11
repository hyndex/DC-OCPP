// SPDX-License-Identifier: Apache-2.0
#include "plc_can.hpp"

#include "hardware_sim.hpp"

#include <everest/logging.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cerrno>

namespace charger {
namespace {

constexpr int kPollMs = 200;
constexpr int kTxLimitsMs = 500;
constexpr int kTxPresentMs = 300;

uint64_t steady_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

} // namespace

PlcCanHardware::PlcCanHardware(const ChargerConfig& cfg) : cfg_(cfg) {
    use_crc_ = cfg_.plc_use_crc8;
    telemetry_timeout_ms_ = cfg_.telemetry_timeout_ms > 0 ? cfg_.telemetry_timeout_ms : 2000;
    diag_helper_ = std::make_unique<SimulatedHardware>(cfg_);

    for (const auto& c : cfg_.connectors) {
        auto& st = connectors_[c.id]; // default construct in-place (std::atomic is not copyable)
        st.cfg = c;
        st.plc_id = c.plc_id;
        st.iface = c.can_interface.empty() ? cfg_.can_interface : c.can_interface;
        st.limits.max_voltage_v = c.max_voltage_v > 0 ? c.max_voltage_v : cfg_.default_voltage_v;
        st.limits.max_current_a = c.max_current_a > 0 ? c.max_current_a : std::optional<double>{};
        st.limits.max_power_kw = c.max_power_w > 0 ? c.max_power_w / 1000.0 : std::optional<double>{};
        st.present_voltage_v = st.limits.max_voltage_v.value_or(800.0);
        st.present_power_kw = st.limits.max_power_kw.value_or(0.0);
        st.present_current_a = st.limits.max_current_a.value_or(0.0);
        (void)open_socket_for_iface(st.iface);
    }

    running_ = !sockets_.empty();
    if (running_) {
        rx_thread_ = std::thread(&PlcCanHardware::rx_loop, this);
        tx_thread_ = std::thread(&PlcCanHardware::tx_loop, this);
        init_ok_ = true;
    } else {
        EVLOG_warning << "PLC CAN backend disabled: no CAN sockets opened";
    }
}

PlcCanHardware::~PlcCanHardware() {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();
    for (auto& kv : sockets_) {
        if (kv.second >= 0) close(kv.second);
    }
    sockets_.clear();
}

bool PlcCanHardware::open_socket_for_iface(const std::string& iface) {
    if (iface.empty()) return false;
    if (sockets_.count(iface)) return true;
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        EVLOG_error << "Failed to open CAN socket for " << iface << ": " << std::strerror(errno);
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        EVLOG_error << "CAN iface " << iface << " not found: " << std::strerror(errno);
        close(fd);
        return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        EVLOG_error << "Bind CAN iface " << iface << " failed: " << std::strerror(errno);
        close(fd);
        return false;
    }

    int recv_own = 0;
    (void)setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
    sockets_[iface] = fd;
    EVLOG_info << "CAN socket ready on " << iface << " (fd=" << fd << ")";
    return true;
}

bool PlcCanHardware::send_frame(const PlcState& st, uint32_t can_id, const std::array<uint8_t, 8>& data) {
    const auto it = sockets_.find(st.iface);
    if (it == sockets_.end()) return false;
    const int fd = it->second;
    struct can_frame frame {};
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data.data(), 8);
    const ssize_t n = write(fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        return false;
    }
    return true;
}

void PlcCanHardware::rx_loop() {
    while (running_) {
        std::vector<pollfd> pfds;
        pfds.reserve(sockets_.size());
        for (const auto& kv : sockets_) {
            pfds.push_back(pollfd{kv.second, POLLIN, 0});
        }
        if (pfds.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
            continue;
        }
        const int rv = poll(pfds.data(), pfds.size(), kPollMs);
        if (rv <= 0) {
            continue;
        }
        for (const auto& p : pfds) {
            if (!(p.revents & POLLIN)) continue;
            struct can_frame cf {};
            while (read(p.fd, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf))) {
                if (!(cf.can_id & CAN_EFF_FLAG)) continue;
                handle_frame(cf.can_id & CAN_EFF_MASK, cf.data);
            }
        }
    }
}

void PlcCanHardware::tx_loop() {
    while (running_) {
        const auto now = std::chrono::steady_clock::now();
        for (auto& kv : connectors_) {
            update_limits_tx(kv.second, now);
            update_present_tx(kv.second, now);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

PlcCanHardware::PlcState* PlcCanHardware::find_state_by_plc(uint8_t plc_id) {
    for (auto& kv : connectors_) {
        if (kv.second.plc_id == static_cast<int>(plc_id)) {
            return &kv.second;
        }
    }
    return nullptr;
}

std::int32_t PlcCanHardware::connector_from_plc(uint8_t plc_id) const {
    for (const auto& kv : connectors_) {
        if (kv.second.plc_id == static_cast<int>(plc_id)) return kv.first;
    }
    return 0;
}

void PlcCanHardware::handle_frame(uint32_t can_id, const uint8_t data[8]) {
    const uint8_t plc_id = static_cast<uint8_t>(can_id & 0x0Fu);
    PlcState* st = find_state_by_plc(plc_id);
    if (!st) return;

    const auto now = std::chrono::steady_clock::now();
    if (can_id == can_contract::relay_status_id(plc_id)) {
        st->last_relay = can_contract::decode_relay_status(data, use_crc_);
        st->last_status_rx = now;
        st->output_enabled = st->last_relay.relay[0] || st->last_relay.relay[1] || st->last_relay.relay[2];
    } else if (can_id == can_contract::safety_status_id(plc_id)) {
        st->last_safety = can_contract::decode_safety_status(data, use_crc_);
        st->last_status_rx = now;
    } else if (can_id == can_contract::energy_meter_id(plc_id)) {
        static std::atomic<uint64_t> seq{0};
        const uint8_t mux = data[0];
        auto m = can_contract::decode_energy_meter(data, ++seq);
        st->last_meter.meter_ok = m.meter_ok;
        st->last_meter.comm_error = m.comm_error;
        st->last_meter.stale = m.stale;
        st->last_meter.overrange = m.overrange;
        st->last_meter.fallback_active = m.fallback_active;
        st->last_meter.seq = m.seq;
        if (mux == 0) {
            st->last_meter.voltage_v = m.voltage_v;
            st->last_meter.current_a = m.current_a;
            st->last_meter.power_kw = m.power_kw;
        } else if (mux == 1) {
            st->last_meter.import_energy_kwh = m.import_energy_kwh;
            st->last_meter.freq_hz = m.freq_hz;
        }
        st->last_meter_rx = now;
    } else if (can_id == can_contract::config_ack_id(plc_id)) {
        auto ack = can_contract::decode_config_ack(data, use_crc_);
        if (!ack.crc_ok) {
            EVLOG_warning << "ConfigAck CRC fail from PLC " << static_cast<int>(plc_id);
        } else if (ack.param_id == can_contract::PARAM_EVSE_LIMIT_ACK) {
            EVLOG_debug << "EVSE limits ack plc=" << static_cast<int>(plc_id) << " count=" << ack.value;
        }
    }
}

uint16_t PlcCanHardware::clamp_to_0p1(double v) {
    if (v < 0.0) v = 0.0;
    v = std::min(v, 6553.5); // fits in 0.1 increments
    return static_cast<uint16_t>(v * 10.0 + 0.5);
}

uint16_t PlcCanHardware::clamp_to_0p1k(double kw) {
    if (kw < 0.0) kw = 0.0;
    kw = std::min(kw, 6553.5);
    return static_cast<uint16_t>(kw * 10.0 + 0.5);
}

uint16_t PlcCanHardware::clamp_to_0p1_current(double a) {
    if (a < 0.0) a = 0.0;
    a = std::min(a, 6553.5);
    return static_cast<uint16_t>(a * 10.0 + 0.5);
}

void PlcCanHardware::update_limits_tx(PlcState& st, std::chrono::steady_clock::time_point now) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_limits_tx).count();
    if (elapsed >= kTxLimitsMs) {
        const uint16_t v = clamp_to_0p1(st.limits.max_voltage_v.value_or(st.present_voltage_v));
        const uint16_t i = clamp_to_0p1_current(st.limits.max_current_a.value_or(st.present_current_a));
        const uint16_t p = clamp_to_0p1k(st.limits.max_power_kw.value_or(st.present_power_kw));
        auto payload = can_contract::build_evse_limits(v, i, p, use_crc_);
        (void)send_frame(st, can_contract::evse_dc_max_limits_id(static_cast<uint8_t>(st.plc_id)), payload);
        st.last_limits_tx = now;
    }
}

void PlcCanHardware::update_present_tx(PlcState& st, std::chrono::steady_clock::time_point now) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_present_tx).count();
    if (elapsed >= kTxPresentMs) {
        const uint16_t v = clamp_to_0p1(st.present_voltage_v);
        const uint16_t i = clamp_to_0p1_current(st.present_current_a);
        const uint16_t p = clamp_to_0p1k(st.present_power_kw);
        auto payload = can_contract::build_evse_present(v, i, p, st.output_enabled, st.regulating, st.fault_bits,
                                                        use_crc_);
        (void)send_frame(st, can_contract::evse_dc_reg_limits_id(static_cast<uint8_t>(st.plc_id)), payload);
        st.last_present_tx = now;
    }
}

bool PlcCanHardware::enable(std::int32_t connector) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.sys_enable = true;
    st.output_enabled = true;
    auto payload = can_contract::build_relay_control(0x03u, true, st.seq.fetch_add(1), true, false, false, 0,
                                                     use_crc_);
    return send_frame(st, can_contract::relay_control_id(static_cast<uint8_t>(st.plc_id)), payload);
}

bool PlcCanHardware::disable(std::int32_t connector) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.output_enabled = false;
    auto payload = can_contract::build_relay_control(0x07u, false, st.seq.fetch_add(1), false, true, false, 0,
                                                     use_crc_);
    return send_frame(st, can_contract::relay_control_id(static_cast<uint8_t>(st.plc_id)), payload);
}

bool PlcCanHardware::pause_charging(std::int32_t connector) {
    return disable(connector);
}

bool PlcCanHardware::resume_charging(std::int32_t connector) {
    return enable(connector);
}

bool PlcCanHardware::stop_transaction(std::int32_t connector, ocpp::v16::Reason) {
    return disable(connector);
}

ocpp::v16::UnlockStatus PlcCanHardware::unlock(std::int32_t) {
    return ocpp::v16::UnlockStatus::Unlocked;
}

ocpp::v16::ReservationStatus PlcCanHardware::reserve(std::int32_t reservation_id, std::int32_t,
                                                     ocpp::DateTime, const std::string&, const std::optional<std::string>&) {
    (void)reservation_id;
    return ocpp::v16::ReservationStatus::Accepted;
}

bool PlcCanHardware::cancel_reservation(std::int32_t) { return true; }

ocpp::v16::GetLogResponse PlcCanHardware::upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request) {
    return diag_helper_ ? diag_helper_->upload_diagnostics(request) : ocpp::v16::GetLogResponse{};
}

ocpp::v16::GetLogResponse PlcCanHardware::upload_logs(const ocpp::v16::GetLogRequest& request) {
    return diag_helper_ ? diag_helper_->upload_logs(request) : ocpp::v16::GetLogResponse{};
}

bool PlcCanHardware::update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) {
    return diag_helper_ ? diag_helper_->update_firmware(request) : false;
}

ocpp::v16::UpdateFirmwareStatusEnumType
PlcCanHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) {
    return diag_helper_ ? diag_helper_->update_firmware_signed(request)
                        : ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
}

void PlcCanHardware::set_connection_timeout(std::int32_t seconds) { connection_timeout_s_ = seconds; }

bool PlcCanHardware::is_reset_allowed(const ocpp::v16::ResetType&) { return true; }

void PlcCanHardware::reset(const ocpp::v16::ResetType&) {
    // Nothing to do for a stateless backend.
}

void PlcCanHardware::on_remote_start_token(const std::string& id_token,
                                           const std::vector<std::int32_t>& referenced_connectors, bool prevalidated) {
    EVLOG_info << "Remote start token (CAN backend): " << id_token << " prevalidated=" << prevalidated
               << " connectors=" << referenced_connectors.size();
}

ocpp::Measurement PlcCanHardware::sample_meter(std::int32_t connector) {
    ocpp::Measurement m{};
    const auto it = connectors_.find(connector);
    if (it == connectors_.end()) return m;
    const auto& st = it->second;
    m.power_meter.timestamp = ocpp::DateTime().to_rfc3339();
    const double scale = st.cfg.meter_scale;
    m.power_meter.energy_Wh_import.total = st.last_meter.import_energy_kwh * 1000.0 * scale +
                                           st.cfg.meter_offset_wh;
    m.power_meter.power_W.emplace();
    m.power_meter.power_W->total = static_cast<float>(st.last_meter.power_kw * 1000.0 * scale);
    m.power_meter.current_A.emplace();
    m.power_meter.current_A->DC = static_cast<float>(st.last_meter.current_a * scale);
    m.power_meter.voltage_V.emplace();
    m.power_meter.voltage_V->DC = static_cast<float>(st.last_meter.voltage_v);
    return m;
}

GunStatus PlcCanHardware::get_status(std::int32_t connector) {
    GunStatus gs{};
    const auto it = connectors_.find(connector);
    if (it == connectors_.end()) return gs;
    const auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    const bool comm_stale =
        st.last_status_rx.time_since_epoch().count() == 0 ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_status_rx).count() >
            telemetry_timeout_ms_;
    gs.comm_fault = comm_stale || st.last_relay.comm_fault;
    gs.safety_ok = st.last_relay.safety_ok && st.last_safety.safety_ok && !st.last_relay.earth_fault &&
                   !st.last_safety.earth_fault && !comm_stale;
    gs.estop = st.last_relay.estop_latched || st.last_safety.estop_latched;
    gs.earth_fault = st.last_relay.earth_fault || st.last_safety.earth_fault;
    gs.relay_closed = st.last_relay.relay[0] || st.last_relay.relay[1] || st.last_relay.relay[2];
    gs.plugged_in = true; // No CP feedback on this contract; assume connected.
    gs.cp_state = gs.relay_closed ? 'C' : 'B';
    gs.lock_engaged = st.cfg.require_lock ? true : true;
    gs.authorization_granted = st.authorized;
    gs.module_healthy_mask = 0x03;
    gs.module_fault_mask = 0x00;
    gs.hlc_power_ready = gs.authorization_granted && gs.relay_closed;
    gs.hlc_stage = gs.relay_closed ? 5 : 2;
    gs.last_telemetry = st.last_status_rx;
    gs.meter_stale = st.last_meter_rx.time_since_epoch().count() == 0 ||
                     std::chrono::duration_cast<std::chrono::seconds>(now - st.last_meter_rx).count() >
                         cfg_.meter_keepalive_s;

    const double pv = st.last_meter.voltage_v > 0.0 ? st.last_meter.voltage_v : st.present_voltage_v;
    const double pc = st.last_meter.current_a > 0.0 ? st.last_meter.current_a : st.present_current_a;
    const double pp_kw = st.last_meter.power_kw > 0.0 ? st.last_meter.power_kw : st.present_power_kw;
    gs.present_voltage_v = pv;
    gs.present_current_a = pc;
    gs.present_power_w = pp_kw * 1000.0;
    gs.target_voltage_v = st.limits.max_voltage_v;
    gs.target_current_a = st.limits.max_current_a;
    gs.evse_max_voltage_v = st.limits.max_voltage_v;
    gs.evse_max_current_a = st.limits.max_current_a;
    gs.evse_max_power_kw = st.limits.max_power_kw;
    gs.comm_fault = gs.comm_fault || !st.last_relay.crc_ok || !st.last_safety.crc_ok;
    return gs;
}

void PlcCanHardware::set_authorization_state(std::int32_t connector, bool authorized) {
    set_authorization_state(connector, authorized ? AuthorizationState::Granted : AuthorizationState::Denied);
}

void PlcCanHardware::set_authorization_state(std::int32_t connector, AuthorizationState state) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.authorized = (state == AuthorizationState::Granted);
    const uint32_t val = st.authorized ? 1u : 0u;
    auto payload = can_contract::build_config_cmd(can_contract::PARAM_AUTH_STATE, 0, val, use_crc_);
    (void)send_frame(st, can_contract::config_cmd_id(static_cast<uint8_t>(st.plc_id)), payload);
    if (state == AuthorizationState::Pending) {
        auto pend = can_contract::build_config_cmd(can_contract::PARAM_AUTH_PENDING, 0, 1u, use_crc_);
        (void)send_frame(st, can_contract::config_cmd_id(static_cast<uint8_t>(st.plc_id)), pend);
    }
}

void PlcCanHardware::apply_power_command(const PowerCommand& cmd) {
    auto it = connectors_.find(cmd.connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    const bool want_power = cmd.module_count > 0 && (cmd.gc_closed || cmd.mc_closed);
    st.output_enabled = want_power;
    st.regulating = want_power;
    if (cmd.voltage_set_v > 0.0) st.present_voltage_v = cmd.voltage_set_v;
    if (cmd.current_limit_a > 0.0) st.present_current_a = cmd.current_limit_a;
    if (cmd.power_kw > 0.0) st.present_power_kw = cmd.power_kw;
    EvseLimits limits{};
    limits.max_voltage_v = cmd.voltage_set_v > 0.0 ? cmd.voltage_set_v : st.limits.max_voltage_v;
    limits.max_current_a = cmd.current_limit_a > 0.0 ? cmd.current_limit_a : st.limits.max_current_a;
    limits.max_power_kw = cmd.power_kw > 0.0 ? cmd.power_kw : st.limits.max_power_kw;
    set_evse_limits(cmd.connector, limits);
    if (want_power) {
        (void)enable(cmd.connector);
    } else {
        (void)disable(cmd.connector);
    }
}

void PlcCanHardware::apply_power_allocation(std::int32_t connector, int modules) {
    (void)modules;
    (void)connector;
}

void PlcCanHardware::set_evse_limits(std::int32_t connector, const EvseLimits& limits) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.limits = limits;
    st.last_limits_tx = std::chrono::steady_clock::time_point{}; // force immediate send
}

void PlcCanHardware::publish_evse_present(std::int32_t connector, double voltage_v, double current_a, double power_kw,
                                          bool output_enabled, bool regulating) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.present_voltage_v = voltage_v;
    st.present_current_a = current_a;
    st.present_power_kw = power_kw;
    st.output_enabled = output_enabled;
    st.regulating = regulating;
    st.last_present_tx = std::chrono::steady_clock::time_point{}; // force immediate send
}

void PlcCanHardware::publish_fault_state(std::int32_t connector, uint8_t fault_bits) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    it->second.fault_bits = fault_bits;
    it->second.last_present_tx = std::chrono::steady_clock::time_point{};
}

void PlcCanHardware::clear_faults(std::int32_t connector) {
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    auto payload = can_contract::build_relay_control(0x07u, false, st.seq.fetch_add(1), st.sys_enable, false, true,
                                                     0, use_crc_);
    (void)send_frame(st, can_contract::relay_control_id(static_cast<uint8_t>(st.plc_id)), payload);
}

std::vector<AuthToken> PlcCanHardware::poll_auth_tokens() { return {}; }

bool PlcCanHardware::supports_cross_slot_islands() const { return false; }

} // namespace charger
