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
#include <cctype>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace charger {
namespace {

constexpr int kPollMs = 200;
constexpr int kTxLimitsMs = 500;
constexpr int kTxPresentMs = 300;
constexpr int kTxRelayMs = 100;
constexpr int kSegmentTimeoutMs = 2000;

constexpr uint8_t kRelayGunMask = 0x01u;
constexpr uint8_t kRelayModule0Mask = 0x02u;
constexpr uint8_t kRelayModule1Mask = 0x04u;

bool is_printable_ascii(const std::vector<uint8_t>& bytes) {
    if (bytes.empty()) return false;
    for (auto b : bytes) {
        if (b == 0) return false;
        if (!std::isprint(static_cast<unsigned char>(b))) return false;
    }
    return true;
}

std::string bytes_to_hex(const std::vector<uint8_t>& bytes, const char* sep = "") {
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i > 0) {
            oss << sep;
        }
        oss << std::setw(2) << static_cast<int>(bytes[i]);
    }
    return oss.str();
}

std::string format_token_bytes(const std::vector<uint8_t>& bytes) {
    if (is_printable_ascii(bytes)) {
        return std::string(bytes.begin(), bytes.end());
    }
    return bytes_to_hex(bytes);
}

std::string format_mac_token(const std::vector<uint8_t>& bytes) {
    if (bytes.size() == 6) {
        return bytes_to_hex(bytes, ":");
    }
    return format_token_bytes(bytes);
}

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
        st.connector_id = c.id;
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
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (auto& kv : connectors_) {
                auto& st = kv.second;
                update_limits_tx(st, now);
                update_present_tx(st, now);
                update_relay_tx(st, now);
                if (!st.protocol_sent ||
                    (!st.protocol_ok &&
                     std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_protocol_tx).count() > 1000)) {
                    const auto payload =
                        can_contract::build_config_cmd(can_contract::PARAM_PROTO_VERSION, 0,
                                                       can_contract::PROTOCOL_VERSION, use_crc_);
                    (void)send_frame(st, can_contract::config_cmd_id(static_cast<uint8_t>(st.plc_id)), payload);
                    st.protocol_sent = true;
                    st.last_protocol_tx = now;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

PlcCanHardware::PlcState* PlcCanHardware::find_state_by_plc(uint8_t plc_id) {
    // Caller must hold state_mutex_.
    for (auto& kv : connectors_) {
        if (kv.second.plc_id == static_cast<int>(plc_id)) {
            return &kv.second;
        }
    }
    return nullptr;
}

std::int32_t PlcCanHardware::connector_from_plc(uint8_t plc_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto& kv : connectors_) {
        if (kv.second.plc_id == static_cast<int>(plc_id)) return kv.first;
    }
    return 0;
}

void PlcCanHardware::handle_frame(uint32_t can_id, const uint8_t data[8]) {
    const uint8_t plc_id = static_cast<uint8_t>(can_id & 0x0Fu);
    std::lock_guard<std::mutex> lock(state_mutex_);
    PlcState* st = find_state_by_plc(plc_id);
    if (!st) return;

    const auto now = std::chrono::steady_clock::now();
    if (can_id == can_contract::charge_info_id(plc_id)) {
        const auto info = can_contract::decode_charge_info(data);
        st->hlc_stage = info.hlc_stage;
        st->hlc_charge_complete = info.charge_complete;
        st->hlc_precharge_active = info.precharge_active;
        st->hlc_cable_checked = info.cable_checked;
        st->hlc_auth_granted = info.auth_granted;
        st->hlc_auth_pending = info.auth_pending;
        st->lock_engaged = info.lock_engaged;
        st->lock_engaged_valid = true;
        st->last_chargeinfo_rx = now;
        st->last_status_rx = now;
        st->authorized = info.auth_granted;
        st->auth_pending = info.auth_pending;
    } else if (can_id == can_contract::relay_status_id(plc_id)) {
        st->last_relay = can_contract::decode_relay_status(data, use_crc_);
        st->last_relay_rx = now;
        st->last_status_rx = now;
        st->output_enabled = st->last_relay.relay[0];
    } else if (can_id == can_contract::safety_status_id(plc_id)) {
        st->last_safety = can_contract::decode_safety_status(data, use_crc_);
        st->last_safety_rx = now;
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
            st->evse_limit_ack_count = ack.value;
            st->last_evse_limit_ack = now;
        } else if (ack.param_id == can_contract::PARAM_PROTO_VERSION) {
            const bool ok = (ack.status == 0) && (ack.value == can_contract::PROTOCOL_VERSION);
            st->protocol_ok = ok;
            if (!ok) {
                EVLOG_error << "PLC protocol version mismatch plc=" << static_cast<int>(plc_id)
                            << " status=" << static_cast<int>(ack.status) << " value=" << ack.value
                            << " expected=" << static_cast<int>(can_contract::PROTOCOL_VERSION);
            }
        }
    } else if (can_id == can_contract::cp_voltage_levels_id(plc_id)) {
        const auto cp = can_contract::decode_cp_voltage_levels(data);
        st->cp_state_raw = cp.cp_state;
        st->cp_duty_raw = cp.duty_pct;
        st->last_cp_rx = now;
    } else if (can_id == can_contract::charging_session_id(plc_id)) {
        const auto session = can_contract::decode_charging_session(data);
        st->cp_state_session = session.cp_state;
        st->cp_duty_session = session.duty_pct;
        st->hlc_stage = session.hlc_stage;
        st->hlc_auth_pending = session.auth_pending;
        st->last_session_rx = now;
    } else if (can_id == can_contract::boot_config_id(plc_id)) {
        const auto cfg = can_contract::decode_boot_config(data);
        st->boot_feature_flags = cfg.feature_flags;
        st->meter_available = (cfg.feature_flags & can_contract::FEATURE_METER) != 0;
        st->last_boot_rx = now;
    } else if (can_id == can_contract::evdc_targets_id(plc_id)) {
        const auto targets = can_contract::decode_evdc_targets(data);
        st->ev_target_voltage_v = targets.target_v;
        st->ev_target_current_a = targets.target_a;
        st->ev_present_voltage_v = targets.present_v;
        st->ev_present_current_a = targets.present_a;
        st->last_ev_targets_rx = now;
    } else if (can_id == can_contract::rfid_event_id(plc_id)) {
        const auto seg = can_contract::decode_rfid_event(data);
        if (seg.event_type == 0) {
            std::vector<uint8_t> uid;
            if (assemble_rfid_segment(st->rfid, seg, now, uid)) {
                AuthToken token;
                token.id_token = bytes_to_hex(uid);
                token.source = AuthTokenSource::RFID;
                token.connector_hint = st->connector_id;
                token.received_at = now;
                std::lock_guard<std::mutex> tok_lock(token_mutex_);
                pending_tokens_.push_back(std::move(token));
            }
        }
    } else if (can_id == can_contract::evccid_id(plc_id) ||
               can_id == can_contract::evemaid0_id(plc_id) ||
               can_id == can_contract::evemaid1_id(plc_id) ||
               can_id == can_contract::evmac_id(plc_id)) {
        const auto seg = can_contract::decode_identity_segment(data);
        PlcCanHardware::IdentityAssembly* target = nullptr;
        bool is_mac = false;
        if (can_id == can_contract::evccid_id(plc_id)) {
            target = &st->evccid;
        } else if (can_id == can_contract::evemaid0_id(plc_id)) {
            target = &st->evemaid0;
        } else if (can_id == can_contract::evemaid1_id(plc_id)) {
            target = &st->evemaid1;
        } else if (can_id == can_contract::evmac_id(plc_id)) {
            target = &st->evmac;
            is_mac = true;
        }
        if (target) {
            std::vector<uint8_t> payload;
            if (assemble_identity_segment(*target, seg, now, payload)) {
                AuthToken token;
                token.id_token = is_mac ? format_mac_token(payload) : format_token_bytes(payload);
                token.source = AuthTokenSource::Autocharge;
                token.connector_hint = st->connector_id;
                token.received_at = now;
                std::lock_guard<std::mutex> tok_lock(token_mutex_);
                pending_tokens_.push_back(std::move(token));
            }
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

void PlcCanHardware::set_relay_command(PlcState& st, bool gun_on, uint8_t module_mask, bool force_off) {
    uint8_t enable_mask = 0;
    uint8_t cmd_mask = 0;
    if (!cfg_.plc_owns_gun_relay) {
        enable_mask |= kRelayGunMask;
        if (gun_on) {
            cmd_mask |= kRelayGunMask;
        }
    }
    if (cfg_.plc_module_relays_enabled) {
        enable_mask |= static_cast<uint8_t>(kRelayModule0Mask | kRelayModule1Mask);
        if (module_mask & 0x01u) {
            cmd_mask |= kRelayModule0Mask;
        }
        if (module_mask & 0x02u) {
            cmd_mask |= kRelayModule1Mask;
        }
    }
    const bool any_requested = gun_on || (cfg_.plc_module_relays_enabled && (module_mask & 0x03u) != 0);
    st.sys_enable = any_requested || cfg_.plc_owns_gun_relay;
    st.relay_cmd_mask = cmd_mask;
    st.relay_enable_mask = enable_mask;
    st.relay_force_off = force_off && !cfg_.plc_owns_gun_relay;
    if (force_off) {
        st.relay_cmd_mask = 0;
    }
    st.last_relay_tx = std::chrono::steady_clock::time_point{};
}

void PlcCanHardware::set_lock_command(PlcState& st, bool lock) {
    if (st.lock_command_set && st.lock_command == lock) {
        return;
    }
    st.lock_command = lock;
    st.lock_command_set = true;
    const uint32_t val = lock ? 1u : 0u;
    auto payload = can_contract::build_config_cmd(can_contract::PARAM_LOCK_CMD, 0, val, use_crc_);
    (void)send_frame(st, can_contract::config_cmd_id(static_cast<uint8_t>(st.plc_id)), payload);
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

void PlcCanHardware::update_relay_tx(PlcState& st, std::chrono::steady_clock::time_point now) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_relay_tx).count();
    if (elapsed < kTxRelayMs) {
        return;
    }
    const uint8_t seq = st.seq.fetch_add(1);
    auto payload = can_contract::build_relay_control_masks(st.relay_cmd_mask,
                                                           st.relay_enable_mask,
                                                           seq,
                                                           st.sys_enable,
                                                           st.relay_force_off,
                                                           st.relay_clear_faults,
                                                           0,
                                                           use_crc_);
    (void)send_frame(st, can_contract::relay_control_id(static_cast<uint8_t>(st.plc_id)), payload);
    st.last_relay_tx = now;
    st.relay_clear_faults = false;
}

bool PlcCanHardware::assemble_identity_segment(IdentityAssembly& asmbl,
                                               const can_contract::IdentitySegment& seg,
                                               std::chrono::steady_clock::time_point now,
                                               std::vector<uint8_t>& out) {
    out.clear();
    if (seg.len == 0 || seg.seg_cnt == 0 || seg.seg_idx >= seg.seg_cnt) {
        return false;
    }
    if (asmbl.updated.time_since_epoch().count() != 0) {
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - asmbl.updated).count();
        if (age > kSegmentTimeoutMs) {
            asmbl.reset();
        }
    }
    if (asmbl.len != seg.len || asmbl.seg_cnt != seg.seg_cnt || asmbl.data.empty()) {
        asmbl.len = seg.len;
        asmbl.seg_cnt = seg.seg_cnt;
        asmbl.data.assign(seg.len, 0);
        asmbl.received.assign(seg.seg_cnt, false);
    }
    const std::size_t offset = static_cast<std::size_t>(seg.seg_idx) * 5u;
    for (std::size_t i = 0; i < seg.payload.size(); ++i) {
        const std::size_t idx = offset + i;
        if (idx >= asmbl.data.size()) break;
        asmbl.data[idx] = seg.payload[i];
    }
    if (seg.seg_idx < asmbl.received.size()) {
        asmbl.received[seg.seg_idx] = true;
    }
    asmbl.updated = now;
    const bool complete = std::all_of(asmbl.received.begin(), asmbl.received.end(), [](bool v) { return v; });
    if (!complete) {
        return false;
    }
    out = asmbl.data;
    asmbl.reset();
    return true;
}

bool PlcCanHardware::assemble_rfid_segment(RfidAssembly& asmbl,
                                           const can_contract::RfidEventSegment& seg,
                                           std::chrono::steady_clock::time_point now,
                                           std::vector<uint8_t>& out) {
    out.clear();
    if (seg.uid_len == 0 || seg.seg_cnt == 0 || seg.seg_idx >= seg.seg_cnt) {
        return false;
    }
    if (asmbl.updated.time_since_epoch().count() != 0) {
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - asmbl.updated).count();
        if (age > kSegmentTimeoutMs) {
            asmbl.reset();
        }
    }
    if (asmbl.event_id != seg.event_id || asmbl.uid_len != seg.uid_len || asmbl.seg_cnt != seg.seg_cnt ||
        asmbl.data.empty()) {
        asmbl.uid_len = seg.uid_len;
        asmbl.seg_cnt = seg.seg_cnt;
        asmbl.event_id = seg.event_id;
        asmbl.data.assign(seg.uid_len, 0);
        asmbl.received.assign(seg.seg_cnt, false);
    }
    const std::size_t offset = static_cast<std::size_t>(seg.seg_idx) * 5u;
    for (std::size_t i = 0; i < seg.payload.size(); ++i) {
        const std::size_t idx = offset + i;
        if (idx >= asmbl.data.size()) break;
        asmbl.data[idx] = seg.payload[i];
    }
    if (seg.seg_idx < asmbl.received.size()) {
        asmbl.received[seg.seg_idx] = true;
    }
    asmbl.updated = now;
    const bool complete = std::all_of(asmbl.received.begin(), asmbl.received.end(), [](bool v) { return v; });
    if (!complete) {
        return false;
    }
    out = asmbl.data;
    asmbl.reset();
    return true;
}

bool PlcCanHardware::enable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.output_enabled = true;
    st.regulating = true;
    if (st.cfg.require_lock) {
        set_lock_command(st, true);
    }
    set_relay_command(st, true, 0x03u, false);
    update_relay_tx(st, std::chrono::steady_clock::now());
    return true;
}

bool PlcCanHardware::disable(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.output_enabled = false;
    st.regulating = false;
    set_relay_command(st, false, 0x00u, true);
    update_relay_tx(st, std::chrono::steady_clock::now());
    return true;
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

ocpp::v16::UnlockStatus PlcCanHardware::unlock(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return ocpp::v16::UnlockStatus::UnlockFailed;
    auto& st = it->second;
    if (!st.cfg.require_lock) return ocpp::v16::UnlockStatus::Unlocked;
    set_lock_command(st, false);
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
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = connectors_.find(connector);
    if (it == connectors_.end()) return m;
    auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    const bool use_plc_meter = (st.cfg.meter_source == "plc") && st.meter_available;
    const bool meter_ok = use_plc_meter && st.last_meter.meter_ok && !st.last_meter.stale && !st.last_meter.comm_error;
    if (meter_ok) {
        st.energy_kwh = st.last_meter.import_energy_kwh;
    } else {
        if (st.last_energy_update.time_since_epoch().count() != 0) {
            const auto dt = std::chrono::duration_cast<std::chrono::seconds>(now - st.last_energy_update).count();
            if (dt > 0) {
                // If no energy meter is available, estimate imported energy from the best available
                // power telemetry (typically derived from module V/I via publish_evse_present()).
                //
                // Clamp power to 0 when output is not enabled to avoid accumulating energy from stale
                // or noise readings while contactors are open.
                const bool present_fresh =
                    st.last_evse_present_update.time_since_epoch().count() != 0 &&
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_evse_present_update).count() <=
                        telemetry_timeout_ms_;
                const double power_kw =
                    (st.output_enabled && present_fresh && st.present_power_kw > 0.0) ? st.present_power_kw : 0.0;
                st.energy_kwh += power_kw * (static_cast<double>(dt) / 3600.0);
            }
        }
        st.last_energy_update = now;
    }
    m.power_meter.timestamp = ocpp::DateTime().to_rfc3339();
    const double scale = st.cfg.meter_scale;
    const double energy_kwh = meter_ok ? st.last_meter.import_energy_kwh : st.energy_kwh;
    m.power_meter.energy_Wh_import.total = energy_kwh * 1000.0 * scale +
                                           st.cfg.meter_offset_wh;
    m.power_meter.power_W.emplace();
    const double power_kw = meter_ok ? st.last_meter.power_kw : st.present_power_kw;
    m.power_meter.power_W->total = static_cast<float>(power_kw * 1000.0 * scale);
    m.power_meter.current_A.emplace();
    const double current_a = meter_ok ? st.last_meter.current_a : st.present_current_a;
    m.power_meter.current_A->DC = static_cast<float>(current_a * scale);
    m.power_meter.voltage_V.emplace();
    const double voltage_v = meter_ok ? st.last_meter.voltage_v : st.present_voltage_v;
    m.power_meter.voltage_V->DC = static_cast<float>(voltage_v);
    const double freq_hz = meter_ok ? st.last_meter.freq_hz : st.freq_hz;
    if (freq_hz > 0.0) {
        m.power_meter.frequency_Hz.emplace();
        m.power_meter.frequency_Hz->L1 = static_cast<float>(freq_hz);
    }
    return m;
}

GunStatus PlcCanHardware::get_status(std::int32_t connector) {
    GunStatus gs{};
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = connectors_.find(connector);
    if (it == connectors_.end()) return gs;
    const auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    const bool comm_stale =
        st.last_status_rx.time_since_epoch().count() == 0 ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_status_rx).count() >
            telemetry_timeout_ms_;
    gs.comm_fault = comm_stale || st.last_relay.comm_fault || !st.protocol_ok;
    gs.safety_ok = st.last_relay.safety_ok && st.last_safety.safety_ok && !st.last_relay.earth_fault &&
                   !st.last_safety.earth_fault && !comm_stale;
    gs.estop = st.last_relay.estop_latched || st.last_safety.estop_latched;
    gs.earth_fault = st.last_relay.earth_fault || st.last_safety.earth_fault;
    if (cfg_.plc_owns_gun_relay) {
        gs.relay_closed = st.output_enabled || st.regulating;
    } else {
        gs.relay_closed = st.last_relay.relay[0];
    }
    const bool cp_session_fresh =
        st.last_session_rx.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_session_rx).count() <=
            telemetry_timeout_ms_;
    const bool cp_raw_fresh =
        st.last_cp_rx.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_cp_rx).count() <=
            telemetry_timeout_ms_;
    const bool cp_fresh = cp_session_fresh || cp_raw_fresh;
    const char cp_state = cp_session_fresh ? st.cp_state_session : (cp_raw_fresh ? st.cp_state_raw : 'U');
    gs.cp_state = cp_state;
    const uint8_t duty = cp_session_fresh ? st.cp_duty_session : (cp_raw_fresh ? st.cp_duty_raw : 0);
    gs.pilot_duty_pct = cp_fresh ? static_cast<double>(duty) : 0.0;
    const bool cp_connected = cp_state != 'A' && cp_state != 'U';
    const bool chargeinfo_fresh =
        st.last_chargeinfo_rx.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_chargeinfo_rx).count() <=
            telemetry_timeout_ms_;
    const auto hlc_source_ts = std::max(st.last_chargeinfo_rx, st.last_session_rx);
    const bool hlc_fresh =
        hlc_source_ts.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - hlc_source_ts).count() <=
            telemetry_timeout_ms_;
    gs.plugged_in = cp_fresh ? cp_connected : (hlc_fresh && st.hlc_stage > 0);
    gs.cp_fault = cp_fresh && (cp_state == 'E' || cp_state == 'F');
    const bool lock_required = st.cfg.require_lock;
    const bool lock_fresh = st.lock_engaged_valid && chargeinfo_fresh;
    gs.lock_engaged = lock_required ? (lock_fresh && st.lock_engaged) : true;
    const bool auth_granted = chargeinfo_fresh ? st.hlc_auth_granted : st.authorized;
    gs.authorization_granted = auth_granted;
    gs.module_healthy_mask = 0x03;
    gs.module_fault_mask = 0x00;
    gs.hlc_stage = hlc_fresh ? st.hlc_stage : 0;
    gs.hlc_cable_check_ok = chargeinfo_fresh ? st.hlc_cable_checked : false;
    gs.hlc_precharge_active = chargeinfo_fresh ? st.hlc_precharge_active : false;
    gs.hlc_charge_complete = chargeinfo_fresh ? st.hlc_charge_complete : false;
    const bool hlc_ready = hlc_fresh && st.hlc_stage >= 9 && gs.hlc_cable_check_ok &&
                           !gs.hlc_precharge_active && !gs.hlc_charge_complete;
    gs.hlc_power_ready = hlc_ready;
    if (st.last_relay_rx.time_since_epoch().count() != 0 && st.last_safety_rx.time_since_epoch().count() != 0) {
        gs.last_telemetry = std::min(st.last_relay_rx, st.last_safety_rx);
    }
    const bool meter_expected = (st.cfg.meter_source == "plc") && st.meter_available;
    const bool have_meter_sample = st.last_meter_rx.time_since_epoch().count() != 0;
    const bool meter_flagged_stale = have_meter_sample && (st.last_meter.comm_error || st.last_meter.stale);
    if (meter_expected) {
        if (meter_flagged_stale) {
            gs.meter_stale = true;
        } else if (have_meter_sample) {
            gs.meter_stale =
                std::chrono::duration_cast<std::chrono::seconds>(now - st.last_meter_rx).count() > cfg_.meter_keepalive_s;
        } else if (st.last_status_rx.time_since_epoch().count() != 0) {
            // Avoid an immediate meter fault at boot: allow up to meter_keepalive_s after the first status frame
            // before declaring the meter stale if we have not received any meter sample yet.
            gs.meter_stale =
                std::chrono::duration_cast<std::chrono::seconds>(now - st.last_status_rx).count() > cfg_.meter_keepalive_s;
        } else {
            gs.meter_stale = false;
        }
    }

    const bool meter_ok = st.last_meter.meter_ok && !st.last_meter.stale && !st.last_meter.comm_error;
    const bool evse_present_fresh =
        st.last_evse_present_update.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_evse_present_update).count() <=
            telemetry_timeout_ms_;
    const bool ev_targets_fresh =
        st.last_ev_targets_rx.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_ev_targets_rx).count() <=
            telemetry_timeout_ms_;
    double pv = 0.0;
    double pc = 0.0;
    double pp_kw = 0.0;
    if (meter_ok) {
        pv = st.last_meter.voltage_v;
        pc = st.last_meter.current_a;
        pp_kw = st.last_meter.power_kw;
    } else if (evse_present_fresh &&
               (st.present_voltage_v > 0.0 || st.present_current_a > 0.0 || st.present_power_kw > 0.0)) {
        pv = st.present_voltage_v;
        pc = st.present_current_a;
        pp_kw = st.present_power_kw;
    } else if (ev_targets_fresh) {
        pv = st.ev_present_voltage_v;
        pc = st.ev_present_current_a;
        pp_kw = (pv > 0.0 && pc > 0.0) ? (pv * pc) / 1000.0 : 0.0;
    } else {
        pv = st.present_voltage_v;
        pc = st.present_current_a;
        pp_kw = st.present_power_kw;
    }
    if (pp_kw <= 0.0 && pv > 0.0 && pc > 0.0) {
        pp_kw = (pv * pc) / 1000.0;
    }
    gs.present_voltage_v = pv;
    gs.present_current_a = pc;
    gs.present_power_w = pp_kw * 1000.0;
    if (ev_targets_fresh) {
        gs.target_voltage_v = st.ev_target_voltage_v;
        gs.target_current_a = st.ev_target_current_a;
    } else {
        gs.target_current_a = 0.0;
    }
    gs.evse_max_voltage_v = st.limits.max_voltage_v;
    gs.evse_max_current_a = st.limits.max_current_a;
    gs.evse_max_power_kw = st.limits.max_power_kw;
    gs.evse_limit_ack_count = st.evse_limit_ack_count;
    gs.last_evse_limit_ack = st.last_evse_limit_ack;
    gs.comm_fault = gs.comm_fault || !st.last_relay.crc_ok || !st.last_safety.crc_ok;
    return gs;
}

void PlcCanHardware::set_authorization_state(std::int32_t connector, bool authorized) {
    set_authorization_state(connector, authorized ? AuthorizationState::Granted : AuthorizationState::Denied);
}

void PlcCanHardware::set_authorization_state(std::int32_t connector, AuthorizationState state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.authorized = (state == AuthorizationState::Granted);
    st.auth_pending = (state == AuthorizationState::Pending);
    const uint32_t auth_val = st.authorized ? 1u : 0u;
    const uint32_t pending_val = st.auth_pending ? 1u : 0u;
    auto auth_payload = can_contract::build_config_cmd(can_contract::PARAM_AUTH_STATE, 0, auth_val, use_crc_);
    (void)send_frame(st, can_contract::config_cmd_id(static_cast<uint8_t>(st.plc_id)), auth_payload);
    auto pend_payload = can_contract::build_config_cmd(can_contract::PARAM_AUTH_PENDING, 0, pending_val, use_crc_);
    (void)send_frame(st, can_contract::config_cmd_id(static_cast<uint8_t>(st.plc_id)), pend_payload);
}

void PlcCanHardware::apply_power_command(const PowerCommand& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(cmd.connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    uint8_t module_mask = cmd.module_mask;
    if (!cmd.mc_closed) {
        module_mask = 0;
    }
    module_mask &= 0x03u;
    const bool gun_on = cmd.gc_closed;
    const bool any_relays = gun_on || (cfg_.plc_module_relays_enabled && module_mask != 0);
    st.output_enabled = gun_on;
    st.regulating = any_relays;
    if (any_relays && st.cfg.require_lock) {
        set_lock_command(st, true);
    }
    EvseLimits limits{};
    limits.max_voltage_v = cmd.voltage_set_v > 0.0 ? cmd.voltage_set_v : st.limits.max_voltage_v;
    limits.max_current_a = cmd.current_limit_a > 0.0 ? cmd.current_limit_a : st.limits.max_current_a;
    limits.max_power_kw = cmd.power_kw > 0.0 ? cmd.power_kw : st.limits.max_power_kw;
    st.limits = limits;
    st.last_limits_tx = std::chrono::steady_clock::time_point{};
    set_relay_command(st, gun_on, module_mask, !any_relays);
    update_relay_tx(st, std::chrono::steady_clock::now());
}

void PlcCanHardware::apply_power_allocation(std::int32_t connector, int modules) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    const int capped = std::max(0, std::min(modules, 2));
    const uint8_t module_mask = capped > 0 ? static_cast<uint8_t>((1u << capped) - 1u) : 0u;
    const bool want_power = capped > 0;
    st.output_enabled = want_power;
    st.regulating = want_power;
    if (want_power && st.cfg.require_lock) {
        set_lock_command(st, true);
    }
    set_relay_command(st, want_power, module_mask, !want_power);
    update_relay_tx(st, std::chrono::steady_clock::now());
}

void PlcCanHardware::set_evse_limits(std::int32_t connector, const EvseLimits& limits) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.limits = limits;
    st.last_limits_tx = std::chrono::steady_clock::time_point{}; // force immediate send
}

void PlcCanHardware::publish_evse_present(std::int32_t connector, double voltage_v, double current_a, double power_kw,
                                          bool output_enabled, bool regulating) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.present_voltage_v = voltage_v;
    st.present_current_a = current_a;
    st.present_power_kw = power_kw;
    st.output_enabled = output_enabled;
    st.regulating = regulating;
    st.last_evse_present_update = std::chrono::steady_clock::now();
    st.last_present_tx = std::chrono::steady_clock::time_point{}; // force immediate send
}

void PlcCanHardware::publish_fault_state(std::int32_t connector, uint8_t fault_bits) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    it->second.fault_bits = fault_bits;
    it->second.last_present_tx = std::chrono::steady_clock::time_point{};
}

void PlcCanHardware::clear_faults(std::int32_t connector) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.relay_clear_faults = true;
    update_relay_tx(st, std::chrono::steady_clock::now());
}

std::vector<AuthToken> PlcCanHardware::poll_auth_tokens() {
    std::lock_guard<std::mutex> lock(token_mutex_);
    std::vector<AuthToken> tokens;
    tokens.swap(pending_tokens_);
    return tokens;
}

bool PlcCanHardware::supports_cross_slot_islands() const { return false; }

} // namespace charger
