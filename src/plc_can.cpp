// SPDX-License-Identifier: Apache-2.0
#include "plc_can.hpp"

#include <everest/logging.hpp>

#ifdef __linux__
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
#include <cmath>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>

namespace charger {
namespace {

constexpr int kPollMs = 200;
constexpr int kTxPresentActiveMs = 300;
constexpr int kTxPresentIdleMs = 500;
constexpr int kTxLimitsMs = 1500;
constexpr int kMinTxLimitsMs = 200;
constexpr int kMinTxPresentMs = 50;
// Keep relay commands at 1 Hz to avoid extra CAN load while staying below the 3 s PLC watchdog.
constexpr int kTxRelayMs = 1000;
constexpr int kRelayMaxIntervalMs = 2000;
constexpr int kSegmentTimeoutMs = 2000;
constexpr int kPlugInDebounceMs = 200;
constexpr int kPlugOutDebounceMs = 1000;
constexpr int kCpFaultOnDebounceMs = 500;
constexpr int kCpFaultOffDebounceMs = 1000;
constexpr uint8_t kHlcMinPowerStage = 9;
constexpr int kHlcTransitionGraceMs = 1500;

constexpr double kPresentVoltageEps = 1.0;
constexpr double kPresentCurrentEps = 0.5;
constexpr double kPresentPowerEps = 0.5;

constexpr double kDefaultVoltageV = 800.0; // fallback when config/telemetry missing
constexpr double kDefaultCurrentA = 50.0;  // fallback when config/telemetry missing

constexpr uint8_t kRelayGunMask = 0x01u;
constexpr uint8_t kRelayModule0Mask = 0x02u;
constexpr uint8_t kRelayModule1Mask = 0x04u;
constexpr std::chrono::milliseconds kBackpressureHoldMs(1500);
constexpr int kBackpressureMaxLevel = 3;
constexpr int kBackpressureLogIntervalMs = 1000;
constexpr int kPresentMaxIntervalMs = 500;
constexpr int kLimitsMaxIntervalMs = 1500;
constexpr auto kCanStatsLogInterval = std::chrono::seconds(30);
constexpr int kCanStatsBitsPerFrameEst = 150; // extended ID, 8-byte payload; excludes bit-stuff worst-case

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
        // Use plain hex without separators: many CSMS implementations treat ':' as invalid in an OCPP idTag.
        return bytes_to_hex(bytes);
    }
    return format_token_bytes(bytes);
}

uint64_t steady_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool opt_equal(const std::optional<double>& a, const std::optional<double>& b) {
    if (a.has_value() != b.has_value()) return false;
    if (!a) return true;
    return *a == *b;
}

bool limits_equal(const EvseLimits& a, const EvseLimits& b) {
    return opt_equal(a.max_voltage_v, b.max_voltage_v) && opt_equal(a.max_current_a, b.max_current_a) &&
           opt_equal(a.max_power_kw, b.max_power_kw);
}

} // namespace

int PlcCanHardware::compute_interval_ms(int base_ms, int min_ms, int max_ms, int backoff_factor) const {
    const int clamped_factor = std::max(1, backoff_factor);
    const int scaled = base_ms * clamped_factor;
    return std::min(max_ms, std::max(min_ms, scaled));
}

int PlcCanHardware::backpressure_factor(uint64_t now_ms) {
    int level = backpressure_level_.load(std::memory_order_relaxed);
    const uint64_t until = backpressure_until_ms_.load(std::memory_order_relaxed);
    if (level > 0 && until > 0 && now_ms > until) {
        level = std::max(0, level - 1);
        backpressure_level_.store(level, std::memory_order_relaxed);
        if (level > 0) {
            backpressure_until_ms_.store(now_ms + static_cast<uint64_t>(kBackpressureHoldMs.count()),
                                         std::memory_order_relaxed);
        } else {
            backpressure_until_ms_.store(0, std::memory_order_relaxed);
        }
    }
    return 1 + level;
}

void PlcCanHardware::note_tx_backpressure(bool severe) {
    const uint64_t now_ms = steady_ms();
    int level = backpressure_level_.load(std::memory_order_relaxed);
    level = std::min(kBackpressureMaxLevel, level + (severe ? 2 : 1));
    backpressure_level_.store(level, std::memory_order_relaxed);
    backpressure_until_ms_.store(now_ms + static_cast<uint64_t>(kBackpressureHoldMs.count()),
                                 std::memory_order_relaxed);
    const uint64_t last_log = last_backpressure_log_ms_.load(std::memory_order_relaxed);
    if (last_log == 0 || now_ms > last_log + static_cast<uint64_t>(kBackpressureLogIntervalMs)) {
        EVLOG_warning << "CAN backpressure detected (level=" << level
                      << ", hold_ms=" << kBackpressureHoldMs.count() << ")";
        last_backpressure_log_ms_.store(now_ms, std::memory_order_relaxed);
    }
}

void PlcCanHardware::collect_can_stats_lines(std::chrono::steady_clock::time_point now, std::vector<std::string>& out) {
    // Caller must hold state_mutex_.
    struct Totals {
        int plcs{0};
        uint64_t tx_fast{0};
        uint64_t tx_slow{0};
        uint64_t rx_tlm_v3{0};
        uint64_t tx_errno_enobufs{0};
        uint64_t tx_errno_eagain{0};
        uint64_t tx_errno_other{0};
        std::array<uint64_t, 4> tx_backpressure_level_hits{};
    };

    std::map<std::string, Totals> totals_by_iface;
    for (const auto& kv : connectors_) {
        const auto& st = kv.second;
        auto& t = totals_by_iface[st.iface];
        t.plcs++;
        t.tx_fast += st.tx_fast_ok + st.tx_fast_fail;
        t.tx_slow += st.tx_slow_ok + st.tx_slow_fail;
        t.rx_tlm_v3 += st.rx_tlm_v3;
        t.tx_errno_enobufs += st.tx_errno_enobufs;
        t.tx_errno_eagain += st.tx_errno_eagain;
        t.tx_errno_other += st.tx_errno_other;
        for (std::size_t i = 0; i < t.tx_backpressure_level_hits.size(); ++i) {
            t.tx_backpressure_level_hits[i] += st.tx_backpressure_level_hits[i];
        }
    }

    // Prevent unbounded growth if interfaces are removed at runtime.
    for (auto it = iface_stats_.begin(); it != iface_stats_.end();) {
        if (!totals_by_iface.count(it->first)) {
            it = iface_stats_.erase(it);
        } else {
            ++it;
        }
    }

    for (const auto& kv : totals_by_iface) {
        const std::string& iface = kv.first;
        const Totals& cur = kv.second;
        auto& snap = iface_stats_[iface];
        if (snap.last_log.time_since_epoch().count() == 0) {
            snap.last_log = now;
            snap.tx_fast = cur.tx_fast;
            snap.tx_slow = cur.tx_slow;
            snap.rx_tlm_v3 = cur.rx_tlm_v3;
            snap.tx_errno_enobufs = cur.tx_errno_enobufs;
            snap.tx_errno_eagain = cur.tx_errno_eagain;
            snap.tx_errno_other = cur.tx_errno_other;
            snap.tx_backpressure_level_hits = cur.tx_backpressure_level_hits;
            continue;
        }
        const auto elapsed = now - snap.last_log;
        if (elapsed < kCanStatsLogInterval) continue;
        const double dt_s =
            std::max(1e-3, std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count());

        const uint64_t d_tx_fast = cur.tx_fast - snap.tx_fast;
        const uint64_t d_tx_slow = cur.tx_slow - snap.tx_slow;
        const uint64_t d_rx_tlm_v3 = cur.rx_tlm_v3 - snap.rx_tlm_v3;
        const uint64_t d_enobufs = cur.tx_errno_enobufs - snap.tx_errno_enobufs;
        const uint64_t d_eagain = cur.tx_errno_eagain - snap.tx_errno_eagain;
        const uint64_t d_other = cur.tx_errno_other - snap.tx_errno_other;
        std::array<uint64_t, 4> d_bp{};
        for (std::size_t i = 0; i < d_bp.size(); ++i) {
            d_bp[i] = cur.tx_backpressure_level_hits[i] - snap.tx_backpressure_level_hits[i];
        }

        const uint64_t total_frames =
            d_tx_fast + d_tx_slow + d_rx_tlm_v3;
        const double est_kbps = (static_cast<double>(total_frames) * kCanStatsBitsPerFrameEst) / dt_s / 1000.0;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1);
        oss << "PLC CAN stats"
            << " iface=" << iface
            << " plcs=" << cur.plcs
            << " est_kbps=" << est_kbps
            << " tx_fast_fps=" << (static_cast<double>(d_tx_fast) / dt_s)
            << " tx_slow_fps=" << (static_cast<double>(d_tx_slow) / dt_s)
            << " rx_tlm_fps=" << (static_cast<double>(d_rx_tlm_v3) / dt_s)
            << " tx_err{enobufs=" << d_enobufs << ",eagain=" << d_eagain << ",other=" << d_other << "}"
            << " bp_hits{l1=" << d_bp[1] << ",l2=" << d_bp[2] << ",l3=" << d_bp[3] << "}"
            << " bp_lvl=" << backpressure_level_.load(std::memory_order_relaxed);
        out.push_back(oss.str());

        snap.last_log = now;
        snap.tx_fast = cur.tx_fast;
        snap.tx_slow = cur.tx_slow;
        snap.rx_tlm_v3 = cur.rx_tlm_v3;
        snap.tx_errno_enobufs = cur.tx_errno_enobufs;
        snap.tx_errno_eagain = cur.tx_errno_eagain;
        snap.tx_errno_other = cur.tx_errno_other;
        snap.tx_backpressure_level_hits = cur.tx_backpressure_level_hits;
    }
}

PlcCanHardware::PlcCanHardware(const ChargerConfig& cfg) : cfg_(cfg) {
    started_at_ = std::chrono::steady_clock::now();
    if (!cfg_.plc_use_crc8) {
        EVLOG_warning << "Ignoring plc_use_crc8=false; CRC8 is mandatory for PLC CAN frames";
    }
    use_crc_ = true;
    telemetry_timeout_ms_ = cfg_.telemetry_timeout_ms > 0 ? cfg_.telemetry_timeout_ms : 2000;
    auto clamp_ms = [](int value, int min_ms, int max_ms) {
        return std::min(max_ms, std::max(min_ms, value));
    };
    tx_present_base_ms_ = clamp_ms(kTxPresentActiveMs, kMinTxPresentMs, kPresentMaxIntervalMs);
    tx_present_idle_ms_ = clamp_ms(kTxPresentIdleMs, kMinTxPresentMs, kPresentMaxIntervalMs);
    tx_limits_base_ms_ = clamp_ms(kTxLimitsMs, kMinTxLimitsMs, kLimitsMaxIntervalMs);
    if (cfg_.autocharge_id_source == "evccid") {
        autocharge_source_ = AutochargeIdSource::Evccid;
    } else if (cfg_.autocharge_id_source == "emaid") {
        autocharge_source_ = AutochargeIdSource::Emaid;
    } else {
        autocharge_source_ = AutochargeIdSource::Evmac;
    }
    // Phase-offset initial TX timestamps so multiple PLCs don't align their EVSE_FAST/EVSE_SLOW bursts.
    // Use the number of PLCs on a given interface to spread timestamps over the full cadence window.
    std::map<int, std::pair<int, int>> phase_info; // connector_id -> (phase_idx, phase_cnt)
    {
        std::map<std::string, std::vector<std::pair<int, int>>> iface_order; // iface -> (plc_id, connector_id)
        for (const auto& c : cfg_.connectors) {
            const std::string iface = c.can_interface.empty() ? cfg_.can_interface : c.can_interface;
            iface_order[iface].push_back(std::make_pair(c.plc_id, c.id));
        }
        for (auto& kv : iface_order) {
            auto& v = kv.second;
            std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) {
                if (a.first != b.first) return a.first < b.first;
                return a.second < b.second;
            });
            const int count = std::max(1, static_cast<int>(v.size()));
            for (int idx = 0; idx < count; ++idx) {
                phase_info[v[static_cast<std::size_t>(idx)].second] = std::make_pair(idx, count);
            }
        }
    }
    const auto now = std::chrono::steady_clock::now();
    for (const auto& c : cfg_.connectors) {
        auto& st = connectors_[c.id]; // default construct in-place (std::atomic is not copyable)
        st.cfg = c;
        st.connector_id = c.id;
        st.plc_id = c.plc_id;
        st.iface = c.can_interface.empty() ? cfg_.can_interface : c.can_interface;
        st.limits.max_voltage_v = c.max_voltage_v > 0 ? c.max_voltage_v : cfg_.default_voltage_v;
        st.limits.max_current_a = c.max_current_a > 0 ? c.max_current_a : std::optional<double>{};
        st.limits.max_power_kw = c.max_power_w > 0 ? c.max_power_w / 1000.0 : std::optional<double>{};
        // Present values represent measured output. Default to 0 until real telemetry arrives.
        st.present_voltage_v = 0.0;
        st.present_power_kw = 0.0;
        st.present_current_a = 0.0;
        // Initialize relay masks so the TX loop can immediately start sending keepalives with the
        // expected enable mask, even before the first planner dispatch.
        uint8_t enable_mask = 0;
        if (!cfg_.plc_owns_gun_relay) {
            enable_mask |= kRelayGunMask;
        }
        if (cfg_.plc_module_relays_enabled) {
            enable_mask |= static_cast<uint8_t>(kRelayModule0Mask | kRelayModule1Mask);
        }
        st.desired_sys_enable = true;
        st.sys_enable = true;
        st.desired_relay_enable_mask = enable_mask;
        st.relay_enable_mask = enable_mask;
        st.desired_relay_cmd_mask = 0;
        st.relay_cmd_mask = 0;
        st.desired_relay_force_off = false;
        st.relay_force_off = false;
        st.relay_state_dirty = true;
        st.relay_tx_urgent = true;
        int phase_idx = 0;
        int phase_cnt = 1;
        if (const auto it = phase_info.find(c.id); it != phase_info.end()) {
            phase_idx = it->second.first;
            phase_cnt = std::max(1, it->second.second);
        }
        const int present_phase_ms = (tx_present_idle_ms_ * phase_idx) / phase_cnt;
        const int limits_phase_ms = (tx_limits_base_ms_ * phase_idx) / phase_cnt;
        st.last_present_tx = now - std::chrono::milliseconds(std::max(0, tx_present_idle_ms_ - present_phase_ms));
        st.last_limits_tx = now - std::chrono::milliseconds(std::max(0, tx_limits_base_ms_ - limits_phase_ms));
        (void)open_socket_for_iface(st.iface);
    }

    running_ = !sockets_.empty();
    if (running_) {
        rx_thread_ = std::thread(&PlcCanHardware::rx_loop, this);
        tx_thread_ = std::thread(&PlcCanHardware::tx_loop, this);
        init_ok_ = true;
        EVLOG_info << "PLC CAN TX cadence: present_active=" << tx_present_base_ms_
                   << "ms present_idle=" << tx_present_idle_ms_
                   << "ms limits=" << tx_limits_base_ms_ << "ms relay=" << kTxRelayMs << "ms";
        EVLOG_info << "PLC CAN compact protocol enabled";
    } else {
        EVLOG_warning << "PLC CAN backend disabled: no CAN sockets opened";
    }
}

PlcCanHardware::~PlcCanHardware() {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();
    {
        std::lock_guard<std::mutex> lock(sockets_mutex_);
        for (auto& kv : sockets_) {
            if (kv.second >= 0) close(kv.second);
        }
        sockets_.clear();
    }
}

bool PlcCanHardware::open_socket_for_iface(const std::string& iface) {
    if (iface.empty()) return false;
    {
        std::lock_guard<std::mutex> lock(sockets_mutex_);
        if (sockets_.count(iface)) return true;
    }
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

    // Filter out high-traffic power-module frames (e.g. Maxwell 0x060xxxxx) so PLC telemetry like PLC_TLM_V3
    // is less likely to be dropped under load. PLC contract frames use protocol field 0x0, plus boot config (0x9).
    {
        struct can_filter filters[2] {};
        // Contract/EVSE frames: protocol field (bits 20..28) = 0x0.
        filters[0].can_id = CAN_EFF_FLAG;
        filters[0].can_mask = CAN_EFF_FLAG | (0x1FFu << 20);
        // BootConfig frames: protocol field (bits 20..28) = 0x9.
        filters[1].can_id = CAN_EFF_FLAG | (0x9u << 20);
        filters[1].can_mask = CAN_EFF_FLAG | (0x1FFu << 20);
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) {
            EVLOG_warning << "Failed to set CAN filter on " << iface << ": " << std::strerror(errno);
        }
    }

    // Enlarge receive buffer to reduce frame loss when multiple processes are listening.
    {
        const int rcvbuf = 512 * 1024;
        if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
            EVLOG_warning << "Failed to set SO_RCVBUF on " << iface << ": " << std::strerror(errno);
        }
    }
    // Enlarge send buffer to reduce ENOBUFS during short bursts when the driver queues frames.
    {
        const int sndbuf = 512 * 1024;
        if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
            EVLOG_warning << "Failed to set SO_SNDBUF on " << iface << ": " << std::strerror(errno);
        }
    }

    int recv_own = 0;
    (void)setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
    {
        std::lock_guard<std::mutex> lock(sockets_mutex_);
        auto it = sockets_.find(iface);
        if (it != sockets_.end()) {
            close(fd);
            fd = it->second;
        } else {
            sockets_[iface] = fd;
        }
    }
    EVLOG_info << "CAN socket ready on " << iface << " (fd=" << fd << ")";
    return true;
}

bool PlcCanHardware::send_frame_raw(const std::string& iface,
                                    int plc_id,
                                    uint32_t can_id,
                                    const std::array<uint8_t, 8>& data,
                                    TxPriority pri,
                                    int* out_errno) {
    int fd = -1;
    {
        std::lock_guard<std::mutex> lock(sockets_mutex_);
        const auto it = sockets_.find(iface);
        if (it == sockets_.end()) {
            if (out_errno) *out_errno = ENODEV;
            return false;
        }
        fd = it->second;
    }
    struct can_frame frame {};
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data.data(), 8);
    static std::chrono::steady_clock::time_point last_err_log{};
    static std::chrono::steady_clock::time_point last_reopen{};
    static int enobufs_count = 0;
    const auto now_tp = std::chrono::steady_clock::now();
    bool noted_backpressure = false;
    int last_errno = 0;
    for (int attempt = 0; attempt < 3; ++attempt) {
        const ssize_t n = write(fd, &frame, sizeof(frame));
        if (n == static_cast<ssize_t>(sizeof(frame))) {
            enobufs_count = 0;
            if (out_errno) *out_errno = 0;
            return true;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            last_errno = errno;
            note_tx_backpressure(false);
            noted_backpressure = true;
            if (pri <= TxPriority::Control) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            break; // drop lower-priority frames instead of piling up
        }
        if (n < 0 && errno == ENOBUFS) {
            last_errno = errno;
            enobufs_count++;
            note_tx_backpressure(true);
            noted_backpressure = true;
            if (pri <= TxPriority::Heartbeat) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                // If TX queue is wedged, try reopening the socket once in a while.
                if (enobufs_count >= 5) {
                    const bool can_reopen = last_reopen.time_since_epoch().count() == 0 ||
                                            (now_tp - last_reopen) > std::chrono::seconds(2);
                    if (can_reopen) {
                        last_reopen = now_tp;
                        {
                            std::lock_guard<std::mutex> lock(sockets_mutex_);
                            const auto it = sockets_.find(iface);
                            if (it != sockets_.end()) {
                                if (it->second >= 0) close(it->second);
                                sockets_.erase(it);
                            }
                        }
                        open_socket_for_iface(iface);
                        {
                            std::lock_guard<std::mutex> lock(sockets_mutex_);
                            const auto it = sockets_.find(iface);
                            fd = (it == sockets_.end()) ? -1 : it->second;
                        }
                    }
                    enobufs_count = 0;
                    if (fd < 0) break;
                    continue;
                }
            } else {
                break; // shed telemetry/debug immediately on ENOBUFS
            }
        }
        if (n < 0 && !noted_backpressure) {
            last_errno = errno;
            note_tx_backpressure(false);
            noted_backpressure = true;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_err_log).count();
        if (since > 500) {
            EVLOG_warning << "CAN write failed iface=" << iface << " plc=" << plc_id << " id=0x" << std::hex
                          << can_id << std::dec << " errno=" << errno << " (" << std::strerror(errno) << ")";
            last_err_log = now;
        }
        break;
    }
    if (out_errno) *out_errno = (last_errno != 0) ? last_errno : errno;
    return false;
}

void PlcCanHardware::rx_loop() {
    while (running_) {
        std::vector<int> fds;
        {
            std::lock_guard<std::mutex> lock(sockets_mutex_);
            fds.reserve(sockets_.size());
            for (const auto& kv : sockets_) {
                fds.push_back(kv.second);
            }
        }
        std::vector<pollfd> pfds;
        pfds.reserve(fds.size());
        for (const int fd : fds) {
            pfds.push_back(pollfd{fd, POLLIN, 0});
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
        const uint64_t now_ms = steady_ms();
        const int backoff = backpressure_factor(now_ms);
        const int backpressure_level = backpressure_level_.load(std::memory_order_relaxed);
        const uint64_t backpressure_until = backpressure_until_ms_.load(std::memory_order_relaxed);
        const bool severe_backpressure =
            backpressure_level >= kBackpressureMaxLevel && backpressure_until > now_ms;
        std::vector<std::string> stats_lines;
        {
            std::unique_lock<std::mutex> lock(state_mutex_);
            for (auto& kv : connectors_) {
                auto& st = kv.second;
                const int eff_backoff = backoff;
                const bool allow_fast = !severe_backpressure || st.relay_tx_urgent || st.relay_state_dirty;
                if (allow_fast) {
                    update_fast_tx(st, now, eff_backoff, lock);
                }
                if (!severe_backpressure) {
                    update_slow_tx(st, now, eff_backoff, lock);
                }
            }
            if (last_can_stats_check_.time_since_epoch().count() == 0 ||
                (now - last_can_stats_check_) >= std::chrono::seconds(1)) {
                collect_can_stats_lines(now, stats_lines);
                last_can_stats_check_ = now;
            }
        }
        for (const auto& line : stats_lines) {
            EVLOG_info << line;
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
    if (can_id == can_contract::plc_tlm_v3_id(plc_id)) {
        st->rx_tlm_v3++;
        const auto tlm = can_contract::decode_plc_tlm_v3(data, use_crc_);
        if (use_crc_ && !tlm.crc_ok) {
            st->tlm_v3_crc_fail_count++;
            if (st->last_tlm_crc_warn.time_since_epoch().count() == 0 ||
                (now - st->last_tlm_crc_warn) > std::chrono::seconds(2)) {
                EVLOG_warning << "PLC_TLM_V3 CRC fail from PLC " << static_cast<int>(plc_id)
                              << " count=" << st->tlm_v3_crc_fail_count;
                st->last_tlm_crc_warn = now;
            }
            return;
        }
        st->cp_state_raw = tlm.cp_state;
        st->cp_state_session = tlm.cp_state;
        st->cp_duty_raw = 0;
        st->cp_duty_session = 0;
        st->hlc_stage = tlm.hlc_stage;
        st->hlc_charge_complete = tlm.charge_complete;
        st->hlc_precharge_active = tlm.precharge_active;
        st->hlc_cable_checked = tlm.cable_checked;
        st->lock_engaged = tlm.lock_engaged;
        st->lock_engaged_valid = true;
        st->last_cp_rx = now;
        st->last_session_rx = now;
        st->last_chargeinfo_rx = now;
        st->last_status_rx = now;

        // Populate Relay/Safety status so existing fault gating logic continues to work.
        st->last_relay.relay[0] = (tlm.relay_state_mask & 0x01u) != 0;
        st->last_relay.relay[1] = (tlm.relay_state_mask & 0x02u) != 0;
        st->last_relay.relay[2] = (tlm.relay_state_mask & 0x04u) != 0;
        st->last_relay.relay_fault[0] = (tlm.relay_fault_mask & 0x01u) != 0;
        st->last_relay.relay_fault[1] = (tlm.relay_fault_mask & 0x02u) != 0;
        st->last_relay.relay_fault[2] = (tlm.relay_fault_mask & 0x04u) != 0;
        st->last_relay.safety_ok = tlm.safety_ok;
        st->last_relay.estop_latched = tlm.estop;
        st->last_relay.estop_input = tlm.estop;
        st->last_relay.earth_fault = tlm.earth_fault;
        st->last_relay.comm_fault = tlm.comm_fault;
        st->last_relay.fault_reason = tlm.fault_reason;
        st->last_relay.crc_ok = tlm.crc_ok;

        st->last_safety.safety_ok = tlm.safety_ok;
        st->last_safety.estop_latched = tlm.estop;
        st->last_safety.estop_input = tlm.estop;
        st->last_safety.earth_fault = tlm.earth_fault;
        st->last_safety.crc_ok = tlm.crc_ok;

        st->last_relay_rx = now;
        st->last_safety_rx = now;

        st->ev_target_voltage_v = tlm.ev_target_voltage_v;
        st->ev_target_current_a = tlm.ev_target_current_a;
        st->last_ev_targets_rx = now;

        // In protocol v3, EVSE limit ACKs are conveyed by the low-byte of the PLC's received-limits counter.
        bool ack_advanced = false;
        const uint32_t extended =
            can_contract::extend_counter_lsb(st->evse_limit_ack_count, tlm.limits_rx_count_lsb, &ack_advanced);
        if (ack_advanced) {
            st->evse_limit_ack_count = extended;
            st->last_evse_limit_ack = now;
        }

        // Authorization is controller-owned; keep the HLC echo state aligned so get_status() semantics remain stable.
        st->hlc_auth_granted = st->authorized;
        st->hlc_auth_pending = st->auth_pending;
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
    } else if (can_id == can_contract::boot_config_id(plc_id)) {
        const auto cfg = can_contract::decode_boot_config(data);
        st->boot_feature_flags = cfg.feature_flags;
        st->meter_available = (cfg.feature_flags & can_contract::FEATURE_METER) != 0;
        st->last_boot_rx = now;
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
    } else if (can_id == can_contract::evmac_id(plc_id)) {
        const auto seg = can_contract::decode_identity_segment(data);
        std::vector<uint8_t> payload;
        if (assemble_identity_segment(st->evmac, seg, now, payload)) {
            if (autocharge_source_ == AutochargeIdSource::Evmac) {
                emit_autocharge_token(*st, format_mac_token(payload), now);
            } else {
                EVLOG_debug << "Ignoring EVMAC identity (autochargeIdSource=" << cfg_.autocharge_id_source << ")";
            }
        }
    } else if (can_id == can_contract::evccid_id(plc_id)) {
        const auto seg = can_contract::decode_identity_segment(data);
        std::vector<uint8_t> payload;
        if (assemble_identity_segment(st->evccid, seg, now, payload)) {
            if (autocharge_source_ == AutochargeIdSource::Evccid) {
                emit_autocharge_token(*st, format_token_bytes(payload), now);
            } else if (autocharge_source_ == AutochargeIdSource::Evmac) {
                // Some PLC stacks publish the EV's MAC via the EVCCID identity frame.
                // Treat this as compatible with MAC-based AutoCharge to avoid deadlocking HLC authorization.
                emit_autocharge_token(*st, format_mac_token(payload), now);
            } else {
                EVLOG_debug << "Ignoring EVCCID identity (autochargeIdSource=" << cfg_.autocharge_id_source << ")";
            }
        }
    } else if (can_id == can_contract::evemaid0_id(plc_id)) {
        const auto seg = can_contract::decode_identity_segment(data);
        std::vector<uint8_t> payload;
        if (assemble_identity_segment(st->evemaid0, seg, now, payload)) {
            if (autocharge_source_ == AutochargeIdSource::Emaid) {
                st->emaid0_cache = payload;
                st->emaid0_rx = now;
                maybe_emit_emaid(*st, now);
            } else {
                EVLOG_debug << "Ignoring EMAID0 identity (autochargeIdSource=" << cfg_.autocharge_id_source << ")";
            }
        }
    } else if (can_id == can_contract::evemaid1_id(plc_id)) {
        const auto seg = can_contract::decode_identity_segment(data);
        std::vector<uint8_t> payload;
        if (assemble_identity_segment(st->evemaid1, seg, now, payload)) {
            if (autocharge_source_ == AutochargeIdSource::Emaid) {
                st->emaid1_cache = payload;
                st->emaid1_rx = now;
                maybe_emit_emaid(*st, now);
            } else {
                EVLOG_debug << "Ignoring EMAID1 identity (autochargeIdSource=" << cfg_.autocharge_id_source << ")";
            }
        }
    }
    if (autocharge_source_ == AutochargeIdSource::Emaid &&
        (!st->emaid0_cache.empty() || !st->emaid1_cache.empty())) {
        maybe_emit_emaid(*st, now);
    }
}

uint16_t PlcCanHardware::clamp_to_0p5(double v) {
    if (v < 0.0) v = 0.0;
    v = std::min(v, 1023.5); // 11 bits at 0.5 V resolution
    return static_cast<uint16_t>(v * 2.0 + 0.5);
}

uint16_t PlcCanHardware::clamp_to_0p5k(double kw) {
    if (kw < 0.0) kw = 0.0;
    kw = std::min(kw, 255.5); // 9 bits at 0.5 kW resolution
    return static_cast<uint16_t>(kw * 2.0 + 0.5);
}

uint16_t PlcCanHardware::clamp_to_0p2_current(double a) {
    if (a < 0.0) a = 0.0;
    a = std::min(a, 409.4); // 11 bits at 0.2 A resolution
    return static_cast<uint16_t>(a * 5.0 + 0.5);
}

bool PlcCanHardware::set_relay_command(PlcState& st, bool gun_on, uint8_t module_mask, bool force_off) {
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
    const bool relay_force_off = force_off && !cfg_.plc_owns_gun_relay;
    if (relay_force_off) {
        cmd_mask = 0;
    }
    const bool changed = (st.desired_relay_cmd_mask != cmd_mask) ||
                         (st.desired_relay_enable_mask != enable_mask) ||
                         (st.desired_relay_force_off != relay_force_off);
    if (changed) {
        const bool would_open_relays = relay_force_off && !st.relay_force_off;
        st.desired_relay_cmd_mask = cmd_mask;
        st.desired_relay_enable_mask = enable_mask;
        st.desired_relay_force_off = relay_force_off;
        st.relay_state_dirty = true;
        if (would_open_relays) {
            st.relay_tx_urgent = true;
        }
    }
    return changed;
}

void PlcCanHardware::set_lock_command(PlcState& st, bool lock) {
    if (st.lock_command_set && st.lock_command == lock) {
        return;
    }
    st.lock_command = lock;
    st.lock_command_set = true;
    st.last_limits_tx = std::chrono::steady_clock::time_point{};
    st.last_limits_payload_valid = false;
}


void PlcCanHardware::update_fast_tx(PlcState& st,
                                    std::chrono::steady_clock::time_point now,
                                    int backoff_factor,
                                    std::unique_lock<std::mutex>& state_lock) {
    // Keep CAN load bounded. Relay edges bypass cadence gating via relay_tx_urgent.
    const std::array<uint8_t, 3> relay_bits{{kRelayGunMask, kRelayModule0Mask, kRelayModule1Mask}};

    // Apply relay transitions with per-relay minimum dwell times to reduce chatter.
    const uint8_t enable_mask = st.desired_relay_enable_mask;
    const bool want_sys_enable = st.desired_sys_enable;
    const bool want_force_off = st.desired_relay_force_off;
    uint8_t want_cmd_mask = static_cast<uint8_t>(st.desired_relay_cmd_mask & enable_mask);
    if (!want_sys_enable || want_force_off) {
        want_cmd_mask = 0;
    }

    bool state_changed = false;
    bool urgent = false;

    auto relay_hold_for_mask = [&](uint8_t mask) -> std::chrono::milliseconds {
        if (mask == kRelayGunMask) {
            return std::chrono::milliseconds(std::max(0, cfg_.min_gc_hold_ms));
        }
        return std::chrono::milliseconds(std::max(0, cfg_.min_mc_hold_ms));
    };
    auto relay_index_for_mask = [&](uint8_t mask) -> int {
        if (mask == kRelayGunMask) return 0;
        if (mask == kRelayModule0Mask) return 1;
        if (mask == kRelayModule1Mask) return 2;
        return -1;
    };

    const bool emergency_open = (!want_sys_enable) || want_force_off;

    const uint8_t cur_cmd_mask = static_cast<uint8_t>(st.relay_cmd_mask & st.relay_enable_mask);
    uint8_t next_cmd_mask = cur_cmd_mask;
    const uint8_t to_off = static_cast<uint8_t>(cur_cmd_mask & static_cast<uint8_t>(~want_cmd_mask));
    if (to_off != 0) {
        for (const uint8_t bit : relay_bits) {
            if (!(to_off & bit)) continue;
            const int idx = relay_index_for_mask(bit);
            if (idx < 0) continue;
            const auto hold = relay_hold_for_mask(bit);
            const auto last = st.relay_last_change[static_cast<std::size_t>(idx)];
            const bool allowed = emergency_open || last.time_since_epoch().count() == 0 || (now - last) >= hold;
            if (allowed) {
                next_cmd_mask = static_cast<uint8_t>(next_cmd_mask & static_cast<uint8_t>(~bit));
                state_changed = true;
                urgent = urgent || emergency_open;
                st.relay_last_change[static_cast<std::size_t>(idx)] = now;
            }
        }
    }

    const uint8_t to_on = static_cast<uint8_t>(want_cmd_mask & static_cast<uint8_t>(~cur_cmd_mask));
    if (to_on != 0) {
        for (const uint8_t bit : relay_bits) {
            if (!(to_on & bit)) continue;
            const int idx = relay_index_for_mask(bit);
            if (idx < 0) continue;
            const auto hold = relay_hold_for_mask(bit);
            const auto last = st.relay_last_change[static_cast<std::size_t>(idx)];
            const bool allowed = last.time_since_epoch().count() == 0 || (now - last) >= hold;
            if (allowed) {
                next_cmd_mask = static_cast<uint8_t>(next_cmd_mask | bit);
                state_changed = true;
                st.relay_last_change[static_cast<std::size_t>(idx)] = now;
            }
        }
    }

    if (st.relay_enable_mask != enable_mask) {
        st.relay_enable_mask = enable_mask;
        state_changed = true;
        urgent = true;
    }
    if (st.sys_enable != want_sys_enable) {
        st.sys_enable = want_sys_enable;
        state_changed = true;
        urgent = true;
    }
    if (st.relay_force_off != want_force_off) {
        st.relay_force_off = want_force_off;
        state_changed = true;
        urgent = true;
    }
    if (st.relay_cmd_mask != next_cmd_mask) {
        st.relay_cmd_mask = next_cmd_mask;
        state_changed = true;
    }

    if (state_changed) {
        st.relay_state_dirty = true;
        if (urgent) {
            st.relay_tx_urgent = true;
        }
    }

    double v_f = std::max(0.0, st.present_voltage_v);
    double i_f = std::max(0.0, st.present_current_a);
    double p_kw_f = std::max(0.0, st.present_power_kw);
    if (!st.output_enabled && !st.regulating) {
        i_f = 0.0;
        p_kw_f = 0.0;
    }
    if (p_kw_f <= 0.0 && v_f > 0.0 && i_f > 0.0) {
        p_kw_f = (v_f * i_f) / 1000.0;
    }

    const uint16_t v = clamp_to_0p5(v_f);
    const uint16_t i = clamp_to_0p2_current(i_f);
    const uint16_t p = clamp_to_0p5k(p_kw_f);
    const uint8_t seq = st.seq.fetch_add(1);
    const auto payload = can_contract::build_evse_fast(v, i, p, st.output_enabled, st.regulating, st.fault_bits,
                                                       st.relay_cmd_mask, st.relay_enable_mask, st.sys_enable,
                                                       st.relay_force_off, st.relay_clear_faults, seq, use_crc_);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_present_tx).count();
    // EVSE_FAST includes a sequence number and CRC; compare on a masked payload so keepalive cadence can be respected.
    auto cmp = payload;
    // seq spans bits 44..51 => byte5 upper nibble + byte6 lower nibble
    cmp[5] &= 0x0Fu;
    cmp[6] &= 0xF0u;
    // Ignore CRC8 in comparisons.
    cmp[7] = 0;
    const bool payload_changed = !st.last_present_payload_valid || (cmp != st.last_present_payload);
    const bool active = st.hlc_stage > 0 || st.relay_state_dirty || st.relay_tx_urgent;
    const int base_ms = active ? tx_present_base_ms_ : tx_present_idle_ms_;
    const int keepalive_ms = compute_interval_ms(base_ms, kMinTxPresentMs, kPresentMaxIntervalMs, backoff_factor);
    const bool allow_change_tx = elapsed >= keepalive_ms;
    const bool allow_keepalive_tx = elapsed >= keepalive_ms;
    if (st.tx_quiet_until.time_since_epoch().count() != 0 && now < st.tx_quiet_until && !st.relay_tx_urgent &&
        !st.relay_state_dirty) {
        return;
    }
    if (!(payload_changed ? allow_change_tx : allow_keepalive_tx) && !st.relay_tx_urgent) {
        return;
    }
    const TxPriority pri = (st.relay_tx_urgent || st.relay_state_dirty) ? TxPriority::Control : TxPriority::Heartbeat;
    const bool had_dirty = st.relay_state_dirty;
    const bool had_urgent = st.relay_tx_urgent;
    const bool had_clear_faults = st.relay_clear_faults;
    // Optimistically clear so a concurrent update while unlocked won't be lost.
    st.relay_state_dirty = false;
    st.relay_tx_urgent = false;
    state_lock.unlock();
    int tx_errno = 0;
    const bool sent =
        send_frame_raw(st.iface,
                       st.plc_id,
                       can_contract::evse_fast_id(static_cast<uint8_t>(st.plc_id)),
                       payload,
                       pri,
                       &tx_errno);
    state_lock.lock();
    if (!sent) {
        st.tx_fast_fail++;
        if (tx_errno == ENOBUFS) {
            st.tx_errno_enobufs++;
        } else if (tx_errno == EAGAIN || tx_errno == EWOULDBLOCK) {
            st.tx_errno_eagain++;
        } else if (tx_errno != 0) {
            st.tx_errno_other++;
        }
        if (tx_errno == ENOBUFS || tx_errno == EAGAIN || tx_errno == EWOULDBLOCK) {
            const int level = std::max(0, std::min(3, backpressure_level_.load(std::memory_order_relaxed)));
            st.tx_backpressure_level_hits[static_cast<std::size_t>(level)]++;
        }
        if (had_dirty) st.relay_state_dirty = true;
        if (had_urgent) st.relay_tx_urgent = true;
        const auto warn_now = std::chrono::steady_clock::now();
        const bool log_now = st.last_tx_warn.time_since_epoch().count() == 0 ||
                             (warn_now - st.last_tx_warn) > std::chrono::milliseconds(500);
        if (log_now) {
            EVLOG_warning << "Failed to TX EVSE_FAST on CAN for plc_id=" << st.plc_id;
            st.last_tx_warn = warn_now;
        }
        st.tx_failures++;
        st.tx_errors_recent++;
        st.last_present_tx = now; // throttle retry cadence under backpressure
        st.tx_failure_streak++;
        if (st.tx_failure_streak >= 5) {
            const auto quiet_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
            st.tx_quiet_until = std::max(st.tx_quiet_until, quiet_until);
            const auto now_quiet = std::chrono::steady_clock::now();
            const bool log_quiet = st.last_tx_quiet_log.time_since_epoch().count() == 0 ||
                                   (now_quiet - st.last_tx_quiet_log) > std::chrono::seconds(2);
            if (log_quiet) {
                EVLOG_warning << "Entering CAN TX quiet mode on plc_id=" << st.plc_id
                              << " after repeated ENOBUFS";
                st.last_tx_quiet_log = now_quiet;
            }
        }
        return;
    }
    st.tx_fast_ok++;
    st.tx_failure_streak = 0;
    st.tx_quiet_until = std::chrono::steady_clock::time_point{};
    st.last_tx_ok = std::chrono::steady_clock::now();
    st.last_tx_warn = std::chrono::steady_clock::time_point{};
    st.last_present_payload = cmp;
    st.last_present_payload_valid = true;
    st.last_present_tx = now;
    st.last_relay_tx = now;
    if (had_clear_faults) {
        st.relay_clear_faults = false;
    }
}

void PlcCanHardware::update_slow_tx(PlcState& st,
                                    std::chrono::steady_clock::time_point now,
                                    int backoff_factor,
                                    std::unique_lock<std::mutex>& state_lock) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_limits_tx).count();
    const double cfg_v = (st.cfg.max_voltage_v > 0.0) ? st.cfg.max_voltage_v
                                                     : (cfg_.default_voltage_v > 0.0 ? cfg_.default_voltage_v
                                                                                     : kDefaultVoltageV);
    const double cfg_i = (st.cfg.max_current_a > 0.0)
                             ? st.cfg.max_current_a
                             : (st.cfg.max_power_w > 0.0 && cfg_v > 0.0 ? st.cfg.max_power_w / 1000.0 / cfg_v
                                                                        : kDefaultCurrentA);
    const double cfg_p_kw = (st.cfg.max_power_w > 0.0) ? st.cfg.max_power_w / 1000.0 : 0.0;
    double max_v = (st.limits.max_voltage_v && st.limits.max_voltage_v.value() > 0.0)
                       ? st.limits.max_voltage_v.value()
                       : cfg_v;
    double max_i = (st.limits.max_current_a && st.limits.max_current_a.value() > 0.0)
                       ? st.limits.max_current_a.value()
                       : cfg_i;
    double max_p_kw =
        (st.limits.max_power_kw && st.limits.max_power_kw.value() > 0.0)
            ? st.limits.max_power_kw.value()
            : ((cfg_p_kw > 0.0) ? cfg_p_kw : (max_v * max_i / 1000.0));
    if (max_v <= 0.0) max_v = cfg_v;
    if (max_i <= 0.0) max_i = cfg_i;
    if (max_p_kw <= 0.0) max_p_kw = (max_v > 0.0 && max_i > 0.0) ? (max_v * max_i / 1000.0) : cfg_p_kw;

    const uint16_t v = clamp_to_0p5(max_v);
    const uint16_t i = clamp_to_0p2_current(max_i);
    const uint16_t p = clamp_to_0p5k(max_p_kw);

    bool want_auth = st.hlc_auth_granted;
    bool want_pending = st.hlc_auth_pending;
    if (st.desired_auth_state != AuthorizationState::Unknown) {
        want_auth = (st.desired_auth_state == AuthorizationState::Granted);
        want_pending = (st.desired_auth_state == AuthorizationState::Pending);
    }

    const auto payload = can_contract::build_evse_slow(v,
                                                       i,
                                                       p,
                                                       want_auth,
                                                       want_pending,
                                                       st.hlc_enabled,
                                                       st.pnc_blocked,
                                                       st.lock_command,
                                                       use_crc_);
    const bool payload_changed = !st.last_limits_payload_valid || (payload != st.last_limits_payload);
    const int keepalive_ms =
        compute_interval_ms(tx_limits_base_ms_, kMinTxLimitsMs, kLimitsMaxIntervalMs, backoff_factor);
    const int change_ms = compute_interval_ms(kMinTxLimitsMs, kMinTxLimitsMs, kLimitsMaxIntervalMs, backoff_factor);
    const bool allow_change_tx = elapsed >= change_ms;
    const bool allow_keepalive_tx = elapsed >= keepalive_ms;
    if (st.tx_quiet_until.time_since_epoch().count() != 0 && now < st.tx_quiet_until) {
        return;
    }
    if (!(payload_changed ? allow_change_tx : allow_keepalive_tx)) {
        return;
    }
    state_lock.unlock();
    int tx_errno = 0;
    const bool sent =
        send_frame_raw(st.iface,
                       st.plc_id,
                       can_contract::evse_slow_id(static_cast<uint8_t>(st.plc_id)),
                       payload,
                       TxPriority::Telemetry,
                       &tx_errno);
    state_lock.lock();
    if (!sent) {
        st.tx_slow_fail++;
        if (tx_errno == ENOBUFS) {
            st.tx_errno_enobufs++;
        } else if (tx_errno == EAGAIN || tx_errno == EWOULDBLOCK) {
            st.tx_errno_eagain++;
        } else if (tx_errno != 0) {
            st.tx_errno_other++;
        }
        if (tx_errno == ENOBUFS || tx_errno == EAGAIN || tx_errno == EWOULDBLOCK) {
            const int level = std::max(0, std::min(3, backpressure_level_.load(std::memory_order_relaxed)));
            st.tx_backpressure_level_hits[static_cast<std::size_t>(level)]++;
        }
        const auto warn_now = std::chrono::steady_clock::now();
        const bool log_now = st.last_tx_warn.time_since_epoch().count() == 0 ||
                             (warn_now - st.last_tx_warn) > std::chrono::milliseconds(500);
        if (log_now) {
            EVLOG_warning << "Failed to TX EVSE_SLOW on CAN for plc_id=" << st.plc_id;
            st.last_tx_warn = warn_now;
        }
        st.tx_failures++;
        st.tx_errors_recent++;
    } else {
        st.tx_slow_ok++;
        st.tx_failure_streak = 0;
        st.tx_quiet_until = std::chrono::steady_clock::time_point{};
        st.last_tx_ok = std::chrono::steady_clock::now();
        st.last_tx_warn = std::chrono::steady_clock::time_point{};
        st.last_limits_payload = payload;
        st.last_limits_payload_valid = true;
        st.last_limits_tx = now;
    }
}

void PlcCanHardware::emit_autocharge_token(PlcState& st, const std::string& id_token,
                                           std::chrono::steady_clock::time_point now) {
    if (id_token.empty()) {
        return;
    }
    AuthToken token;
    token.id_token = id_token;
    token.source = AuthTokenSource::Autocharge;
    token.connector_hint = st.connector_id;
    token.received_at = now;
    std::lock_guard<std::mutex> tok_lock(token_mutex_);
    pending_tokens_.push_back(std::move(token));
}

void PlcCanHardware::maybe_emit_emaid(PlcState& st, std::chrono::steady_clock::time_point now) {
    if (autocharge_source_ != AutochargeIdSource::Emaid) {
        return;
    }
    const bool have0 = !st.emaid0_cache.empty();
    const bool have1 = !st.emaid1_cache.empty();
    if (have0 && have1) {
        std::vector<uint8_t> combined = st.emaid0_cache;
        combined.insert(combined.end(), st.emaid1_cache.begin(), st.emaid1_cache.end());
        st.emaid0_cache.clear();
        st.emaid1_cache.clear();
        emit_autocharge_token(st, format_token_bytes(combined), now);
        return;
    }
    const auto timeout = std::chrono::milliseconds(kSegmentTimeoutMs);
    if (have0 && (now - st.emaid0_rx) > timeout) {
        auto token = format_token_bytes(st.emaid0_cache);
        st.emaid0_cache.clear();
        emit_autocharge_token(st, token, now);
        return;
    }
    if (have1 && (now - st.emaid1_rx) > timeout) {
        auto token = format_token_bytes(st.emaid1_cache);
        st.emaid1_cache.clear();
        emit_autocharge_token(st, token, now);
    }
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
    std::unique_lock<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    // "Enable EVSE" should not energize contactors/modules. Power delivery is handled by apply_power_command()
    // when a session is ready and the planner requests relays.
    st.output_enabled = false;
    st.regulating = false;
    st.desired_sys_enable = true;
    if (st.cfg.require_lock) {
        set_lock_command(st, true);
    }
    (void)set_relay_command(st, false, 0x00u, false);
    st.relay_state_dirty = true;
    st.relay_tx_urgent = true;
    update_fast_tx(st, std::chrono::steady_clock::now(), 1, lock);
    return true;
}

bool PlcCanHardware::disable(std::int32_t connector) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.output_enabled = false;
    st.regulating = false;
    st.desired_sys_enable = false;
    (void)set_relay_command(st, false, 0x00u, true);
    st.relay_state_dirty = true;
    st.relay_tx_urgent = true;
    update_fast_tx(st, std::chrono::steady_clock::now(), 1, lock);
    return true;
}

bool PlcCanHardware::pause_charging(std::int32_t connector) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.output_enabled = false;
    st.regulating = false;
    // Pause should stop power delivery but keep the comms/pilot plane alive.
    st.desired_sys_enable = true;
    (void)set_relay_command(st, false, 0x00u, false);
    st.relay_state_dirty = true;
    st.relay_tx_urgent = true;
    update_fast_tx(st, std::chrono::steady_clock::now(), 1, lock);
    return true;
}

bool PlcCanHardware::resume_charging(std::int32_t connector) {
    return enable(connector);
}

bool PlcCanHardware::stop_transaction(std::int32_t connector, ocpp::v16::Reason) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return false;
    auto& st = it->second;
    st.output_enabled = false;
    st.regulating = false;
    // StopTransaction should stop power delivery but keep the comms/pilot plane alive so the EV
    // can transition to finishing/unplug cleanly without flapping CP presence.
    st.desired_sys_enable = true;
    (void)set_relay_command(st, false, 0x00u, true);
    st.relay_state_dirty = true;
    st.relay_tx_urgent = true;
    update_fast_tx(st, std::chrono::steady_clock::now(), 1, lock);
    return true;
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
    (void)request;
    EVLOG_warning << "Diagnostics upload not supported in PLC backend";
    ocpp::v16::GetLogResponse resp{};
    resp.status = ocpp::v16::LogStatusEnumType::Rejected;
    return resp;
}

ocpp::v16::GetLogResponse PlcCanHardware::upload_logs(const ocpp::v16::GetLogRequest& request) {
    (void)request;
    EVLOG_warning << "Log upload not supported in PLC backend";
    ocpp::v16::GetLogResponse resp{};
    resp.status = ocpp::v16::LogStatusEnumType::Rejected;
    return resp;
}

bool PlcCanHardware::update_firmware(const ocpp::v16::UpdateFirmwareRequest& request) {
    (void)request;
    EVLOG_warning << "Firmware update not supported in PLC backend";
    return false;
}

ocpp::v16::UpdateFirmwareStatusEnumType
PlcCanHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest& request) {
    (void)request;
    EVLOG_warning << "Signed firmware update not supported in PLC backend";
    return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
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
    auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    const bool relay_feedback = cfg_.plc_relay_feedback;

    const bool status_seen = st.last_status_rx.time_since_epoch().count() != 0;
    const bool status_stale =
        status_seen &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_status_rx).count() >
            telemetry_timeout_ms_;

    const bool relay_seen = st.last_relay_rx.time_since_epoch().count() != 0;
    const bool relay_stale =
        relay_seen &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_relay_rx).count() >
            telemetry_timeout_ms_;

    const bool safety_seen = st.last_safety_rx.time_since_epoch().count() != 0;
    const bool safety_stale =
        safety_seen &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_safety_rx).count() >
            telemetry_timeout_ms_;

    // Give the PLC extra time to start streaming status before flagging comm faults to avoid false positives.
    const auto startup_grace = std::chrono::milliseconds(std::max(5000, telemetry_timeout_ms_ * 2));
    const bool within_startup_grace =
        started_at_.time_since_epoch().count() != 0 && (now - started_at_) < startup_grace;

    // Treat "never received any status yet" as unknown during startup grace rather than a comm fault.
    // After the grace window, missing/stale frames become actionable.
    const bool comm_missing = !status_seen && !within_startup_grace;
    // RelayStatus comm_fault is transport health (not contactor aux feedback), so honor it even when
    // relay feedback contacts are unavailable on the harness.
    const bool relay_comm_fault = relay_seen && !relay_stale && st.last_relay.comm_fault;
    const bool comm_relevant = st.output_enabled || st.regulating || st.plugged_in || st.hlc_stage > 0 ||
                               st.hlc_precharge_active || st.hlc_charge_complete || st.desired_relay_cmd_mask != 0 ||
                               st.desired_sys_enable;
    const bool raw_comm_fault =
        comm_relevant &&
        (status_stale || comm_missing || relay_stale || safety_stale || relay_comm_fault);

    // Safety inputs are only authoritative when their respective telemetry is fresh.
    const bool relay_status_fresh = relay_feedback && relay_seen && !relay_stale;
    const bool safety_fresh = safety_seen && !safety_stale;
    const bool relay_safety_ok =
        (!relay_feedback || !relay_status_fresh) ? true : (st.last_relay.safety_ok && !st.last_relay.earth_fault);
    const bool safety_status_ok =
        !safety_fresh ? true : (st.last_safety.safety_ok && !st.last_safety.earth_fault);
    const bool raw_safety_ok = relay_safety_ok && safety_status_ok;
    const bool raw_estop =
        (relay_feedback ? (st.last_relay.estop_latched || st.last_relay.estop_input) : false) ||
        st.last_safety.estop_latched || st.last_safety.estop_input;
    const bool raw_earth_fault = (relay_feedback ? st.last_relay.earth_fault : false) || st.last_safety.earth_fault;

    auto debounced_active = [&](bool active, std::chrono::steady_clock::time_point& since,
                                std::chrono::milliseconds debounce) -> bool {
        if (!active) {
            since = std::chrono::steady_clock::time_point{};
            return false;
        }
        if (since.time_since_epoch().count() == 0) {
            since = now;
            return false;
        }
        return (now - since) >= debounce;
    };

	    constexpr std::chrono::milliseconds kSafetyTripDebounceMs(200);
	    constexpr std::chrono::milliseconds kCriticalTripDebounceMs(100);
	    // Relax comm debounce to reduce false positives when PLC is slow to report.
	    constexpr std::chrono::milliseconds kCommTripDebounceMs(6000);
	    const bool safety_trip = debounced_active(!raw_safety_ok, st.safety_trip_since, kSafetyTripDebounceMs);
	    const bool estop_trip = debounced_active(raw_estop, st.estop_trip_since, kCriticalTripDebounceMs);
	    const bool earth_trip = debounced_active(raw_earth_fault, st.earth_trip_since, kCriticalTripDebounceMs);
	    bool comm_trip = debounced_active(raw_comm_fault, st.comm_trip_since, kCommTripDebounceMs);

	    gs.comm_fault = comm_trip;
	    gs.safety_ok = !safety_trip;
	    gs.estop = estop_trip;
	    gs.earth_fault = earth_trip;
        // Map earth/IMD faults into the higher-level isolation fault used by the controller/PLC contract.
        gs.isolation_fault = earth_trip;
    if (relay_feedback) {
        gs.relay_closed = st.last_relay.relay[0];
    } else if (cfg_.plc_owns_gun_relay) {
        // No contact feedback and gun relay is owned by the PLC: fall back to EVSE-side output enable hint.
        gs.relay_closed = st.output_enabled;
    } else {
        // No contact feedback: report our last commanded (TX-applied) gun relay state.
        gs.relay_closed = st.sys_enable && !st.relay_force_off && (st.relay_cmd_mask & kRelayGunMask);
    }
    const bool idle_paths =
        !st.regulating && !st.output_enabled && !st.plugged_in && st.hlc_stage == 0 && !st.hlc_precharge_active &&
        !st.hlc_charge_complete;
    if (idle_paths) {
        gs.relay_closed = false; // Assume open when no feedback hardware and no active session.
    }
    const bool relay_fresh =
        relay_feedback &&
        st.last_relay_rx.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_relay_rx).count() <=
            telemetry_timeout_ms_;
    if (relay_fresh && cfg_.plc_module_relays_enabled) {
        struct RelayBit {
            uint8_t mask;
            int idx;
        };
        const RelayBit module_bits[] = {
            {kRelayModule0Mask, 1},
            {kRelayModule1Mask, 2},
        };
        for (const auto& bit : module_bits) {
            const bool expected_on =
                (st.relay_enable_mask & bit.mask) && (st.relay_cmd_mask & bit.mask) && !st.relay_force_off;
            const bool actual_on = st.last_relay.relay[bit.idx] && !st.last_relay.relay_fault[bit.idx];
            // Only treat "expected OFF but still ON" as a critical weld/stuck contactor condition.
            // "expected ON but still OFF" can occur transiently during power-up sequencing (e.g., contactor supply
            // comes up after the PLC/EVSE) and should not immediately trip OCPP Faulted.
            if (!expected_on && actual_on) {
                if (st.relay_mismatch_since[bit.idx].time_since_epoch().count() == 0) {
                    st.relay_mismatch_since[bit.idx] = now;
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(now - st.relay_mismatch_since[bit.idx])
                               .count() >= 500) {
                    st.relay_conflict_count++;
                    st.relay_mismatch_since[bit.idx] = now;
                }
            } else {
                st.relay_mismatch_since[bit.idx] = std::chrono::steady_clock::time_point{};
            }
        }
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

    const bool plugged_raw = cp_fresh ? cp_connected : (hlc_fresh && st.hlc_stage > 0);
    if (st.plugged_raw_changed.time_since_epoch().count() == 0) {
        st.plugged_raw = plugged_raw;
        st.plugged_in = plugged_raw;
        st.plugged_raw_changed = now;
    } else {
        if (plugged_raw != st.plugged_raw) {
            st.plugged_raw = plugged_raw;
            st.plugged_raw_changed = now;
        }
        const int debounce_ms = st.plugged_raw ? kPlugInDebounceMs : kPlugOutDebounceMs;
        if (st.plugged_in != st.plugged_raw &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.plugged_raw_changed).count() >=
                debounce_ms) {
            const bool prev = st.plugged_in;
            st.plugged_in = st.plugged_raw;
            EVLOG_debug << "Connector " << connector << " plugged_in=" << (st.plugged_in ? "true" : "false")
                        << " (was " << (prev ? "true" : "false") << ", cp_state=" << cp_state
                        << ", cp_fresh=" << (cp_fresh ? "true" : "false") << ")";
        }
    }

    const bool cp_fault_raw = cp_fresh && (cp_state == 'E' || cp_state == 'F');
    if (st.cp_fault_raw_changed.time_since_epoch().count() == 0) {
        st.cp_fault_raw = cp_fault_raw;
        st.cp_fault = cp_fault_raw;
        st.cp_fault_raw_changed = now;
    } else {
        if (cp_fault_raw != st.cp_fault_raw) {
            st.cp_fault_raw = cp_fault_raw;
            st.cp_fault_raw_changed = now;
        }
        const int debounce_ms = st.cp_fault_raw ? kCpFaultOnDebounceMs : kCpFaultOffDebounceMs;
        if (st.cp_fault != st.cp_fault_raw &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.cp_fault_raw_changed).count() >=
                debounce_ms) {
            const bool prev = st.cp_fault;
            st.cp_fault = st.cp_fault_raw;
            EVLOG_debug << "Connector " << connector << " cp_fault=" << (st.cp_fault ? "true" : "false")
                        << " (was " << (prev ? "true" : "false") << ", cp_state=" << cp_state
                        << ", cp_fresh=" << (cp_fresh ? "true" : "false") << ")";
        }
    }

    gs.plugged_in = st.plugged_in;
    gs.cp_fault = st.cp_fault;
    const bool lock_required = st.cfg.require_lock;
    const bool lock_fresh = st.lock_engaged_valid && chargeinfo_fresh;
    // Treat lock status as "unknown" until we have fresh ChargeInfo frames.
    // We must not fault/abort sessions purely because the PLC hasn't yet reported lock feedback,
    // especially during early ISO15118 stages where the EV is still authorizing.
    //
    // Once ChargeInfo is fresh, lock feedback becomes authoritative.
    if (!lock_required) {
        gs.lock_engaged = true;
    } else if (!lock_fresh) {
        gs.lock_engaged = true;
    } else {
        gs.lock_engaged = st.lock_engaged;
    }
    const bool auth_granted = chargeinfo_fresh ? st.hlc_auth_granted : st.authorized;
    gs.authorization_granted = auth_granted;
    gs.module_healthy_mask = 0x03;
    gs.module_fault_mask = 0x00;
    gs.hlc_stage = hlc_fresh ? st.hlc_stage : 0;
    // `cable_checked` comes from the PLC HLC state machine. Treat it as "ok" when completed.
    gs.hlc_cable_check_ok = chargeinfo_fresh ? st.hlc_cable_checked : false;
    const bool precharge_from_stage =
        chargeinfo_fresh && st.hlc_stage == static_cast<uint8_t>(kHlcMinPowerStage - 1) && !st.hlc_charge_complete;
    gs.hlc_precharge_active =
        chargeinfo_fresh ? (st.hlc_precharge_active || precharge_from_stage) : false;
    gs.hlc_charge_complete = chargeinfo_fresh ? st.hlc_charge_complete : false;
    // Treat stage >= POWER_DELIVERY as power-ready even if `precharge_active` is stuck high.
    const bool hlc_ready = hlc_fresh && st.hlc_stage >= kHlcMinPowerStage && !gs.hlc_charge_complete;
    gs.hlc_power_ready = hlc_ready;
    if (relay_feedback && st.last_relay_rx.time_since_epoch().count() != 0 &&
        st.last_safety_rx.time_since_epoch().count() != 0) {
        gs.last_telemetry = std::min(st.last_relay_rx, st.last_safety_rx);
    } else if (st.last_safety_rx.time_since_epoch().count() != 0) {
        gs.last_telemetry = st.last_safety_rx;
    }
    gs.relay_conflict_count = st.relay_conflict_count;
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
    } else if (evse_present_fresh) {
        pv = st.present_voltage_v;
        pc = st.present_current_a;
        pp_kw = st.present_power_kw;
    } else if (ev_targets_fresh) {
        // PLC telemetry provides target V/I; use these as a fallback estimate when no present reading is available.
        pv = st.ev_target_voltage_v;
        pc = st.ev_target_current_a;
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
        gs.target_voltage_v.reset();
        gs.target_current_a.reset();
    }
    const double cfg_v = (st.cfg.max_voltage_v > 0.0) ? st.cfg.max_voltage_v
                                                      : (cfg_.default_voltage_v > 0.0 ? cfg_.default_voltage_v
                                                                                      : kDefaultVoltageV);
    const double cfg_i = (st.cfg.max_current_a > 0.0)
                             ? st.cfg.max_current_a
                             : (st.cfg.max_power_w > 0.0 && cfg_v > 0.0 ? st.cfg.max_power_w / 1000.0 / cfg_v
                                                                        : kDefaultCurrentA);
    const double cfg_p_kw = (st.cfg.max_power_w > 0.0) ? st.cfg.max_power_w / 1000.0 : 0.0;
    double max_v = (st.limits.max_voltage_v && st.limits.max_voltage_v.value() > 0.0)
                       ? st.limits.max_voltage_v.value()
                       : cfg_v;
    double max_i = (st.limits.max_current_a && st.limits.max_current_a.value() > 0.0)
                       ? st.limits.max_current_a.value()
                       : cfg_i;
    double max_p_kw =
        (st.limits.max_power_kw && st.limits.max_power_kw.value() > 0.0)
            ? st.limits.max_power_kw.value()
            : ((cfg_p_kw > 0.0) ? cfg_p_kw : (max_v > 0.0 && max_i > 0.0 ? (max_v * max_i / 1000.0) : 0.0));
    if (max_v <= 0.0) max_v = cfg_v;
    if (max_i <= 0.0) max_i = cfg_i;
    if (max_p_kw <= 0.0) {
        max_p_kw = (max_v > 0.0 && max_i > 0.0) ? (max_v * max_i / 1000.0) : cfg_p_kw;
    }
    gs.evse_max_voltage_v = max_v > 0.0 ? std::optional<double>(max_v) : std::nullopt;
    gs.evse_max_current_a = max_i > 0.0 ? std::optional<double>(max_i) : std::nullopt;
    gs.evse_max_power_kw = max_p_kw > 0.0 ? std::optional<double>(max_p_kw) : std::nullopt;
    gs.evse_limit_ack_count = st.evse_limit_ack_count;
    gs.last_evse_limit_ack = st.last_evse_limit_ack;
    const bool crc_relay_bad = relay_seen ? !st.last_relay.crc_ok : false;
    const bool crc_safety_bad = safety_seen ? !st.last_safety.crc_ok : false;
    gs.comm_fault = gs.comm_fault || crc_relay_bad || crc_safety_bad;
    const bool idle_system = idle_paths && !st.regulating && !st.output_enabled;
    if (idle_system) {
        // Idle + no active session: suppress transport-induced comm faults and safety/earth trips to avoid
        // spurious OCPP Faulted while the site is quiescent. Keep estop authoritative even while idle.
        st.comm_trip_since = std::chrono::steady_clock::time_point{};
        st.safety_trip_since = std::chrono::steady_clock::time_point{};
        st.earth_trip_since = std::chrono::steady_clock::time_point{};
        gs.comm_fault = false;
        gs.safety_ok = true;
        gs.earth_fault = false;
        gs.isolation_fault = false;
        gs.cp_fault = false;
    }
    gs.backpressure_level = static_cast<uint8_t>(backpressure_level_.load(std::memory_order_relaxed));
    gs.degraded_mode = gs.comm_fault || gs.backpressure_level > 0;
    gs.tx_error_count = st.tx_failures;
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
    st.desired_auth_state = state;
    st.authorized = (state == AuthorizationState::Granted);
    st.auth_pending = (state == AuthorizationState::Pending);
    st.last_limits_tx = std::chrono::steady_clock::time_point{};
    st.last_limits_payload_valid = false;
}

void PlcCanHardware::set_digital_comm_enabled(std::int32_t connector, bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) {
        return;
    }
    auto& st = it->second;
    st.hlc_enabled = enabled;
    st.last_limits_tx = std::chrono::steady_clock::time_point{};
    st.last_limits_payload_valid = false;
}

void PlcCanHardware::set_pnc_blocked(std::int32_t connector, bool blocked) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) {
        return;
    }
    auto& st = it->second;
    st.pnc_blocked = blocked;
    st.last_limits_tx = std::chrono::steady_clock::time_point{};
    st.last_limits_payload_valid = false;
}

void PlcCanHardware::apply_power_command(const PowerCommand& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(cmd.connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    // `cmd.module_mask` drives the two auxiliary relay outputs as KM_A/KM_B
    // (module bus sectionalizers owned by this PLC).
    uint8_t relay_mask = static_cast<uint8_t>(cmd.module_mask & 0x03u);
    if (relay_mask == 0u && cmd.module_count > 0 && cmd.mc_closed && cfg_.plc_module_relays_enabled) {
        const int capped = std::max(0, std::min(cmd.module_count, 2));
        const uint8_t fallback_mask = capped > 0 ? static_cast<uint8_t>((1u << capped) - 1u) : 0u;
        if (fallback_mask != 0u) {
            static std::map<int, std::chrono::steady_clock::time_point> last_log;
            auto& last_ts = last_log[cmd.connector];
            if (last_ts.time_since_epoch().count() == 0 || (now - last_ts) > std::chrono::seconds(1)) {
                EVLOG_warning << "PLC fallback module mask on connector " << cmd.connector
                              << " mask=0x" << std::hex << static_cast<int>(fallback_mask) << std::dec
                              << " module_count=" << cmd.module_count;
                last_ts = now;
            }
        }
        relay_mask = fallback_mask;
    }
    bool gun_on = cmd.gc_closed;
    bool force_off = false;
    if (cfg_.use_plc) {
        const bool hlc_fresh =
            st.last_chargeinfo_rx.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_chargeinfo_rx).count() <=
                telemetry_timeout_ms_;
        // Treat only the explicit PRECHARGE stage (kHlcMinPowerStage-1) as "precharge".
        // CableCheck and earlier stages must not energize DC relays/contactors.
        const bool precharge_from_stage =
            hlc_fresh &&
            st.hlc_stage == static_cast<uint8_t>(kHlcMinPowerStage - 1) &&
            !st.hlc_charge_complete;
        const bool hlc_precharge_phase = hlc_fresh && (st.hlc_precharge_active || precharge_from_stage);
        const bool hlc_power_phase = hlc_fresh && st.hlc_stage >= kHlcMinPowerStage && !st.hlc_charge_complete;
        const bool hlc_active = hlc_precharge_phase || hlc_power_phase;
        if (hlc_active) {
            st.last_hlc_active = now;
        }
        if (st.hlc_charge_complete || !st.plugged_in) {
            st.last_hlc_active = std::chrono::steady_clock::time_point{};
        }
        const bool hlc_recent =
            st.last_hlc_active.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_hlc_active) <=
                std::chrono::milliseconds(kHlcTransitionGraceMs);
        const bool ev_targets_fresh =
            st.last_ev_targets_rx.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_ev_targets_rx).count() <=
                telemetry_timeout_ms_;
        const bool cp_ready = (st.cp_state_session == 'C' || st.cp_state_session == 'D');
        const bool ev_requesting =
            ev_targets_fresh && cp_ready && st.ev_target_current_a > 0.5 && !st.hlc_charge_complete;
        if (ev_requesting) {
            st.last_hlc_active = now;
        }
        bool relays_closed = false;
        if (cfg_.plc_relay_feedback) {
            const bool relay_fresh =
                st.last_relay_rx.time_since_epoch().count() != 0 &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_relay_rx).count() <=
                    telemetry_timeout_ms_;
            if (relay_fresh) {
                relays_closed = st.last_relay.relay[0] || st.last_relay.relay[1] || st.last_relay.relay[2];
            }
        } else {
            relays_closed = st.relay_cmd_mask != 0u;
        }
        const bool hlc_allows_relays =
            (hlc_active || (hlc_recent && relays_closed) || ev_requesting) && st.plugged_in && !st.hlc_charge_complete;
        if (!hlc_allows_relays) {
            if (gun_on || relay_mask != 0u) {
                static std::map<int, std::chrono::steady_clock::time_point> last_log;
                auto& last_ts = last_log[cmd.connector];
                if (last_ts.time_since_epoch().count() == 0 || (now - last_ts) > std::chrono::seconds(1)) {
                    EVLOG_info << "Relay close blocked (HLC inactive)"
                               << " connector=" << cmd.connector
                               << " hlc_stage=" << static_cast<int>(st.hlc_stage)
                               << " precharge=" << (st.hlc_precharge_active ? "1" : "0")
                               << " charge_complete=" << (st.hlc_charge_complete ? "1" : "0")
                               << " fresh=" << (hlc_fresh ? "1" : "0")
                               << " ev_req=" << (ev_requesting ? "1" : "0")
                               << " cp=" << st.cp_state_session
                               << " targetI=" << st.ev_target_current_a
                               << " targetV=" << st.ev_target_voltage_v
                               << " relays_closed=" << (relays_closed ? "1" : "0");
                    last_ts = now;
                }
            }
            gun_on = false;
            relay_mask = 0u;
            force_off = true;
        }
    }
    const bool any_relays = gun_on || (cfg_.plc_module_relays_enabled && relay_mask != 0);
    st.output_enabled = gun_on;
    st.regulating = any_relays;
    if (!st.desired_sys_enable) {
        st.output_enabled = false;
        st.regulating = false;
    }
    if (any_relays && st.cfg.require_lock) {
        set_lock_command(st, true);
    }
    EvseLimits limits{};
    limits.max_voltage_v = cmd.voltage_set_v > 0.0 ? cmd.voltage_set_v : st.limits.max_voltage_v;
    limits.max_current_a = cmd.current_limit_a > 0.0 ? cmd.current_limit_a : st.limits.max_current_a;
    limits.max_power_kw = cmd.power_kw > 0.0 ? cmd.power_kw : st.limits.max_power_kw;
    if (!limits_equal(st.limits, limits)) {
        st.limits = limits;
    }
    (void)set_relay_command(st, gun_on, relay_mask, force_off);
}

void PlcCanHardware::apply_power_allocation(std::int32_t connector, int modules) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    const int capped = std::max(0, std::min(modules, 2));
    uint8_t module_mask = capped > 0 ? static_cast<uint8_t>((1u << capped) - 1u) : 0u;
    bool want_power = capped > 0;
    bool force_off = false;
    if (cfg_.use_plc) {
        const bool hlc_fresh =
            st.last_chargeinfo_rx.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_chargeinfo_rx).count() <=
                telemetry_timeout_ms_;
        // Treat only the explicit PRECHARGE stage (kHlcMinPowerStage-1) as "precharge".
        // CableCheck and earlier stages must not energize DC relays/contactors.
        const bool precharge_from_stage =
            hlc_fresh &&
            st.hlc_stage == static_cast<uint8_t>(kHlcMinPowerStage - 1) &&
            !st.hlc_charge_complete;
        const bool hlc_precharge_phase = hlc_fresh && (st.hlc_precharge_active || precharge_from_stage);
        const bool hlc_power_phase = hlc_fresh && st.hlc_stage >= kHlcMinPowerStage && !st.hlc_charge_complete;
        const bool hlc_active = hlc_precharge_phase || hlc_power_phase;
        if (hlc_active) {
            st.last_hlc_active = now;
        }
        if (st.hlc_charge_complete || !st.plugged_in) {
            st.last_hlc_active = std::chrono::steady_clock::time_point{};
        }
        const bool hlc_recent =
            st.last_hlc_active.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_hlc_active) <=
                std::chrono::milliseconds(kHlcTransitionGraceMs);
        const bool ev_targets_fresh =
            st.last_ev_targets_rx.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_ev_targets_rx).count() <=
                telemetry_timeout_ms_;
        const bool cp_ready = (st.cp_state_session == 'C' || st.cp_state_session == 'D');
        const bool ev_requesting =
            ev_targets_fresh && cp_ready && st.ev_target_current_a > 0.5 && !st.hlc_charge_complete;
        if (ev_requesting) {
            st.last_hlc_active = now;
        }
        bool relays_closed = false;
        if (cfg_.plc_relay_feedback) {
            const bool relay_fresh =
                st.last_relay_rx.time_since_epoch().count() != 0 &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_relay_rx).count() <=
                    telemetry_timeout_ms_;
            if (relay_fresh) {
                relays_closed = st.last_relay.relay[0] || st.last_relay.relay[1] || st.last_relay.relay[2];
            }
        } else {
            relays_closed = st.relay_cmd_mask != 0u;
        }
        const bool hlc_allows_relays =
            (hlc_active || (hlc_recent && relays_closed) || ev_requesting) && st.plugged_in && !st.hlc_charge_complete;
        if (!hlc_allows_relays) {
            if (want_power) {
                static std::map<int, std::chrono::steady_clock::time_point> last_log;
                auto& last_ts = last_log[connector];
                if (last_ts.time_since_epoch().count() == 0 || (now - last_ts) > std::chrono::seconds(1)) {
                    EVLOG_info << "Relay close blocked (HLC inactive)"
                               << " connector=" << connector
                               << " hlc_stage=" << static_cast<int>(st.hlc_stage)
                               << " precharge=" << (st.hlc_precharge_active ? "1" : "0")
                               << " charge_complete=" << (st.hlc_charge_complete ? "1" : "0")
                               << " fresh=" << (hlc_fresh ? "1" : "0")
                               << " ev_req=" << (ev_requesting ? "1" : "0")
                               << " cp=" << st.cp_state_session
                               << " targetI=" << st.ev_target_current_a
                               << " targetV=" << st.ev_target_voltage_v
                               << " relays_closed=" << (relays_closed ? "1" : "0");
                    last_ts = now;
                }
            }
            want_power = false;
            module_mask = 0u;
            force_off = true;
        }
    }
    st.output_enabled = want_power;
    st.regulating = want_power;
    if (!st.desired_sys_enable) {
        st.output_enabled = false;
        st.regulating = false;
    }
    if (want_power && st.cfg.require_lock) {
        set_lock_command(st, true);
    }
    (void)set_relay_command(st, want_power, module_mask, force_off);
}

void PlcCanHardware::set_evse_limits(std::int32_t connector, const EvseLimits& limits) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    if (!limits_equal(st.limits, limits)) {
        st.limits = limits;
    }
}

void PlcCanHardware::publish_evse_present(std::int32_t connector, double voltage_v, double current_a, double power_kw,
                                          bool output_enabled, bool regulating) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    const auto now = std::chrono::steady_clock::now();
    st.present_voltage_v = voltage_v;
    st.present_current_a = current_a;
    st.present_power_kw = power_kw;
    if (!st.desired_sys_enable) {
        output_enabled = false;
        regulating = false;
    }
    st.output_enabled = output_enabled;
    st.regulating = regulating;
    st.last_evse_present_update = now;
}

void PlcCanHardware::publish_fault_state(std::int32_t connector, uint8_t fault_bits) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    if (st.fault_bits != fault_bits) {
        st.fault_bits = fault_bits;
    }
}

void PlcCanHardware::clear_faults(std::int32_t connector) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    auto it = connectors_.find(connector);
    if (it == connectors_.end()) return;
    auto& st = it->second;
    st.relay_clear_faults = true;
    st.relay_tx_urgent = true;
    update_fast_tx(st, std::chrono::steady_clock::now(), 1, lock);
}

std::vector<AuthToken> PlcCanHardware::poll_auth_tokens() {
    std::lock_guard<std::mutex> lock(token_mutex_);
    std::vector<AuthToken> tokens;
    tokens.swap(pending_tokens_);
    return tokens;
}

bool PlcCanHardware::supports_cross_slot_islands() const { return true; }

} // namespace charger

#else

namespace charger {

PlcCanHardware::PlcCanHardware(const ChargerConfig& cfg) : cfg_(cfg) {
    EVLOG_warning << "PLC CAN backend disabled: unsupported platform";
    init_ok_ = false;
}

PlcCanHardware::~PlcCanHardware() = default;

bool PlcCanHardware::enable(std::int32_t) { return false; }
bool PlcCanHardware::disable(std::int32_t) { return false; }
bool PlcCanHardware::pause_charging(std::int32_t) { return false; }
bool PlcCanHardware::resume_charging(std::int32_t) { return false; }
bool PlcCanHardware::stop_transaction(std::int32_t, ocpp::v16::Reason) { return false; }
ocpp::v16::UnlockStatus PlcCanHardware::unlock(std::int32_t) { return ocpp::v16::UnlockStatus::NotSupported; }
ocpp::v16::ReservationStatus PlcCanHardware::reserve(std::int32_t, std::int32_t, ocpp::DateTime,
                                                     const std::string&, const std::optional<std::string>&) {
    return ocpp::v16::ReservationStatus::Rejected;
}
bool PlcCanHardware::cancel_reservation(std::int32_t) { return false; }
ocpp::v16::GetLogResponse PlcCanHardware::upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest&) {
    ocpp::v16::GetLogResponse response;
    response.status = ocpp::v16::LogStatusEnumType::Rejected;
    return response;
}
ocpp::v16::GetLogResponse PlcCanHardware::upload_logs(const ocpp::v16::GetLogRequest&) {
    ocpp::v16::GetLogResponse response;
    response.status = ocpp::v16::LogStatusEnumType::Rejected;
    return response;
}
bool PlcCanHardware::update_firmware(const ocpp::v16::UpdateFirmwareRequest&) { return false; }
ocpp::v16::UpdateFirmwareStatusEnumType
PlcCanHardware::update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest&) {
    return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
}
void PlcCanHardware::set_connection_timeout(std::int32_t) {}
bool PlcCanHardware::is_reset_allowed(const ocpp::v16::ResetType&) { return false; }
void PlcCanHardware::reset(const ocpp::v16::ResetType&) {}
void PlcCanHardware::on_remote_start_token(const std::string&, const std::vector<std::int32_t>&, bool) {}
ocpp::Measurement PlcCanHardware::sample_meter(std::int32_t) { return {}; }
GunStatus PlcCanHardware::get_status(std::int32_t) { return {}; }
void PlcCanHardware::set_authorization_state(std::int32_t, bool) {}
void PlcCanHardware::set_authorization_state(std::int32_t, AuthorizationState) {}
void PlcCanHardware::set_digital_comm_enabled(std::int32_t, bool) {}
void PlcCanHardware::set_pnc_blocked(std::int32_t, bool) {}
void PlcCanHardware::apply_power_command(const PowerCommand&) {}
void PlcCanHardware::apply_power_allocation(std::int32_t, int) {}
void PlcCanHardware::set_evse_limits(std::int32_t, const EvseLimits&) {}
void PlcCanHardware::publish_evse_present(std::int32_t, double, double, double, bool, bool) {}
void PlcCanHardware::publish_fault_state(std::int32_t, uint8_t) {}
void PlcCanHardware::clear_faults(std::int32_t) {}
std::vector<AuthToken> PlcCanHardware::poll_auth_tokens() { return {}; }
bool PlcCanHardware::supports_cross_slot_islands() const { return false; }

} // namespace charger

#endif
