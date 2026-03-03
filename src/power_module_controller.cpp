// SPDX-License-Identifier: Apache-2.0
#include "power_module_controller.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <everest/logging.hpp>

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#else
// Minimal definitions to allow non-Linux builds to compile; no CAN I/O.
struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};
constexpr uint32_t CAN_EFF_FLAG = 0x80000000U;
constexpr uint32_t CAN_EFF_MASK = 0x1FFFFFFFU;
#endif

namespace charger {

namespace {
constexpr uint16_t MAXWELL_PROT_NO = 0x060;
constexpr uint8_t RECTIFIER_PROTO_NO = 0x01;
constexpr uint8_t MAXWELL_FUNC_SET = 0x03;
constexpr uint8_t MAXWELL_FUNC_READ = 0x10;
constexpr uint8_t MAXWELL_TYPE_FLOAT = 0x41;
constexpr uint8_t MAXWELL_TYPE_INT = 0x42;
constexpr uint8_t MAXWELL_OK = 0xF0;
constexpr uint8_t MAXWELL_CONTROLLER_ADDR = 0xF0;
constexpr double MAXWELL_ABSOLUTE_CURRENT_SCALE = 1024.0;
constexpr float MAXWELL_CURRENT_LIMIT_RATIO_MAX = 3.5f;
// Severe faults that should mark modules unusable.
constexpr uint32_t MAXWELL_ALARM_SEVERE_MASK =
    (1u << 0) | (1u << 1) | (1u << 4) | (1u << 5) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 14) |
    (1u << 16) | (1u << 17) | (1u << 27) | (1u << 28) | (1u << 30) | (1u << 31);
constexpr uint8_t MAXWELL_ALARM_ONOFF_BIT = 22; // 0=On, 1=Off per V1.50 table.
constexpr uint8_t MAXWELL_ALARM_POWER_LIMIT_BIT = 23;
constexpr uint8_t MAXWELL_ALARM_TEMP_DERATE_BIT = 24;
constexpr uint8_t MAXWELL_ALARM_AC_LIMIT_BIT = 25;
// Maxwell V1.50 (Table 1 + response format) reports read/write failures via non-F0 status,
// while actionable module fault semantics are exposed through the 0x0040 alarm/status bits.
// Use a wider confirmation window so transient response errors do not immediately trip modules.
constexpr int MAXWELL_STATUS_ERROR_DEBOUNCE_COUNT = 12;
constexpr auto MAXWELL_STATUS_ERROR_DEBOUNCE_TIME = std::chrono::milliseconds(3000);
constexpr auto MAXWELL_START_TIMEOUT = std::chrono::seconds(8);
constexpr auto MAXWELL_LIMIT_READBACK_SETTLE_TIME = std::chrono::milliseconds(2500);
constexpr int MAXWELL_LIMIT_MISMATCH_CONFIRM_COUNT = 6;
constexpr auto MAXWELL_LIMIT_MISMATCH_CONFIRM_TIME = std::chrono::milliseconds(5000);
constexpr double MAXWELL_LIMIT_MISMATCH_ABS_DELTA = 0.25;
constexpr double MAXWELL_LIMIT_MISMATCH_REL_DELTA = 0.35;
constexpr double MAXWELL_OUTPUT_VOLTAGE_MIN_SET_V = 80.0;
constexpr auto MODULE_IDLE_STATUS_POLL_INTERVAL = std::chrono::milliseconds(800);
constexpr auto MODULE_IDLE_TELEMETRY_POLL_INTERVAL = std::chrono::milliseconds(1500);
// Keep OFF-command keepalive sparse in lab to avoid unnecessary command churn on inactive modules.
constexpr auto MODULE_OFF_COMMAND_KEEPALIVE_INTERVAL = std::chrono::milliseconds(10000);
constexpr auto MODULE_OFF_STATE_RETRY_INTERVAL = std::chrono::milliseconds(2000);
constexpr auto RECTIFIER_START_TIMEOUT = std::chrono::seconds(8);
constexpr auto TONHE_START_TIMEOUT = std::chrono::seconds(8);
constexpr double MODULE_START_FAULT_MIN_CURRENT_A = 0.5;
// Keep steady-state control refresh aligned with configured cmdIntervalMs.
constexpr int MODULE_STABLE_CONTROL_REFRESH_FACTOR = 1;
constexpr int MODULE_STABLE_CONTROL_REFRESH_MIN_MS = 500;
constexpr int MODULE_STABLE_CONTROL_REFRESH_MAX_MS = 2000;
constexpr double MODULE_TRACKING_CURRENT_RATIO = 0.80;
constexpr double MODULE_TRACKING_CURRENT_MARGIN_A = 1.5;
constexpr int MODULE_CURRENT_PRIORITY_MIN_INTERVAL_MS = 180;

inline bool startup_load_requested(double current_a) {
    return current_a > MODULE_START_FAULT_MIN_CURRENT_A;
}

inline std::chrono::milliseconds current_priority_poll_interval(std::chrono::milliseconds poll_interval) {
    return std::chrono::milliseconds(
        std::max<int64_t>(MODULE_CURRENT_PRIORITY_MIN_INTERVAL_MS, poll_interval.count()));
}

inline std::chrono::milliseconds control_refresh_interval(const ModuleSpec& spec, bool stable_tracking) {
    const int base_ms = std::max(500, spec.cmd_interval_ms);
    if (!stable_tracking) {
        return std::chrono::milliseconds(base_ms);
    }
    const int64_t scaled_ms = static_cast<int64_t>(base_ms) * MODULE_STABLE_CONTROL_REFRESH_FACTOR;
    const int stable_ms = static_cast<int>(std::clamp<int64_t>(scaled_ms,
                                                                MODULE_STABLE_CONTROL_REFRESH_MIN_MS,
                                                                MODULE_STABLE_CONTROL_REFRESH_MAX_MS));
    return std::chrono::milliseconds(stable_ms);
}

inline bool module_current_tracking(double measured_a, double requested_a) {
    const double requested = std::max(0.0, requested_a);
    if (requested <= 0.5) {
        return true;
    }
    const double threshold = (requested <= 2.0)
                                 ? 0.5
                                 : std::max(MODULE_TRACKING_CURRENT_MARGIN_A,
                                            requested * MODULE_TRACKING_CURRENT_RATIO);
    return measured_a >= threshold;
}

constexpr uint32_t RECTIFIER_STATUS_IGNORE_MASK =
    (1u << 22) | (1u << 23) | (1u << 24) | (1u << 25) | (1u << 26) | (1u << 27) | (1u << 30);
constexpr uint32_t RECTIFIER_STATUS_FAULT_MASK = 0xFFFFFFFFu & ~RECTIFIER_STATUS_IGNORE_MASK;
constexpr uint8_t RECTIFIER_ALARM_AC_DERATE_BIT = 22;
constexpr uint8_t RECTIFIER_ALARM_TEMP_DERATE_BIT = 23;
constexpr uint8_t RECTIFIER_ALARM_PFC_OFF_BIT = 24;
constexpr uint8_t RECTIFIER_ALARM_MODULE_OFF_BIT = 25;

constexpr uint8_t RECTIFIER_MSG_SET = 0x00;
constexpr uint8_t RECTIFIER_MSG_SET_RESP = 0x01;
constexpr uint8_t RECTIFIER_MSG_READ = 0x02;
constexpr uint8_t RECTIFIER_MSG_READ_RESP = 0x03;
constexpr uint8_t RECTIFIER_MSG_READ_SN = 0x04;

constexpr uint8_t TONHE_STATE_OFF = 0x00;
constexpr uint8_t TONHE_STATE_ON = 0x01;
constexpr uint8_t TONHE_STATE_FAULT_OFF = 0x11;
constexpr uint8_t TONHE_CMD_STOP = 0x55;
constexpr uint8_t TONHE_CMD_START = 0xAA;
constexpr uint16_t TONHE_FAULT_SEVERE_MASK =
    (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 7) |
    (1u << 8) | (1u << 9) | (1u << 10) | (1u << 11) | (1u << 15);
constexpr uint16_t TONHE_EXT_FAULT_SEVERE_MASK =
    (1u << 0) | (1u << 1) | (1u << 2) | (1u << 4) | (1u << 6) | (1u << 7) | (1u << 10);
constexpr uint16_t TONHE_EXT_FAULT_INPUT_POWER_LIMIT_BIT = (1u << 8);
constexpr uint16_t TONHE_EXT_FAULT_TEMP_POWER_LIMIT_BIT = (1u << 9);

constexpr uint32_t TONHE_PGN_STATE = 0x000100;
constexpr uint32_t TONHE_PGN_STARTSTOP = 0x000600;
constexpr uint32_t TONHE_PGN_AC_INFO = 0x000B00;
constexpr uint32_t TONHE_PGN_EXT_STATUS = 0x009100;

struct ModuleSetpoint {
    bool enable{false};
    double voltage_v{0.0};
    double current_a{0.0};
};

std::string hex_u32(uint32_t value, int width) {
    std::ostringstream os;
    os << std::uppercase << std::hex << std::setfill('0') << std::setw(width) << value;
    return os.str();
}

struct ModuleTelemetryState {
    bool healthy{false};
    bool fault{false};
    double temperature_c{0.0};
    double voltage_v{0.0};
    double current_a{0.0};
    double current_limit_point{0.0};
    bool current_capability_valid{false};
    double current_capability_a{0.0};
    bool capability_flags_valid{false};
    bool module_off{false};
    bool power_limited{false};
    bool temp_derated{false};
    bool ac_limited{false};
    uint32_t alarms{0};
    uint16_t reported_group{0};
    uint16_t reported_address{0};
    uint32_t input_mode{0};
    uint8_t healthy_mask{0};
    uint8_t fault_mask{0};
    std::chrono::steady_clock::time_point last_voltage_update{};
    std::chrono::steady_clock::time_point last_current_update{};
    std::chrono::steady_clock::time_point last_update{};
};

int popcount(uint8_t v) {
    int count = 0;
    while (v) {
        count += (v & 0x1);
        v >>= 1U;
    }
    return count;
}

std::chrono::milliseconds telemetry_stale_interval(const ModuleSpec& spec) {
    int ms = spec.telemetry_stale_ms;
    if (ms <= 0) {
        const int fallback = std::max(3000, spec.poll_interval_ms * 4);
        ms = fallback;
    }
    return std::chrono::milliseconds(ms);
}

uint32_t encode_float_be(float value) {
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(float));
    return raw;
}

float decode_float_be(const uint8_t* buf) {
    uint32_t raw = (static_cast<uint32_t>(buf[0]) << 24) |
                   (static_cast<uint32_t>(buf[1]) << 16) |
                   (static_cast<uint32_t>(buf[2]) << 8) |
                   (static_cast<uint32_t>(buf[3]));
    float val = 0.0f;
    std::memcpy(&val, &raw, sizeof(float));
    return val;
}

uint32_t decode_u32_be(const uint8_t* buf) {
    return (static_cast<uint32_t>(buf[0]) << 24) |
           (static_cast<uint32_t>(buf[1]) << 16) |
           (static_cast<uint32_t>(buf[2]) << 8) |
           static_cast<uint32_t>(buf[3]);
}

uint16_t decode_u16_le(const uint8_t* buf) {
    return static_cast<uint16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8));
}

uint32_t build_tonhe_id(uint8_t priority, uint32_t pgn, uint8_t dest, uint8_t src) {
    const uint8_t pf = static_cast<uint8_t>((pgn >> 8) & 0xFF);
    const uint8_t ps = dest;
    const uint32_t id = (static_cast<uint32_t>(priority & 0x7) << 26) |
                        (static_cast<uint32_t>(pf) << 16) |
                        (static_cast<uint32_t>(ps) << 8) |
                        static_cast<uint32_t>(src);
    return id | CAN_EFF_FLAG;
}

struct CanFilterSpec {
    uint32_t id{0};
    uint32_t mask{0};
    bool enabled{false};
};

enum class TxClass { SafetyUrgent, Control, Telemetry };

class CanTrafficGovernor {
public:
    struct IfaceStatus {
        double total_kbps{0.0};
        double module_budget_kbps{0.0};
        bool budget_limited{false};
        bool overload_latched{false};
        uint64_t module_tx_frames{0};
        uint64_t module_rx_frames{0};
        uint64_t budget_drops{0};
        int64_t control_backlog_age_ms{0};
    };

    static CanTrafficGovernor& instance() {
        static CanTrafficGovernor g;
        return g;
    }

    void configure(const ModuleCanTrafficPolicy& policy, const std::set<std::string>& ifaces) {
        std::lock_guard<std::mutex> lock(mtx_);
        default_policy_ = policy;
        for (const auto& iface : ifaces) {
            auto& st = states_[iface];
            apply_policy_locked(st, iface);
            ensure_monitor_socket_locked(iface, st);
        }
        start_monitor_if_needed_locked();
    }

    void ensure_iface(const std::string& iface) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto& st = states_[iface];
        apply_policy_locked(st, iface);
        ensure_monitor_socket_locked(iface, st);
        start_monitor_if_needed_locked();
    }

    bool allow_send(const std::string& iface, TxClass tx_class, int expected_rx_frames) {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mtx_);
        auto& st = states_[iface];
        apply_policy_locked(st, iface);
        prune_locked(st, now);
        refresh_overcap_locked(st, now);
        maybe_log_locked(iface, st, now);

        const int rx_frames = std::max(0, expected_rx_frames);
        const uint64_t estimate_bits =
            static_cast<uint64_t>(st.bits_per_frame_estimate) * static_cast<uint64_t>(1 + rx_frames);
        if (tx_class == TxClass::SafetyUrgent) {
            return true;
        }

        // Do not budget-drop control writes. Dropping them causes stale setpoints and
        // can destabilize real sessions under transient CAN pressure.
        if (tx_class == TxClass::Control) {
            st.reserved_bits += estimate_bits;
            st.reserved_events.push_back({now, estimate_bits});
            st.control_backlog_since = std::chrono::steady_clock::time_point{};
            return true;
        }

        if (st.overload_latched && st.enforce) {
            st.budget_drops++;
            st.last_budget_drop = now;
            return false;
        }

        const uint64_t projected_bits = st.observed_bits + st.reserved_bits + estimate_bits;
        const double projected_kbps = window_kbps(st, projected_bits);

        if (st.enforce && projected_kbps > st.module_budget_kbps) {
            st.budget_drops++;
            st.last_budget_drop = now;
            return false;
        }

        st.reserved_bits += estimate_bits;
        st.reserved_events.push_back({now, estimate_bits});
        if (tx_class == TxClass::Control) {
            st.control_backlog_since = std::chrono::steady_clock::time_point{};
        }
        return true;
    }

    void note_send_result(const std::string& iface,
                          TxClass tx_class,
                          bool success,
                          int expected_rx_frames,
                          bool had_reservation) {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mtx_);
        auto& st = states_[iface];
        apply_policy_locked(st, iface);
        prune_locked(st, now);
        if (!success && had_reservation) {
            const int rx_frames = std::max(0, expected_rx_frames);
            const uint64_t estimate_bits =
                static_cast<uint64_t>(st.bits_per_frame_estimate) * static_cast<uint64_t>(1 + rx_frames);
            release_reserved_locked(st, estimate_bits);
        }
        if (success) {
            st.module_tx_frames++;
            st.tx_events.push_back(now);
            if (tx_class == TxClass::Control) {
                st.control_backlog_since = std::chrono::steady_clock::time_point{};
            }
        } else if (tx_class == TxClass::Control && st.control_backlog_since.time_since_epoch().count() == 0) {
            st.control_backlog_since = now;
        }
        refresh_overcap_locked(st, now);
        maybe_log_locked(iface, st, now);
    }

    bool overload_latched(const std::string& iface) {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mtx_);
        const auto it = states_.find(iface);
        if (it == states_.end()) {
            return false;
        }
        prune_locked(it->second, now);
        refresh_overcap_locked(it->second, now);
        return it->second.overload_latched;
    }

    IfaceStatus status(const std::string& iface) {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mtx_);
        const auto it = states_.find(iface);
        if (it == states_.end()) {
            return {};
        }
        auto& st = it->second;
        prune_locked(st, now);
        refresh_overcap_locked(st, now);
        IfaceStatus out;
        out.total_kbps = window_kbps(st, st.observed_bits + st.reserved_bits);
        out.module_budget_kbps = st.module_budget_kbps;
        out.budget_limited = st.last_budget_drop.time_since_epoch().count() != 0 &&
                             (now - st.last_budget_drop) <= std::chrono::milliseconds(st.window_ms);
        out.overload_latched = st.overload_latched;
        out.module_tx_frames = st.module_tx_frames;
        out.module_rx_frames = st.module_rx_frames;
        out.budget_drops = st.budget_drops;
        if (st.control_backlog_since.time_since_epoch().count() != 0) {
            out.control_backlog_age_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - st.control_backlog_since).count();
        }
        return out;
    }

    ~CanTrafficGovernor() {
        running_.store(false);
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
#ifdef __linux__
        for (auto& kv : states_) {
            if (kv.second.monitor_sock >= 0) {
                ::close(kv.second.monitor_sock);
                kv.second.monitor_sock = -1;
            }
        }
#endif
    }

private:
    struct BitEvent {
        std::chrono::steady_clock::time_point ts{};
        uint64_t bits{0};
    };

    struct IfaceState {
        double max_total_kbps{20.0};
        double plc_reserve_kbps{0.0};
        double module_budget_kbps{20.0};
        int window_ms{10000};
        int bits_per_frame_estimate{150};
        int over_cap_debounce_ms{5000};
        double over_cap_clear_ratio{0.85};
        int over_cap_clear_hold_ms{5000};
        bool enforce{true};
        std::deque<BitEvent> observed_events;
        std::deque<BitEvent> reserved_events;
        std::deque<std::chrono::steady_clock::time_point> tx_events;
        std::deque<std::chrono::steady_clock::time_point> rx_events;
        uint64_t observed_bits{0};
        uint64_t reserved_bits{0};
        uint64_t module_tx_frames{0};
        uint64_t module_rx_frames{0};
        uint64_t budget_drops{0};
        std::chrono::steady_clock::time_point last_budget_drop{};
        std::chrono::steady_clock::time_point control_backlog_since{};
        std::chrono::steady_clock::time_point over_cap_since{};
        std::chrono::steady_clock::time_point under_cap_since{};
        bool overload_latched{false};
        bool overcap_log_latched{false};
        std::chrono::steady_clock::time_point last_log{};
#ifdef __linux__
        int monitor_sock{-1};
#endif
    };

    CanTrafficGovernor() = default;
    CanTrafficGovernor(const CanTrafficGovernor&) = delete;
    CanTrafficGovernor& operator=(const CanTrafficGovernor&) = delete;

    static double window_kbps(const IfaceState& st, uint64_t bits) {
        const double window_s = std::max(0.001, static_cast<double>(st.window_ms) / 1000.0);
        return static_cast<double>(bits) / window_s / 1000.0;
    }

    static void prune_locked(IfaceState& st, const std::chrono::steady_clock::time_point& now) {
        const auto horizon = now - std::chrono::milliseconds(std::max(1000, st.window_ms));
        while (!st.observed_events.empty() && st.observed_events.front().ts < horizon) {
            st.observed_bits -= st.observed_events.front().bits;
            st.observed_events.pop_front();
        }
        while (!st.reserved_events.empty() && st.reserved_events.front().ts < horizon) {
            st.reserved_bits -= st.reserved_events.front().bits;
            st.reserved_events.pop_front();
        }
        while (!st.tx_events.empty() && st.tx_events.front() < horizon) {
            st.tx_events.pop_front();
        }
        while (!st.rx_events.empty() && st.rx_events.front() < horizon) {
            st.rx_events.pop_front();
        }
    }

    static void release_reserved_locked(IfaceState& st, uint64_t bits) {
        while (bits > 0 && !st.reserved_events.empty()) {
            auto& ev = st.reserved_events.back();
            const uint64_t take = std::min(bits, ev.bits);
            ev.bits -= take;
            st.reserved_bits -= take;
            bits -= take;
            if (ev.bits == 0) {
                st.reserved_events.pop_back();
            }
        }
        if (bits > 0) {
            st.reserved_bits = (st.reserved_bits > bits) ? (st.reserved_bits - bits) : 0;
        }
    }

    static void consume_reserved_locked(IfaceState& st, uint64_t bits) {
        while (bits > 0 && !st.reserved_events.empty()) {
            auto& ev = st.reserved_events.front();
            const uint64_t take = std::min(bits, ev.bits);
            ev.bits -= take;
            st.reserved_bits -= take;
            bits -= take;
            if (ev.bits == 0) {
                st.reserved_events.pop_front();
            }
        }
    }

    static void refresh_overcap_locked(IfaceState& st, const std::chrono::steady_clock::time_point& now) {
        const double observed_kbps = window_kbps(st, st.observed_bits);
        if (observed_kbps > st.max_total_kbps) {
            st.under_cap_since = std::chrono::steady_clock::time_point{};
            if (st.over_cap_since.time_since_epoch().count() == 0) {
                st.over_cap_since = now;
            } else if (!st.overload_latched && st.enforce &&
                       (now - st.over_cap_since) >= std::chrono::milliseconds(std::max(0, st.over_cap_debounce_ms))) {
                st.overload_latched = true;
            }
            return;
        }
        if (st.overload_latched) {
            const double clear_threshold = st.max_total_kbps * std::clamp(st.over_cap_clear_ratio, 0.5, 0.99);
            if (observed_kbps <= clear_threshold) {
                if (st.under_cap_since.time_since_epoch().count() == 0) {
                    st.under_cap_since = now;
                } else if ((now - st.under_cap_since) >=
                           std::chrono::milliseconds(std::max(0, st.over_cap_clear_hold_ms))) {
                    st.overload_latched = false;
                    st.overcap_log_latched = false;
                    st.over_cap_since = std::chrono::steady_clock::time_point{};
                    st.under_cap_since = std::chrono::steady_clock::time_point{};
                }
            } else {
                st.under_cap_since = std::chrono::steady_clock::time_point{};
            }
        } else {
            st.over_cap_since = std::chrono::steady_clock::time_point{};
            st.under_cap_since = std::chrono::steady_clock::time_point{};
        }
    }

    void apply_policy_locked(IfaceState& st, const std::string& iface) {
        st.max_total_kbps = default_policy_.max_total_kbps_per_interface > 0.0
                                ? default_policy_.max_total_kbps_per_interface
                                : 20.0;
        st.window_ms = std::max(1000, default_policy_.window_ms);
        st.bits_per_frame_estimate = std::max(80, default_policy_.bits_per_frame_estimate);
        st.over_cap_debounce_ms = std::max(0, default_policy_.over_cap_debounce_ms);
        st.over_cap_clear_ratio = std::clamp(default_policy_.over_cap_clear_ratio, 0.50, 0.99);
        st.over_cap_clear_hold_ms = std::max(0, default_policy_.over_cap_clear_hold_ms);
        st.enforce = default_policy_.enforce;
        (void)iface;
        st.plc_reserve_kbps = 0.0;
        st.module_budget_kbps = st.max_total_kbps;
    }

    void maybe_log_locked(const std::string& iface, IfaceState& st,
                          const std::chrono::steady_clock::time_point& now) {
        if (st.last_log.time_since_epoch().count() != 0 && (now - st.last_log) < std::chrono::seconds(10)) {
            return;
        }
        const double total_kbps = window_kbps(st, st.observed_bits + st.reserved_bits);
        const double window_s = std::max(0.001, static_cast<double>(st.window_ms) / 1000.0);
        const double tx_fps = static_cast<double>(st.tx_events.size()) / window_s;
        const double rx_fps = static_cast<double>(st.rx_events.size()) / window_s;
        const bool budget_limited = st.last_budget_drop.time_since_epoch().count() != 0 &&
                                    (now - st.last_budget_drop) <= std::chrono::milliseconds(st.window_ms);
        const int64_t backlog_age_ms = st.control_backlog_since.time_since_epoch().count() == 0
                                           ? 0
                                           : std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 now - st.control_backlog_since)
                                                 .count();
        std::ostringstream oss;
        oss << "Module CAN stats"
            << " iface=" << iface
            << " total_kbps=" << total_kbps
            << " module_budget_kbps=" << st.module_budget_kbps
            << " module_tx_fps=" << tx_fps
            << " module_rx_fps=" << rx_fps
            << " budget_drops=" << st.budget_drops
            << " control_backlog_age_ms=" << backlog_age_ms
            << " overcap_state=" << (st.overload_latched ? "latched" : (budget_limited ? "limited" : "ok"));
        EVLOG_info << oss.str();
        st.last_log = now;
    }

    void start_monitor_if_needed_locked() {
#ifdef __linux__
        if (running_.load()) {
            return;
        }
        bool have_socket = false;
        for (const auto& kv : states_) {
            if (kv.second.monitor_sock >= 0) {
                have_socket = true;
                break;
            }
        }
        if (!have_socket) {
            return;
        }
        running_.store(true);
        monitor_thread_ = std::thread([this]() { monitor_loop(); });
#endif
    }

    void ensure_monitor_socket_locked(const std::string& iface, IfaceState& st) {
#ifdef __linux__
        if (st.monitor_sock >= 0) {
            return;
        }
        const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0) {
            EVLOG_warning << "CAN monitor socket open failed on " << iface;
            return;
        }
        struct ifreq ifr {};
        std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            EVLOG_warning << "CAN monitor ioctl failed on " << iface;
            ::close(fd);
            return;
        }
        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            EVLOG_warning << "CAN monitor bind failed on " << iface;
            ::close(fd);
            return;
        }
        const int rcvbuf = 512 * 1024;
        ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        const int flags = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        st.monitor_sock = fd;
#else
        (void)iface;
        (void)st;
#endif
    }

    void note_observed_frame(const std::string& iface, const std::chrono::steady_clock::time_point& now) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto& st = states_[iface];
        apply_policy_locked(st, iface);
        prune_locked(st, now);
        const uint64_t bits = static_cast<uint64_t>(std::max(80, st.bits_per_frame_estimate));
        st.observed_bits += bits;
        st.observed_events.push_back({now, bits});
        consume_reserved_locked(st, bits);
        st.module_rx_frames++;
        st.rx_events.push_back(now);
        refresh_overcap_locked(st, now);
        maybe_log_locked(iface, st, now);
        if (st.overload_latched && st.enforce && !st.overcap_log_latched) {
            EVLOG_error << "Module CAN overload latched on iface=" << iface
                        << " total_kbps=" << window_kbps(st, st.observed_bits)
                        << " cap_kbps=" << st.max_total_kbps;
            st.overcap_log_latched = true;
        }
    }

    void monitor_loop() {
#ifdef __linux__
        while (running_.load()) {
            std::vector<std::pair<std::string, int>> sockets;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                for (const auto& kv : states_) {
                    if (kv.second.monitor_sock >= 0) {
                        sockets.emplace_back(kv.first, kv.second.monitor_sock);
                    }
                }
            }

            bool saw_frame = false;
            for (const auto& sock : sockets) {
                can_frame frame{};
                while (::recv(sock.second, &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame)) {
                    saw_frame = true;
                    note_observed_frame(sock.first, std::chrono::steady_clock::now());
                }
            }
            if (!saw_frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
#endif
    }

    mutable std::mutex mtx_;
    ModuleCanTrafficPolicy default_policy_{};
    std::unordered_map<std::string, IfaceState> states_;
    std::atomic<bool> running_{false};
    std::thread monitor_thread_;
};

class CanChannel {
public:
    explicit CanChannel(std::string iface, CanFilterSpec filter = {}, int poll_budget_fps = 0) :
        iface_(std::move(iface)), filter_(filter), poll_budget_fps_(std::max(0, poll_budget_fps)) {
#ifdef __linux__
        sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            EVLOG_error << "Failed to open CAN socket on " << iface_;
            return;
        }
        const int recv_own = 0;
        ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));
        struct ifreq ifr {};
        std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
        if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            EVLOG_error << "CAN ioctl failed for " << iface_;
            ::close(sock_);
            sock_ = -1;
            return;
        }
        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (filter_.enabled) {
            struct can_filter filter {};
            filter.can_id = filter_.id;
            filter.can_mask = filter_.mask;
            ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
        }
        if (::bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            EVLOG_error << "Failed to bind CAN socket on " << iface_;
            ::close(sock_);
            sock_ = -1;
            return;
        }
        // Increase buffers to reduce drops/ENOBUFS when multiple sockets share a busy bus.
        {
            const int rcvbuf = 256 * 1024;
            ::setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
            const int sndbuf = 256 * 1024;
            ::setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        }
        const int flags = ::fcntl(sock_, F_GETFL, 0);
        ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
#endif
        configure_poll_budget();
        CanTrafficGovernor::instance().ensure_iface(iface_);
    }

    ~CanChannel() {
#ifdef __linux__
        if (sock_ >= 0) {
            ::close(sock_);
        }
#endif
    }

    bool valid() const { return sock_ >= 0; }

    bool send(const can_frame& frame, TxClass tx_class, int expected_rx_frames = 0) {
#ifdef __linux__
        if (sock_ < 0) return false;
        bool had_reservation = false;
        if (tx_class == TxClass::Telemetry && !consume_poll_budget_token()) {
            CanTrafficGovernor::instance().note_send_result(iface_, tx_class, false, expected_rx_frames, false);
            return false;
        }
        if (tx_class != TxClass::SafetyUrgent) {
            if (!CanTrafficGovernor::instance().allow_send(iface_, tx_class, expected_rx_frames)) {
                CanTrafficGovernor::instance().note_send_result(iface_, tx_class, false, expected_rx_frames, false);
                return false;
            }
            had_reservation = true;
        }
        const int attempts = tx_class == TxClass::Telemetry ? 1 : 3;
        for (int attempt = 0; attempt < attempts; ++attempt) {
            const auto n = ::write(sock_, &frame, sizeof(frame));
            if (n == sizeof(frame)) {
                CanTrafficGovernor::instance().note_send_result(
                    iface_, tx_class, true, expected_rx_frames, had_reservation);
                return true;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)) {
                if (tx_class == TxClass::Telemetry) {
                    CanTrafficGovernor::instance().note_send_result(
                        iface_, tx_class, false, expected_rx_frames, had_reservation);
                    return false; // drop low-priority reads instead of queueing
                }
                const int sleep_ms = (errno == ENOBUFS) ? 5 : 2;
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                continue;
            }
            CanTrafficGovernor::instance().note_send_result(iface_, tx_class, false, expected_rx_frames, had_reservation);
            return false;
        }
        CanTrafficGovernor::instance().note_send_result(iface_, tx_class, false, expected_rx_frames, had_reservation);
        return false;
#else
        (void)frame;
        (void)tx_class;
        (void)expected_rx_frames;
        return false;
#endif
    }

    bool send(const can_frame& frame, bool critical) {
        return send(frame, critical ? TxClass::Control : TxClass::Telemetry, 0);
    }

    bool recv(can_frame& frame) {
#ifdef __linux__
        if (sock_ < 0) return false;
        const auto n = ::recv(sock_, &frame, sizeof(frame), MSG_DONTWAIT);
        return n == sizeof(frame);
#else
        (void)frame;
        return false;
#endif
    }

private:
    struct PollBudgetState {
        int fps{0};
        double tokens{0.0};
        std::chrono::steady_clock::time_point last_refill{};
    };

    void configure_poll_budget() {
        if (poll_budget_fps_ <= 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(budget_mutex_);
        auto& st = poll_budgets_[iface_];
        if (st.fps <= 0) {
            st.fps = poll_budget_fps_;
            st.tokens = static_cast<double>(poll_budget_fps_);
            st.last_refill = std::chrono::steady_clock::now();
            return;
        }
        if (poll_budget_fps_ < st.fps) {
            st.fps = poll_budget_fps_;
            st.tokens = std::min(st.tokens, static_cast<double>(st.fps));
        }
    }

    bool consume_poll_budget_token() {
        std::lock_guard<std::mutex> lock(budget_mutex_);
        auto it = poll_budgets_.find(iface_);
        if (it == poll_budgets_.end() || it->second.fps <= 0) {
            return true;
        }
        auto& st = it->second;
        const auto now = std::chrono::steady_clock::now();
        if (st.last_refill.time_since_epoch().count() == 0) {
            st.last_refill = now;
            st.tokens = static_cast<double>(st.fps);
        } else {
            const double elapsed_s =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - st.last_refill).count();
            if (elapsed_s > 0.0) {
                st.tokens = std::min(static_cast<double>(st.fps), st.tokens + elapsed_s * static_cast<double>(st.fps));
                st.last_refill = now;
            }
        }
        if (st.tokens < 1.0) {
            return false;
        }
        st.tokens -= 1.0;
        return true;
    }

    std::string iface_;
    CanFilterSpec filter_;
    int poll_budget_fps_{0};
    int sock_{-1};
    static std::mutex budget_mutex_;
    static std::map<std::string, PollBudgetState> poll_budgets_;
};

std::mutex CanChannel::budget_mutex_{};
std::map<std::string, CanChannel::PollBudgetState> CanChannel::poll_budgets_{};

class ModuleDriver {
public:
    explicit ModuleDriver(ModuleSpec spec) : spec_(std::move(spec)) {}
    virtual ~ModuleDriver() = default;

    virtual void apply(const ModuleSetpoint& sp) = 0;
    virtual void poll() = 0;

    const ModuleSpec& spec() const { return spec_; }
    const ModuleTelemetryState& telemetry() const { return telemetry_; }

protected:
    ModuleSpec spec_;
    ModuleTelemetryState telemetry_{};
};

// Simulation driver: provides "fresh" telemetry without CAN I/O.
//
// This is used by unit tests and dev builds where SocketCAN/real modules are not present.
// It intentionally models a healthy module with telemetry derived from the last setpoint.
class SimulatedModuleDriver : public ModuleDriver {
public:
    explicit SimulatedModuleDriver(const ModuleSpec& spec) : ModuleDriver(spec) {
        const auto now = std::chrono::steady_clock::now();
        const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                : 0x00;
        telemetry_.healthy = true;
        telemetry_.fault = false;
        telemetry_.temperature_c = 25.0;
        telemetry_.voltage_v = 0.0;
        telemetry_.current_a = 0.0;
        telemetry_.current_limit_point = 0.0;
        telemetry_.current_capability_valid = spec_.rated_current_a > 0.0;
        telemetry_.current_capability_a = telemetry_.current_capability_valid ? spec_.rated_current_a : 0.0;
        telemetry_.capability_flags_valid = true;
        telemetry_.module_off = false;
        telemetry_.power_limited = false;
        telemetry_.temp_derated = false;
        telemetry_.ac_limited = false;
        telemetry_.alarms = 0;
        telemetry_.healthy_mask = bit;
        telemetry_.fault_mask = 0;
        telemetry_.last_voltage_update = now;
        telemetry_.last_current_update = now;
        telemetry_.last_update = now;
        telemetry_.reported_group = static_cast<uint16_t>(std::max(0, spec_.group));
        telemetry_.reported_address = static_cast<uint16_t>(std::max(0, spec_.address));
        telemetry_.input_mode = static_cast<uint32_t>(std::max(0, spec_.input_mode));
    }

    void apply(const ModuleSetpoint& sp) override {
        desired_ = sp;
        const auto now = std::chrono::steady_clock::now();
        const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                : 0x00;
        telemetry_.healthy = true;
        telemetry_.fault = false;
        telemetry_.healthy_mask = bit;
        telemetry_.fault_mask = 0;
        telemetry_.voltage_v = sp.enable ? std::max(0.0, sp.voltage_v) : 0.0;
        telemetry_.current_a = sp.enable ? std::max(0.0, sp.current_a) : 0.0;
        if (spec_.rated_current_a > 0.0) {
            telemetry_.current_limit_point = std::clamp(telemetry_.current_a / spec_.rated_current_a, 0.0, 1.0);
            telemetry_.current_capability_valid = true;
            telemetry_.current_capability_a = spec_.rated_current_a;
        } else {
            telemetry_.current_limit_point = sp.enable ? 1.0 : 0.0;
            telemetry_.current_capability_valid = false;
            telemetry_.current_capability_a = 0.0;
        }
        telemetry_.capability_flags_valid = true;
        // Sim modules represent idealized available hardware in unit/sim tests; keep them allocatable.
        telemetry_.module_off = false;
        telemetry_.power_limited = false;
        telemetry_.temp_derated = false;
        telemetry_.ac_limited = false;
        telemetry_.last_voltage_update = now;
        telemetry_.last_current_update = now;
        telemetry_.last_update = now;
    }

    void poll() override {
        // Keep telemetry fresh even if setpoints are not changing.
        const auto now = std::chrono::steady_clock::now();
        telemetry_.last_voltage_update = now;
        telemetry_.last_current_update = now;
        telemetry_.last_update = now;
    }

private:
    ModuleSetpoint desired_{};
};

class MaxwellModuleDriver : public ModuleDriver {
public:
    MaxwellModuleDriver(const ModuleSpec& spec, std::shared_ptr<CanChannel> channel) :
        ModuleDriver(spec), channel_(std::move(channel)) {
        last_sent_.voltage_v = 0.0;
        // Phase-offset poll timestamps so modules don't all transmit in the same tick window.
        const auto now = std::chrono::steady_clock::now();
        const int base_ms = std::max(100, spec_.poll_interval_ms);
        const int key = (spec_.address >= 0) ? spec_.address : spec_.slot_index;
        const int phase_ms = (key * 73) % base_ms;
        const auto offset = std::chrono::milliseconds(std::max(0, base_ms - phase_ms));
        last_poll_status_ = now - offset;
        last_poll_voltage_ = now - offset;
        last_poll_current_ = now - offset;
        last_poll_temp_ = now;
        last_poll_limit_point_ = now;
        last_poll_input_mode_ = now;
    }

    void apply(const ModuleSetpoint& sp) override {
        desired_ = sp;
        const auto now = std::chrono::steady_clock::now();
        if (sp.enable && !last_desired_enable_) {
            enable_requested_at_ = now;
        } else if (!sp.enable && last_desired_enable_) {
            enable_requested_at_ = std::chrono::steady_clock::time_point{};
        }
        last_desired_enable_ = sp.enable;
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto control_retry_interval = std::chrono::milliseconds(std::max(100, spec_.cmd_interval_ms));
        const bool control_retry_due =
            last_control_attempt_.time_since_epoch().count() == 0 ||
            (now - last_control_attempt_) >= control_retry_interval;
        const bool enable_edge_on = sp.enable && !last_sent_.enable;
        const bool enable_edge_off = !sp.enable && last_sent_.enable;
        const double requested_voltage_v =
            (sp.enable && sp.voltage_v > 0.0 && sp.voltage_v < MAXWELL_OUTPUT_VOLTAGE_MIN_SET_V)
                ? MAXWELL_OUTPUT_VOLTAGE_MIN_SET_V
                : (sp.voltage_v > 0.0 ? sp.voltage_v : 0.0);
        const bool voltage_changed = std::fabs(requested_voltage_v - last_sent_.voltage_v) > 0.5;
        // Keep command granularity aligned with the 0.1A EV/PLC contract.
        const bool current_changed = std::fabs(sp.current_a - last_sent_.current_a) > 0.1;
        const bool invalid_setpoint = (!std::isfinite(sp.voltage_v) || sp.voltage_v < 0.0) ||
                                      (!std::isfinite(sp.current_a) || sp.current_a < 0.0);
        if (invalid_setpoint) {
            const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                    ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                    : 0x00;
            telemetry_.fault = true;
            telemetry_.healthy = false;
            telemetry_.fault_mask = bit;
            telemetry_.healthy_mask = 0;
            telemetry_.voltage_v = 0.0;
            telemetry_.current_a = 0.0;
            telemetry_.last_voltage_update = now;
            telemetry_.last_current_update = now;
            telemetry_.last_update = now;
            const bool sent = send_set_int(0x0030, 0x00010000, TxClass::SafetyUrgent); // shutdown (fail-safe)
            if (sent) {
                last_sent_ = ModuleSetpoint{};
                last_tx_ = now;
            }
            desired_ = ModuleSetpoint{};
            return;
        }

        const bool have_recent_status =
            last_status_update_.time_since_epoch().count() != 0 &&
            (now - last_status_update_) <= telemetry_stale_interval(spec_);
        const bool module_off_known = have_recent_status;
        const bool module_off =
            have_recent_status && ((telemetry_.alarms & (1u << MAXWELL_ALARM_ONOFF_BIT)) != 0);
        const bool severe_alarm_active = (telemetry_.alarms & MAXWELL_ALARM_SEVERE_MASK) != 0;
        const bool steady_tracking =
            sp.enable && have_recent_status && !module_off && !severe_alarm_active &&
            module_current_tracking(telemetry_.current_a, sp.current_a);
        const auto cmd_refresh_interval = control_refresh_interval(spec_, steady_tracking);
        const bool periodic_refresh = (now - last_tx_) >= cmd_refresh_interval;
        const bool off_keepalive_due =
            !sp.enable &&
            (last_off_keepalive_tx_.time_since_epoch().count() == 0 ||
             (now - last_off_keepalive_tx_) >= MODULE_OFF_COMMAND_KEEPALIVE_INTERVAL);
        const bool off_state_unconfirmed = module_off_known && !module_off;
        const bool off_retry_due =
            last_off_retry_tx_.time_since_epoch().count() == 0 ||
            (now - last_off_retry_tx_) >= MODULE_OFF_STATE_RETRY_INTERVAL;
        const bool need_off_command =
            !sp.enable &&
            (off_keepalive_due || ((enable_edge_off || off_state_unconfirmed) && off_retry_due));

        const bool control_update_due = sp.enable &&
                                        (enable_edge_on ||
                                         (control_retry_due &&
                                          (voltage_changed || current_changed || periodic_refresh)));

        const bool should_send = control_update_due || need_off_command;

        if (!should_send) {
            return;
        }

        double voltage_v = requested_voltage_v;
        const double current_a = sp.current_a > 0.0 ? sp.current_a : 0.0;
        bool sent_control = false;
        bool attempted_control = false;

        const bool need_input_mode = spec_.input_mode >= 0;
        if (need_input_mode) {
            const auto input_mode_matches = [&](uint32_t reported) {
                if (spec_.input_mode == 3) {
                    return reported == 3;
                }
                if (spec_.input_mode == 2) {
                    return reported == 2;
                }
                return reported == 1;
            };
            const uint32_t commanded_input_mode =
                (spec_.input_mode == 2) ? 2U : ((spec_.input_mode == 3) ? 3U : 1U);
            const auto mode_interval =
                std::chrono::milliseconds(std::max<int64_t>(5000, std::max(200, spec_.cmd_interval_ms) * 5));
            const bool mode_stale = (now - last_input_mode_tx_) >= mode_interval;
            const bool mismatch = input_mode_reported_ && !input_mode_matches(telemetry_.input_mode);
            // Avoid pushing mode writes before the first readback is available. This prevents noisy
            // startup retries; once readback is known, enforce configured mode with bounded retries.
            if (mode_stale && mismatch) {
                attempted_control = true;
                if (send_set_int(0x0046, commanded_input_mode)) {
                    last_input_mode_tx_ = now;
                    sent_control = true;
                }
            }
        }

        if (sp.enable) {
            if (sp.voltage_v > 0.0 && sp.voltage_v < MAXWELL_OUTPUT_VOLTAGE_MIN_SET_V) {
                static std::map<std::string, std::chrono::steady_clock::time_point> last_low_voltage_log;
                auto& last_log = last_low_voltage_log[spec_.id];
                if (last_log.time_since_epoch().count() == 0 || (now - last_log) >= std::chrono::seconds(2)) {
                    EVLOG_warning << "MXR module " << spec_.id
                                  << " clamping low 0x0021 voltage setpoint from " << sp.voltage_v
                                  << "V to protocol-safe floor " << MAXWELL_OUTPUT_VOLTAGE_MIN_SET_V << "V";
                    last_log = now;
                }
                voltage_v = MAXWELL_OUTPUT_VOLTAGE_MIN_SET_V;
            }
            last_off_retry_tx_ = std::chrono::steady_clock::time_point{};
            // Start once when transitioning to enable, and retry during periodic updates if the module still reports OFF.
            // Keep retrying startup while status is still OFF or status readback is not yet available.
            // This closes the gap where one startup command is lost on CAN and no retry is sent.
            const bool startup_retry = periodic_refresh && (!have_recent_status || module_off);
            if (enable_edge_on || startup_retry) {
                attempted_control = true;
                if (send_set_int(0x0030, 0x00000000)) { // startup
                    last_sent_.enable = true;
                    sent_control = true;
                }
            }
            const double rated_current = spec_.rated_current_a > 0.0
                                             ? spec_.rated_current_a
                                             : (spec_.rated_power_kw > 0.0 && voltage_v > 1.0
                                                    ? (spec_.rated_power_kw * 1000.0) / voltage_v
                                                    : 0.0);
            float frac = 0.0f;
            if (current_a <= 0.0) {
                frac = 0.0f;
            } else if (rated_current > 0.0) {
                frac = static_cast<float>(std::clamp(current_a / rated_current,
                                                     0.0,
                                                     static_cast<double>(MAXWELL_CURRENT_LIMIT_RATIO_MAX)));
            } else {
                // Fail safe: do not drive current if rated current is unknown at low bus voltage.
                frac = 0.0f;
                if (current_a > 0.0 &&
                    (last_missing_rated_current_log_.time_since_epoch().count() == 0 ||
                     (now - last_missing_rated_current_log_) >= std::chrono::seconds(2))) {
                    EVLOG_error << "MXR module " << spec_.id
                                << " missing rated current at low voltage; forcing current ratio=0";
                    last_missing_rated_current_log_ = now;
                }
            }
            // For MXR, avoid driving both 0x0022 (limit ratio) and 0x001B (absolute current)
            // in the same cycle; mixed current-control paths can cause unstable low-current behavior.
            const bool use_limit_ratio = !spec_.send_output_current;
            if (use_limit_ratio && (enable_edge_on || (control_retry_due && (current_changed || periodic_refresh)))) {
                attempted_control = true;
                if (send_set_float(0x0022, frac)) {
                    last_limit_fraction_ = frac;
                    last_limit_set_tx_ = now;
                    last_sent_.current_a = sp.current_a;
                    sent_control = true;
                }
            }
            if (enable_edge_on || (control_retry_due && (voltage_changed || periodic_refresh))) {
                attempted_control = true;
                if (send_set_float(0x0021, static_cast<float>(voltage_v))) {
                    last_sent_.voltage_v = voltage_v;
                    sent_control = true;
                }
            }
            if (spec_.send_output_current &&
                (enable_edge_on || (control_retry_due && (current_changed || periodic_refresh)))) {
                const double raw_d = std::llround(std::max(0.0, current_a) * MAXWELL_ABSOLUTE_CURRENT_SCALE);
                const uint32_t val = static_cast<uint32_t>(
                    std::clamp(raw_d, 0.0, static_cast<double>(std::numeric_limits<uint32_t>::max())));
                attempted_control = true;
                if (send_set_int(0x001B, val)) {
                    last_set_output_current_a_ = sp.current_a;
                    last_limit_set_tx_ = now;
                    last_sent_.current_a = sp.current_a;
                    sent_control = true;
                }
            }
            last_off_keepalive_tx_ = std::chrono::steady_clock::time_point{};
        } else {
            if (need_off_command) {
                last_off_keepalive_tx_ = now;
                last_off_retry_tx_ = now;
                attempted_control = true;
                if (send_set_int(0x0030, 0x00010000)) { // shutdown
                    last_sent_ = ModuleSetpoint{};
                    limit_mismatch_count_ = 0;
                    limit_mismatch_since_ = std::chrono::steady_clock::time_point{};
                    last_limit_fraction_ = 0.0;
                    last_set_output_current_a_ = 0.0;
                    sent_control = true;
                }
            }
        }
        if (attempted_control) {
            last_control_attempt_ = now;
        }
        // Advance refresh timing only when we actually attempted/sent module control traffic.
        // This prevents non-actionable planner deltas from starving periodic current/voltage keepalive commands.
        if (sent_control || attempted_control) {
            last_tx_ = now;
        }
    }

    void poll() override {
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto poll_interval = std::chrono::milliseconds(std::max(100, spec_.poll_interval_ms));
        const auto slow_interval = std::chrono::milliseconds(std::max<int64_t>(1000, poll_interval.count() * 4));
        const auto idle_status_interval =
            std::chrono::milliseconds(std::max<int64_t>(MODULE_IDLE_STATUS_POLL_INTERVAL.count(),
                                                        poll_interval.count() * 6));
        const auto idle_telemetry_interval =
            std::chrono::milliseconds(std::max<int64_t>(MODULE_IDLE_TELEMETRY_POLL_INTERVAL.count(),
                                                        poll_interval.count() * 12));
        const bool direct = !spec_.broadcast || resolved_addr_.has_value();
        const bool have_recent_status =
            last_status_update_.time_since_epoch().count() != 0 &&
            (now - last_status_update_) <= telemetry_stale_interval(spec_);
        const bool module_off =
            have_recent_status && ((telemetry_.alarms & (1u << MAXWELL_ALARM_ONOFF_BIT)) != 0);
        // Keep active telemetry polling after disable requests until OFF is confirmed by status.
        const bool run_context = desired_.enable || !module_off;

        auto due = [&](const std::chrono::steady_clock::time_point& last,
                       const std::chrono::milliseconds interval) -> bool {
            return last.time_since_epoch().count() == 0 || (now - last) >= interval;
        };

        // Fair poll schedule: send at most one successful read per tick while rotating priority
        // so voltage/current cannot starve when status is due at the same cadence.
        bool sent_priority_current = false;
        if (run_context) {
            // Guarantee periodic current-register polls even under heavy status/voltage polling.
            // This prevents stale 0A telemetry from persisting during active charging.
            const auto current_overdue_interval = current_priority_poll_interval(poll_interval);
            if (due(last_poll_current_, current_overdue_interval) && send_read(0x0002)) {
                last_poll_current_ = now;
                poll_rr_cursor_ = 3; // continue RR after the "current" task
                sent_priority_current = true;
            }
        }
        if (direct && !sent_priority_current) {
            constexpr int kPollTaskCount = 7;
            const auto probe_interval = std::chrono::milliseconds(1000);
            const auto input_interval = std::chrono::milliseconds(1000);
            for (int step = 0; step < kPollTaskCount; ++step) {
                const int task = (static_cast<int>(poll_rr_cursor_) + step) % kPollTaskCount;
                bool sent = false;
                switch (task) {
                case 0: // alarm/status
                    if (due(last_poll_status_, run_context ? poll_interval : idle_status_interval)) {
                        sent = send_read(0x0040);
                        if (sent) last_poll_status_ = now;
                    }
                    break;
                case 1: // voltage
                    if (run_context && due(last_poll_voltage_, poll_interval)) {
                        sent = send_read(0x0001);
                        if (sent) last_poll_voltage_ = now;
                    }
                    break;
                case 2: // current
                    if (run_context && due(last_poll_current_, poll_interval)) {
                        sent = send_read(0x0002);
                        if (sent) last_poll_current_ = now;
                    }
                    break;
                case 3: // temperature
                    if (due(last_poll_temp_, run_context ? slow_interval : idle_telemetry_interval)) {
                        sent = send_read(0x0004);
                        if (sent) last_poll_temp_ = now;
                    }
                    break;
                case 4: // current limit point
                    if (run_context && spec_.readback_limits && due(last_poll_limit_point_, slow_interval)) {
                        sent = send_read(0x0003);
                        if (sent) last_poll_limit_point_ = now;
                    }
                    break;
                case 5: // group/address probe
                    if (spec_.probe_on_startup && !addr_reported_ && due(last_probe_tx_, probe_interval)) {
                        sent = send_read(0x0043);
                        if (sent) last_probe_tx_ = now;
                    }
                    break;
                case 6: // input mode
                    if (spec_.input_mode >= 0 && !input_mode_reported_ && due(last_poll_input_mode_, input_interval)) {
                        sent = send_read(0x004B);
                        if (sent) last_poll_input_mode_ = now;
                    }
                    break;
                default:
                    break;
                }
                if (sent) {
                    poll_rr_cursor_ = static_cast<uint8_t>((task + 1) % kPollTaskCount);
                    break;
                }
            }
        }
        can_frame frame{};
        while (channel_->recv(frame)) {
            handle_frame(frame);
        }
    }

private:
    uint32_t build_can_id() const {
        uint32_t id = 0;
        id |= (static_cast<uint32_t>(MAXWELL_PROT_NO & 0x1FF) << 20);
        const bool direct = !spec_.broadcast || resolved_addr_.has_value();
        id |= (static_cast<uint32_t>(direct ? 1 : 0) << 19); // PTP (0=broadcast, 1=point-to-point)
        uint32_t dst = static_cast<uint32_t>(spec_.address & 0xFF);
        if (direct && resolved_addr_) {
            dst = static_cast<uint32_t>(*resolved_addr_);
        }
        if (!direct) {
            if (spec_.group <= 7) {
                dst = 0xFE;
            } else {
                const int ext_group = std::min(60, std::max(0, spec_.group));
                dst = static_cast<uint32_t>(0xFD - ext_group);
            }
        }
        id |= (dst << 11);
        id |= (static_cast<uint32_t>(MAXWELL_CONTROLLER_ADDR) << 3);
        // Extended group broadcasts use GROUP=0; intra-group broadcast uses GROUP=0..7.
        uint32_t group = 0u;
        if (direct) {
            if (resolved_group_.has_value()) {
                group = static_cast<uint32_t>(*resolved_group_ & 0x07u);
            } else {
                group = static_cast<uint32_t>(spec_.group & 0x07);
            }
        } else {
            group = (spec_.group > 7) ? 0u : static_cast<uint32_t>(spec_.group & 0x07);
        }
        id |= group;
        return id | CAN_EFF_FLAG;
    }

    bool send_set_float(uint16_t reg, float value, TxClass tx_class = TxClass::Control) {
        can_frame frame{};
        frame.can_id = build_can_id();
        frame.can_dlc = 8;
        frame.data[0] = MAXWELL_FUNC_SET;
        frame.data[1] = 0x00;
        frame.data[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
        frame.data[3] = static_cast<uint8_t>(reg & 0xFF);
        const uint32_t raw = encode_float_be(value);
        frame.data[4] = static_cast<uint8_t>((raw >> 24) & 0xFF);
        frame.data[5] = static_cast<uint8_t>((raw >> 16) & 0xFF);
        frame.data[6] = static_cast<uint8_t>((raw >> 8) & 0xFF);
        frame.data[7] = static_cast<uint8_t>(raw & 0xFF);
        if (reg == 0x0021 || reg == 0x0022 || reg == 0x0046) {
            EVLOG_debug << "MXR tx set-float module=" << spec_.id
                        << " can_id=0x" << hex_u32(frame.can_id & CAN_EFF_MASK, 8)
                        << " reg=0x" << hex_u32(reg, 4)
                        << " raw=0x" << hex_u32(raw, 8)
                        << " val=" << value;
        }
        const int expected_rx = tx_class == TxClass::SafetyUrgent ? 0 : 1;
        return channel_->send(frame, tx_class, expected_rx);
    }

    bool send_set_int(uint16_t reg, uint32_t value, TxClass tx_class = TxClass::Control) {
        can_frame frame{};
        frame.can_id = build_can_id();
        frame.can_dlc = 8;
        frame.data[0] = MAXWELL_FUNC_SET;
        frame.data[1] = 0x00;
        frame.data[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
        frame.data[3] = static_cast<uint8_t>(reg & 0xFF);
        frame.data[4] = static_cast<uint8_t>((value >> 24) & 0xFF);
        frame.data[5] = static_cast<uint8_t>((value >> 16) & 0xFF);
        frame.data[6] = static_cast<uint8_t>((value >> 8) & 0xFF);
        frame.data[7] = static_cast<uint8_t>(value & 0xFF);
        if (reg == 0x001B || reg == 0x0030) {
            EVLOG_debug << "MXR tx set-int module=" << spec_.id
                        << " can_id=0x" << hex_u32(frame.can_id & CAN_EFF_MASK, 8)
                        << " reg=0x" << hex_u32(reg, 4)
                        << " raw=0x" << hex_u32(value, 8)
                        << " val=" << value;
        }
        const int expected_rx = tx_class == TxClass::SafetyUrgent ? 0 : 1;
        return channel_->send(frame, tx_class, expected_rx);
    }

    bool send_read(uint16_t reg) {
        can_frame frame{};
        frame.can_id = build_can_id();
        frame.can_dlc = 8;
        frame.data[0] = MAXWELL_FUNC_READ;
        frame.data[1] = 0x00;
        frame.data[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
        frame.data[3] = static_cast<uint8_t>(reg & 0xFF);
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        return channel_->send(frame, TxClass::Telemetry, 1);
    }

    void handle_frame(const can_frame& frame) {
        const uint32_t id = frame.can_id & CAN_EFF_MASK;
        const uint16_t prot = static_cast<uint16_t>((id >> 20) & 0x1FF);
        const uint8_t dst_addr = static_cast<uint8_t>((id >> 11) & 0xFF);
        const uint8_t src_addr = static_cast<uint8_t>((id >> 3) & 0xFF);
        const uint8_t group = static_cast<uint8_t>(id & 0x07u);
        uint8_t expected_group = 0;
        if (resolved_group_.has_value()) {
            expected_group = static_cast<uint8_t>(*resolved_group_ & 0x07u);
        } else if (spec_.group <= 7) {
            expected_group = static_cast<uint8_t>(spec_.group & 0x07u);
        } else {
            expected_group = 0;
        }
        bool group_match = (spec_.group <= 7) ? (group == expected_group) : (group == 0);
        const uint8_t match_addr = resolved_addr_.has_value() ? *resolved_addr_ : static_cast<uint8_t>(spec_.address & 0xFF);
        const bool src_match = src_addr == match_addr;
        const bool broadcast_accept = spec_.broadcast &&
                                      (spec_.address == 0xFE || spec_.address == 0xFF);
        if (!group_match && !resolved_group_.has_value() && spec_.group <= 7 && src_match) {
            resolved_group_ = group;
            expected_group = static_cast<uint8_t>(*resolved_group_ & 0x07u);
            group_match = true;
            EVLOG_warning << "MXR module " << spec_.id
                          << " group mismatch (cfg=" << static_cast<int>(spec_.group & 0x07u)
                          << " rx=" << static_cast<int>(group)
                          << "); auto-adopting group " << static_cast<int>(expected_group);
        }
        if (prot != MAXWELL_PROT_NO || dst_addr != MAXWELL_CONTROLLER_ADDR || !group_match ||
            (!src_match && !broadcast_accept)) {
            return;
        }
        if (frame.can_dlc < 8) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const uint8_t type = frame.data[0];
        const uint8_t status = frame.data[1];
        const uint16_t reg = static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]);
        if (status != MAXWELL_OK) {
            if (type == MAXWELL_TYPE_FLOAT && reg == 0x0021) {
                EVLOG_warning << "MXR module " << spec_.id
                              << " rejected voltage setpoint reg=0x0021 status=0x"
                              << std::hex << static_cast<int>(status) << std::dec;
            }
            if (status_error_since_.time_since_epoch().count() == 0) {
                status_error_since_ = now;
            }
            status_error_count_++;
            const bool persistent_status_error =
                status_error_count_ >= MAXWELL_STATUS_ERROR_DEBOUNCE_COUNT &&
                (now - status_error_since_) >= MAXWELL_STATUS_ERROR_DEBOUNCE_TIME;
            if (persistent_status_error && !status_error_latched_) {
                EVLOG_warning << "MXR module " << spec_.id
                              << " persistent non-F0 responses (status=0x"
                              << std::hex << static_cast<int>(status) << std::dec
                              << ", reg=0x" << std::hex << reg << std::dec
                              << ", count=" << status_error_count_
                              << "); waiting for stable 0x0040 alarm evidence before faulting";
                status_error_latched_ = true;
            }
            return;
        }
        if (status_error_latched_) {
            EVLOG_info << "MXR module " << spec_.id << " response status recovered to F0";
            status_error_latched_ = false;
        }
        status_error_since_ = std::chrono::steady_clock::time_point{};
        status_error_count_ = 0;
        if (type == MAXWELL_TYPE_FLOAT) {
            const float val = decode_float_be(&frame.data[4]);
            if (!std::isfinite(val)) {
                telemetry_.last_update = now;
                return;
            }
            if (reg == 0x0001) {
                if (val >= -0.1f && val <= 2000.0f) {
                    telemetry_.voltage_v = std::max(0.0, static_cast<double>(val));
                    telemetry_.last_voltage_update = now;
                }
            } else if (reg == 0x0002) {
                if (val >= -0.1f && val <= 500.0f) {
                    telemetry_.current_a = std::max(0.0, static_cast<double>(val));
                    telemetry_.last_current_update = now;
                }
            } else if (reg == 0x0004) {
                if (val >= -50.0f && val <= 200.0f) {
                    telemetry_.temperature_c = static_cast<double>(val);
                }
            } else if (reg == 0x0003) {
                if (val >= -0.1f && val <= 1.5f) {
                    telemetry_.current_limit_point = std::clamp(static_cast<double>(val), 0.0, 1.0);
                    if (spec_.rated_current_a > 0.1) {
                        telemetry_.current_capability_valid = true;
                        telemetry_.current_capability_a =
                            std::max(0.0, spec_.rated_current_a * telemetry_.current_limit_point);
                    } else {
                        telemetry_.current_capability_valid = false;
                        telemetry_.current_capability_a = 0.0;
                    }
                    if (!spec_.readback_limits) {
                        limit_mismatch_count_ = 0;
                        limit_mismatch_since_ = std::chrono::steady_clock::time_point{};
                    } else {
                        const bool settling =
                            last_limit_set_tx_.time_since_epoch().count() != 0 &&
                            (now - last_limit_set_tx_) < MAXWELL_LIMIT_READBACK_SETTLE_TIME;
                        const double expected_frac_from_ratio = std::clamp(last_limit_fraction_, 0.0, 1.0);
                        bool have_expected = expected_frac_from_ratio > 0.0;
                        double expected_best = expected_frac_from_ratio;
                        double expected_frac_from_current = -1.0;
                        if (spec_.send_output_current && spec_.rated_current_a > 0.5 &&
                            last_set_output_current_a_ > 0.0) {
                            expected_frac_from_current =
                                std::clamp(last_set_output_current_a_ / spec_.rated_current_a, 0.0, 1.0);
                            if (!have_expected ||
                                std::fabs(telemetry_.current_limit_point - expected_frac_from_current) <
                                    std::fabs(telemetry_.current_limit_point - expected_best)) {
                                expected_best = expected_frac_from_current;
                            }
                            have_expected = true;
                        }

                        if (settling || !have_expected) {
                            limit_mismatch_count_ = 0;
                            limit_mismatch_since_ = std::chrono::steady_clock::time_point{};
                        } else {
                            const double diff = std::fabs(telemetry_.current_limit_point - expected_best);
                            const double denom = std::max(0.1, std::fabs(expected_best));
                            const bool mismatch = diff > MAXWELL_LIMIT_MISMATCH_ABS_DELTA &&
                                                  (diff / denom) > MAXWELL_LIMIT_MISMATCH_REL_DELTA;
                            if (mismatch) {
                                if (limit_mismatch_since_.time_since_epoch().count() == 0) {
                                    limit_mismatch_since_ = now;
                                }
                                limit_mismatch_count_++;
                                if (limit_mismatch_count_ >= MAXWELL_LIMIT_MISMATCH_CONFIRM_COUNT &&
                                    (now - limit_mismatch_since_) >= MAXWELL_LIMIT_MISMATCH_CONFIRM_TIME) {
                                    EVLOG_warning << "MXR module " << spec_.id
                                                  << " persistent current-limit readback mismatch"
                                                  << " (read=" << telemetry_.current_limit_point
                                                  << " expected_best=" << expected_best
                                                  << " expected_0022=" << expected_frac_from_ratio
                                                  << " expected_001B="
                                                  << (expected_frac_from_current >= 0.0
                                                          ? expected_frac_from_current
                                                          : expected_frac_from_ratio)
                                                  << " diff=" << diff
                                                  << " count=" << limit_mismatch_count_ << ")";
                                    limit_mismatch_count_ = 0;
                                    limit_mismatch_since_ = std::chrono::steady_clock::time_point{};
                                }
                            } else {
                                limit_mismatch_count_ = 0;
                                limit_mismatch_since_ = std::chrono::steady_clock::time_point{};
                            }
                        }
                    }
                }
            }
        } else if (type == MAXWELL_TYPE_INT) {
            const uint32_t val = decode_u32_be(&frame.data[4]);
            if (reg == 0x0040) {
                telemetry_.alarms = val;
                last_status_update_ = now;
                telemetry_.capability_flags_valid = true;
                telemetry_.module_off = (val & (1u << MAXWELL_ALARM_ONOFF_BIT)) != 0;
                telemetry_.power_limited = (val & (1u << MAXWELL_ALARM_POWER_LIMIT_BIT)) != 0;
                telemetry_.temp_derated = (val & (1u << MAXWELL_ALARM_TEMP_DERATE_BIT)) != 0;
                telemetry_.ac_limited = (val & (1u << MAXWELL_ALARM_AC_LIMIT_BIT)) != 0;
                const bool module_off = (val & (1u << MAXWELL_ALARM_ONOFF_BIT)) != 0;
                const bool severe = (val & MAXWELL_ALARM_SEVERE_MASK) != 0;
                const bool startup_stalled = !severe && desired_.enable &&
                    startup_load_requested(desired_.current_a) &&
                    enable_requested_at_.time_since_epoch().count() != 0 && module_off &&
                    (now - enable_requested_at_) > MAXWELL_START_TIMEOUT;
                if (startup_stalled &&
                    (last_start_stall_log_.time_since_epoch().count() == 0 ||
                     (now - last_start_stall_log_) >= std::chrono::seconds(2))) {
                    EVLOG_warning << "MXR module " << spec_.id
                                  << " still reports OFF after startup request; keeping module in recovery state";
                    last_start_stall_log_ = now;
                }
                const bool fault = severe;
                telemetry_.fault = fault;
                uint8_t bit = 0x01;
                if (spec_.slot_index >= 0 && spec_.slot_index < 8) {
                    bit = static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index));
                }
                telemetry_.healthy_mask = fault ? 0x00 : bit;
                telemetry_.fault_mask = fault ? bit : 0x00;
            } else if (reg == 0x0043) {
                telemetry_.reported_group = static_cast<uint16_t>((val >> 16) & 0xFFFF);
                telemetry_.reported_address = static_cast<uint16_t>(val & 0xFFFF);
                if (!addr_reported_) {
                    EVLOG_info << "MXR module " << spec_.id << " reported group="
                               << telemetry_.reported_group << " address=" << telemetry_.reported_address;
                    addr_reported_ = true;
                }
                if (spec_.broadcast && !resolved_addr_.has_value() && telemetry_.reported_address <= 63) {
                    resolved_addr_ = static_cast<uint8_t>(telemetry_.reported_address & 0xFFu);
                    resolved_group_ = static_cast<uint8_t>(telemetry_.reported_group & 0x07u);
                    EVLOG_info << "MXR module " << spec_.id << " switching to direct addr="
                               << static_cast<int>(*resolved_addr_) << " group="
                               << static_cast<int>(*resolved_group_);
                }
            } else if (reg == 0x004B) {
                telemetry_.input_mode = val;
                if (!input_mode_reported_) {
                    EVLOG_info << "MXR module " << spec_.id << " input mode=" << telemetry_.input_mode;
                }
                const bool input_mode_match =
                    (spec_.input_mode < 0) ||
                    (spec_.input_mode == 2
                         ? (telemetry_.input_mode == 2)
                         : (spec_.input_mode == 3 ? (telemetry_.input_mode == 3) : (telemetry_.input_mode == 1)));
                if (!input_mode_match) {
                    EVLOG_warning << "MXR module " << spec_.id << " input mode mismatch (reported "
                                  << telemetry_.input_mode << ", expected "
                                  << (spec_.input_mode == 2 ? "2(DC)"
                                                            : (spec_.input_mode == 3 ? "3(3-phase AC)"
                                                                                     : "1(AC)"))
                                  << ")";
                }
                input_mode_reported_ = true;
            }
        }
        telemetry_.last_update = now;
        if (telemetry_.fault_mask != 0) {
            telemetry_.fault = true;
        }
        telemetry_.healthy = !telemetry_.fault;
    }

    ModuleSetpoint last_sent_{};
    ModuleSetpoint desired_{};
    bool last_desired_enable_{false};
    std::chrono::steady_clock::time_point enable_requested_at_{};
    std::chrono::steady_clock::time_point last_status_update_{};
    std::chrono::steady_clock::time_point last_tx_{};
    std::chrono::steady_clock::time_point last_control_attempt_{};
    std::chrono::steady_clock::time_point last_off_keepalive_tx_{};
    std::chrono::steady_clock::time_point last_off_retry_tx_{};
    std::chrono::steady_clock::time_point last_poll_voltage_{};
    std::chrono::steady_clock::time_point last_poll_current_{};
    std::chrono::steady_clock::time_point last_poll_temp_{};
    std::chrono::steady_clock::time_point last_poll_status_{};
    std::chrono::steady_clock::time_point last_poll_limit_point_{};
    std::chrono::steady_clock::time_point last_poll_input_mode_{};
    std::chrono::steady_clock::time_point last_input_mode_tx_{};
    uint8_t poll_rr_cursor_{0};
    std::shared_ptr<CanChannel> channel_;
    bool addr_reported_{false};
    bool input_mode_reported_{false};
    std::optional<uint8_t> resolved_addr_{};
    std::optional<uint8_t> resolved_group_{};
    std::chrono::steady_clock::time_point last_probe_tx_{};
    std::chrono::steady_clock::time_point last_missing_rated_current_log_{};
    std::chrono::steady_clock::time_point last_start_stall_log_{};
    std::chrono::steady_clock::time_point status_error_since_{};
    std::chrono::steady_clock::time_point last_limit_set_tx_{};
    std::chrono::steady_clock::time_point limit_mismatch_since_{};
    double last_limit_fraction_{0.0};
    double last_set_output_current_a_{0.0};
    int limit_mismatch_count_{0};
    int status_error_count_{0};
    bool status_error_latched_{false};
};

class RectifierModuleDriver : public ModuleDriver {
public:
    RectifierModuleDriver(const ModuleSpec& spec, std::shared_ptr<CanChannel> channel) :
        ModuleDriver(spec), channel_(std::move(channel)) {
        last_sent_.voltage_v = 0.0;
        // Phase-offset poll timestamps so modules don't all transmit in the same tick window.
        const auto now = std::chrono::steady_clock::now();
        const int base_ms = std::max(100, spec_.poll_interval_ms);
        const int key = (spec_.address >= 0) ? spec_.address : spec_.slot_index;
        const int phase_ms = (key * 71) % base_ms;
        const auto offset = std::chrono::milliseconds(std::max(0, base_ms - phase_ms));
        last_poll_status_ = now - offset;
        last_poll_voltage_ = now - offset;
        last_poll_current_ = now - offset;
        last_poll_temp_ = now;
        last_poll_capability_ = now;
    }

    void apply(const ModuleSetpoint& sp) override {
        desired_ = sp;
        const auto now = std::chrono::steady_clock::now();
        if (sp.enable && !last_desired_enable_) {
            enable_requested_at_ = now;
        } else if (!sp.enable && last_desired_enable_) {
            enable_requested_at_ = std::chrono::steady_clock::time_point{};
        }
        last_desired_enable_ = sp.enable;
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const bool enable_edge_on = sp.enable && !last_sent_.enable;
        const bool enable_edge_off = !sp.enable && last_sent_.enable;
        const bool voltage_changed = std::fabs(sp.voltage_v - last_sent_.voltage_v) > 0.5;
        // Keep command granularity aligned with the 0.1A EV/PLC contract.
        const bool current_changed = std::fabs(sp.current_a - last_sent_.current_a) > 0.1;
        const bool invalid_setpoint = (!std::isfinite(sp.voltage_v) || sp.voltage_v < 0.0) ||
                                      (!std::isfinite(sp.current_a) || sp.current_a < 0.0);
        if (invalid_setpoint) {
            const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                    ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                    : 0x00;
            telemetry_.fault = true;
            telemetry_.healthy = false;
            telemetry_.fault_mask = bit;
            telemetry_.healthy_mask = 0;
            telemetry_.voltage_v = 0.0;
            telemetry_.current_a = 0.0;
            telemetry_.last_voltage_update = now;
            telemetry_.last_current_update = now;
            telemetry_.last_update = now;
            const bool sent = send_set(4, 1, TxClass::SafetyUrgent); // shutdown
            if (sent) {
                last_sent_ = ModuleSetpoint{};
                last_tx_ = now;
            }
            desired_ = ModuleSetpoint{};
            return;
        }

        const bool have_recent_status =
            last_status_update_.time_since_epoch().count() != 0 &&
            (now - last_status_update_) <= telemetry_stale_interval(spec_);
        const bool module_off_known = have_recent_status;
        const bool module_off = have_recent_status && ((last_status_bits_ & (1u << 25)) != 0);
        const bool steady_tracking =
            sp.enable && have_recent_status && !module_off && !telemetry_.fault &&
            module_current_tracking(telemetry_.current_a, sp.current_a);
        const auto cmd_refresh_interval = control_refresh_interval(spec_, steady_tracking);
        const bool periodic_refresh = (now - last_tx_) >= cmd_refresh_interval;
        const bool off_keepalive_due =
            !sp.enable &&
            (last_off_keepalive_tx_.time_since_epoch().count() == 0 ||
             (now - last_off_keepalive_tx_) >= MODULE_OFF_COMMAND_KEEPALIVE_INTERVAL);
        const bool need_off_command =
            !sp.enable && (enable_edge_off || (module_off_known && !module_off) || off_keepalive_due);

        const bool should_send =
            enable_edge_on || (sp.enable && (voltage_changed || current_changed || periodic_refresh)) ||
            need_off_command;
        if (!should_send) {
            return;
        }

        const double voltage_v = std::max(0.0, sp.voltage_v);
        const double current_a = std::max(0.0, sp.current_a);
        bool sent_control = false;

        if (sp.enable) {
            // Retry startup until status becomes available and ON.
            const bool startup_retry = periodic_refresh && (!have_recent_status || module_off);
            if (enable_edge_on || startup_retry) {
                if (send_set(4, 0)) { // power on
                    last_sent_.enable = true;
                    sent_control = true;
                }
            }
            if (enable_edge_on || voltage_changed || periodic_refresh) {
                const uint32_t mv = static_cast<uint32_t>(std::clamp(voltage_v * 1000.0, 0.0, 4.0e9));
                if (send_set(2, mv)) { // voltage reference
                    last_sent_.voltage_v = sp.voltage_v;
                    sent_control = true;
                }
            }
            if (enable_edge_on || current_changed || periodic_refresh) {
                const uint32_t ma = static_cast<uint32_t>(std::clamp(current_a * 1000.0, 0.0, 4.0e9));
                if (send_set(3, ma)) { // current limit
                    last_sent_.current_a = sp.current_a;
                    sent_control = true;
                }
            }
            last_off_keepalive_tx_ = std::chrono::steady_clock::time_point{};
        } else {
            if (need_off_command) {
                last_off_keepalive_tx_ = now;
                if (send_set(4, 1)) { // power off
                    last_sent_ = ModuleSetpoint{};
                    sent_control = true;
                }
            }
        }
        if (sent_control || should_send) {
            last_tx_ = now;
        }
    }

    void poll() override {
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto poll_interval = std::chrono::milliseconds(std::max(100, spec_.poll_interval_ms));
        const auto slow_interval = std::chrono::milliseconds(std::max<int64_t>(1000, poll_interval.count() * 4));
        const auto idle_status_interval =
            std::chrono::milliseconds(std::max<int64_t>(MODULE_IDLE_STATUS_POLL_INTERVAL.count(),
                                                        poll_interval.count() * 6));
        const auto idle_telemetry_interval =
            std::chrono::milliseconds(std::max<int64_t>(MODULE_IDLE_TELEMETRY_POLL_INTERVAL.count(),
                                                        poll_interval.count() * 12));
        const bool direct = !spec_.broadcast || resolved_addr_.has_value();
        const bool have_recent_status =
            last_status_update_.time_since_epoch().count() != 0 &&
            (now - last_status_update_) <= telemetry_stale_interval(spec_);
        const bool module_off = have_recent_status && ((last_status_bits_ & (1u << 25)) != 0);
        const bool run_context = desired_.enable || !module_off;

        auto due = [&](const std::chrono::steady_clock::time_point& last,
                       const std::chrono::milliseconds interval) -> bool {
            return last.time_since_epoch().count() == 0 || (now - last) >= interval;
        };

        bool sent_priority_current = false;
        if (run_context) {
            const auto current_overdue_interval = current_priority_poll_interval(poll_interval);
            if (due(last_poll_current_, current_overdue_interval) && send_read(1)) {
                last_poll_current_ = now;
                poll_rr_cursor_ = 3; // continue RR after the "current" task
                sent_priority_current = true;
            }
        }
        if (direct && !sent_priority_current) {
            constexpr int kPollTaskCount = 5;
            for (int step = 0; step < kPollTaskCount; ++step) {
                const int task = (static_cast<int>(poll_rr_cursor_) + step) % kPollTaskCount;
                bool sent = false;
                switch (task) {
                case 0: // status
                    if (due(last_poll_status_, run_context ? poll_interval : idle_status_interval)) {
                        sent = send_read(8);
                        if (sent) last_poll_status_ = now;
                    }
                    break;
                case 1: // output voltage
                    if (run_context && due(last_poll_voltage_, poll_interval)) {
                        sent = send_read(0);
                        if (sent) last_poll_voltage_ = now;
                    }
                    break;
                case 2: // output current
                    if (run_context && due(last_poll_current_, poll_interval)) {
                        sent = send_read(1);
                        if (sent) last_poll_current_ = now;
                    }
                    break;
                case 3: // inlet temperature
                    if (due(last_poll_temp_, run_context ? slow_interval : idle_telemetry_interval)) {
                        sent = send_read(30);
                        if (sent) last_poll_temp_ = now;
                    }
                    break;
                case 4: // output current capability
                    if (run_context && spec_.readback_limits && due(last_poll_capability_, slow_interval)) {
                        sent = send_read(104);
                        if (sent) last_poll_capability_ = now;
                    }
                    break;
                default:
                    break;
                }
                if (sent) {
                    poll_rr_cursor_ = static_cast<uint8_t>((task + 1) % kPollTaskCount);
                    break;
                }
            }
        }
        can_frame frame{};
        while (channel_->recv(frame)) {
            handle_frame(frame);
        }
    }

private:
    uint32_t build_can_id(bool broadcast) const {
        uint32_t id = 0;
        id |= (static_cast<uint32_t>(RECTIFIER_PROTO_NO & 0x0F) << 25);
        id |= (static_cast<uint32_t>(spec_.monitor_address & 0x0F) << 21);
        uint8_t mod = broadcast ? 0x00 : static_cast<uint8_t>(spec_.address & 0x7F);
        if (!broadcast && resolved_addr_) {
            mod = *resolved_addr_;
        }
        id |= (static_cast<uint32_t>(mod & 0x7F) << 14);
        id |= (static_cast<uint32_t>(spec_.production_day & 0x1F) << 9);
        id |= static_cast<uint32_t>(spec_.serial_low & 0x1FF);
        return id | CAN_EFF_FLAG;
    }

    bool send_frame(uint8_t msg_type, uint8_t cmd, uint32_t data, TxClass tx_class, int expected_rx) {
        const bool broadcast = spec_.broadcast && !resolved_addr_.has_value();
        can_frame frame{};
        frame.can_id = build_can_id(broadcast);
        frame.can_dlc = 8;
        const uint8_t group = static_cast<uint8_t>(std::clamp(spec_.group, 0, 0x0F));
        frame.data[0] = static_cast<uint8_t>(((group & 0x0F) << 4) | (msg_type & 0x0F));
        frame.data[1] = cmd;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = static_cast<uint8_t>((data >> 24) & 0xFF);
        frame.data[5] = static_cast<uint8_t>((data >> 16) & 0xFF);
        frame.data[6] = static_cast<uint8_t>((data >> 8) & 0xFF);
        frame.data[7] = static_cast<uint8_t>(data & 0xFF);
        return channel_->send(frame, tx_class, expected_rx);
    }

    bool send_set(uint8_t cmd, uint32_t data, TxClass tx_class = TxClass::Control) {
        const int expected_rx = tx_class == TxClass::SafetyUrgent ? 0 : 1;
        return send_frame(RECTIFIER_MSG_SET, cmd, data, tx_class, expected_rx);
    }
    bool send_read(uint8_t cmd) { return send_frame(RECTIFIER_MSG_READ, cmd, 0, TxClass::Telemetry, 1); }

    void handle_frame(const can_frame& frame) {
        if (frame.can_dlc < 8) return;
        const uint32_t id = frame.can_id & CAN_EFF_MASK;
        const uint8_t proto = static_cast<uint8_t>((id >> 25) & 0x0F);
        const uint8_t mon = static_cast<uint8_t>((id >> 21) & 0x0F);
        const uint8_t mod = static_cast<uint8_t>((id >> 14) & 0x7F);
        const uint8_t prod = static_cast<uint8_t>((id >> 9) & 0x1F);
        const uint16_t serial_low = static_cast<uint16_t>(id & 0x1FF);
        if (proto != RECTIFIER_PROTO_NO) return;
        if (spec_.monitor_address > 0 && mon != static_cast<uint8_t>(spec_.monitor_address & 0x0F)) {
            return;
        }
        if (spec_.production_day > 0 && prod != static_cast<uint8_t>(spec_.production_day & 0x1F)) {
            return;
        }
        if (spec_.serial_low > 0 && serial_low != static_cast<uint16_t>(spec_.serial_low & 0x1FF)) {
            return;
        }
        const bool direct = !spec_.broadcast || resolved_addr_.has_value();
        if (direct) {
            const uint8_t expect = resolved_addr_.has_value() ? *resolved_addr_ : static_cast<uint8_t>(spec_.address & 0x7F);
            if (mod != expect) return;
        } else if (spec_.broadcast && !resolved_addr_.has_value()) {
            if (mod == 0) return;
            const bool identity_hint =
                (spec_.address > 0 && spec_.address <= 0x7F) || spec_.production_day > 0 || spec_.serial_low > 0;
            if (!identity_hint) {
                if (!warned_no_identity_) {
                    EVLOG_warning << "Rectifier module " << spec_.id
                                  << " is in broadcast mode without address/serial hints; ignoring responses";
                    warned_no_identity_ = true;
                }
                return;
            }
            if (spec_.address > 0 && spec_.address <= 0x7F && mod != static_cast<uint8_t>(spec_.address & 0x7F)) {
                return;
            }
            resolved_addr_ = mod;
            EVLOG_info << "Rectifier module " << spec_.id << " switching to direct addr="
                       << static_cast<int>(*resolved_addr_);
        }

        const uint8_t group = static_cast<uint8_t>((frame.data[0] >> 4) & 0x0F);
        if (spec_.group > 0 && group != static_cast<uint8_t>(spec_.group & 0x0F)) {
            return;
        }
        const uint8_t msg_type = static_cast<uint8_t>(frame.data[0] & 0x0F);
        const uint8_t cmd = frame.data[1];
        const uint32_t val = decode_u32_be(&frame.data[4]);
        const auto now = std::chrono::steady_clock::now();

        if (msg_type == RECTIFIER_MSG_READ_RESP || msg_type == RECTIFIER_MSG_SET_RESP ||
            msg_type == RECTIFIER_MSG_READ_SN) {
            if (cmd == 0) {
                telemetry_.voltage_v = static_cast<double>(val) / 1000.0;
                telemetry_.last_voltage_update = now;
            } else if (cmd == 1) {
                telemetry_.current_a = static_cast<double>(val) / 1000.0;
                telemetry_.last_current_update = now;
            } else if (cmd == 3) {
                // ENR/UUGreen command type 3 reports output current limit in mA.
                const double limit_current_a = static_cast<double>(val) / 1000.0;
                if (spec_.rated_current_a > 0.1 && std::isfinite(limit_current_a) && limit_current_a >= 0.0) {
                    telemetry_.current_limit_point =
                        std::clamp(limit_current_a / std::max(0.1, spec_.rated_current_a), 0.0, 1.0);
                }
            } else if (cmd == 30) {
                telemetry_.temperature_c = static_cast<double>(val) / 1000.0;
            } else if (cmd == 104) {
                // ENR/UUGreen command type 104 reports available output current capability in mA.
                const double capability_a = static_cast<double>(val) / 1000.0;
                if (std::isfinite(capability_a) && capability_a >= 0.0) {
                    telemetry_.current_capability_valid = true;
                    telemetry_.current_capability_a = capability_a;
                    if (spec_.rated_current_a > 0.1) {
                        telemetry_.current_limit_point =
                            std::clamp(capability_a / std::max(0.1, spec_.rated_current_a), 0.0, 1.0);
                    }
                }
            } else if (cmd == 114) {
                // ENR/UUGreen command type 114 packs output current (high 16b) and capability (low 16b), in 0.1A.
                const double measured_a = static_cast<double>((val >> 16) & 0xFFFFu) * 0.1;
                const double capability_a = static_cast<double>(val & 0xFFFFu) * 0.1;
                if (std::isfinite(measured_a) && measured_a >= 0.0) {
                    telemetry_.current_a = measured_a;
                    telemetry_.last_current_update = now;
                }
                if (std::isfinite(capability_a) && capability_a >= 0.0) {
                    telemetry_.current_capability_valid = true;
                    telemetry_.current_capability_a = capability_a;
                    if (spec_.rated_current_a > 0.1) {
                        telemetry_.current_limit_point =
                            std::clamp(capability_a / std::max(0.1, spec_.rated_current_a), 0.0, 1.0);
                    }
                }
            } else if (cmd == 8) {
                telemetry_.alarms = val;
                last_status_bits_ = val;
                last_status_update_ = now;
                const bool ac_derated = (val & (1u << RECTIFIER_ALARM_AC_DERATE_BIT)) != 0;
                const bool temp_derated = (val & (1u << RECTIFIER_ALARM_TEMP_DERATE_BIT)) != 0;
                const bool pfc_off = (val & (1u << RECTIFIER_ALARM_PFC_OFF_BIT)) != 0;
                const bool module_off = (val & (1u << RECTIFIER_ALARM_MODULE_OFF_BIT)) != 0;
                telemetry_.capability_flags_valid = true;
                telemetry_.module_off = module_off;
                telemetry_.ac_limited = ac_derated || pfc_off;
                telemetry_.temp_derated = temp_derated;
                telemetry_.power_limited = ac_derated || temp_derated || pfc_off;
                bool fault = (val & RECTIFIER_STATUS_FAULT_MASK) != 0;
                if (!fault && desired_.enable &&
                    startup_load_requested(desired_.current_a) &&
                    enable_requested_at_.time_since_epoch().count() != 0 && module_off &&
                    (now - enable_requested_at_) > RECTIFIER_START_TIMEOUT) {
                    fault = true;
                }
                const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                        ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                        : 0x00;
                telemetry_.fault = fault;
                telemetry_.healthy = !fault;
                telemetry_.fault_mask = fault ? bit : 0x00;
                telemetry_.healthy_mask = fault ? 0x00 : bit;
            }
            telemetry_.last_update = now;
        }
    }

    ModuleSetpoint last_sent_{};
    ModuleSetpoint desired_{};
    bool last_desired_enable_{false};
    std::chrono::steady_clock::time_point enable_requested_at_{};
    std::chrono::steady_clock::time_point last_status_update_{};
    std::chrono::steady_clock::time_point last_tx_{};
    std::chrono::steady_clock::time_point last_off_keepalive_tx_{};
    std::chrono::steady_clock::time_point last_poll_voltage_{};
    std::chrono::steady_clock::time_point last_poll_current_{};
    std::chrono::steady_clock::time_point last_poll_status_{};
    std::chrono::steady_clock::time_point last_poll_temp_{};
    std::chrono::steady_clock::time_point last_poll_capability_{};
    uint8_t poll_rr_cursor_{0};
    std::shared_ptr<CanChannel> channel_;
    std::optional<uint8_t> resolved_addr_{};
    uint32_t last_status_bits_{0};
    bool warned_no_identity_{false};
};

class TonheModuleDriver : public ModuleDriver {
public:
    TonheModuleDriver(const ModuleSpec& spec, std::shared_ptr<CanChannel> channel) :
        ModuleDriver(spec), channel_(std::move(channel)) {}

    void apply(const ModuleSetpoint& sp) override {
        desired_ = sp;
        const auto now = std::chrono::steady_clock::now();
        if (sp.enable && !last_desired_enable_) {
            enable_requested_at_ = now;
        } else if (!sp.enable && last_desired_enable_) {
            enable_requested_at_ = std::chrono::steady_clock::time_point{};
        }
        last_desired_enable_ = sp.enable;
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const bool enable_edge_on = sp.enable && !last_sent_.enable;
        const bool enable_edge_off = !sp.enable && last_sent_.enable;
        const bool voltage_changed = std::fabs(sp.voltage_v - last_sent_.voltage_v) > 0.5;
        // Keep command granularity aligned with the 0.1A EV/PLC contract.
        const bool current_changed = std::fabs(sp.current_a - last_sent_.current_a) > 0.1;
        const bool module_off = last_state_ != TONHE_STATE_ON;
        const bool have_recent_status =
            telemetry_.last_update.time_since_epoch().count() != 0 &&
            (now - telemetry_.last_update) <= telemetry_stale_interval(spec_);
        const bool steady_tracking =
            sp.enable && have_recent_status && !module_off && !telemetry_.fault &&
            module_current_tracking(telemetry_.current_a, sp.current_a);
        const auto cmd_refresh_interval = control_refresh_interval(spec_, steady_tracking);
        const bool periodic_refresh = (now - last_tx_) >= cmd_refresh_interval;
        const bool off_keepalive_due =
            !sp.enable &&
            (last_off_keepalive_tx_.time_since_epoch().count() == 0 ||
             (now - last_off_keepalive_tx_) >= MODULE_OFF_COMMAND_KEEPALIVE_INTERVAL);
        const bool need_off_command = !sp.enable && (enable_edge_off || !module_off || off_keepalive_due);

        const bool invalid_setpoint = (!std::isfinite(sp.voltage_v) || sp.voltage_v < 0.0) ||
                                      (!std::isfinite(sp.current_a) || sp.current_a < 0.0);
        if (invalid_setpoint) {
            const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                    ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                    : 0x00;
            telemetry_.fault = true;
            telemetry_.healthy = false;
            telemetry_.fault_mask = bit;
            telemetry_.healthy_mask = 0;
            telemetry_.voltage_v = 0.0;
            telemetry_.current_a = 0.0;
            telemetry_.last_voltage_update = now;
            telemetry_.last_current_update = now;
            telemetry_.last_update = now;
            const bool sent = send_startstop(false, 0.0, 0.0, TxClass::SafetyUrgent);
            if (sent) {
                last_sent_ = ModuleSetpoint{};
                last_tx_ = now;
            }
            desired_ = ModuleSetpoint{};
            return;
        }

        const bool should_send =
            enable_edge_on || (sp.enable && (voltage_changed || current_changed || periodic_refresh)) ||
            need_off_command;
        if (should_send) {
            const bool enable_cmd = sp.enable;
            const double voltage_v = enable_cmd ? std::max(0.0, sp.voltage_v) : 0.0;
            const double current_a = enable_cmd ? std::max(0.0, sp.current_a) : 0.0;
            if (!enable_cmd) {
                last_off_keepalive_tx_ = now;
            }
            if (send_startstop(enable_cmd, voltage_v, current_a, TxClass::Control)) {
                last_sent_ = sp;
                last_tx_ = now;
                if (sp.enable) {
                    last_off_keepalive_tx_ = std::chrono::steady_clock::time_point{};
                } else {
                    last_off_keepalive_tx_ = now;
                }
            } else {
                last_tx_ = now;
            }
        }
    }

    void poll() override {
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        can_frame frame{};
        while (channel_->recv(frame)) {
            handle_frame(frame);
        }
    }

private:
    bool send_startstop(bool enable, double voltage_v, double current_a, TxClass tx_class) {
        can_frame frame{};
        frame.can_id = build_tonhe_id(2, TONHE_PGN_STARTSTOP,
                                      static_cast<uint8_t>(spec_.address & 0xFF),
                                      static_cast<uint8_t>(spec_.source_address & 0xFF));
        frame.can_dlc = 8;
        frame.data[0] = enable ? TONHE_CMD_START : TONHE_CMD_STOP;
        frame.data[1] = 0x00;
        const uint16_t v = static_cast<uint16_t>(std::clamp(voltage_v * 10.0, 0.0, 65535.0));
        const uint16_t i = static_cast<uint16_t>(std::clamp(current_a * 100.0, 0.0, 65535.0));
        frame.data[2] = static_cast<uint8_t>(v & 0xFF);
        frame.data[3] = static_cast<uint8_t>((v >> 8) & 0xFF);
        frame.data[4] = static_cast<uint8_t>(i & 0xFF);
        frame.data[5] = static_cast<uint8_t>((i >> 8) & 0xFF);
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;
        const int expected_rx = tx_class == TxClass::SafetyUrgent ? 0 : 0;
        return channel_->send(frame, tx_class, expected_rx);
    }

    void update_fault_state(const std::chrono::steady_clock::time_point& now, bool update_last) {
        bool fault = (last_fault_bits_ & TONHE_FAULT_SEVERE_MASK) != 0 ||
                     (last_pfc_fault_ != 0) ||
                     (last_ext_fault_bits_ & TONHE_EXT_FAULT_SEVERE_MASK) != 0 ||
                     (last_state_ == TONHE_STATE_FAULT_OFF);
        const bool module_off = last_state_ != TONHE_STATE_ON;
        if (!fault && desired_.enable &&
            startup_load_requested(desired_.current_a) &&
            enable_requested_at_.time_since_epoch().count() != 0 && module_off &&
            (now - enable_requested_at_) > TONHE_START_TIMEOUT) {
            fault = true;
        }
        const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                : 0x00;
        telemetry_.fault = fault;
        telemetry_.healthy = !fault;
        telemetry_.fault_mask = fault ? bit : 0x00;
        telemetry_.healthy_mask = fault ? 0x00 : bit;
        telemetry_.capability_flags_valid = true;
        telemetry_.module_off = module_off;
        telemetry_.ac_limited = (last_ext_fault_bits_ & TONHE_EXT_FAULT_INPUT_POWER_LIMIT_BIT) != 0;
        telemetry_.temp_derated = (last_ext_fault_bits_ & TONHE_EXT_FAULT_TEMP_POWER_LIMIT_BIT) != 0;
        telemetry_.power_limited = telemetry_.ac_limited || telemetry_.temp_derated;
        telemetry_.current_capability_valid = false;
        telemetry_.current_capability_a = 0.0;
        telemetry_.alarms = (static_cast<uint32_t>(last_ext_fault_bits_) << 16) |
                            (static_cast<uint32_t>(last_fault_bits_) << 8) |
                            static_cast<uint32_t>(last_pfc_fault_);
        if (update_last) {
            telemetry_.last_update = now;
        }
    }

    void handle_frame(const can_frame& frame) {
        if (frame.can_dlc < 8) return;
        const uint32_t id = frame.can_id & CAN_EFF_MASK;
        const uint8_t pf = static_cast<uint8_t>((id >> 16) & 0xFF);
        const uint8_t ps = static_cast<uint8_t>((id >> 8) & 0xFF);
        const uint8_t src = static_cast<uint8_t>(id & 0xFF);
        const uint32_t pgn = (pf < 0xF0) ? (static_cast<uint32_t>(pf) << 8)
                                         : ((static_cast<uint32_t>(pf) << 8) | ps);
        const uint8_t dest = ps;
        if (spec_.source_address > 0 && dest != static_cast<uint8_t>(spec_.source_address & 0xFF) && dest != 0xFF) {
            return;
        }
        if (spec_.address >= 0 && src != static_cast<uint8_t>(spec_.address & 0xFF)) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (pgn == TONHE_PGN_STATE) {
            last_state_ = frame.data[0];
            const uint16_t v_raw = decode_u16_le(&frame.data[1]);
            const uint16_t i_raw = decode_u16_le(&frame.data[3]);
            last_fault_bits_ = decode_u16_le(&frame.data[5]);
            last_pfc_fault_ = frame.data[7];
            telemetry_.voltage_v = static_cast<double>(v_raw) * 0.1;
            telemetry_.current_a = static_cast<double>(i_raw) * 0.01;
            telemetry_.last_voltage_update = now;
            telemetry_.last_current_update = now;
            update_fault_state(now, true);
        } else if (pgn == TONHE_PGN_AC_INFO) {
            const uint16_t temp_raw = decode_u16_le(&frame.data[6]);
            telemetry_.temperature_c = static_cast<double>(temp_raw);
        } else if (pgn == TONHE_PGN_EXT_STATUS) {
            last_ext_fault_bits_ = decode_u16_le(&frame.data[2]);
            const bool fault_active = (last_ext_fault_bits_ & TONHE_EXT_FAULT_SEVERE_MASK) != 0;
            update_fault_state(now, fault_active);
        }
    }

    ModuleSetpoint last_sent_{};
    ModuleSetpoint desired_{};
    bool last_desired_enable_{false};
    std::chrono::steady_clock::time_point enable_requested_at_{};
    std::chrono::steady_clock::time_point last_tx_{};
    std::chrono::steady_clock::time_point last_off_keepalive_tx_{};
    std::shared_ptr<CanChannel> channel_;
    uint8_t last_state_{TONHE_STATE_OFF};
    uint16_t last_fault_bits_{0};
    uint8_t last_pfc_fault_{0};
    uint16_t last_ext_fault_bits_{0};
};

class PowerModuleControllerImpl {
public:
    PowerModuleControllerImpl() = default;
    explicit PowerModuleControllerImpl(std::vector<ModuleSpec> specs) { set_modules(std::move(specs)); }

    void set_can_traffic_policy(const ModuleCanTrafficPolicy& policy) {
        std::lock_guard<std::mutex> lock(mtx_);
        policy_ = policy;
        std::set<std::string> ifaces;
        for (const auto& mod : modules_) {
            if (mod.spec.can_interface.empty() || mod.spec.type == "sim" || mod.spec.type == "simulated") {
                continue;
            }
            ifaces.insert(mod.spec.can_interface);
        }
        CanTrafficGovernor::instance().configure(policy_, ifaces);
    }

    void set_modules(std::vector<ModuleSpec> specs) {
        std::lock_guard<std::mutex> lock(mtx_);
        modules_.clear();
        slot_index_.clear();
        poll_rr_cursor_by_iface_.clear();
        std::set<std::string> ifaces;
        for (auto& spec : specs) {
            if (spec.type.empty()) {
                continue;
            }

            std::string type_lower = spec.type;
            std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            spec.type = type_lower;

            const bool address_required = !(spec.type == "sim" || spec.type == "simulated");
            if (address_required && spec.address < 0) {
                EVLOG_warning << "Skipping module " << spec.id << " (slot " << spec.slot_id
                              << ") because no address was provided";
                continue;
            }
            if (spec.slot_index < 0 || spec.slot_index >= 8) {
                EVLOG_warning << "Skipping module " << spec.id << " (slot " << spec.slot_id
                              << ") because slot_index is invalid (" << spec.slot_index << ")";
                continue;
            }

            // Use a dedicated SocketCAN channel per module to avoid receive-drain races between drivers.
            // (Each socket will still see the bus, but they don't steal frames from each other.)
            std::shared_ptr<CanChannel> chan;
            if (!(spec.type == "sim" || spec.type == "simulated")) {
                chan = std::make_shared<CanChannel>(spec.can_interface, filter_for_type(spec), spec.poll_budget_fps);
                if (!spec.can_interface.empty()) {
                    ifaces.insert(spec.can_interface);
                }
            }
            ModuleRuntime rt;
            rt.spec = spec;
            rt.driver = make_driver(spec, chan);
            if (!rt.driver) {
                EVLOG_warning << "No driver registered for module type '" << spec.type
                              << "'; module " << spec.id << " will be treated as unavailable";
                continue;
            }
            const size_t idx = modules_.size();
            modules_.push_back(std::move(rt));
            slot_index_[spec.slot_id].push_back(idx);
        }
        CanTrafficGovernor::instance().configure(policy_, ifaces);
    }

    void apply(const ModuleCommandRequest& req) {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto it = slot_index_.find(req.slot_id);
        if (it == slot_index_.end()) {
            return;
        }
        const auto& indices = it->second;
        int active_count = 0;
        for (auto idx : indices) {
            if (idx >= modules_.size()) continue;
            const auto& mod = modules_[idx];
            if (!mod.driver) continue;
            const auto& spec = mod.spec;
            const bool selected = (spec.slot_index >= 0 && spec.slot_index < 8) &&
                                  (((req.mask >> spec.slot_index) & 0x1) != 0);
            if (req.enable && selected) {
                active_count++;
            }
        }
        const double current_per_module =
            (active_count > 0) ? (req.current_a / static_cast<double>(active_count)) : 0.0;
        for (auto idx : indices) {
            if (idx >= modules_.size()) continue;
            auto& mod = modules_[idx];
            const bool selected = (mod.spec.slot_index >= 0 && mod.spec.slot_index < 8) &&
                                  (((req.mask >> mod.spec.slot_index) & 0x1) != 0);
            const bool active = req.enable && selected && mod.driver;
            ModuleSetpoint sp;
            sp.enable = active;
            sp.voltage_v = req.voltage_v;
            sp.current_a = active ? current_per_module : 0.0;
            if (mod.driver) {
                mod.driver->apply(sp);
            }
        }
    }

    ModuleHealthSnapshot snapshot(int slot_id) const {
        std::lock_guard<std::mutex> lock(mtx_);
        ModuleHealthSnapshot snap{};
        const auto it = slot_index_.find(slot_id);
        if (it == slot_index_.end()) {
            return snap;
        }
        snap.valid = true;
        const auto now = std::chrono::steady_clock::now();
        double voltage_sum = 0.0;
        int voltage_count = 0;
        double current_sum = 0.0;
        int current_expected_count = 0;
        int current_fresh_count = 0;
        bool any_telem = false;
        bool any_fresh = false;
        for (auto idx : it->second) {
            if (idx >= modules_.size()) continue;
            const auto& mod = modules_[idx];
            const auto iface_status = mod.spec.can_interface.empty()
                                          ? CanTrafficGovernor::IfaceStatus{}
                                          : CanTrafficGovernor::instance().status(mod.spec.can_interface);
            snap.can_total_kbps = std::max(snap.can_total_kbps, iface_status.total_kbps);
            snap.can_budget_limited = snap.can_budget_limited || iface_status.budget_limited;
            snap.can_overload_latched = snap.can_overload_latched || iface_status.overload_latched;
            const auto& telem = mod.driver ? mod.driver->telemetry() : ModuleTelemetryState{};
            const auto stale_interval = telemetry_stale_interval(mod.spec);
            const bool fresh = telem.last_update.time_since_epoch().count() > 0 &&
                               (now - telem.last_update) <= stale_interval;
            const bool voltage_fresh = telem.last_voltage_update.time_since_epoch().count() > 0 &&
                                       (now - telem.last_voltage_update) <= stale_interval;
            const bool current_fresh = telem.last_current_update.time_since_epoch().count() > 0 &&
                                       (now - telem.last_current_update) <= stale_interval;
            if (fresh) {
                any_fresh = true;
            }
            const bool module_off = fresh && telem.capability_flags_valid && telem.module_off;
            const bool module_power_limited = fresh && telem.capability_flags_valid && telem.power_limited;
            const bool module_temp_derated = fresh && telem.capability_flags_valid && telem.temp_derated;
            const bool module_ac_limited = fresh && telem.capability_flags_valid && telem.ac_limited;
            snap.module_off = snap.module_off || module_off;
            snap.module_power_limited = snap.module_power_limited || module_power_limited;
            snap.module_temp_derated = snap.module_temp_derated || module_temp_derated;
            snap.module_ac_limited = snap.module_ac_limited || module_ac_limited;

            const bool healthy = fresh && !telem.fault && telem.fault_mask == 0;
            const uint8_t bit = (mod.spec.slot_index >= 0 && mod.spec.slot_index < 8)
                                    ? static_cast<uint8_t>(1U << static_cast<uint8_t>(mod.spec.slot_index))
                                    : 0x00;
            if (fresh) {
                if (healthy) {
                    if (telem.healthy_mask) {
                        snap.healthy_mask |= telem.healthy_mask;
                    } else {
                        snap.healthy_mask |= bit;
                    }
                } else {
                    if (telem.fault_mask) {
                        snap.fault_mask |= telem.fault_mask;
                    } else {
                        snap.fault_mask |= bit;
                    }
                }
            }
            if (mod.spec.slot_index >= 0 && mod.spec.slot_index < static_cast<int>(snap.temperatures_c.size())) {
                snap.temperatures_c[mod.spec.slot_index] = telem.temperature_c;
            }
            if (voltage_fresh) {
                any_telem = true;
                voltage_sum += telem.voltage_v;
                voltage_count++;
                current_expected_count++;
                if (current_fresh) {
                    current_sum += telem.current_a;
                    current_fresh_count++;
                }
            }

            if (fresh && !telem.fault && !module_off) {
                const bool have_limit_point = current_fresh && std::isfinite(telem.current_limit_point) &&
                                              telem.current_limit_point > 0.0 && telem.current_limit_point <= 1.0;
                const double limit_scale = have_limit_point ? std::clamp(telem.current_limit_point, 0.0, 1.0) : 1.0;
                const bool have_current_capability =
                    telem.current_capability_valid && std::isfinite(telem.current_capability_a) &&
                    telem.current_capability_a >= 0.0;
                if (have_limit_point || have_current_capability) {
                    snap.limit_fresh = true;
                }
                double module_available_current_a = 0.0;
                if (have_current_capability) {
                    module_available_current_a = std::max(0.0, telem.current_capability_a);
                } else if (mod.spec.rated_current_a > 0.1) {
                    module_available_current_a = mod.spec.rated_current_a * limit_scale;
                }
                double module_available_power_kw = 0.0;
                if (mod.spec.rated_power_kw > 0.1) {
                    if (mod.spec.rated_current_a > 0.1 && have_current_capability) {
                        const double capability_scale =
                            std::clamp(module_available_current_a / std::max(0.1, mod.spec.rated_current_a), 0.0, 1.0);
                        module_available_power_kw = mod.spec.rated_power_kw * capability_scale;
                    } else {
                        module_available_power_kw = mod.spec.rated_power_kw * limit_scale;
                    }
                } else if (have_current_capability && module_available_current_a > 0.0 &&
                           voltage_fresh && telem.voltage_v > 0.0) {
                    module_available_power_kw = (telem.voltage_v * module_available_current_a) / 1000.0;
                }
                snap.available_current_a += std::max(0.0, module_available_current_a);
                snap.available_power_kw += std::max(0.0, module_available_power_kw);
            }
        }
        if (any_telem && voltage_count > 0) {
            snap.telemetry_valid = true;
            snap.voltage_v = voltage_sum / static_cast<double>(voltage_count);
        }
        if (snap.telemetry_valid && current_expected_count > 0 && current_fresh_count == current_expected_count) {
            snap.current_valid = true;
            snap.current_a = current_sum;
            snap.power_kw = (snap.voltage_v * snap.current_a) / 1000.0;
        }
        snap.health_valid = any_fresh;
        return snap;
    }

    void poll() {
        std::lock_guard<std::mutex> lock(mtx_);
        std::map<std::string, std::vector<size_t>> by_iface;
        for (size_t idx = 0; idx < modules_.size(); ++idx) {
            if (!modules_[idx].driver) {
                continue;
            }
            const std::string iface = modules_[idx].spec.can_interface.empty() ? "__no_iface__" : modules_[idx].spec.can_interface;
            by_iface[iface].push_back(idx);
        }
        for (auto& kv : by_iface) {
            auto& indices = kv.second;
            if (indices.empty()) {
                continue;
            }
            const size_t start = poll_rr_cursor_by_iface_[kv.first] % indices.size();
            const bool iface_overloaded = kv.first != "__no_iface__" &&
                                          CanTrafficGovernor::instance().overload_latched(kv.first);
            const size_t poll_budget = iface_overloaded ? std::min<size_t>(indices.size(), 1) : indices.size();
            for (size_t i = 0; i < poll_budget; ++i) {
                const size_t pos = (start + i) % indices.size();
                auto& mod = modules_[indices[pos]];
                if (mod.driver) {
                    mod.driver->poll();
                }
            }
            poll_rr_cursor_by_iface_[kv.first] = (start + poll_budget) % indices.size();
        }
    }

private:
    struct ModuleRuntime {
        ModuleSpec spec;
        std::unique_ptr<ModuleDriver> driver;
    };

    static CanFilterSpec filter_for_type(const ModuleSpec& spec) {
        if (spec.type == "maxwell-mxr" || spec.type == "maxwell" || spec.type == "maxwell-max") {
            return {CAN_EFF_FLAG | (static_cast<uint32_t>(MAXWELL_PROT_NO) << 20),
                    CAN_EFF_FLAG | (0x1FFu << 20), true};
        }
        if (spec.type == "maxwell-enr" || spec.type == "enr" || spec.type == "uugreen" ||
            spec.type == "uugreenpower") {
            return {CAN_EFF_FLAG | (static_cast<uint32_t>(RECTIFIER_PROTO_NO) << 25),
                    CAN_EFF_FLAG | (0x0Fu << 25), true};
        }
        if (spec.type == "tonhe" && spec.address >= 0) {
            // Tonhe/J1939-style frames carry module address in SRC (lowest 8 bits).
            return {CAN_EFF_FLAG | static_cast<uint32_t>(spec.address & 0xFF),
                    CAN_EFF_FLAG | 0xFFu, true};
        }
        return {};
    }

    static std::unique_ptr<ModuleDriver> make_driver(const ModuleSpec& spec,
                                                     const std::shared_ptr<CanChannel>& channel) {
        if (spec.type == "sim" || spec.type == "simulated") {
            return std::make_unique<SimulatedModuleDriver>(spec);
        }
        if (spec.type == "maxwell-mxr" || spec.type == "maxwell" || spec.type == "maxwell-max") {
            return std::make_unique<MaxwellModuleDriver>(spec, channel);
        }
        if (spec.type == "maxwell-enr" || spec.type == "enr" || spec.type == "uugreen" ||
            spec.type == "uugreenpower") {
            return std::make_unique<RectifierModuleDriver>(spec, channel);
        }
        if (spec.type == "tonhe") {
            return std::make_unique<TonheModuleDriver>(spec, channel);
        }
        return nullptr;
    }

    mutable std::mutex mtx_;
    std::vector<ModuleRuntime> modules_;
    std::map<int, std::vector<size_t>> slot_index_;
    std::map<std::string, size_t> poll_rr_cursor_by_iface_;
    ModuleCanTrafficPolicy policy_{};
};
} // namespace

struct PowerModuleController::Impl {
    PowerModuleControllerImpl impl;
};

PowerModuleController::PowerModuleController() : impl_(std::make_unique<Impl>()) {}

PowerModuleController::PowerModuleController(const std::vector<ModuleSpec>& specs) :
    impl_(std::make_unique<Impl>()) {
    impl_->impl.set_modules(specs);
}

PowerModuleController::PowerModuleController(const std::vector<ModuleSpec>& specs,
                                             const ModuleCanTrafficPolicy& policy) :
    impl_(std::make_unique<Impl>()) {
    impl_->impl.set_can_traffic_policy(policy);
    impl_->impl.set_modules(specs);
}

PowerModuleController::~PowerModuleController() = default;

void PowerModuleController::set_can_traffic_policy(const ModuleCanTrafficPolicy& policy) {
    if (!impl_) return;
    impl_->impl.set_can_traffic_policy(policy);
}

void PowerModuleController::set_modules(const std::vector<ModuleSpec>& specs) {
    if (!impl_) return;
    impl_->impl.set_modules(specs);
}

void PowerModuleController::apply_command(const ModuleCommandRequest& req) {
    if (!impl_) return;
    impl_->impl.apply(req);
}

ModuleHealthSnapshot PowerModuleController::snapshot_for_slot(int slot_id) const {
    if (!impl_) return {};
    return impl_->impl.snapshot(slot_id);
}

void PowerModuleController::poll() {
    if (!impl_) return;
    impl_->impl.poll();
}

} // namespace charger
