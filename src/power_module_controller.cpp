// SPDX-License-Identifier: Apache-2.0
#include "power_module_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <map>
#include <mutex>
#include <optional>
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
constexpr std::chrono::milliseconds MAXWELL_PERIODIC_TX(500);
constexpr std::chrono::milliseconds MAXWELL_POLL_PERIOD(500);
// Severe faults that should mark modules unusable.
constexpr uint32_t MAXWELL_ALARM_SEVERE_MASK =
    (1u << 0) | (1u << 1) | (1u << 4) | (1u << 5) | (1u << 7) | (1u << 8) | (1u << 9) | (1u << 14) |
    (1u << 16) | (1u << 17) | (1u << 27) | (1u << 28) | (1u << 30) | (1u << 31);
constexpr uint8_t MAXWELL_ALARM_ONOFF_BIT = 22; // 0=On, 1=Off per V1.50 table.
constexpr auto MAXWELL_START_TIMEOUT = std::chrono::seconds(2);
constexpr auto RECTIFIER_START_TIMEOUT = std::chrono::seconds(3);
constexpr auto TONHE_START_TIMEOUT = std::chrono::seconds(3);

constexpr uint32_t RECTIFIER_STATUS_IGNORE_MASK =
    (1u << 22) | (1u << 23) | (1u << 24) | (1u << 25) | (1u << 26) | (1u << 27) | (1u << 30);
constexpr uint32_t RECTIFIER_STATUS_FAULT_MASK = 0xFFFFFFFFu & ~RECTIFIER_STATUS_IGNORE_MASK;

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

constexpr uint32_t TONHE_PGN_STATE = 0x000100;
constexpr uint32_t TONHE_PGN_CONFIRM = 0x000200;
constexpr uint32_t TONHE_PGN_STARTSTOP = 0x000600;
constexpr uint32_t TONHE_PGN_AC_INFO = 0x000B00;
constexpr uint32_t TONHE_PGN_EXT_STATUS = 0x009100;
constexpr uint32_t TONHE_PGN_INPUT_MODE = 0x00AA00;

struct ModuleSetpoint {
    bool enable{false};
    double voltage_v{0.0};
    double current_a{0.0};
    double power_kw{0.0};
};

struct ModuleTelemetryState {
    bool healthy{false};
    bool fault{false};
    double temperature_c{0.0};
    double voltage_v{0.0};
    double current_a{0.0};
    double current_limit_point{0.0};
    uint32_t alarms{0};
    uint16_t reported_group{0};
    uint16_t reported_address{0};
    uint32_t input_mode{0};
    uint8_t healthy_mask{0};
    uint8_t fault_mask{0};
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
        const int fallback = std::max(2000, spec.poll_interval_ms * 3);
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

uint32_t decode_u32_be_from_bytes(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return (static_cast<uint32_t>(b0) << 24) |
           (static_cast<uint32_t>(b1) << 16) |
           (static_cast<uint32_t>(b2) << 8) |
           static_cast<uint32_t>(b3);
}

uint32_t encode_u32_be_bytes(uint32_t v) { return v; }

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

class CanChannel {
public:
    explicit CanChannel(std::string iface, CanFilterSpec filter = {}) : iface_(std::move(iface)), filter_(filter) {
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
        const int flags = ::fcntl(sock_, F_GETFL, 0);
        ::fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
#endif
    }

    ~CanChannel() {
#ifdef __linux__
        if (sock_ >= 0) {
            ::close(sock_);
        }
#endif
    }

    bool valid() const { return sock_ >= 0; }

    bool send(const can_frame& frame) {
#ifdef __linux__
        if (sock_ < 0) return false;
        const auto n = ::write(sock_, &frame, sizeof(frame));
        return n == sizeof(frame);
#else
        (void)frame;
        return false;
#endif
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
    std::string iface_;
    CanFilterSpec filter_;
    int sock_{-1};
};

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

class MaxwellModuleDriver : public ModuleDriver {
public:
    MaxwellModuleDriver(const ModuleSpec& spec, std::shared_ptr<CanChannel> channel) :
        ModuleDriver(spec), channel_(std::move(channel)) {
        last_sent_.voltage_v = 0.0;
    }

    void apply(const ModuleSetpoint& sp) override {
        desired_ = sp;
        const auto now = std::chrono::steady_clock::now();
        if (confirm_pending_ && now > confirm_deadline_ && !confirm_received_) {
            confirm_pending_ = false;
            confirm_miss_count_++;
            if (confirm_miss_count_ >= 3) {
                EVLOG_warning << "Tonhe module " << spec_.id
                              << " did not confirm start/stop command after retries";
                confirm_miss_count_ = 0;
            }
        }
        if (sp.enable && !last_desired_enable_) {
            enable_requested_at_ = now;
        } else if (!sp.enable && last_desired_enable_) {
            enable_requested_at_ = std::chrono::steady_clock::time_point{};
        }
        last_desired_enable_ = sp.enable;
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto cmd_interval = std::chrono::milliseconds(std::max(100, spec_.cmd_interval_ms));
        const bool periodic = (now - last_tx_) >= cmd_interval;
        const bool enable_edge_on = sp.enable && !last_sent_.enable;
        const bool enable_edge_off = !sp.enable && last_sent_.enable;
        const bool voltage_changed = std::fabs(sp.voltage_v - last_sent_.voltage_v) > 0.5;
        const bool current_changed = std::fabs(sp.current_a - last_sent_.current_a) > 0.5;
        const bool power_changed = std::fabs(sp.power_kw - last_sent_.power_kw) > 0.1;

        const bool have_recent_status =
            last_status_update_.time_since_epoch().count() != 0 &&
            (now - last_status_update_) <= telemetry_stale_interval(spec_);
        const bool module_off =
            have_recent_status && ((telemetry_.alarms & (1u << MAXWELL_ALARM_ONOFF_BIT)) != 0);

        const bool should_send =
            enable_edge_on || enable_edge_off ||
            (sp.enable && (voltage_changed || current_changed || periodic)) ||
            (!sp.enable && periodic);

        if (!should_send) {
            return;
        }

        const bool invalid_setpoint = (!std::isfinite(sp.voltage_v) || sp.voltage_v < 0.0) ||
                                      (!std::isfinite(sp.current_a) || sp.current_a < 0.0) ||
                                      (!std::isfinite(sp.power_kw) || sp.power_kw < 0.0);
        if (invalid_setpoint) {
            const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                    ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                    : 0x00;
            telemetry_.fault = true;
            telemetry_.healthy = false;
            telemetry_.fault_mask = bit;
            telemetry_.healthy_mask = 0;
            telemetry_.last_update = now;
            send_set_int(0x0030, 0x00010000); // shutdown (fail-safe)
            last_sent_ = ModuleSetpoint{};
            last_tx_ = now;
            desired_ = ModuleSetpoint{};
            return;
        }

        const double voltage_v = sp.voltage_v > 0.0 ? sp.voltage_v : 0.0;
        const double current_a = sp.current_a > 0.0 ? sp.current_a : 0.0;

        const bool need_input_mode = spec_.input_mode >= 0;
        if (need_input_mode) {
            const auto mode_interval = std::chrono::milliseconds(std::max(200, spec_.cmd_interval_ms));
            const bool mode_stale = (now - last_input_mode_tx_) >= mode_interval;
            if (mode_stale) {
                const bool mismatch = (input_mode_reported_ && telemetry_.input_mode !=
                                                          static_cast<uint32_t>(spec_.input_mode));
                if (!input_mode_reported_ || mismatch) {
                    send_set_int(0x0046, static_cast<uint32_t>(spec_.input_mode));
                    last_input_mode_tx_ = now;
                }
            }
        }

        if (sp.enable) {
            // Start once when transitioning to enable, and retry during periodic updates if the module still reports OFF.
            if (enable_edge_on || (periodic && module_off)) {
                send_set_int(0x0030, 0x00000000); // startup
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
                frac = static_cast<float>(std::clamp(current_a / rated_current, 0.0, 1.0));
            } else {
                // If the module doesn't report a rated current and voltage is too low to infer it,
                // keep Ilim at max only for non-zero current requests.
                frac = 1.0f;
            }
            if (enable_edge_on || current_changed || periodic) {
                send_set_float(0x0022, frac);
                last_limit_fraction_ = frac;
            }
            if (enable_edge_on || voltage_changed || periodic) {
                send_set_float(0x0021, static_cast<float>(voltage_v));
            }
            if (spec_.send_output_current && (enable_edge_on || current_changed || periodic)) {
                const double clamped_a = std::clamp(sp.current_a, 0.0, 4096.0);
                const uint32_t val = static_cast<uint32_t>(clamped_a * 1024.0);
                send_set_int(0x001B, val);
            }
            if (spec_.send_output_power && (enable_edge_on || power_changed || periodic)) {
                send_set_float(0x0020, static_cast<float>(std::max(0.0, sp.power_kw)));
            }
        } else {
            if (enable_edge_off || periodic) {
                send_set_int(0x0030, 0x00010000); // shutdown
            }
        }
        last_sent_ = sp;
        last_tx_ = now;
    }

    void poll() override {
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto poll_interval = std::chrono::milliseconds(std::max(100, spec_.poll_interval_ms));
        const bool direct = !spec_.broadcast || resolved_addr_.has_value();
        if (direct && (now - last_poll_) >= poll_interval) {
            send_read(0x0001); // voltage
            send_read(0x0002); // current
            send_read(0x0004); // DC board temperature
            send_read(0x0040); // alarm/status
            if (spec_.readback_limits) {
                send_read(0x0003); // current limit point
            }
            if (spec_.probe_on_startup && !addr_reported_) {
                send_read(0x0043); // group/address
            }
            if (spec_.input_mode >= 0) {
                send_read(0x004B); // input mode
            }
            last_poll_ = now;
        } else if (!direct && spec_.probe_on_startup) {
            const auto probe_interval = std::chrono::milliseconds(1000);
            if ((now - last_probe_tx_) >= probe_interval) {
                send_read(0x0043);
                last_probe_tx_ = now;
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

    void send_set_float(uint16_t reg, float value) {
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
        channel_->send(frame);
    }

    void send_set_int(uint16_t reg, uint32_t value) {
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
        channel_->send(frame);
    }

    void send_read(uint16_t reg) {
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
        channel_->send(frame);
    }

    void handle_frame(const can_frame& frame) {
        const uint32_t id = frame.can_id & CAN_EFF_MASK;
        const uint16_t prot = static_cast<uint16_t>((id >> 20) & 0x1FF);
        const uint8_t dst_addr = static_cast<uint8_t>((id >> 11) & 0xFF);
        const uint8_t src_addr = static_cast<uint8_t>((id >> 3) & 0xFF);
        const uint8_t group = static_cast<uint8_t>(id & 0x07u);
        const bool group_match = (spec_.group <= 7) ? (group == (spec_.group & 0x07))
                                                    : (group == 0);
        const uint8_t match_addr = resolved_addr_.has_value() ? *resolved_addr_ : static_cast<uint8_t>(spec_.address & 0xFF);
        const bool src_match = src_addr == match_addr;
        const bool broadcast_accept = spec_.broadcast &&
                                      (spec_.address == 0xFE || spec_.address == 0xFF);
        if (prot != MAXWELL_PROT_NO || dst_addr != MAXWELL_CONTROLLER_ADDR || !group_match ||
            (!src_match && !broadcast_accept)) {
            return;
        }
        if (frame.can_dlc < 8) {
            return;
        }
        const uint8_t type = frame.data[0];
        const uint8_t status = frame.data[1];
        const uint16_t reg = static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]);
        if (status != MAXWELL_OK) {
            const uint8_t bit = (spec_.slot_index >= 0 && spec_.slot_index < 8)
                                    ? static_cast<uint8_t>(1U << static_cast<uint8_t>(spec_.slot_index))
                                    : 0x00;
            telemetry_.fault = true;
            telemetry_.healthy = false;
            telemetry_.fault_mask = bit;
            telemetry_.healthy_mask = 0;
            telemetry_.last_update = std::chrono::steady_clock::now();
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (type == MAXWELL_TYPE_FLOAT) {
            const float val = decode_float_be(&frame.data[4]);
            if (!std::isfinite(val)) {
                telemetry_.last_update = now;
                return;
            }
            if (reg == 0x0001) {
                if (val >= -0.1f && val <= 2000.0f) {
                    telemetry_.voltage_v = std::max(0.0, static_cast<double>(val));
                }
            } else if (reg == 0x0002) {
                if (val >= -0.1f && val <= 500.0f) {
                    telemetry_.current_a = std::max(0.0, static_cast<double>(val));
                }
            } else if (reg == 0x0004) {
                if (val >= -50.0f && val <= 200.0f) {
                    telemetry_.temperature_c = static_cast<double>(val);
                }
            } else if (reg == 0x0003) {
                if (val >= -0.1f && val <= 1.5f) {
                    telemetry_.current_limit_point = std::clamp(static_cast<double>(val), 0.0, 1.0);
                    if (spec_.readback_limits && last_limit_fraction_ > 0.0) {
                        const double diff = std::fabs(telemetry_.current_limit_point - last_limit_fraction_);
                        if (diff > 0.2) {
                            limit_mismatch_count_++;
                            if (limit_mismatch_count_ >= 3) {
                                EVLOG_warning << "MXR module " << spec_.id
                                              << " current limit mismatch (set=" << last_limit_fraction_
                                              << " read=" << telemetry_.current_limit_point << ")";
                                limit_mismatch_count_ = 0;
                            }
                        } else {
                            limit_mismatch_count_ = 0;
                        }
                    }
                }
            }
        } else if (type == MAXWELL_TYPE_INT) {
            const uint32_t val = decode_u32_be(&frame.data[4]);
            if (reg == 0x0040) {
                telemetry_.alarms = val;
                last_status_update_ = now;
                const bool module_off = (val & (1u << MAXWELL_ALARM_ONOFF_BIT)) != 0;
                const bool severe = (val & MAXWELL_ALARM_SEVERE_MASK) != 0;
                bool fault = severe;
                if (!fault && desired_.enable && enable_requested_at_.time_since_epoch().count() != 0 && module_off &&
                    (now - enable_requested_at_) > MAXWELL_START_TIMEOUT) {
                    fault = true;
                }
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
                if (spec_.input_mode >= 0 &&
                    telemetry_.input_mode != static_cast<uint32_t>(spec_.input_mode)) {
                    EVLOG_warning << "MXR module " << spec_.id << " input mode mismatch (reported "
                                  << telemetry_.input_mode << ", expected " << spec_.input_mode << ")";
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
    std::chrono::steady_clock::time_point last_poll_{};
    std::chrono::steady_clock::time_point last_input_mode_tx_{};
    std::shared_ptr<CanChannel> channel_;
    bool addr_reported_{false};
    bool input_mode_reported_{false};
    std::optional<uint8_t> resolved_addr_{};
    std::optional<uint8_t> resolved_group_{};
    std::chrono::steady_clock::time_point last_probe_tx_{};
    double last_limit_fraction_{0.0};
    int limit_mismatch_count_{0};
};

class RectifierModuleDriver : public ModuleDriver {
public:
    RectifierModuleDriver(const ModuleSpec& spec, std::shared_ptr<CanChannel> channel) :
        ModuleDriver(spec), channel_(std::move(channel)) {
        last_sent_.voltage_v = 0.0;
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
        const auto cmd_interval = std::chrono::milliseconds(std::max(100, spec_.cmd_interval_ms));
        const bool periodic = (now - last_tx_) >= cmd_interval;
        const bool enable_edge_on = sp.enable && !last_sent_.enable;
        const bool enable_edge_off = !sp.enable && last_sent_.enable;
        const bool voltage_changed = std::fabs(sp.voltage_v - last_sent_.voltage_v) > 0.5;
        const bool current_changed = std::fabs(sp.current_a - last_sent_.current_a) > 0.5;

        const bool have_recent_status =
            last_status_update_.time_since_epoch().count() != 0 &&
            (now - last_status_update_) <= telemetry_stale_interval(spec_);
        const bool module_off = have_recent_status && ((last_status_bits_ & (1u << 25)) != 0);

        const bool should_send =
            enable_edge_on || enable_edge_off ||
            (sp.enable && (voltage_changed || current_changed || periodic)) ||
            (!sp.enable && periodic);
        if (!should_send) {
            return;
        }

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
            telemetry_.last_update = now;
            send_set(4, 1); // shutdown
            last_sent_ = ModuleSetpoint{};
            last_tx_ = now;
            desired_ = ModuleSetpoint{};
            return;
        }

        maybe_send_modes(now);

        const double voltage_v = std::max(0.0, sp.voltage_v);
        const double current_a = std::max(0.0, sp.current_a);

        if (sp.enable) {
            if (enable_edge_on || (periodic && module_off)) {
                send_set(4, 0); // power on
            }
            if (enable_edge_on || voltage_changed || periodic) {
                const uint32_t mv = static_cast<uint32_t>(std::clamp(voltage_v * 1000.0, 0.0, 4.0e9));
                send_set(2, mv); // voltage reference
            }
            if (enable_edge_on || current_changed || periodic) {
                const uint32_t ma = static_cast<uint32_t>(std::clamp(current_a * 1000.0, 0.0, 4.0e9));
                send_set(3, ma); // current limit
            }
        } else {
            if (enable_edge_off || periodic) {
                send_set(4, 1); // power off
            }
        }
        last_sent_ = sp;
        last_tx_ = now;
    }

    void poll() override {
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto poll_interval = std::chrono::milliseconds(std::max(100, spec_.poll_interval_ms));
        if ((now - last_poll_) >= poll_interval) {
            send_read(0);   // output voltage
            send_read(1);   // output current
            send_read(8);   // status
            send_read(30);  // inlet temperature
            if (spec_.hi_lo_mode >= 0) {
                send_read(101); // actual hi/lo mode
            }
            if (spec_.silent_mode >= 0) {
                send_read(62); // silent mode
            }
            if (spec_.probe_on_startup) {
                send_read(89); // group address (best-effort)
            }
            last_poll_ = now;
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

    void send_frame(uint8_t msg_type, uint8_t cmd, uint32_t data) {
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
        channel_->send(frame);
    }

    void send_set(uint8_t cmd, uint32_t data) { send_frame(RECTIFIER_MSG_SET, cmd, data); }
    void send_read(uint8_t cmd) { send_frame(RECTIFIER_MSG_READ, cmd, 0); }

    void maybe_send_modes(const std::chrono::steady_clock::time_point& now) {
        if (!channel_ || !channel_->valid()) return;
        const auto interval = std::chrono::milliseconds(std::max(200, spec_.cmd_interval_ms));
        if (spec_.hi_lo_mode >= 0 && !desired_.enable && (now - last_mode_tx_) >= interval) {
            if (!mode_known_ || mode_reported_ != static_cast<uint8_t>(spec_.hi_lo_mode)) {
                send_set(95, static_cast<uint32_t>(spec_.hi_lo_mode));
                last_mode_tx_ = now;
            }
        }
        if (spec_.silent_mode >= 0 && (now - last_silent_tx_) >= interval) {
            if (!silent_known_ || silent_reported_ != static_cast<uint8_t>(spec_.silent_mode)) {
                send_set(62, static_cast<uint32_t>(spec_.silent_mode));
                last_silent_tx_ = now;
            }
        }
        if (spec_.group > 0 && !desired_.enable && (now - last_group_tx_) >= interval) {
            if (!group_known_ || group_reported_ != static_cast<uint8_t>(spec_.group)) {
                send_set(89, static_cast<uint32_t>(spec_.group & 0x0F));
                last_group_tx_ = now;
            }
        }
    }

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
            } else if (cmd == 1) {
                telemetry_.current_a = static_cast<double>(val) / 1000.0;
            } else if (cmd == 30) {
                telemetry_.temperature_c = static_cast<double>(val) / 1000.0;
            } else if (cmd == 8) {
                telemetry_.alarms = val;
                last_status_bits_ = val;
                last_status_update_ = now;
                const bool module_off = (val & (1u << 25)) != 0;
                bool fault = (val & RECTIFIER_STATUS_FAULT_MASK) != 0;
                if (!fault && desired_.enable && enable_requested_at_.time_since_epoch().count() != 0 && module_off &&
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
            } else if (cmd == 101 || cmd == 96) {
                mode_reported_ = static_cast<uint8_t>(val & 0xFF);
                mode_reported_ = static_cast<uint8_t>(std::clamp<int>(mode_reported_, 0, 3));
                mode_known_ = true;
            } else if (cmd == 62) {
                silent_reported_ = static_cast<uint8_t>(val & 0xFF);
                silent_reported_ = static_cast<uint8_t>(std::clamp<int>(silent_reported_, 0, 2));
                silent_known_ = true;
            } else if (cmd == 89) {
                group_reported_ = static_cast<uint8_t>(val & 0x0F);
                group_known_ = true;
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
    std::chrono::steady_clock::time_point last_poll_{};
    std::shared_ptr<CanChannel> channel_;
    std::optional<uint8_t> resolved_addr_{};
    uint32_t last_status_bits_{0};
    std::chrono::steady_clock::time_point last_mode_tx_{};
    std::chrono::steady_clock::time_point last_silent_tx_{};
    std::chrono::steady_clock::time_point last_group_tx_{};
    bool mode_known_{false};
    bool silent_known_{false};
    bool group_known_{false};
    uint8_t mode_reported_{0};
    uint8_t silent_reported_{0};
    uint8_t group_reported_{0};
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
        const auto cmd_interval = std::chrono::milliseconds(std::max(100, spec_.cmd_interval_ms));
        const bool periodic = (now - last_tx_) >= cmd_interval;
        const bool enable_edge_on = sp.enable && !last_sent_.enable;
        const bool enable_edge_off = !sp.enable && last_sent_.enable;
        const bool voltage_changed = std::fabs(sp.voltage_v - last_sent_.voltage_v) > 0.5;
        const bool current_changed = std::fabs(sp.current_a - last_sent_.current_a) > 0.5;

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
            telemetry_.last_update = now;
            send_startstop(false, 0.0, 0.0);
            last_sent_ = ModuleSetpoint{};
            last_tx_ = now;
            desired_ = ModuleSetpoint{};
            return;
        }

        maybe_send_input_mode(now);

        const bool should_send =
            enable_edge_on || enable_edge_off ||
            (sp.enable && (voltage_changed || current_changed || periodic)) ||
            (!sp.enable && periodic);
        if (should_send) {
            const double voltage_v = std::max(0.0, sp.voltage_v);
            const double current_a = std::max(0.0, sp.current_a);
            send_startstop(sp.enable, voltage_v, current_a);
            last_sent_ = sp;
            last_tx_ = now;
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
    void send_startstop(bool enable, double voltage_v, double current_a) {
        const auto now = std::chrono::steady_clock::now();
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
        channel_->send(frame);
        confirm_pending_ = true;
        confirm_received_ = false;
        confirm_deadline_ = now + std::chrono::milliseconds(1000);
    }

    void maybe_send_input_mode(const std::chrono::steady_clock::time_point& now) {
        if (spec_.input_mode < 0) return;
        if (desired_.enable) return;
        if (last_state_ == TONHE_STATE_ON) return;
        const auto interval = std::chrono::milliseconds(std::max(1000, spec_.cmd_interval_ms));
        if (input_mode_sent_ && (now - last_input_mode_tx_) < interval) {
            return;
        }
        const int desired_mode = (spec_.input_mode == 2) ? 0 : 1; // 0=DC, 1=AC
        if (input_mode_sent_ && desired_mode == last_input_mode_) {
            return;
        }
        can_frame frame{};
        frame.can_id = build_tonhe_id(6, TONHE_PGN_INPUT_MODE, 0xFF,
                                      static_cast<uint8_t>(spec_.source_address & 0xFF));
        frame.can_dlc = 8;
        frame.data[0] = static_cast<uint8_t>(desired_mode & 0xFF);
        for (int i = 1; i < 8; ++i) {
            frame.data[i] = 0x00;
        }
        channel_->send(frame);
        input_mode_sent_ = true;
        last_input_mode_ = desired_mode;
        last_input_mode_tx_ = now;
    }

    void update_fault_state(const std::chrono::steady_clock::time_point& now, bool update_last) {
        bool fault = (last_fault_bits_ & TONHE_FAULT_SEVERE_MASK) != 0 ||
                     (last_pfc_fault_ != 0) ||
                     (last_ext_fault_bits_ & TONHE_EXT_FAULT_SEVERE_MASK) != 0 ||
                     (last_state_ == TONHE_STATE_FAULT_OFF);
        const bool module_off = last_state_ != TONHE_STATE_ON;
        if (!fault && desired_.enable && enable_requested_at_.time_since_epoch().count() != 0 && module_off &&
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
            update_fault_state(now, true);
        } else if (pgn == TONHE_PGN_CONFIRM) {
            confirm_received_ = frame.data[0] == 0x01;
            if (confirm_received_) {
                confirm_pending_ = false;
                confirm_miss_count_ = 0;
            }
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
    std::shared_ptr<CanChannel> channel_;
    uint8_t last_state_{TONHE_STATE_OFF};
    uint16_t last_fault_bits_{0};
    uint8_t last_pfc_fault_{0};
    uint16_t last_ext_fault_bits_{0};
    bool confirm_received_{false};
    bool confirm_pending_{false};
    int confirm_miss_count_{0};
    std::chrono::steady_clock::time_point confirm_deadline_{};
    bool input_mode_sent_{false};
    int last_input_mode_{-1};
    std::chrono::steady_clock::time_point last_input_mode_tx_{};
};

class PowerModuleControllerImpl {
public:
    PowerModuleControllerImpl() = default;
    explicit PowerModuleControllerImpl(std::vector<ModuleSpec> specs) { set_modules(std::move(specs)); }

    void set_modules(std::vector<ModuleSpec> specs) {
        std::lock_guard<std::mutex> lock(mtx_);
        modules_.clear();
        slot_index_.clear();
        for (auto& spec : specs) {
            if (spec.type.empty()) {
                continue;
            }
            if (spec.address < 0) {
                EVLOG_warning << "Skipping module " << spec.id << " (slot " << spec.slot_id
                              << ") because no address was provided";
                continue;
            }
            if (spec.slot_index < 0 || spec.slot_index >= 8) {
                EVLOG_warning << "Skipping module " << spec.id << " (slot " << spec.slot_id
                              << ") because slot_index is invalid (" << spec.slot_index << ")";
                continue;
            }
            std::string type_lower = spec.type;
            std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            spec.type = type_lower;
            // Use a dedicated SocketCAN channel per module to avoid receive-drain races between drivers.
            // (Each socket will still see the bus, but they don't steal frames from each other.)
            auto chan = std::make_shared<CanChannel>(spec.can_interface, filter_for_type(spec));
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
    }

    void apply(const ModuleCommandRequest& req) {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto it = slot_index_.find(req.slot_id);
        if (it == slot_index_.end()) {
            return;
        }
        const auto& indices = it->second;
        int enabled = 0;
        for (auto idx : indices) {
            if (idx >= modules_.size()) continue;
            const auto& spec = modules_[idx].spec;
            const bool selected = (spec.slot_index >= 0 && spec.slot_index < 8) &&
                                  (((req.mask >> spec.slot_index) & 0x1) != 0);
            if (req.enable && selected) {
                enabled++;
            }
        }
        const double current_per_module = (enabled > 0) ? (req.current_a / static_cast<double>(enabled)) : 0.0;
        const double power_per_module = (enabled > 0) ? (req.power_kw / static_cast<double>(enabled)) : 0.0;
        for (auto idx : indices) {
            if (idx >= modules_.size()) continue;
            auto& mod = modules_[idx];
            const bool selected = (mod.spec.slot_index >= 0 && mod.spec.slot_index < 8) &&
                                  (((req.mask >> mod.spec.slot_index) & 0x1) != 0);
            const bool active = req.enable && selected;
            ModuleSetpoint sp;
            sp.enable = active;
            sp.voltage_v = req.voltage_v;
            sp.current_a = active ? current_per_module : 0.0;
            sp.power_kw = active ? power_per_module : 0.0;
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
        bool any_telem = false;
        bool any_fresh = false;
        for (auto idx : it->second) {
            if (idx >= modules_.size()) continue;
            const auto& mod = modules_[idx];
            const auto& telem = mod.driver ? mod.driver->telemetry() : ModuleTelemetryState{};
            const bool fresh = telem.last_update.time_since_epoch().count() > 0 &&
                               (now - telem.last_update) <= telemetry_stale_interval(mod.spec);
            if (fresh) {
                any_fresh = true;
            }
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
            if (fresh) {
                any_telem = true;
                voltage_sum += telem.voltage_v;
                voltage_count++;
                current_sum += telem.current_a;
            }
        }
        if (any_telem && voltage_count > 0) {
            snap.telemetry_valid = true;
            snap.voltage_v = voltage_sum / static_cast<double>(voltage_count);
            snap.current_a = current_sum;
            snap.power_kw = (snap.voltage_v * snap.current_a) / 1000.0;
        }
        snap.health_valid = any_fresh;
        return snap;
    }

    void poll() {
        std::lock_guard<std::mutex> lock(mtx_);
        for (auto& mod : modules_) {
            if (mod.driver) {
                mod.driver->poll();
            }
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
        return {};
    }

    static std::unique_ptr<ModuleDriver> make_driver(const ModuleSpec& spec,
                                                     const std::shared_ptr<CanChannel>& channel) {
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

PowerModuleController::~PowerModuleController() = default;

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
