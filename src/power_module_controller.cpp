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
constexpr uint8_t MAXWELL_FUNC_SET = 0x03;
constexpr uint8_t MAXWELL_FUNC_READ = 0x10;
constexpr uint8_t MAXWELL_TYPE_FLOAT = 0x41;
constexpr uint8_t MAXWELL_TYPE_INT = 0x42;
constexpr uint8_t MAXWELL_OK = 0xF0;
constexpr uint8_t MAXWELL_CONTROLLER_ADDR = 0xF0;
constexpr std::chrono::milliseconds MAXWELL_PERIODIC_TX(500);
constexpr std::chrono::milliseconds MAXWELL_POLL_PERIOD(500);
constexpr std::chrono::seconds TELEMETRY_STALE(2);

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
    uint32_t alarms{0};
    uint8_t healthy_mask{0};
    uint8_t fault_mask{0};
    std::chrono::steady_clock::time_point last_update{};
};

void map_maxwell_alarms(uint32_t alarms, uint8_t& healthy_mask, uint8_t& fault_mask) {
    if (alarms == 0) {
        healthy_mask = 0x03;
        fault_mask = 0x00;
        return;
    }
    // Severe faults that should mark modules unusable
    constexpr uint32_t SEVERE_MASK =
        (1u << 0) |  (1u << 1) |  (1u << 3) |  (1u << 4) |  (1u << 5) |
        (1u << 7) |  (1u << 8) |  (1u << 9) |  (1u << 14) | (1u << 16) |
        (1u << 17) | (1u << 22) | (1u << 27) | (1u << 28) | (1u << 30) |
        (1u << 31);
    if ((alarms & SEVERE_MASK) != 0) {
        healthy_mask = 0x00;
        fault_mask = 0x03; // both modules in slot
    } else {
        healthy_mask = 0x03;
        fault_mask = 0x00;
    }
}

int popcount(uint8_t v) {
    int count = 0;
    while (v) {
        count += (v & 0x1);
        v >>= 1U;
    }
    return count;
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

class CanChannel {
public:
    explicit CanChannel(std::string iface) : iface_(std::move(iface)) {
#ifdef __linux__
        sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            EVLOG_error << "Failed to open CAN socket on " << iface_;
            return;
        }
        const int on = 1;
        ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on));
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
        struct can_filter filter {};
        filter.can_id = (static_cast<uint32_t>(MAXWELL_PROT_NO) << 20) | CAN_EFF_FLAG;
        filter.can_mask = (static_cast<uint32_t>(0x1FF) << 20) | CAN_EFF_FLAG;
        ::setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
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
        if (!channel_ || !channel_->valid() || spec_.address < 0) {
            return;
        }
        const auto cmd_interval = std::chrono::milliseconds(std::max(100, spec_.cmd_interval_ms));
        const bool state_change = (sp.enable != last_sent_.enable) ||
                                  (std::fabs(sp.voltage_v - last_sent_.voltage_v) > 0.5) ||
                                  (std::fabs(sp.current_a - last_sent_.current_a) > 0.5) ||
                                  (std::fabs(sp.power_kw - last_sent_.power_kw) > 0.5);
        const bool periodic = (now - last_tx_) >= cmd_interval;
        if (!state_change && !periodic) {
            return;
        }
        if (sp.enable) {
            send_set_float(0x0021, static_cast<float>(sp.voltage_v));
            const double rated_current = spec_.rated_current_a > 0.0
                                             ? spec_.rated_current_a
                                             : (spec_.rated_power_kw > 0.0 && sp.voltage_v > 1.0
                                                    ? (spec_.rated_power_kw * 1000.0) / sp.voltage_v
                                                    : 0.0);
            const float frac = rated_current > 0.0
                                   ? static_cast<float>(std::clamp(sp.current_a / rated_current, 0.0, 1.0))
                                   : 1.0f;
            send_set_float(0x0022, frac);
        if (spec_.rated_power_kw > 0.0 && sp.power_kw > 0.0) {
            const float p_frac =
                static_cast<float>(std::clamp(sp.power_kw / spec_.rated_power_kw, 0.0, 1.0));
            send_set_float(0x0020, p_frac);
        }
        send_set_int(0x0030, 0x00000000); // startup
    } else {
        send_set_int(0x0030, 0x00010000); // shutdown
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
        if (!spec_.broadcast && (now - last_poll_) >= poll_interval) {
            send_read(0x0001); // voltage
            send_read(0x0002); // current
            send_read(0x0004); // DC board temperature
            send_read(0x0040); // alarm/status
            last_poll_ = now;
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
        id |= (1U << 19); // PTP
        uint32_t dst = static_cast<uint32_t>(spec_.address & 0xFF);
        if (spec_.broadcast) {
            if (spec_.group <= 7) {
                dst = 0xFE;
            } else {
                const int ext_group = std::min(60, std::max(0, spec_.group));
                dst = static_cast<uint32_t>(0xFD - ext_group);
            }
        }
        id |= (dst << 11);
        id |= (static_cast<uint32_t>(MAXWELL_CONTROLLER_ADDR) << 3);
        id |= (static_cast<uint32_t>(spec_.group & 0x07));
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
        const uint8_t src_addr = static_cast<uint8_t>((id >> 3) & 0xFF);
        if (prot != MAXWELL_PROT_NO || src_addr != static_cast<uint8_t>(spec_.address & 0xFF)) {
            return;
        }
        if (frame.can_dlc < 8) {
            return;
        }
        const uint8_t type = frame.data[0];
        const uint8_t status = frame.data[1];
        const uint16_t reg = static_cast<uint16_t>((frame.data[2] << 8) | frame.data[3]);
        if (status != MAXWELL_OK) {
            telemetry_.fault = true;
            telemetry_.healthy = false;
            telemetry_.last_update = std::chrono::steady_clock::now();
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (type == MAXWELL_TYPE_FLOAT) {
            const float val = decode_float_be(&frame.data[4]);
            if (reg == 0x0001) {
                telemetry_.voltage_v = val;
            } else if (reg == 0x0002) {
                telemetry_.current_a = val;
            } else if (reg == 0x0004) {
                telemetry_.temperature_c = val;
            }
        } else if (type == MAXWELL_TYPE_INT) {
            const uint32_t val = decode_u32_be(&frame.data[4]);
            if (reg == 0x0040) {
                telemetry_.alarms = val;
                telemetry_.fault = val != 0;
                uint8_t healthy_mask = 0x00;
                uint8_t fault_mask = 0x00;
                map_maxwell_alarms(val, healthy_mask, fault_mask);
                telemetry_.healthy_mask = healthy_mask;
                telemetry_.fault_mask = fault_mask;
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
    std::chrono::steady_clock::time_point last_tx_{};
    std::chrono::steady_clock::time_point last_poll_{};
    std::shared_ptr<CanChannel> channel_;
};

class PowerModuleControllerImpl {
public:
    PowerModuleControllerImpl() = default;
    explicit PowerModuleControllerImpl(std::vector<ModuleSpec> specs) { set_modules(std::move(specs)); }

    void set_modules(std::vector<ModuleSpec> specs) {
        std::lock_guard<std::mutex> lock(mtx_);
        modules_.clear();
        slot_index_.clear();
        channels_.clear();
        for (auto& spec : specs) {
            if (spec.type.empty()) {
                continue;
            }
            if (spec.address < 0) {
                EVLOG_warning << "Skipping module " << spec.id << " (slot " << spec.slot_id
                              << ") because no address was provided";
                continue;
            }
            std::string type_lower = spec.type;
            std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            spec.type = type_lower;
            auto chan = get_or_create_channel(spec.can_interface);
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
            if (req.enable && ((req.mask >> spec.slot_index) & 0x1)) {
                enabled++;
            }
        }
        const double current_per_module = (enabled > 0) ? (req.current_a / static_cast<double>(enabled)) : 0.0;
        const double power_per_module = (enabled > 0) ? (req.power_kw / static_cast<double>(enabled)) : 0.0;
        for (auto idx : indices) {
            if (idx >= modules_.size()) continue;
            auto& mod = modules_[idx];
            const bool active = req.enable && ((req.mask >> mod.spec.slot_index) & 0x1);
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
        for (auto idx : it->second) {
            if (idx >= modules_.size()) continue;
            const auto& mod = modules_[idx];
            const auto& telem = mod.driver ? mod.driver->telemetry() : ModuleTelemetryState{};
            const bool fresh = telem.last_update.time_since_epoch().count() > 0 &&
                               (now - telem.last_update) <= TELEMETRY_STALE;
            const bool healthy = fresh && !telem.fault && telem.fault_mask == 0 && telem.alarms == 0;
            const uint8_t bit = static_cast<uint8_t>(1U << mod.spec.slot_index);
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
            if (mod.spec.slot_index < static_cast<int>(snap.temperatures_c.size())) {
                snap.temperatures_c[mod.spec.slot_index] = telem.temperature_c;
            }
        }
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

    std::shared_ptr<CanChannel> get_or_create_channel(const std::string& iface) {
        const auto it = channels_.find(iface);
        if (it != channels_.end()) {
            return it->second;
        }
        auto chan = std::make_shared<CanChannel>(iface);
        channels_[iface] = chan;
        return chan;
    }

    static std::unique_ptr<ModuleDriver> make_driver(const ModuleSpec& spec,
                                                     const std::shared_ptr<CanChannel>& channel) {
        if (spec.type == "maxwell-mxr" || spec.type == "maxwell") {
            return std::make_unique<MaxwellModuleDriver>(spec, channel);
        }
        return nullptr;
    }

    mutable std::mutex mtx_;
    std::vector<ModuleRuntime> modules_;
    std::map<int, std::vector<size_t>> slot_index_;
    std::map<std::string, std::shared_ptr<CanChannel>> channels_;
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
