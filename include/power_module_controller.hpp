// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace charger {

struct ModuleSpec {
    std::string id;
    int slot_id{0};
    int slot_index{0};
    std::string type;
    std::string can_interface;
    int address{-1};
    int group{0};
    int monitor_address{1};
    int production_day{0};
    int serial_low{0};
    int source_address{0xA0};
    int input_mode{-1};
    int hi_lo_mode{-1};
    int silent_mode{-1};
    double rated_power_kw{0.0};
    double rated_current_a{0.0};
    int poll_interval_ms{500};
    int cmd_interval_ms{500};
    int poll_budget_fps{0};
    int telemetry_stale_ms{0};
    bool broadcast{false}; // send via broadcast DST (0xFE or extended)
    bool probe_on_startup{true};
    bool readback_limits{false};
    bool send_output_current{false};
    bool send_output_power{false};
};

struct ModuleCommandRequest {
    int slot_id{0};
    uint8_t mask{0};
    double voltage_v{0.0};
    double current_a{0.0};
    double power_kw{0.0};
    bool enable{false};
};

struct ModuleHealthSnapshot {
    bool valid{false};
    bool health_valid{false};
    uint8_t healthy_mask{0};
    uint8_t fault_mask{0};
    std::array<double, 2> temperatures_c{{0.0, 0.0}};
    bool telemetry_valid{false};
    bool current_valid{false};
    double voltage_v{0.0};
    double current_a{0.0};
    double power_kw{0.0};
    bool can_budget_limited{false};
    bool can_overload_latched{false};
    double can_total_kbps{0.0};
};

struct ModuleCanTrafficPolicy {
    double max_total_kbps_per_interface{20.0};
    int window_ms{10000};
    int bits_per_frame_estimate{150};
    int over_cap_debounce_ms{5000};
    bool enforce{true};
};

/// \brief Externalized power-module controller with pluggable module drivers (e.g. Maxwell MXR).
class PowerModuleController {
public:
    PowerModuleController();
    explicit PowerModuleController(const std::vector<ModuleSpec>& specs);
    PowerModuleController(const std::vector<ModuleSpec>& specs, const ModuleCanTrafficPolicy& policy);
    ~PowerModuleController();

    void set_can_traffic_policy(const ModuleCanTrafficPolicy& policy);
    void set_modules(const std::vector<ModuleSpec>& specs);
    void apply_command(const ModuleCommandRequest& req);
    ModuleHealthSnapshot snapshot_for_slot(int slot_id) const;
    void poll();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace charger
