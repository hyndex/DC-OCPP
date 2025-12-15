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
    double rated_power_kw{0.0};
    double rated_current_a{0.0};
    int poll_interval_ms{500};
    int cmd_interval_ms{500};
    bool broadcast{false}; // send via broadcast DST (0xFE or extended)
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
    uint8_t healthy_mask{0};
    uint8_t fault_mask{0};
    std::array<double, 2> temperatures_c{{0.0, 0.0}};
};

/// \brief Externalized power-module controller with pluggable module drivers (e.g. Maxwell MXR).
class PowerModuleController {
public:
    PowerModuleController();
    explicit PowerModuleController(const std::vector<ModuleSpec>& specs);
    ~PowerModuleController();

    void set_modules(const std::vector<ModuleSpec>& specs);
    void apply_command(const ModuleCommandRequest& req);
    ModuleHealthSnapshot snapshot_for_slot(int slot_id) const;
    void poll();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace charger
