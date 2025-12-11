// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <limits>
#include <vector>

namespace charger {

enum class ContactorState { Open, Closed };

enum class GunFsmState { Idle, EvDetected, Ready, IslandReady, Charging, RampDown, Fault };

struct Slot {
    int id{0};
    std::vector<std::string> modules; // e.g. {"M1_0", "M1_1"}
    int gun_id{0};                    // gun mapped to this slot
    std::string gc_id;                // gun contactor id
    std::string mc_id;                // bus cut contactor id
    int cw_id{0};
    int ccw_id{0};
};

struct ModuleState {
    std::string id;
    int slot_id{0};
    std::string mn_id;
    bool healthy{true};
    bool enabled{false};
};

struct GunState {
    int id{0};
    int slot_id{0};
    std::string gc_id;
    GunFsmState fsm_state{GunFsmState::Idle};
    bool ev_session_active{false};
    double ev_req_power_kw{0.0};
    double ev_req_voltage_v{0.0};
    double gun_power_limit_kw{0.0};
    double gun_current_limit_a{0.0};
    int priority{0};
    double i_meas_a{0.0};
    double i_set_a{0.0};
    double connector_temp_c{0.0};
};

struct IslandState {
    int id{0};
    std::vector<int> slot_ids;
    std::vector<std::string> module_ids;
    std::optional<int> gun_id;
    double v_set_v{0.0};
    double p_set_kw{0.0};
};

struct PlannerConfig {
    double module_power_kw{30.0};
    double grid_limit_kw{1000.0};
    double ov_current_factor{1.1};
    double ramp_step_a{10.0};
};

struct GunDispatch {
    int gun_id{0};
    double p_budget_kw{0.0};
    int modules_assigned{0};
    double current_limit_a{0.0};
    double voltage_set_v{0.0};
};

struct Plan {
    std::map<std::string, ContactorState> mc_commands;
    std::map<std::string, ContactorState> gc_commands;
    std::map<std::string, ContactorState> mn_commands;
    std::vector<IslandState> islands;
    std::vector<GunDispatch> guns;
};

class PowerManager {
public:
    explicit PowerManager(PlannerConfig cfg);

    void set_slots(std::vector<Slot> slots);
    void update_modules(const std::vector<ModuleState>& modules);
    void update_guns(const std::vector<GunState>& guns);

    Plan compute_plan();

private:
    PlannerConfig cfg_;
    std::vector<Slot> slots_;
    std::map<std::string, ModuleState> modules_;
    std::map<int, GunState> guns_;
    int next_island_id_{1};

    std::vector<int> active_guns() const;
    std::map<int, double> compute_power_budgets(const std::vector<int>& active,
                                                const std::map<int, double>& req_limited,
                                                int healthy_modules) const;
    std::map<int, int> compute_module_allocation(const std::vector<int>& active,
                                                 const std::map<int, double>& budgets,
                                                 const std::map<int, int>& ideal,
                                                 int healthy_modules) const;
    Plan build_plan(const std::vector<int>& active, const std::map<int, double>& budgets,
                    const std::map<int, int>& modules_per_gun);
};

} // namespace charger
