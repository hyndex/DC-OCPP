// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <chrono>
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
    int slot_index{0};
    std::string type;
    std::string can_interface;
    int address{-1};
    int group{0};
    double rated_power_kw{0.0};
    double rated_current_a{0.0};
    bool healthy{true};
    bool enabled{false};
    double temperature_c{0.0};
};

struct GunState {
    int id{0};
    int slot_id{0};
    std::string gc_id;
    GunFsmState fsm_state{GunFsmState::Idle};
    bool ev_session_active{false};
    bool safety_ok{true};
    bool gc_welded{false};
    bool mc_welded{false};
    double ev_req_power_kw{0.0};
    double ev_req_voltage_v{0.0};
    double min_voltage_v{0.0};
    double max_voltage_v{0.0};
    double gun_power_limit_kw{0.0};
    double gun_current_limit_a{0.0};
    int priority{0};
    double i_meas_a{0.0};
    double i_set_a{0.0};
    double connector_temp_c{0.0};
    bool plugged_in{false};
    bool reserved{false};
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
    double min_voltage_v_for_div{5.0};
    double ideal_low_factor{0.8};
    double ideal_high_factor{1.6};
    double default_voltage_v{800.0};
    double connector_derate_start_c{80.0};
    double connector_derate_trip_c{90.0};
    double module_derate_start_c{75.0};
    double module_derate_trip_c{85.0};
    bool allow_cross_slot_islands{false};
    int min_module_hold_ms{1000};
    int max_modules_per_gun{2};
    int min_modules_per_active_gun{1};
    int max_island_radius{6};
    int min_mc_hold_ms{1000};
    int min_gc_hold_ms{500};
    double mc_open_current_a{1.0};
    double gc_open_current_a{1.0};
};

struct GunDispatch {
    int gun_id{0};
    double p_budget_kw{0.0};
    double p_set_kw{0.0};
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
    std::map<int, Slot> slot_lookup_;
    std::map<int, int> last_modules_assigned_;
    std::map<int, std::vector<std::string>> last_module_ids_;
    std::map<int, std::chrono::steady_clock::time_point> last_alloc_change_;

    std::vector<int> active_guns() const;
    Plan blank_plan() const;
    const Slot* find_slot(int slot_id) const;
    int count_healthy_modules_in_slot(int slot_id) const;
    int ideal_modules_for_gun(const GunState& g, double p_budget) const;
    std::vector<std::string> select_modules_for_slot(const Slot& slot, int n_needed, Plan& plan,
                                                     const std::vector<std::string>& preferred = {}) const;
    std::vector<int> build_island_slots_for_gun(const GunState& g, int n_needed,
                                                const std::set<int>& active_home_slots,
                                                const std::set<int>& reserved_slots,
                                                std::set<int>& claimed_slots) const;
    std::vector<std::string> assign_modules_for_island(const std::vector<int>& slot_ids, int n_needed, Plan& plan,
                                                       const std::vector<std::string>& preferred = {}) const;
    std::map<int, double> compute_power_budgets(const std::vector<int>& active,
                                                const std::map<int, double>& req_limited,
                                                int healthy_modules) const;
    std::map<int, int> compute_module_allocation(const std::vector<int>& active,
                                                 const std::map<int, double>& budgets,
                                                const std::map<int, int>& ideal,
                                                 int healthy_modules) const;
    Plan build_plan(const std::vector<int>& active, const std::map<int, double>& budgets,
                    const std::map<int, int>& modules_per_gun,
                    const std::set<int>& reserved_slots);
    void apply_hysteresis(Plan& plan, std::chrono::steady_clock::time_point now);
    bool validate_plan(const Plan& plan) const;
};

} // namespace charger
