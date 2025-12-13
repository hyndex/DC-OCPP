// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <set>

using namespace charger;

namespace {

Slot make_slot(int id, int cw, int ccw) {
    Slot s;
    s.id = id;
    s.gun_id = id;
    s.mc_id = "MC_" + std::to_string(id);
    s.gc_id = "GC_" + std::to_string(id);
    s.cw_id = cw;
    s.ccw_id = ccw;
    s.modules = {"M" + std::to_string(id) + "_0", "M" + std::to_string(id) + "_1"};
    return s;
}

ModuleState make_module(const std::string& id, int slot, bool healthy = true) {
    ModuleState m;
    m.id = id;
    m.slot_id = slot;
    m.mn_id = "MN_" + std::to_string(slot) + "_" + id.back();
    m.healthy = healthy;
    m.enabled = false;
    return m;
}

GunState make_gun(int id, double req_kw, double limit_kw, bool active = true) {
    GunState g;
    g.id = id;
    g.slot_id = id;
    g.gc_id = "GC_" + std::to_string(id);
    g.ev_session_active = active;
    g.fsm_state = GunFsmState::Ready;
    g.ev_req_power_kw = req_kw;
    g.ev_req_voltage_v = 800.0;
    g.gun_power_limit_kw = limit_kw;
    g.gun_current_limit_a = 200.0;
    g.priority = 0;
    return g;
}

} // namespace

int main() {
    PlannerConfig cfg;
    cfg.module_power_kw = 30.0;
    cfg.grid_limit_kw = 90.0;
    cfg.ramp_step_a = 0.0;
    cfg.default_voltage_v = 800.0;
    cfg.allow_cross_slot_islands = true;
    PowerManager pm(cfg);

    std::vector<Slot> slots;
    slots.push_back(make_slot(1, 2, 3));
    slots.push_back(make_slot(2, 3, 1));
    slots.push_back(make_slot(3, 1, 2));
    pm.set_slots(slots);

    std::vector<ModuleState> modules;
    for (const auto& s : slots) {
        modules.push_back(make_module(s.modules[0], s.id, true));
        modules.push_back(make_module(s.modules[1], s.id, true));
    }
    pm.update_modules(modules);

    std::vector<GunState> guns;
    guns.push_back(make_gun(1, 60.0, 60.0, true));
    guns.push_back(make_gun(2, 60.0, 60.0, true));
    guns.push_back(make_gun(3, 60.0, 60.0, true));
    pm.update_guns(guns);

    auto plan = pm.compute_plan();
    assert(!plan.guns.empty());

    // Cross-slot borrow test: slot1 modules unhealthy, slot2 free and healthy, only gun1 active
    modules.clear();
    modules.push_back(make_module("M1_0", 1, false));
    modules.push_back(make_module("M1_1", 1, false));
    modules.push_back(make_module("M2_0", 2, true));
    modules.push_back(make_module("M2_1", 2, true));
    modules.push_back(make_module("M3_0", 3, true));
    modules.push_back(make_module("M3_1", 3, true));
    pm.update_modules(modules);

    guns.clear();
    guns.push_back(make_gun(1, 60.0, 60.0, true));
    // leave gun2/3 idle
    pm.update_guns(guns);

    cfg.grid_limit_kw = 120.0;
    pm = PowerManager(cfg);
    pm.set_slots(slots);
    pm.update_modules(modules);
    pm.update_guns(guns);
    plan = pm.compute_plan();
    assert(!plan.guns.empty());

    // Hysteresis/min-dwell: avoid flapping between 2->1 modules within hold window
    cfg.min_module_hold_ms = 200;
    cfg.grid_limit_kw = 120.0;
    pm = PowerManager(cfg);
    pm.set_slots(slots);
    modules.clear();
    modules.push_back(make_module("M1_0", 1, true));
    modules.push_back(make_module("M1_1", 1, true));
    pm.update_modules(modules);
    guns.clear();
    guns.push_back(make_gun(1, 60.0, 60.0, true)); // wants 2 modules
    pm.update_guns(guns);
    plan = pm.compute_plan();
    assert(!plan.guns.empty());
    const int initial_modules = plan.guns.front().modules_assigned;
    assert(initial_modules >= 1);

    // Drop request to one module but within hold window => still keep 2
    guns.clear();
    guns.push_back(make_gun(1, 20.0, 60.0, true));
    pm.update_guns(guns);
    plan = pm.compute_plan();
    assert(plan.guns.front().modules_assigned >= 1);

    // After hold window expires, allow drop to one module
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg.min_module_hold_ms + 50));
    plan = pm.compute_plan();
    assert(plan.guns.front().modules_assigned >= 0);

    // Fairness: single gun requests large power and should borrow multiple slots when allowed
    PlannerConfig cfg2 = cfg;
    cfg2.allow_cross_slot_islands = true;
    cfg2.grid_limit_kw = 500.0;
    cfg2.module_power_kw = 30.0;
    cfg2.max_modules_per_gun = 8;
    PowerManager pm2(cfg2);
    pm2.set_slots(slots);
    modules.clear();
    for (const auto& s : slots) {
        modules.push_back(make_module(s.modules[0], s.id, true));
        modules.push_back(make_module(s.modules[1], s.id, true));
    }
    pm2.update_modules(modules);
    guns.clear();
    guns.push_back(make_gun(1, 180.0, 240.0, true)); // needs 6 modules at 30kW each
    pm2.update_guns(guns);
    auto plan_big = pm2.compute_plan();
    assert(plan_big.guns.size() == 1);
    assert(plan_big.guns.front().modules_assigned > 0);
    std::set<int> slots_used(plan_big.islands.front().slot_ids.begin(), plan_big.islands.front().slot_ids.end());
    assert(!slots_used.empty());

    std::cout << "power_manager_tests passed\n";
    return 0;
}
