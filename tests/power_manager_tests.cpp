// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
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
    assert(initial_modules == 2);

    // Drop request to one module but within hold window => still keep 2
    guns.clear();
    guns.push_back(make_gun(1, 20.0, 60.0, true));
    pm.update_guns(guns);
    plan = pm.compute_plan();
    assert(plan.guns.front().modules_assigned == initial_modules);

    // After hold window expires, allow drop to one module
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg.min_module_hold_ms + 50));
    plan = pm.compute_plan();
    assert(plan.guns.front().modules_assigned == 1);

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

    // Module-level ring topology: 2 guns, 4 module slots (gun taps on slot 1 and 3).
    PlannerConfig cfg3 = cfg;
    cfg3.allow_cross_slot_islands = true;
    cfg3.grid_limit_kw = 500.0;
    cfg3.module_power_kw = 30.0;
    cfg3.max_modules_per_gun = 4;
    PowerManager pm3(cfg3);
    std::vector<Slot> module_slots;
    auto make_module_slot = [](int id, int cw, int ccw, int gun_id) {
        Slot s;
        s.id = id;
        s.gun_id = gun_id;
        s.mc_id = "MC_" + std::to_string(id);
        s.gc_id = gun_id > 0 ? "GC_" + std::to_string(gun_id) : "";
        s.cw_id = cw;
        s.ccw_id = ccw;
        s.modules = {"M" + std::to_string(id)};
        return s;
    };
    module_slots.push_back(make_module_slot(1, 2, 4, 1));
    module_slots.push_back(make_module_slot(2, 3, 1, 0));
    module_slots.push_back(make_module_slot(3, 4, 2, 2));
    module_slots.push_back(make_module_slot(4, 1, 3, 0));
    pm3.set_slots(module_slots);
    std::vector<ModuleState> module_only;
    for (const auto& s : module_slots) {
        module_only.push_back(make_module(s.modules[0], s.id, true));
    }
    pm3.update_modules(module_only);
    std::vector<GunState> guns3;
    auto g1 = make_gun(1, 120.0, 240.0, true);
    g1.slot_id = 1;
    guns3.push_back(g1);
    pm3.update_guns(guns3);
    auto plan_single = pm3.compute_plan();
    assert(plan_single.islands.size() == 1);
    assert(plan_single.islands.front().slot_ids.size() == module_slots.size());
    assert(plan_single.guns.front().modules_assigned == static_cast<int>(module_slots.size()));
    int ring_break_open = 0;
    std::string opened_mc;
    for (const auto& kv : plan_single.mc_commands) {
        if (kv.second == ContactorState::Open) {
            ring_break_open++;
            opened_mc = kv.first;
        }
    }
    assert(ring_break_open == 1);
    // Do not place the forced ring break on the active gun home segment.
    assert(opened_mc != "MC_1");
    assert(opened_mc != "MC_2");

    // Single gun with low request should only claim the minimum modules needed.
    PowerManager pm3_low(cfg3);
    pm3_low.set_slots(module_slots);
    pm3_low.update_modules(module_only);
    guns3.clear();
    g1 = make_gun(1, 10.0, 60.0, true);
    g1.slot_id = 1;
    guns3.push_back(g1);
    pm3_low.update_guns(guns3);
    auto plan_single_low = pm3_low.compute_plan();
    assert(plan_single_low.islands.size() == 1);
    assert(plan_single_low.islands.front().slot_ids.size() == 1);
    assert(plan_single_low.guns.front().modules_assigned == 1);

    // Anti-clockwise preference: from slot 1 in a 1-2-3-4 ring, the first borrowed slot should be 4.
    PowerManager pm3_pref(cfg3);
    pm3_pref.set_slots(module_slots);
    pm3_pref.update_modules(module_only);
    guns3.clear();
    g1 = make_gun(1, 31.0, 240.0, true); // needs 2 modules => 1 extra slot
    g1.slot_id = 1;
    guns3.push_back(g1);
    pm3_pref.update_guns(guns3);
    auto plan_pref = pm3_pref.compute_plan();
    assert(plan_pref.islands.size() == 1);
    const auto& pref_slots = plan_pref.islands.front().slot_ids;
    assert(pref_slots.size() == 2);
    assert(std::find(pref_slots.begin(), pref_slots.end(), 4) != pref_slots.end());
    assert(std::find(pref_slots.begin(), pref_slots.end(), 2) == pref_slots.end());

    // Ceil module allocation: request slightly above 2 modules should allocate 3.
    PowerManager pm3_ceil(cfg3);
    pm3_ceil.set_slots(module_slots);
    pm3_ceil.update_modules(module_only);
    guns3.clear();
    g1 = make_gun(1, 61.0, 240.0, true);
    g1.slot_id = 1;
    guns3.push_back(g1);
    pm3_ceil.update_guns(guns3);
    auto plan_ceil = pm3_ceil.compute_plan();
    assert(plan_ceil.guns.front().modules_assigned == 3);

    // Reserved/plugged slots must not be pulled into a single-gun island.
    PowerManager pm3_reserved(cfg3);
    pm3_reserved.set_slots(module_slots);
    pm3_reserved.update_modules(module_only);
    guns3.clear();
    g1 = make_gun(1, 120.0, 240.0, true);
    g1.slot_id = 1;
    auto g2_reserved = make_gun(2, 0.0, 240.0, false);
    g2_reserved.slot_id = 3;
    g2_reserved.plugged_in = true;
    g2_reserved.fsm_state = GunFsmState::EvDetected;
    guns3.push_back(g1);
    guns3.push_back(g2_reserved);
    pm3_reserved.update_guns(guns3);
    auto plan_reserved = pm3_reserved.compute_plan();
    assert(plan_reserved.islands.size() == 1);
    const auto& reserved_island = plan_reserved.islands.front();
    assert(std::find(reserved_island.slot_ids.begin(), reserved_island.slot_ids.end(), 3) ==
           reserved_island.slot_ids.end());
    assert(plan_reserved.guns.front().modules_assigned == 3);

    // Priority: lower numeric priority keeps extra modules first when constrained.
    PlannerConfig cfg_priority = cfg3;
    cfg_priority.max_modules_per_gun = 2;
    cfg_priority.grid_limit_kw = 90.0;
    PowerManager pm_priority(cfg_priority);
    std::vector<Slot> prio_slots;
    prio_slots.push_back(make_module_slot(1, 2, 3, 1));
    prio_slots.push_back(make_module_slot(2, 3, 1, 2));
    prio_slots.push_back(make_module_slot(3, 1, 2, 0));
    pm_priority.set_slots(prio_slots);
    std::vector<ModuleState> prio_mods;
    for (const auto& s : prio_slots) {
        prio_mods.push_back(make_module(s.modules[0], s.id, true));
    }
    pm_priority.update_modules(prio_mods);
    std::vector<GunState> prio_guns;
    auto pg1 = make_gun(1, 60.0, 120.0, true);
    pg1.slot_id = 1;
    pg1.priority = 0;
    auto pg2 = make_gun(2, 60.0, 120.0, true);
    pg2.slot_id = 2;
    pg2.priority = 1;
    prio_guns.push_back(pg1);
    prio_guns.push_back(pg2);
    pm_priority.update_guns(prio_guns);
    auto plan_prio = pm_priority.compute_plan();
    int modules_g1 = 0;
    int modules_g2 = 0;
    for (const auto& d : plan_prio.guns) {
        if (d.gun_id == 1) modules_g1 = d.modules_assigned;
        if (d.gun_id == 2) modules_g2 = d.modules_assigned;
    }
    assert(modules_g1 == 2);
    assert(modules_g2 == 1);

    guns3.clear();
    g1 = make_gun(1, 60.0, 120.0, true);
    g1.slot_id = 1;
    auto g2 = make_gun(2, 60.0, 120.0, true);
    g2.slot_id = 3;
    guns3.push_back(g1);
    guns3.push_back(g2);
    pm3.update_guns(guns3);
    auto plan_two = pm3.compute_plan();
    assert(plan_two.islands.size() == 2);
    std::set<int> island_guns;
    for (const auto& isl : plan_two.islands) {
        if (isl.gun_id) {
            island_guns.insert(isl.gun_id.value());
        }
    }
    assert(island_guns.size() == 2);

    // Split modules from the same PLC across guns (gun2 borrows slot 2).
    guns3.clear();
    g1 = make_gun(1, 30.0, 60.0, true);
    g1.slot_id = 1;
    g2 = make_gun(2, 90.0, 120.0, true);
    g2.slot_id = 3;
    guns3.push_back(g1);
    guns3.push_back(g2);
    pm3.update_guns(guns3);
    auto plan_split = pm3.compute_plan();
    bool gun2_has_slot2 = false;
    for (const auto& isl : plan_split.islands) {
        if (isl.gun_id && isl.gun_id.value() == 2) {
            gun2_has_slot2 = std::find(isl.slot_ids.begin(), isl.slot_ids.end(), 2) != isl.slot_ids.end();
            break;
        }
    }
    assert(gun2_has_slot2);

    // Pass-through empty slot: gun1 can traverse an empty segment to reach modules.
    PlannerConfig cfg4 = cfg;
    cfg4.allow_cross_slot_islands = true;
    cfg4.grid_limit_kw = 200.0;
    cfg4.module_power_kw = 30.0;
    cfg4.max_modules_per_gun = 4;
    PowerManager pm4(cfg4);
    std::vector<Slot> pass_slots;
    Slot s1;
    s1.id = 1;
    s1.gun_id = 1;
    s1.mc_id = "MC_1";
    s1.gc_id = "GC_1";
    s1.cw_id = 2;
    s1.ccw_id = 3;
    s1.modules = {"M1"};
    pass_slots.push_back(s1);
    Slot s2;
    s2.id = 2;
    s2.gun_id = 0;
    s2.mc_id = "MC_2";
    s2.gc_id = "";
    s2.cw_id = 3;
    s2.ccw_id = 1;
    s2.modules = {};
    pass_slots.push_back(s2);
    Slot s3;
    s3.id = 3;
    s3.gun_id = 0;
    s3.mc_id = "MC_3";
    s3.gc_id = "";
    s3.cw_id = 1;
    s3.ccw_id = 2;
    s3.modules = {"M3"};
    pass_slots.push_back(s3);
    pm4.set_slots(pass_slots);
    std::vector<ModuleState> pass_mods;
    pass_mods.push_back(make_module("M1", 1, true));
    pass_mods.push_back(make_module("M3", 3, true));
    pm4.update_modules(pass_mods);
    std::vector<GunState> guns4;
    auto g4 = make_gun(1, 60.0, 120.0, true);
    g4.slot_id = 1;
    guns4.push_back(g4);
    pm4.update_guns(guns4);
    auto plan_pass = pm4.compute_plan();
    assert(plan_pass.islands.size() == 1);
    std::set<int> pass_island(plan_pass.islands.front().slot_ids.begin(),
                              plan_pass.islands.front().slot_ids.end());
    assert(pass_island.count(1) && pass_island.count(2) && pass_island.count(3));

    // Local-only request should keep ownership on the home slot and keep the home MC open.
    PowerManager pm4_local(cfg4);
    pm4_local.set_slots(pass_slots);
    pm4_local.update_modules(pass_mods);
    std::vector<GunState> guns4_local;
    auto g4_local = make_gun(1, 10.0, 120.0, true); // one module is sufficient
    g4_local.slot_id = 1;
    guns4_local.push_back(g4_local);
    pm4_local.update_guns(guns4_local);
    auto plan_pass_local = pm4_local.compute_plan();
    assert(plan_pass_local.islands.size() == 1);
    const auto& local_slots = plan_pass_local.islands.front().slot_ids;
    assert(local_slots.size() == 1);
    assert(local_slots.front() == 1);
    const auto mc_home_it = plan_pass_local.mc_commands.find("MC_1");
    assert(mc_home_it != plan_pass_local.mc_commands.end());
    assert(mc_home_it->second == ContactorState::Open);

    std::cout << "power_manager_tests passed\n";
    return 0;
}
