// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <cassert>
#include <iostream>
#include <set>
#include <vector>

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
    cfg.grid_limit_kw = 120.0;
    cfg.ramp_step_a = 0.0;
    cfg.default_voltage_v = 800.0;
    cfg.allow_cross_slot_islands = true;
    cfg.max_modules_per_gun = 2;
    cfg.min_modules_per_active_gun = 1;

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
    guns.push_back(make_gun(3, 0.0, 60.0, false));
    pm.update_guns(guns);

    const auto plan = pm.compute_plan();
    std::set<std::string> modules_seen;
    std::set<int> slots_seen;
    for (const auto& island : plan.islands) {
        for (const auto& mid : island.module_ids) {
            assert(modules_seen.insert(mid).second && "duplicate module assignment detected");
        }
        for (int sid : island.slot_ids) {
            assert(slots_seen.insert(sid).second && "duplicate slot assignment detected");
        }
    }
    for (const auto& dispatch : plan.guns) {
        auto it = std::find_if(plan.islands.begin(), plan.islands.end(), [&](const IslandState& isl) {
            return isl.gun_id && isl.gun_id.value() == dispatch.gun_id;
        });
        if (it != plan.islands.end()) {
            assert(static_cast<int>(it->module_ids.size()) == dispatch.modules_assigned);
        }
    }

    std::cout << "power_manager_invariants_tests passed\n";
    return 0;
}
