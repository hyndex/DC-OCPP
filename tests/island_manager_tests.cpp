// SPDX-License-Identifier: Apache-2.0
#include "island_manager.hpp"

#include <cassert>
#include <iostream>

using namespace charger;

static Slot make_slot(int id, int cw) {
    Slot s;
    s.id = id;
    s.gun_id = id;
    s.mc_id = "MC_" + std::to_string(id);
    s.gc_id = "GC_" + std::to_string(id);
    s.cw_id = cw;
    s.ccw_id = 0; // unused in island manager
    return s;
}

int main() {
    std::vector<Slot> slots;
    slots.push_back(make_slot(1, 2));
    slots.push_back(make_slot(2, 3));
    slots.push_back(make_slot(3, 1));

    IslandManager mgr(slots);
    std::map<std::string, ContactorState> mc_states;
    mc_states["MC_1"] = ContactorState::Closed;
    mc_states["MC_2"] = ContactorState::Closed;
    mc_states["MC_3"] = ContactorState::Closed;
    mgr.set_mc_states(mc_states);
    auto islands = mgr.compute_islands();
    assert(islands.size() == 1);
    assert(islands.front().slots.size() == 3);

    // Opening two adjacent MCs should split the ring: isolate slot 3
    mc_states["MC_2"] = ContactorState::Open;
    mc_states["MC_1"] = ContactorState::Open;
    mgr.set_mc_states(mc_states);
    islands = mgr.compute_islands();
    assert(!islands.empty());

    std::cout << "island_manager_tests passed\n";
    return 0;
}
