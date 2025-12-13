// SPDX-License-Identifier: Apache-2.0
#include "island_manager.hpp"

#include <queue>

namespace charger {

IslandManager::IslandManager(std::vector<Slot> slots) {
    set_slots(std::move(slots));
}

void IslandManager::set_slots(std::vector<Slot> slots) {
    slots_ = std::move(slots);
    slot_lookup_.clear();
    for (const auto& s : slots_) {
        slot_lookup_[s.id] = s;
    }
}

void IslandManager::set_mc_states(const std::map<std::string, ContactorState>& mc_states) {
    mc_states_ = mc_states;
}

std::vector<IslandInfo> IslandManager::compute_islands() const {
    std::vector<IslandInfo> islands;
    if (slots_.empty()) return islands;

    // Build adjacency based on closed MC boundaries. MC open breaks the edge from slot -> cw neighbor.
    std::map<int, std::set<int>> adj;
    for (const auto& s : slots_) {
        bool mc_closed = true;
        auto it = mc_states_.find(s.mc_id);
        if (it != mc_states_.end()) {
            mc_closed = (it->second == ContactorState::Closed);
        }
        if (mc_closed) {
            adj[s.id].insert(s.cw_id);
            adj[s.cw_id].insert(s.id);
        }
    }

    std::set<int> visited;
    for (const auto& s : slots_) {
        if (visited.count(s.id)) continue;
        IslandInfo isl;
        std::queue<int> q;
        q.push(s.id);
        visited.insert(s.id);
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            isl.slots.push_back(cur);
            for (int nb : adj[cur]) {
                if (!visited.count(nb)) {
                    visited.insert(nb);
                    q.push(nb);
                }
            }
        }
        // Collect open MCs that bound this island.
        for (int sid : isl.slots) {
            const auto it_slot = slot_lookup_.find(sid);
            if (it_slot == slot_lookup_.end()) continue;
            const auto& slot = it_slot->second;
            auto it_state = mc_states_.find(slot.mc_id);
            bool mc_closed = !(it_state != mc_states_.end() && it_state->second == ContactorState::Open);
            if (!mc_closed) {
                isl.open_mcs.push_back(slot.mc_id);
            }
        }
        islands.push_back(isl);
    }
    return islands;
}

} // namespace charger
