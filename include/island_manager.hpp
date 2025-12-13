// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "power_manager.hpp"

#include <map>
#include <set>
#include <vector>

namespace charger {

struct IslandInfo {
    std::vector<int> slots;           // contiguous slots in this island
    std::vector<std::string> open_mcs; // MC ids that bound this island
};

/// \brief Pure software helper to reason about ring segments and islands given MC open/closed states.
class IslandManager {
public:
    IslandManager() = default;
    explicit IslandManager(std::vector<Slot> slots);

    void set_slots(std::vector<Slot> slots);
    void set_mc_states(const std::map<std::string, ContactorState>& mc_states);

    /// \brief Compute island groupings of slots based on MC open boundaries.
    std::vector<IslandInfo> compute_islands() const;

private:
    std::vector<Slot> slots_;
    std::map<std::string, ContactorState> mc_states_;
    std::map<int, Slot> slot_lookup_;
};

} // namespace charger
