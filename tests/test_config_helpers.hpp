// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"

#include <cstddef>
#include <string>

namespace charger {

inline void populate_minimal_slots(ChargerConfig& cfg) {
    if (cfg.connectors.empty()) {
        cfg.connectors.push_back(ConnectorConfig{.id = 1});
    }
    cfg.allow_cross_slot_islands = true;
    cfg.slots.clear();
    const std::size_t count = cfg.connectors.size();
    for (std::size_t i = 0; i < count; ++i) {
        const auto& conn = cfg.connectors[i];
        SlotMapping sm;
        sm.id = conn.id;
        sm.gun_id = conn.id;
        sm.gc_id = "GC_" + std::to_string(conn.id);
        sm.mc_id = "MC_" + std::to_string(conn.id);
        sm.cw_id = cfg.connectors[(i + 1) % count].id;
        sm.ccw_id = cfg.connectors[(i + count - 1) % count].id;
        ModuleConfig m0;
        m0.id = "M" + std::to_string(conn.id) + "_0";
        m0.mn_id = "MN_" + std::to_string(conn.id) + "_0";
        m0.type = "maxwell-mxr";
        ModuleConfig m1;
        m1.id = "M" + std::to_string(conn.id) + "_1";
        m1.mn_id = "MN_" + std::to_string(conn.id) + "_1";
        m1.type = "maxwell-mxr";
        sm.modules = {m0, m1};
        cfg.slots.push_back(sm);
    }
}

} // namespace charger
