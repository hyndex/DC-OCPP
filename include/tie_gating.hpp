// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <chrono>
#include <cmath>
#include <map>
#include <string>

#include "power_manager.hpp"

namespace charger {

struct IslandTelemetryLite {
    bool complete{false};
    double voltage_v{0.0};
    double current_a{0.0};
};

inline ContactorState gate_tie_switch(const std::string& id,
                                      ContactorState desired,
                                      ContactorState prev,
                                      const IslandTelemetryLite& side_a,
                                      const IslandTelemetryLite& side_b,
                                      double switch_i_thresh,
                                      double max_dv_v,
                                      std::chrono::milliseconds stable_ms,
                                      bool merge_ok,
                                      std::map<std::string, std::chrono::steady_clock::time_point>& ready_since,
                                      std::chrono::steady_clock::time_point now) {
    if (desired == prev) {
        ready_since.erase(id);
        return desired;
    }

    const bool have_telem = side_a.complete && side_b.complete;
    bool current_ok = false;
    if (have_telem) {
        const bool ia_ok = std::fabs(side_a.current_a) < switch_i_thresh;
        const bool ib_ok = std::fabs(side_b.current_a) < switch_i_thresh;
        if (desired == ContactorState::Closed && prev == ContactorState::Open) {
            // Closing a tie into an already-loaded island can be safe as long as the voltages match
            // and at least one side is "quiet" (e.g., the island being merged has no load yet).
            // This allows module boosting without forcing the active gun to ramp to ~0A first.
            current_ok = ia_ok || ib_ok;
        } else {
            // Opening (or other transitions) remains conservative: require both sides to be quiet.
            current_ok = ia_ok && ib_ok;
        }
    }
    bool dv_ok = true;
    if (desired == ContactorState::Closed && prev == ContactorState::Open) {
        dv_ok = have_telem && (std::fabs(side_a.voltage_v - side_b.voltage_v) <= max_dv_v);
    }
    const bool safe = current_ok && dv_ok && merge_ok;

    if (!safe) {
        ready_since.erase(id);
        return prev;
    }

    if (stable_ms.count() == 0) {
        ready_since.erase(id);
        return desired;
    }

    auto& since = ready_since[id];
    if (since.time_since_epoch().count() == 0) {
        since = now;
    }
    if ((now - since) >= stable_ms) {
        ready_since.erase(id);
        return desired;
    }
    return prev;
}

} // namespace charger
