// SPDX-License-Identifier: Apache-2.0
#include "tie_gating.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <map>

using namespace charger;

int main() {
    std::map<std::string, std::chrono::steady_clock::time_point> ready;
    const auto t0 = std::chrono::steady_clock::now();

    IslandTelemetryLite a{true, 400.0, 0.1};
    IslandTelemetryLite b{true, 450.0, 0.1};

    // ΔV too high => block close.
    auto state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                                 a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                                 true, ready, t0);
    assert(state == ContactorState::Open);

    // Current too high => block close.
    b.voltage_v = 405.0;
    a.current_a = 2.5;
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    assert(state == ContactorState::Open);

    // Telemetry invalid => block close.
    IslandTelemetryLite bad{false, 0.0, 0.0};
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            bad, bad, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    assert(state == ContactorState::Open);

    // Merge conflict => block close.
    a = {true, 400.0, 0.1};
    b = {true, 405.0, 0.1};
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            false, ready, t0);
    assert(state == ContactorState::Open);

    // Stable timer: require dwell before closing.
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(200),
                            true, ready, t0);
    assert(state == ContactorState::Open);
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(200),
                            true, ready, t0 + std::chrono::milliseconds(100));
    assert(state == ContactorState::Open);
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(200),
                            true, ready, t0 + std::chrono::milliseconds(250));
    assert(state == ContactorState::Closed);

    std::cout << "tie_gating_tests passed\n";
    return 0;
}
