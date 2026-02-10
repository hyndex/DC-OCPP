// SPDX-License-Identifier: Apache-2.0
#include "tie_gating.hpp"

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
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected close blocked by dv\n";
        return 1;
    }

    // Closing tie: allow when one side is quiet and dv is within tolerance.
    b.voltage_v = 405.0;
    a.current_a = 2.5;
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    if (state != ContactorState::Closed) {
        std::cerr << "tie_gating_tests failed: expected close allowed when one side quiet\n";
        return 1;
    }

    // Closing tie: block when both sides are above the threshold.
    b.current_a = 2.0;
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected close blocked when both sides high current\n";
        return 1;
    }

    // Telemetry invalid => block close.
    IslandTelemetryLite bad{false, 0.0, 0.0};
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            bad, bad, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected close blocked by invalid telemetry\n";
        return 1;
    }

    // Merge conflict => block close.
    a = {true, 400.0, 0.1};
    b = {true, 405.0, 0.1};
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            false, ready, t0);
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected close blocked by merge conflict\n";
        return 1;
    }

    // Stable timer: require dwell before closing.
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(200),
                            true, ready, t0);
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected close blocked during stable dwell (t0)\n";
        return 1;
    }
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(200),
                            true, ready, t0 + std::chrono::milliseconds(100));
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected close blocked during stable dwell (t0+100ms)\n";
        return 1;
    }
    state = gate_tie_switch("MC_1", ContactorState::Closed, ContactorState::Open,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(200),
                            true, ready, t0 + std::chrono::milliseconds(250));
    if (state != ContactorState::Closed) {
        std::cerr << "tie_gating_tests failed: expected close allowed after stable dwell\n";
        return 1;
    }

    // Opening tie remains conservative: require both sides to be quiet.
    a = {true, 400.0, 2.5};
    b = {true, 405.0, 0.1};
    state = gate_tie_switch("MC_1", ContactorState::Open, ContactorState::Closed,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    if (state != ContactorState::Closed) {
        std::cerr << "tie_gating_tests failed: expected open blocked when one side high current\n";
        return 1;
    }
    a.current_a = 0.1;
    state = gate_tie_switch("MC_1", ContactorState::Open, ContactorState::Closed,
                            a, b, 1.0, 20.0, std::chrono::milliseconds(0),
                            true, ready, t0);
    if (state != ContactorState::Open) {
        std::cerr << "tie_gating_tests failed: expected open allowed when both sides quiet\n";
        return 1;
    }

    std::cout << "tie_gating_tests passed\n";
    return 0;
}
