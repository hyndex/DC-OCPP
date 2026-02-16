// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "e2e-long-cycle-tests";
    cfg.module_power_kw = 30.0;
    cfg.grid_limit_kw = 60.0;
    cfg.default_voltage_v = 800.0;
    cfg.max_modules_per_gun = 1;
    cfg.min_modules_per_active_gun = 1;
    cfg.precharge_max_current_a = 2.0;
    cfg.precharge_voltage_tolerance_v = 20.0;
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.switch_stable_time_ms = 0;
    cfg.min_module_hold_ms = 0;
    cfg.min_mc_hold_ms = 0;
    cfg.min_gc_hold_ms = 0;
    cfg.use_plc = true;
    cfg.plc_owns_gun_relay = false;
    cfg.plc_module_relays_enabled = true;
    cfg.plc_relay_feedback = false;
    cfg.plc_relay3_enabled = false; // Keep relay-3 out of this topology.
    cfg.plc_relay_mode = PlcRelayMode::Ties;
    cfg.database_dir.clear();

    cfg.connectors = {
        ConnectorConfig{
            .id = 1,
            .max_current_a = 120.0,
            .max_power_w = 60000.0,
            .max_voltage_v = 1000.0,
            .min_voltage_v = 200.0,
            .require_lock = false,
            .meter_source = "module",
        },
    };
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static void seed_session(OcppAdapter& adapter, int connector) {
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = "e2e";
    session.id_token = "TAG";
    session.authorized = true;
    session.ev_connected = true;
    session.transaction_started = true;
    session.connected_at = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
    OcppAdapter::TestHook::sessions(adapter)[connector] = session;
}

static int popcount_u8(uint8_t v) {
    int n = 0;
    while (v != 0u) {
        n += static_cast<int>(v & 0x1u);
        v = static_cast<uint8_t>(v >> 1u);
    }
    return n;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);
    seed_session(adapter, 1);

    constexpr int kConnector = 1;
    constexpr int kSlot = 1;
    constexpr double kTargetV = 384.0;
    constexpr double kTargetI = 45.0;
    constexpr int kDemandLoops = 2000; // strict long-run cycle coverage
    constexpr double kDipToleranceA = 5.0;
    constexpr int kInjectedDipStart = 900;
    constexpr int kInjectedDipLen = 8;
    constexpr double kInjectedDipCurrentA = 6.0;

    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.hlc_stage = 8;
    st.hlc_precharge_active = true;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = false;
    st.authorization_granted = true;
    st.lock_engaged = true;
    st.relay_closed = false;
    st.target_voltage_v = kTargetV;
    st.target_current_a = kTargetI;
    st.present_voltage_v = 0.0;
    st.present_current_a = 0.0;
    st.module_healthy_mask = 0x01;
    st.module_fault_mask = 0x00;

    // Phase 1: precharge sequencing must keep one module path and <= precharge current.
    bool saw_precharge = false;
    for (int i = 0; i < 12; ++i) {
        st.last_telemetry = std::chrono::steady_clock::now();
        st.last_target_update = std::chrono::steady_clock::now();
        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        const auto cmd = hw->last_power_command(kConnector);
        if (!cmd.has_value()) {
            std::cerr << "e2e_long_cycle_tests failed: missing precharge command\n";
            return 1;
        }
        if (!cmd->gc_closed || !cmd->mc_closed) {
            std::cerr << "e2e_long_cycle_tests failed: contactors not closed during precharge\n";
            return 1;
        }
        if (cmd->module_count != 1) {
            std::cerr << "e2e_long_cycle_tests failed: expected single module in precharge, got "
                      << cmd->module_count << "\n";
            return 1;
        }
        if (cmd->module_mask == 0u || popcount_u8(cmd->module_mask) != 1) {
            std::cerr << "e2e_long_cycle_tests failed: expected single relay path in precharge, mask=0x"
                      << std::hex << static_cast<int>(cmd->module_mask) << std::dec << "\n";
            return 1;
        }
        if (cmd->current_limit_a > cfg.precharge_max_current_a + 0.3) {
            std::cerr << "e2e_long_cycle_tests failed: precharge current limit too high: "
                      << cmd->current_limit_a << "A\n";
            return 1;
        }
        saw_precharge = true;

        // Follow commanded precharge voltage in the simulated EV bus.
        st.relay_closed = cmd->gc_closed;
        st.present_voltage_v = std::max(0.0, std::min(kTargetV, cmd->voltage_set_v));
        st.present_current_a = 0.0;
    }
    if (!saw_precharge) {
        std::cerr << "e2e_long_cycle_tests failed: precharge phase never observed\n";
        return 1;
    }

    // Phase 2: power phase with constant EV demand for 1500 loops.
    st.hlc_stage = 9;
    st.hlc_precharge_active = false;
    st.hlc_power_ready = true;
    st.relay_closed = true;
    st.present_voltage_v = kTargetV;
    st.present_current_a = 0.0;

    double baseline_cmd_a = -1.0;
    double min_cmd_a = 1e9;
    double max_cmd_a = 0.0;
    bool saw_injected_delivery_dip = false;
    bool saw_vehicle_noise = false;
    bool saw_plc_noise = false;
    bool saw_module_noise = false;
    bool saw_false_positive_data = false;
    for (int loop = 0; loop < kDemandLoops; ++loop) {
        const auto now = std::chrono::steady_clock::now();
        st.last_telemetry = now;
        st.last_target_update = now;
        st.target_voltage_v = kTargetV;
        st.target_current_a = kTargetI;
        st.cp_state = 'C';
        st.meter_stale = false;

        // Vehicle-side noise: short request jitter/outliers around a constant demand envelope.
        if (loop >= 260 && loop < 360) {
            st.target_current_a = kTargetI + ((loop % 2 == 0) ? 2.0 : -2.0);
            saw_vehicle_noise = true;
        }
        if (loop == 520 || loop == 780) {
            st.target_voltage_v = kTargetV + 8.0;
            st.target_current_a = kTargetI + 1.0;
            saw_vehicle_noise = true;
        }

        // PLC-side noise: short CP unknown blips and one stale-target metadata blip.
        if (loop >= 650 && loop < 658 && (loop % 2 == 0)) {
            st.cp_state = 'U';
            saw_plc_noise = true;
        }
        if (loop == 1333) {
            st.last_target_update = now - std::chrono::milliseconds(700);
            saw_plc_noise = true;
        }

        // False-positive data noise: temporary meter stale flag while power is active.
        if (loop >= 1200 && loop < 1210) {
            st.meter_stale = true;
            saw_false_positive_data = true;
        }

        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        const auto cmd = hw->last_power_command(kConnector);
        const auto mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter, kSlot);
        if (!cmd.has_value() || !mreq.has_value()) {
            std::cerr << "e2e_long_cycle_tests failed: missing power/module command at loop " << loop << "\n";
            return 1;
        }
        if (!cmd->gc_closed || !cmd->mc_closed) {
            std::cerr << "e2e_long_cycle_tests failed: contactor dropped during steady demand at loop " << loop << "\n";
            return 1;
        }
        if (cmd->module_mask == 0u || popcount_u8(cmd->module_mask) != 1) {
            std::cerr << "e2e_long_cycle_tests failed: invalid module mask during steady demand at loop " << loop
                      << " mask=0x" << std::hex << static_cast<int>(cmd->module_mask) << std::dec << "\n";
            return 1;
        }
        if (cmd->current_limit_a < 1.0) {
            std::cerr << "e2e_long_cycle_tests failed: unexpected near-zero current offer at loop " << loop
                      << " cmd=" << cmd->current_limit_a << "A\n";
            return 1;
        }

        min_cmd_a = std::min(min_cmd_a, cmd->current_limit_a);
        max_cmd_a = std::max(max_cmd_a, cmd->current_limit_a);

        if (loop == 50) {
            baseline_cmd_a = cmd->current_limit_a;
        }
        if (loop > 80 && baseline_cmd_a > 0.0) {
            if (cmd->current_limit_a + 1e-6 < baseline_cmd_a - kDipToleranceA) {
                std::cerr << "e2e_long_cycle_tests failed: command dipped without EV demand change at loop " << loop
                          << " baseline=" << baseline_cmd_a << "A cmd=" << cmd->current_limit_a << "A\n";
                return 1;
            }
            if (mreq->enable && (mreq->current_a + 1e-6 < baseline_cmd_a - kDipToleranceA)) {
                std::cerr << "e2e_long_cycle_tests failed: module command dipped without EV demand change at loop " << loop
                          << " baseline=" << baseline_cmd_a << "A module_cmd=" << mreq->current_a << "A\n";
                return 1;
            }
        }

        if (loop % 200 == 0) {
            std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
            if (OcppAdapter::TestHook::sessions(adapter).find(kConnector) ==
                OcppAdapter::TestHook::sessions(adapter).end()) {
                std::cerr << "e2e_long_cycle_tests failed: session dropped during steady demand at loop " << loop << "\n";
                return 1;
            }
        }

        // Simulate delivered current following command with bounded noise.
        double delivered_i = std::max(0.0, cmd->current_limit_a - ((loop % 17 == 0) ? 0.6 : 0.2));

        // Module-side noise: explicit short delivery dip and voltage sag window.
        if (loop >= kInjectedDipStart && loop < (kInjectedDipStart + kInjectedDipLen)) {
            delivered_i = kInjectedDipCurrentA;
            saw_injected_delivery_dip = true;
            saw_module_noise = true;
        }
        if (loop >= 1000 && loop < 1012) {
            delivered_i = std::max(0.0, delivered_i - 12.0);
            st.present_voltage_v = kTargetV - 14.0;
            saw_module_noise = true;
        } else {
            st.present_voltage_v = kTargetV;
        }

        // Corrupted one-shot sample: negative current sign glitch.
        if (loop == 1111) {
            st.present_current_a = -2.0;
            saw_false_positive_data = true;
        } else {
            st.present_current_a = delivered_i;
        }
    }

    if (baseline_cmd_a <= 0.0) {
        std::cerr << "e2e_long_cycle_tests failed: failed to establish power-phase baseline\n";
        return 1;
    }
    if (!saw_injected_delivery_dip) {
        std::cerr << "e2e_long_cycle_tests failed: injected delivery dip path did not execute\n";
        return 1;
    }
    if (!saw_vehicle_noise || !saw_plc_noise || !saw_module_noise || !saw_false_positive_data) {
        std::cerr << "e2e_long_cycle_tests failed: full noise matrix did not execute"
                  << " vehicle=" << (saw_vehicle_noise ? "1" : "0")
                  << " plc=" << (saw_plc_noise ? "1" : "0")
                  << " module=" << (saw_module_noise ? "1" : "0")
                  << " false_positive=" << (saw_false_positive_data ? "1" : "0")
                  << "\n";
        return 1;
    }

    // Phase 2a: low-current tail (real EV taper near end of charge) must not be force-aborted.
    // Design-guide alignment: EVSE should continue processing low CurrentDemand and avoid premature stop.
    for (int i = 0; i < 120; ++i) {
        st.last_telemetry = std::chrono::steady_clock::now();
        st.last_target_update = std::chrono::steady_clock::now();
        st.target_voltage_v = kTargetV;
        st.target_current_a = 1.5;
        st.cp_state = 'C';
        st.meter_stale = false;
        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        const auto cmd = hw->last_power_command(kConnector);
        const auto mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter, kSlot);
        if (!cmd.has_value() || !mreq.has_value()) {
            std::cerr << "e2e_long_cycle_tests failed: missing command in low-current tail at step " << i << "\n";
            return 1;
        }
        if (!cmd->gc_closed || !cmd->mc_closed) {
            std::cerr << "e2e_long_cycle_tests failed: contactor dropped in low-current tail at step " << i << "\n";
            return 1;
        }
        if (cmd->current_limit_a < 0.8) {
            std::cerr << "e2e_long_cycle_tests failed: low-current tail collapsed below 0.8A at step " << i
                      << " cmd=" << cmd->current_limit_a << "A\n";
            return 1;
        }
        if (mreq->enable && mreq->current_a < 0.8) {
            std::cerr << "e2e_long_cycle_tests failed: module command too low in low-current tail at step " << i
                      << " module_cmd=" << mreq->current_a << "A\n";
            return 1;
        }
        if (i % 30 == 0) {
            std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
            if (OcppAdapter::TestHook::sessions(adapter).find(kConnector) ==
                OcppAdapter::TestHook::sessions(adapter).end()) {
                std::cerr << "e2e_long_cycle_tests failed: session dropped during low-current tail at step " << i
                          << "\n";
                return 1;
            }
        }

        st.present_current_a = std::max(0.0, cmd->current_limit_a - 0.1);
        st.present_voltage_v = kTargetV;
    }

    // Phase 2b: short missing-telemetry window (< stale debounce) must not collapse charging.
    constexpr int kTelemetryBlipLoops = 22; // ~2.2s at 100ms cadence; below 3s stale debounce.
    for (int i = 0; i < kTelemetryBlipLoops; ++i) {
        const auto now = std::chrono::steady_clock::now();
        st.last_telemetry = now - std::chrono::milliseconds(cfg.telemetry_timeout_ms + 600);
        st.last_target_update = now;
        st.target_voltage_v = kTargetV;
        st.target_current_a = kTargetI;
        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        const auto cmd = hw->last_power_command(kConnector);
        const auto mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter, kSlot);
        if (!cmd.has_value() || !mreq.has_value()) {
            std::cerr << "e2e_long_cycle_tests failed: missing command during telemetry blip at step " << i << "\n";
            return 1;
        }
        if (!cmd->gc_closed || !cmd->mc_closed) {
            std::cerr << "e2e_long_cycle_tests failed: contactor dropped during short telemetry blip at step " << i
                      << "\n";
            return 1;
        }
        if (cmd->current_limit_a + 1e-6 < baseline_cmd_a - kDipToleranceA) {
            std::cerr << "e2e_long_cycle_tests failed: command collapsed during short telemetry blip at step " << i
                      << " baseline=" << baseline_cmd_a << "A cmd=" << cmd->current_limit_a << "A\n";
            return 1;
        }
        if (mreq->enable && (mreq->current_a + 1e-6 < baseline_cmd_a - kDipToleranceA)) {
            std::cerr << "e2e_long_cycle_tests failed: module command collapsed during short telemetry blip at step "
                      << i << " baseline=" << baseline_cmd_a << "A module_cmd=" << mreq->current_a << "A\n";
            return 1;
        }

        st.present_current_a = std::max(0.0, cmd->current_limit_a - 0.3);
        st.present_voltage_v = kTargetV;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        if (OcppAdapter::TestHook::sessions(adapter).find(kConnector) ==
            OcppAdapter::TestHook::sessions(adapter).end()) {
            std::cerr << "e2e_long_cycle_tests failed: session dropped during short telemetry blip window\n";
            return 1;
        }
    }

    // Telemetry recovers: command must remain stable and contactors stay closed.
    for (int i = 0; i < 5; ++i) {
        st.last_telemetry = std::chrono::steady_clock::now();
        st.last_target_update = std::chrono::steady_clock::now();
        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);
        const auto cmd = hw->last_power_command(kConnector);
        if (!cmd.has_value() || !cmd->gc_closed || !cmd->mc_closed) {
            std::cerr << "e2e_long_cycle_tests failed: contactor/command not healthy after telemetry recovery at step "
                      << i << "\n";
            return 1;
        }
        if (cmd->current_limit_a + 1e-6 < baseline_cmd_a - kDipToleranceA) {
            std::cerr << "e2e_long_cycle_tests failed: command remained collapsed after telemetry recovery at step "
                      << i << " baseline=" << baseline_cmd_a << "A cmd=" << cmd->current_limit_a << "A\n";
            return 1;
        }
        st.present_current_a = std::max(0.0, cmd->current_limit_a - 0.2);
        st.present_voltage_v = kTargetV;
    }

    // Phase 3: stop sequencing (CP B) must gate current and then open contactor after current decay.
    st.cp_state = 'B';
    st.target_current_a = 0.0;
    st.hlc_power_ready = true;
    st.hlc_precharge_active = false;
    st.present_current_a = 8.0;

    bool saw_zero_offer = false;
    for (int i = 0; i < 6; ++i) {
        st.last_telemetry = std::chrono::steady_clock::now();
        st.last_target_update = std::chrono::steady_clock::now() - std::chrono::seconds(3);
        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);
        const auto cmd = hw->last_power_command(kConnector);
        if (!cmd.has_value()) {
            std::cerr << "e2e_long_cycle_tests failed: missing stop-phase command\n";
            return 1;
        }
        if (cmd->current_limit_a <= 0.5) {
            saw_zero_offer = true;
            break;
        }
        st.present_current_a = std::max(0.0, st.present_current_a.value_or(0.0) - 2.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!saw_zero_offer) {
        std::cerr << "e2e_long_cycle_tests failed: current offer did not gate to zero during stop\n";
        return 1;
    }

    bool gc_opened = false;
    st.present_current_a = 0.0;
    for (int i = 0; i < 20; ++i) {
        st.last_telemetry = std::chrono::steady_clock::now();
        st.last_target_update = std::chrono::steady_clock::now() - std::chrono::seconds(3);
        hw->set_status_override(kConnector, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);
        const auto cmd = hw->last_power_command(kConnector);
        if (cmd.has_value() && !cmd->gc_closed) {
            gc_opened = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!gc_opened) {
        std::cerr << "e2e_long_cycle_tests failed: GC did not open after current decayed to zero\n";
        return 1;
    }

    std::cout << "e2e_long_cycle_tests passed"
              << " loops=" << kDemandLoops
              << " baseline_cmd_a=" << baseline_cmd_a
              << " min_cmd_a=" << min_cmd_a
              << " max_cmd_a=" << max_cmd_a
              << "\n";
    return 0;
}
