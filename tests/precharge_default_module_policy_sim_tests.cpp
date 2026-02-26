// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "precharge-default-module-policy-sim";
    cfg.module_power_kw = 30.0;
    cfg.grid_limit_kw = 60.0;
    cfg.default_voltage_v = 800.0;

    // Keep the sim fast and deterministic.
    cfg.min_module_hold_ms = 0;
    cfg.min_mc_hold_ms = 0;
    cfg.min_gc_hold_ms = 0;
    cfg.switch_stable_time_ms = 0;

    cfg.precharge_max_current_a = 2.0;
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.precharge_voltage_tolerance_v = 20.0;

    cfg.allow_cross_slot_islands = true;
    cfg.max_modules_per_gun = 2;
    cfg.min_modules_per_active_gun = 1;

    cfg.plc_owns_gun_relay = false;
    cfg.plc_module_relays_enabled = true;
    cfg.plc_relay_feedback = false;
    cfg.plc_relay_mode = PlcRelayMode::Ties;
    cfg.use_plc = true;

    cfg.connectors = {
        ConnectorConfig{.id = 1,
                        .max_current_a = 200,
                        .max_power_w = 60000,
                        .max_voltage_v = 1000,
                        .min_voltage_v = 200,
                        .require_lock = false,
                        .meter_source = "module"},
        ConnectorConfig{.id = 2,
                        .max_current_a = 200,
                        .max_power_w = 60000,
                        .max_voltage_v = 1000,
                        .min_voltage_v = 200,
                        .require_lock = false,
                        .meter_source = "module"},
    };

    // Physical topology: two guns on a ring, one 30 kW module per gun.
    SlotMapping s1;
    s1.id = 1;
    s1.gun_id = 1;
    s1.gc_id = "GC_1";
    s1.mc_id = "MC_1";
    s1.cw_id = 2;
    s1.ccw_id = 2;
    ModuleConfig m1;
    m1.id = "M1";
    m1.mn_id = "MN_1_0";
    m1.type = "sim";
    s1.modules = {m1};

    SlotMapping s2;
    s2.id = 2;
    s2.gun_id = 2;
    s2.gc_id = "GC_2";
    s2.mc_id = "MC_2";
    s2.cw_id = 1;
    s2.ccw_id = 1;
    ModuleConfig m2;
    m2.id = "M2";
    m2.mn_id = "MN_2_0";
    m2.type = "sim";
    s2.modules = {m2};

    cfg.slots = {s1, s2};

    // Keep persistence off in tests.
    cfg.database_dir.clear();
    cfg.auth_wait_timeout_s = 60;
    cfg.meter_sample_interval_s = 1;
    return cfg;
}

static void seed_session(OcppAdapter& adapter, int connector) {
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = "sess-" + std::to_string(connector);
    session.id_token = "TAG";
    session.authorized = true;
    session.ev_connected = true;
    session.transaction_started = true;
    session.connected_at = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        OcppAdapter::TestHook::sessions(adapter)[connector] = session;
    }
}

static GunStatus make_status(bool plugged_in, bool relay_closed, uint8_t hlc_stage,
                             bool precharge_active, bool power_ready,
                             double present_v, double present_i,
                             double target_v, double target_i) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = plugged_in;
    st.cp_state = plugged_in ? 'C' : 'A';
    st.cp_fault = false;
    st.lock_engaged = true;

    st.hlc_stage = plugged_in ? hlc_stage : 0;
    st.hlc_precharge_active = plugged_in ? precharge_active : false;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = plugged_in ? power_ready : false;
    st.hlc_cable_check_ok = true;
    st.authorization_granted = true;

    st.present_voltage_v = plugged_in ? std::optional<double>(present_v) : std::optional<double>(0.0);
    st.present_current_a = plugged_in ? std::optional<double>(present_i) : std::optional<double>(0.0);
    st.present_power_w = plugged_in ? std::optional<double>(present_v * present_i) : std::optional<double>(0.0);

    st.target_voltage_v = plugged_in ? std::optional<double>(target_v) : std::nullopt;
    st.target_current_a = plugged_in ? std::optional<double>(target_i) : std::nullopt;

    st.relay_closed = relay_closed;
    const auto now = std::chrono::steady_clock::now();
    st.last_telemetry = now;
    if (st.target_voltage_v || st.target_current_a) {
        st.last_target_update = now;
    }
    return st;
}

static void refresh_telem(GunStatus& st) { st.last_telemetry = std::chrono::steady_clock::now(); }

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    seed_session(adapter, 1);

    // Internal expanded slot ids (see OcppAdapter::initialize_slots):
    // slot 1: gun-1 module, slot 3: gun-2 module. (slot 2/4 are pass-through.)
    constexpr int SLOT_M1 = 1;
    constexpr int SLOT_M2 = 3;

    constexpr double V = 400.0;
    constexpr double I_60KW = 150.0;

    // Phase A: ISO15118 precharge. Policy: only the default/home module may be allocated/energized here.
    // Even if the EV sends an absurd current request, we must not allocate a second module during precharge.
    GunStatus st1 = make_status(true, false, /*hlc_stage=*/8, /*precharge_active=*/true, /*power_ready=*/false,
                                /*present_v=*/0.0, /*present_i=*/0.0, /*target_v=*/V, /*target_i=*/I_60KW);
    GunStatus st2 = make_status(false, false, /*hlc_stage=*/0, false, false,
                                /*present_v=*/0.0, /*present_i=*/0.0, /*target_v=*/0.0, /*target_i=*/0.0);

    double prev_precharge_v = -1.0;
    bool saw_precharge_gc_close = false;
    bool reached_precharge_target = false;
    for (int tick = 0; tick < 12; ++tick) {
        refresh_telem(st1);
        refresh_telem(st2);
        hw->set_status_override(1, st1);
        hw->set_status_override(2, st2);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        const auto cmd = hw->last_power_command(1);
        if (!cmd.has_value()) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: missing power command\n";
            return 1;
        }
        if (cmd->module_count != 1) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected module_count=1 during precharge, got "
                      << cmd->module_count << "\n";
            return 1;
        }
        if (!cmd->gc_closed) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected GC closed during precharge\n";
            return 1;
        }
        if (cmd->mc_closed) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected MC open during local-only precharge\n";
            return 1;
        }
        if (cmd->module_mask != 0u) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected relay mask 0 when MC is open, mask=0x"
                      << std::hex << static_cast<int>(cmd->module_mask) << std::dec << "\n";
            return 1;
        }
        saw_precharge_gc_close = true;

        const auto m1 = OcppAdapter::TestHook::last_module_command_for_slot(adapter, SLOT_M1);
        const auto m2 = OcppAdapter::TestHook::last_module_command_for_slot(adapter, SLOT_M2);
        if (!m1.has_value() || !m2.has_value()) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: missing module command(s)\n";
            return 1;
        }
        if (!m1->enable) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected default module enabled in precharge\n";
            return 1;
        }
        if (m2->enable) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected boost module disabled in precharge\n";
            return 1;
        }
        if (m1->voltage_v < -1e-6 || m1->voltage_v > V + 5.0) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: unexpected precharge voltage cmd=" << m1->voltage_v
                      << "V expected within [0," << V << "]\n";
            return 1;
        }
        if (prev_precharge_v >= 0.0 && (m1->voltage_v + 1e-6) < prev_precharge_v) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: precharge voltage ramp regressed from "
                      << prev_precharge_v << "V to " << m1->voltage_v << "V\n";
            return 1;
        }
        if (m1->current_a > cfg.precharge_max_current_a + 1e-6) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: expected precharge current<=2A, got "
                      << m1->current_a << "A\n";
            return 1;
        }
        const double precharge_current_floor = cfg.precharge_max_current_a * 0.75;
        if (m1->current_a + 1e-6 < precharge_current_floor) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: precharge current collapsed to "
                      << m1->current_a << "A (expected >= " << precharge_current_floor << "A)\n";
            return 1;
        }
        prev_precharge_v = m1->voltage_v;
        if (m1->voltage_v >= V - 5.0) {
            reached_precharge_target = true;
        }

        // Simulate EV bus following the commanded precharge voltage.
        st1.relay_closed = cmd->gc_closed;
        st1.present_voltage_v = m1->voltage_v;
    }
    if (!saw_precharge_gc_close) {
        std::cerr << "precharge_default_module_policy_sim_tests failed: GC never closed in precharge\n";
        return 1;
    }
    if (!reached_precharge_target) {
        std::cerr << "precharge_default_module_policy_sim_tests failed: precharge voltage never reached target\n";
        return 1;
    }

    // Phase B: power delivery/current demand. Policy: allocate a second module only now, if demand requires.
    // Also: while adding the second module, the default module must remain enabled and its voltage must not drop.
    st1 = make_status(true, true, /*hlc_stage=*/9, /*precharge_active=*/false, /*power_ready=*/true,
                      /*present_v=*/V, /*present_i=*/0.0, /*target_v=*/V, /*target_i=*/I_60KW);

    bool m2_contributing = false;
    bool m1_reached_delivery_current = false;
    bool saw_power_phase_reconfig = false;
    for (int tick = 0; tick < 60; ++tick) {
        refresh_telem(st1);
        refresh_telem(st2);
        hw->set_status_override(1, st1);
        hw->set_status_override(2, st2);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        const auto cmd = hw->last_power_command(1);
        if (!cmd.has_value()) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: missing power command in power phase\n";
            return 1;
        }
        const auto m1 = OcppAdapter::TestHook::last_module_command_for_slot(adapter, SLOT_M1);
        const auto m2 = OcppAdapter::TestHook::last_module_command_for_slot(adapter, SLOT_M2);
        if (!m1.has_value() || !m2.has_value()) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: missing module command(s) in power phase\n";
            return 1;
        }
        if (!m1->enable) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: default module disabled during boost\n";
            return 1;
        }
        if (m1->voltage_v < V - 5.0 || m1->voltage_v > V + 5.0) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: voltage droop during boost, m1="
                      << m1->voltage_v << "V\n";
            return 1;
        }
        if (cmd->current_limit_a < 1.0) {
            saw_power_phase_reconfig = true;
        }
        // Allow a brief reconfiguration window when transitioning from precharge to power delivery.
        // After that, the default module path must sustain non-trivial current.
        if (tick >= 10 && m1->current_a < 1.0) {
            std::cerr << "precharge_default_module_policy_sim_tests failed: unexpected near-zero current, m1="
                      << m1->current_a << "A\n";
            return 1;
        }
        if (m1->current_a >= 8.0) {
            m1_reached_delivery_current = true;
        }

        if (m2->enable && m2->current_a > 10.0) {
            m2_contributing = true;
            break;
        }
    }

    if (!m1_reached_delivery_current) {
        std::cerr << "precharge_default_module_policy_sim_tests failed: default module never reached delivery current\n";
        return 1;
    }
    if (!saw_power_phase_reconfig) {
        std::cerr << "precharge_default_module_policy_sim_tests failed: expected to observe at least one power-phase reconfig tick\n";
        return 1;
    }
    if (!m2_contributing) {
        std::cerr << "precharge_default_module_policy_sim_tests failed: boost module never contributed in power phase\n";
        return 1;
    }

    std::cout << "precharge_default_module_policy_sim_tests passed\n";
    return 0;
}
