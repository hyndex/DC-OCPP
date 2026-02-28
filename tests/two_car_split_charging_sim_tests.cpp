// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "two-car-split-sim";
    cfg.module_power_kw = 30.0;
    cfg.grid_limit_kw = 60.0;
    cfg.default_voltage_v = 800.0;

    // Fast transitions in simulation: we don't want min-hold timers to mask plan changes.
    cfg.min_module_hold_ms = 0;
    cfg.min_mc_hold_ms = 0;
    cfg.min_gc_hold_ms = 0;
    cfg.switch_stable_time_ms = 0;

    // Safety thresholds: match production defaults (2 A switching threshold, 20 V dv tolerance).
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.precharge_voltage_tolerance_v = 20.0;

    cfg.allow_cross_slot_islands = true;
    cfg.max_modules_per_gun = 2;          // allow one gun to take both modules when alone
    cfg.min_modules_per_active_gun = 1;   // ensure both guns get at least one module when both active

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

static GunStatus make_dc_status(bool plugged_in, bool relay_closed,
                                double target_v, double target_i,
                                uint8_t hlc_stage = 9) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = plugged_in;
    st.cp_state = plugged_in ? 'C' : 'A';
    st.cp_fault = false;
    st.lock_engaged = true;

    st.hlc_stage = plugged_in ? hlc_stage : 0;
    st.hlc_precharge_active = false;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = plugged_in;
    st.hlc_cable_check_ok = true;
    st.authorization_granted = true;

    st.target_voltage_v = plugged_in ? std::optional<double>(target_v) : std::nullopt;
    st.target_current_a = plugged_in ? std::optional<double>(target_i) : std::nullopt;

    // For module-meter simulation we keep present values populated, but the adapter will
    // prefer module telemetry when available.
    st.present_voltage_v = plugged_in ? std::optional<double>(target_v) : std::optional<double>(0.0);
    st.present_current_a = plugged_in ? std::optional<double>(target_i) : std::optional<double>(0.0);
    st.present_power_w = plugged_in ? std::optional<double>(target_v * target_i) : std::optional<double>(0.0);

    st.relay_closed = relay_closed;
    const auto now = std::chrono::steady_clock::now();
    st.last_telemetry = now;
    if (plugged_in && (st.target_voltage_v.has_value() || st.target_current_a.has_value())) {
        st.last_target_update = now;
    }
    return st;
}

static void refresh_telem(GunStatus& st) {
    st.last_telemetry = std::chrono::steady_clock::now();
}

static bool check_module_cmd(OcppAdapter& adapter, int slot_id,
                             double exp_v, double min_i,
                             const std::string& label) {
    const auto cmd = OcppAdapter::TestHook::last_module_command_for_slot(adapter, slot_id);
    if (!cmd.has_value()) {
        std::cerr << "two_car_split_sim failed: missing module command for " << label
                  << " (slot " << slot_id << ")\n";
        return false;
    }
    if (!cmd->enable) {
        std::cerr << "two_car_split_sim failed: expected module enabled for " << label
                  << " (slot " << slot_id << ")\n";
        return false;
    }
    if (cmd->voltage_v < exp_v - 5.0 || cmd->voltage_v > exp_v + 5.0) {
        std::cerr << "two_car_split_sim failed: unexpected module voltage for " << label
                  << " (slot " << slot_id << ") got " << cmd->voltage_v
                  << "V expected ~" << exp_v << "V\n";
        return false;
    }
    if (cmd->current_a < min_i) {
        std::cerr << "two_car_split_sim failed: expected module current >= " << min_i
                  << "A for " << label << " (slot " << slot_id << ") got "
                  << cmd->current_a << "A\n";
        return false;
    }
    return true;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Scenario:
    // 1) EV1 charging at 60 kW (400 V, 150 A) and should receive both modules (2 x 30 kW) while EV2 is absent.
    // 2) EV2 plugs in and starts charging at 60 kW demand; system must split: 30 kW each (1 module each)
    //    without dropping EV1 contactor/transaction.
    constexpr double V = 400.0;
    constexpr double I_60KW = 150.0;

    seed_session(adapter, 1);
    GunStatus st1 = make_dc_status(true, true, V, I_60KW);
    GunStatus st2 = make_dc_status(false, false, 0.0, 0.0);

    // Stabilize initial single-EV state.
    for (int i = 0; i < 6; ++i) {
        refresh_telem(st1);
        refresh_telem(st2);
        hw->set_status_override(1, st1);
        hw->set_status_override(2, st2);
        OcppAdapter::TestHook::apply_power_plan(adapter);
        auto cmd1 = hw->last_power_command(1);
        if (cmd1) {
            st1.relay_closed = cmd1->gc_closed;
        }
    }

    auto cmd1_single = hw->last_power_command(1);
    auto cmd2_single = hw->last_power_command(2);
    if (!cmd1_single.has_value() || !cmd2_single.has_value()) {
        std::cerr << "two_car_split_sim failed: missing initial power command(s)\n";
        return 1;
    }

    // Expect EV1 to have access to 2 modules and EV2 gun contactor open.
    if (cmd1_single->module_count < 2) {
        std::cerr << "two_car_split_sim failed: expected EV1 module_count>=2 initially, got "
                  << cmd1_single->module_count << "\n";
        return 1;
    }
    if (!cmd1_single->gc_closed) {
        std::cerr << "two_car_split_sim failed: expected EV1 gc_closed=true initially\n";
        return 1;
    }
    if (cmd2_single->gc_closed) {
        std::cerr << "two_car_split_sim failed: expected EV2 gc_closed=false initially\n";
        return 1;
    }
    if (cmd2_single->module_count <= 0 || cmd2_single->module_mask == 0u) {
        std::cerr << "two_car_split_sim failed: expected EV2 side to participate (module_count>0, module_mask!=0)"
                  << " while EV1 consumes both modules; got module_count=" << cmd2_single->module_count
                  << " module_mask=0x" << std::hex << static_cast<int>(cmd2_single->module_mask) << std::dec << "\n";
        return 1;
    }
    // In the expanded 2-gun/2-module topology, module slots are 1 (gun1) and 3 (gun2).
    // EV1 taking both modules must result in both module drivers being enabled and commanded with non-zero current.
    if (!check_module_cmd(adapter, 1, V, 1.0, "module-1")) {
        return 1;
    }
    if (!check_module_cmd(adapter, 3, V, 1.0, "module-2")) {
        return 1;
    }

    // EV2 arrives and starts charging.
    seed_session(adapter, 2);
    st2 = make_dc_status(true, false, V, I_60KW);

    bool ev1_never_dropped = true;
    bool reached_split = false;
    for (int tick = 0; tick < 10; ++tick) {
        refresh_telem(st1);
        refresh_telem(st2);
        hw->set_status_override(1, st1);
        hw->set_status_override(2, st2);
        OcppAdapter::TestHook::apply_power_plan(adapter);

        auto c1 = hw->last_power_command(1);
        auto c2 = hw->last_power_command(2);
        if (!c1.has_value() || !c2.has_value()) {
            std::cerr << "two_car_split_sim failed: missing power command(s) during split\n";
            return 1;
        }

        if (!c1->gc_closed) {
            ev1_never_dropped = false;
        }

        // Simulate contactors following commands.
        st1.relay_closed = c1->gc_closed;
        st2.relay_closed = c2->gc_closed;

        // Success criterion: both guns commanded closed, each with exactly 1 module budgeted (30 kW each).
        if (c1->gc_closed && c2->gc_closed && c1->module_count == 1 && c2->module_count == 1) {
            reached_split = true;
            break;
        }

        // Allow the controller two ticks to reconfigure islands without tripping.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!ev1_never_dropped) {
        std::cerr << "two_car_split_sim failed: EV1 gun contactor dropped during reallocation\n";
        return 1;
    }
    if (!reached_split) {
        std::cerr << "two_car_split_sim failed: did not reach expected 1-module-each split state\n";
        return 1;
    }

    std::cout << "two_car_split_charging_sim_tests passed\n";
    return 0;
}
