// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "gc-close-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    cfg.precharge_voltage_tolerance_v = 50.0;
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.switch_stable_time_ms = 0;
    cfg.max_modules_per_gun = 1;
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
    session.session_id = "sess";
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

static GunStatus make_status(double present_v, double target_v, double target_i,
                             uint8_t hlc_stage = 9,
                             bool hlc_power_ready = true,
                             bool hlc_precharge_active = false) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.hlc_stage = hlc_stage;
    st.hlc_precharge_active = hlc_precharge_active;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = hlc_power_ready;
    st.authorization_granted = true;
    st.lock_engaged = true;
    st.module_healthy_mask = 0x01;
    st.module_fault_mask = 0x00;
    st.target_voltage_v = target_v;
    st.target_current_a = target_i;
    st.present_voltage_v = present_v;
    st.present_current_a = 0.0;
    st.last_telemetry = std::chrono::steady_clock::now();
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Case 0: precharge stage should not close GC even if modules are assigned.
    seed_session(adapter, 1);
    hw->set_status_override(1, make_status(350.0, 355.0, 5.0, 6, false, true));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    auto cmd_precharge = hw->last_power_command(1);
    if (!cmd_precharge.has_value() || cmd_precharge->gc_closed) {
        std::cerr << "GC precharge gating test failed: expected gc_closed=false\n";
        return 1;
    }

    // Case 1: telemetry incomplete, but HLC power phase + voltage within tolerance => GC closes.
    seed_session(adapter, 1);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        const auto it = OcppAdapter::TestHook::sessions(adapter).find(1);
        if (it == OcppAdapter::TestHook::sessions(adapter).end() || !it->second.authorized) {
            std::cerr << "GC close fallback test failed: session not active\n";
            return 1;
        }
    }
    hw->set_status_override(1, make_status(355.0, 355.0, 20.0));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    auto cmd = hw->last_power_command(1);
    if (!cmd.has_value() || !cmd->gc_closed) {
        std::cerr << "GC close fallback test failed: expected gc_closed=true\n";
        return 1;
    }

    // Case 2: GC close remains blocked; after timeout we abort the session.
    seed_session(adapter, 1);
    auto blocked = make_status(0.0, 355.0, 20.0);
    hw->set_status_override(1, blocked);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    auto cmd_blocked = hw->last_power_command(1);
    if (!cmd_blocked.has_value() || cmd_blocked->gc_closed) {
        std::cerr << "GC close timeout test failed: expected gc_closed=false\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    blocked.last_telemetry = std::chrono::steady_clock::now();
    hw->set_status_override(1, blocked);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        if (OcppAdapter::TestHook::sessions(adapter).find(1) != OcppAdapter::TestHook::sessions(adapter).end()) {
            std::cerr << "GC close timeout test failed: session not cleared\n";
            return 1;
        }
    }

    std::cout << "gc_close_gating_tests passed\n";
    return 0;
}
