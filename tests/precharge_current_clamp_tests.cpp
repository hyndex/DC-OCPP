// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "precharge-clamp-test";
    cfg.connectors = {ConnectorConfig{.id = 1, .max_current_a = 80.0, .max_voltage_v = 1000.0, .require_lock = false}};
    cfg.precharge_max_current_a = 2.0;
    cfg.planner_voltage_margin_v = 0.0;
    cfg.planner_current_margin_a = 0.0;
    cfg.planner_voltage_guard_band_v = 10.0;
    cfg.planner_ramp_up_min_a_per_s = 1000.0;
    cfg.planner_ramp_up_max_a_per_s = 1000.0;
    cfg.planner_ramp_down_min_a_per_s = 1000.0;
    cfg.planner_ramp_down_max_a_per_s = 1000.0;
    cfg.switch_stable_time_ms = 0;
    cfg.max_modules_per_gun = 1;
    cfg.ramp_step_a = 1000.0; // remove slew limit for test determinism
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

static GunStatus make_status(uint8_t hlc_stage, bool hlc_power_ready, bool hlc_precharge_active,
                             double present_v, double target_v, double target_i) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.hlc_stage = hlc_stage;
    st.hlc_power_ready = hlc_power_ready;
    st.hlc_precharge_active = hlc_precharge_active;
    st.hlc_charge_complete = false;
    st.authorization_granted = true;
    st.lock_engaged = true;
    st.module_healthy_mask = 0x01;
    st.module_fault_mask = 0x00;
    st.present_voltage_v = present_v;
    st.present_current_a = 0.0;
    st.target_voltage_v = target_v;
    st.target_current_a = target_i;
    st.last_target_update = std::chrono::steady_clock::now();
    st.last_telemetry = std::chrono::steady_clock::now();
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    seed_session(adapter, 1);

    // Case 0: In precharge, current must be clamped to <= 2A even if EV requests more.
    hw->set_status_override(1, make_status(8, false, true, 0.0, 400.0, 10.0));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto mreq_pre = OcppAdapter::TestHook::last_module_command_for_slot(adapter, 1);
    if (!mreq_pre.has_value() || !mreq_pre->enable) {
        std::cerr << "Precharge clamp test failed: expected module enable\n";
        return 1;
    }
    if (mreq_pre->current_a > 2.0 + 1e-6) {
        std::cerr << "Precharge clamp test failed: expected current<=2A, got " << mreq_pre->current_a << "A\n";
        return 1;
    }

    // Case 1: In power delivery/current demand, clamp must not apply (allow >2A when requested).
    auto power_st = make_status(9, true, false, 400.0, 400.0, 10.0);
    power_st.relay_closed = true; // emulate closed GC after successful precharge
    hw->set_status_override(1, power_st);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto mreq_power = OcppAdapter::TestHook::last_module_command_for_slot(adapter, 1);
    if (!mreq_power.has_value() || !mreq_power->enable) {
        std::cerr << "Power phase clamp test failed: expected module enable\n";
        return 1;
    }
    if (mreq_power->current_a < 2.0 + 1e-3) {
        std::cerr << "Power phase clamp test failed: expected current>2A, got " << mreq_power->current_a << "A\n";
        return 1;
    }

    std::cout << "precharge_current_clamp_tests passed\n";
    return 0;
}
