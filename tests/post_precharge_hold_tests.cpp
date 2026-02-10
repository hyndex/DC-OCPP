// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <iostream>
#include <mutex>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "post-precharge-hold-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    cfg.precharge_voltage_tolerance_v = 50.0;
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.switch_stable_time_ms = 0;
    cfg.max_modules_per_gun = 2; // verify keepalive allocation stays at 1
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static void seed_pending_session(OcppAdapter& adapter, int connector) {
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = "sess";
    session.id_token = "TAG";
    session.authorized = false;
    session.ev_connected = true;
    session.transaction_started = false;
    session.connected_at = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        OcppAdapter::TestHook::sessions(adapter)[connector] = session;
    }
    OcppAdapter::TestHook::set_auth_state(adapter, connector, AuthorizationState::Pending);
}

static void mark_authorized(OcppAdapter& adapter, int connector) {
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        auto& sess = OcppAdapter::TestHook::sessions(adapter)[connector];
        sess.authorized = true;
        sess.authorized_at = std::chrono::steady_clock::now();
    }
    OcppAdapter::TestHook::set_auth_state(adapter, connector, AuthorizationState::Granted);
}

static GunStatus make_status(double present_v, double target_v, double target_i,
                             uint8_t hlc_stage = 9,
                             bool hlc_power_ready = true) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.hlc_stage = hlc_stage;
    st.hlc_precharge_active = false;
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

    // Post-precharge hold: HLC power phase (stage>=9) but OCPP authorization pending.
    seed_pending_session(adapter, 1);
    hw->set_status_override(1, make_status(399.0, 400.0, 20.0));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto cmd_hold = hw->last_power_command(1);
    if (!cmd_hold.has_value()) {
        std::cerr << "post_precharge_hold_tests failed: missing power command\n";
        return 1;
    }
    if (!cmd_hold->gc_closed) {
        std::cerr << "post_precharge_hold_tests failed: expected GC closed during hold\n";
        return 1;
    }
    if (cmd_hold->module_count != 1) {
        std::cerr << "post_precharge_hold_tests failed: expected 1 module during hold\n";
        return 1;
    }
    if (cmd_hold->current_limit_a != 0.0 || cmd_hold->power_kw != 0.0) {
        std::cerr << "post_precharge_hold_tests failed: expected 0A/0kW during hold\n";
        return 1;
    }
    if (std::abs(cmd_hold->voltage_set_v - 400.0) > 1e-6) {
        std::cerr << "post_precharge_hold_tests failed: expected V_set=400 during hold\n";
        return 1;
    }

    // Once authorized, we should transition to real current delivery without dropping voltage or opening GC.
    mark_authorized(adapter, 1);
    hw->set_status_override(1, make_status(400.0, 400.0, 20.0));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto cmd_charge = hw->last_power_command(1);
    if (!cmd_charge.has_value()) {
        std::cerr << "post_precharge_hold_tests failed: missing power command after auth\n";
        return 1;
    }
    if (!cmd_charge->gc_closed) {
        std::cerr << "post_precharge_hold_tests failed: expected GC closed after auth\n";
        return 1;
    }
    if (cmd_charge->module_count != 1) {
        std::cerr << "post_precharge_hold_tests failed: expected 1 module after auth\n";
        return 1;
    }
    if (std::abs(cmd_charge->voltage_set_v - 400.0) > 1e-6) {
        std::cerr << "post_precharge_hold_tests failed: expected V_set=400 after auth\n";
        return 1;
    }

    std::cout << "post_precharge_hold_tests passed\n";
    return 0;
}
