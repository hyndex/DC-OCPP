#include "ocpp_adapter.hpp"
#include "hardware_sim.hpp"

#include <cassert>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "limit-fallback-test";
    ConnectorConfig c1{};
    c1.id = 1;
    c1.max_power_w = 100000; // 100 kW
    c1.max_voltage_v = 1000.0;
    c1.max_current_a = 200.0;
    cfg.connectors = {c1};
    cfg.meter_sample_interval_s = 1;
    cfg.default_voltage_v = 800.0;
    cfg.max_modules_per_gun = 1;
    return cfg;
}

static GunStatus make_ready_status(const std::chrono::steady_clock::time_point& now, bool relay_closed,
                                   std::chrono::milliseconds ack_age) {
    GunStatus st{};
    st.safety_ok = true;
    st.relay_closed = relay_closed;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.hlc_stage = 9; // HLC_WAIT_CURRENT_DEMAND equivalent threshold
    st.hlc_cable_check_ok = true;
    st.hlc_precharge_active = false;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = true;
    st.authorization_granted = true;
    st.lock_engaged = true;
    st.present_voltage_v = 400.0;
    st.present_current_a = 50.0;
    st.present_power_w = st.present_voltage_v.value() * st.present_current_a.value();
    st.evse_max_voltage_v = 1000.0;
    st.evse_max_current_a = 200.0;
    st.evse_max_power_kw = 150.0;
    st.last_telemetry = now;
    st.last_evse_limit_ack = now - ack_age;
    st.evse_limit_ack_count = 10;
    st.module_healthy_mask = 0x03;
    st.module_fault_mask = 0x00;
    return st;
}

int main() {
    const auto now = std::chrono::steady_clock::now();
    auto cfg = make_cfg();
    auto hw = std::make_shared<SimulatedHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Pending authorization should propagate to hardware stub and keep auth not granted.
    OcppAdapter::TestHook::set_auth_state(adapter, 1, AuthorizationState::Pending);
    auto st_pending = hw->get_status(1);
    assert(st_pending.authorization_granted == false);

    // Seed an authorized session on connector 1.
    OcppAdapter::TestHook::plugged_in_state(adapter)[1] = true;
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = "s1";
    session.authorized = true;
    session.ev_connected = true;
    session.connected_at = now;
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        OcppAdapter::TestHook::sessions(adapter)[1] = session;
    }

    // Case 1: stale EVSE limit ACK should constrain power.
    GunStatus stale = make_ready_status(now, true, std::chrono::milliseconds(2000));
    hw->set_status_override(1, stale);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    assert(OcppAdapter::TestHook::power_constrained(adapter)[1] == true);

    // Case 2: fresh EVSE limit ACK clears constraint.
    GunStatus fresh = make_ready_status(now, true, std::chrono::milliseconds(10));
    hw->set_status_override(1, fresh);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    assert(OcppAdapter::TestHook::power_constrained(adapter)[1] == false);

    std::cout << "Limit fallback tests passed\n";
    return 0;
}
