// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "power-delivery-stall-test";
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
    session.session_id = "stall";
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

static GunStatus make_status(double present_v, double target_v, double target_i) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.hlc_stage = 9;
    st.hlc_precharge_active = false;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = true;
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

    seed_session(adapter, 1);
    auto st = make_status(355.0, 355.0, 30.0);
    st.relay_closed = true; // simulate contactor closed but no current flow
    hw->set_status_override(1, st);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        if (OcppAdapter::TestHook::sessions(adapter).find(1) == OcppAdapter::TestHook::sessions(adapter).end()) {
            std::cerr << "Power delivery stall test failed: session cleared too early\n";
            return 1;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    st.last_telemetry = std::chrono::steady_clock::now();
    hw->set_status_override(1, st);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        if (OcppAdapter::TestHook::sessions(adapter).find(1) != OcppAdapter::TestHook::sessions(adapter).end()) {
            std::cerr << "Power delivery stall test failed: session not cleared\n";
            return 1;
        }
    }

    // Do not arm stall fault if EVSE is intentionally offering very low current.
    auto low_offer_cfg = make_cfg();
    low_offer_cfg.connectors[0].max_current_a = 0.2;
    auto low_offer_hw = std::make_shared<TestHardware>(low_offer_cfg);
    OcppAdapter low_offer_adapter(low_offer_cfg, low_offer_hw);
    seed_session(low_offer_adapter, 1);
    auto low_offer_st = make_status(355.0, 355.0, 30.0);
    low_offer_st.relay_closed = true;
    low_offer_hw->set_status_override(1, low_offer_st);
    OcppAdapter::TestHook::apply_power_plan(low_offer_adapter);
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    low_offer_st.last_telemetry = std::chrono::steady_clock::now();
    low_offer_hw->set_status_override(1, low_offer_st);
    OcppAdapter::TestHook::apply_power_plan(low_offer_adapter);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(low_offer_adapter));
        if (OcppAdapter::TestHook::sessions(low_offer_adapter).find(1) ==
            OcppAdapter::TestHook::sessions(low_offer_adapter).end()) {
            std::cerr << "Power delivery stall test failed: session cleared while EVSE current offer was low\n";
            return 1;
        }
    }

    std::cout << "power_delivery_stall_tests passed\n";
    return 0;
}
