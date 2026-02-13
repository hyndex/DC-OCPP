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

    // When EV stops requesting power (CP drops to B), connector-side current offer is gated to 0A.
    // Module command must follow the same gating and stop commanding charge current.
    auto decay_cfg = make_cfg();
    auto decay_hw = std::make_shared<TestHardware>(decay_cfg);
    OcppAdapter decay_adapter(decay_cfg, decay_hw);
    seed_session(decay_adapter, 1);

    auto decay_st = make_status(369.0, 384.0, 41.0);
    decay_st.relay_closed = true;
    decay_hw->set_status_override(1, decay_st);
    OcppAdapter::TestHook::apply_power_plan(decay_adapter);
    const auto mreq_before = OcppAdapter::TestHook::last_module_command_for_slot(decay_adapter, 1);
    if (!mreq_before.has_value() || mreq_before->current_a < 0.5) {
        std::cerr << "Power delivery stall test failed: expected non-zero module current before CP drop\n";
        return 1;
    }

    decay_st.cp_state = 'B';
    decay_st.last_telemetry = std::chrono::steady_clock::now();
    decay_hw->set_status_override(1, decay_st);
    OcppAdapter::TestHook::apply_power_plan(decay_adapter); // start CP-drop debounce window
    const auto cmd_during_debounce = decay_hw->last_power_command(1);
    if (!cmd_during_debounce.has_value() || cmd_during_debounce->current_limit_a < 0.5) {
        std::cerr << "Power delivery stall test failed: connector current offer dropped during CP-drop debounce\n";
        return 1;
    }
    const auto mreq_during_debounce = OcppAdapter::TestHook::last_module_command_for_slot(decay_adapter, 1);
    if (!mreq_during_debounce.has_value() || mreq_during_debounce->current_a < 0.5) {
        std::cerr << "Power delivery stall test failed: module current command dropped during CP-drop debounce\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(950));
    decay_st.last_telemetry = std::chrono::steady_clock::now();
    decay_hw->set_status_override(1, decay_st);
    OcppAdapter::TestHook::apply_power_plan(decay_adapter);

    const auto cmd_after = decay_hw->last_power_command(1);
    if (!cmd_after.has_value() || cmd_after->current_limit_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: expected connector current offer to gate to 0A after CP drop\n";
        return 1;
    }
    const auto mreq_after = OcppAdapter::TestHook::last_module_command_for_slot(decay_adapter, 1);
    if (!mreq_after.has_value() || mreq_after->current_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: module command did not gate current to 0A after CP drop\n";
        return 1;
    }

    // CP unknown telemetry blips should not immediately collapse current offer.
    auto unknown_cfg = make_cfg();
    auto unknown_hw = std::make_shared<TestHardware>(unknown_cfg);
    OcppAdapter unknown_adapter(unknown_cfg, unknown_hw);
    seed_session(unknown_adapter, 1);

    auto unknown_st = make_status(369.0, 384.0, 41.0);
    unknown_st.relay_closed = true;
    unknown_hw->set_status_override(1, unknown_st);
    OcppAdapter::TestHook::apply_power_plan(unknown_adapter);
    const auto unknown_before = OcppAdapter::TestHook::last_module_command_for_slot(unknown_adapter, 1);
    if (!unknown_before.has_value() || unknown_before->current_a < 0.5) {
        std::cerr << "Power delivery stall test failed: expected non-zero module current before CP unknown\n";
        return 1;
    }

    unknown_st.cp_state = 'U';
    unknown_st.last_telemetry = std::chrono::steady_clock::now();
    unknown_hw->set_status_override(1, unknown_st);
    OcppAdapter::TestHook::apply_power_plan(unknown_adapter); // start CP-unknown debounce
    const auto unknown_cmd_during = unknown_hw->last_power_command(1);
    if (!unknown_cmd_during.has_value() || unknown_cmd_during->current_limit_a < 0.5) {
        std::cerr << "Power delivery stall test failed: connector current offer dropped during CP-unknown debounce\n";
        return 1;
    }
    const auto unknown_mreq_during = OcppAdapter::TestHook::last_module_command_for_slot(unknown_adapter, 1);
    if (!unknown_mreq_during.has_value() || unknown_mreq_during->current_a < 0.5) {
        std::cerr << "Power delivery stall test failed: module current command dropped during CP-unknown debounce\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(950));
    unknown_st.last_telemetry = std::chrono::steady_clock::now();
    unknown_hw->set_status_override(1, unknown_st);
    OcppAdapter::TestHook::apply_power_plan(unknown_adapter);
    const auto unknown_cmd_after = unknown_hw->last_power_command(1);
    if (!unknown_cmd_after.has_value() || unknown_cmd_after->current_limit_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: expected connector current offer to gate to 0A after sustained CP unknown\n";
        return 1;
    }
    const auto unknown_mreq_after = OcppAdapter::TestHook::last_module_command_for_slot(unknown_adapter, 1);
    if (!unknown_mreq_after.has_value() || unknown_mreq_after->current_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: module current command did not gate to 0A after sustained CP unknown\n";
        return 1;
    }

    // CP B blips while fresh CurrentDemand targets are still arriving should not collapse current.
    auto fresh_target_cfg = make_cfg();
    auto fresh_target_hw = std::make_shared<TestHardware>(fresh_target_cfg);
    OcppAdapter fresh_target_adapter(fresh_target_cfg, fresh_target_hw);
    seed_session(fresh_target_adapter, 1);

    auto fresh_target_st = make_status(373.0, 384.0, 51.0);
    fresh_target_st.relay_closed = true;
    fresh_target_st.last_target_update = std::chrono::steady_clock::now();
    fresh_target_hw->set_status_override(1, fresh_target_st);
    OcppAdapter::TestHook::apply_power_plan(fresh_target_adapter);
    const auto fresh_target_before = OcppAdapter::TestHook::last_module_command_for_slot(fresh_target_adapter, 1);
    if (!fresh_target_before.has_value() || fresh_target_before->current_a < 0.5) {
        std::cerr << "Power delivery stall test failed: expected non-zero module current before CP B blip\n";
        return 1;
    }

    fresh_target_st.cp_state = 'B';
    fresh_target_st.last_telemetry = std::chrono::steady_clock::now();
    fresh_target_st.last_target_update = std::chrono::steady_clock::now();
    fresh_target_hw->set_status_override(1, fresh_target_st);
    OcppAdapter::TestHook::apply_power_plan(fresh_target_adapter);
    const auto fresh_target_cmd_during = fresh_target_hw->last_power_command(1);
    if (!fresh_target_cmd_during.has_value() || fresh_target_cmd_during->current_limit_a < 0.5) {
        std::cerr << "Power delivery stall test failed: connector current collapsed despite fresh CurrentDemand targets\n";
        return 1;
    }
    const auto fresh_target_mreq_during =
        OcppAdapter::TestHook::last_module_command_for_slot(fresh_target_adapter, 1);
    if (!fresh_target_mreq_during.has_value() || fresh_target_mreq_during->current_a < 0.5) {
        std::cerr << "Power delivery stall test failed: module current collapsed despite fresh CurrentDemand targets\n";
        return 1;
    }

    // Once target updates stop, fall back to the existing CP-drop debounce behavior and gate to 0A.
    fresh_target_st.last_telemetry = std::chrono::steady_clock::now();
    fresh_target_st.last_target_update = std::chrono::steady_clock::now() - std::chrono::seconds(3);
    fresh_target_hw->set_status_override(1, fresh_target_st);
    OcppAdapter::TestHook::apply_power_plan(fresh_target_adapter);
    std::this_thread::sleep_for(std::chrono::milliseconds(950));
    fresh_target_st.last_telemetry = std::chrono::steady_clock::now();
    fresh_target_hw->set_status_override(1, fresh_target_st);
    OcppAdapter::TestHook::apply_power_plan(fresh_target_adapter);
    const auto fresh_target_cmd_after = fresh_target_hw->last_power_command(1);
    if (!fresh_target_cmd_after.has_value() || fresh_target_cmd_after->current_limit_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: expected 0A after CP B + stale CurrentDemand targets\n";
        return 1;
    }
    const auto fresh_target_mreq_after =
        OcppAdapter::TestHook::last_module_command_for_slot(fresh_target_adapter, 1);
    if (!fresh_target_mreq_after.has_value() || fresh_target_mreq_after->current_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: module current did not gate after stale targets\n";
        return 1;
    }

    // Sustained CP=B must not keep power active indefinitely, even if target timestamps keep refreshing.
    // Gate current after bounded hold and avoid false PowerDeliveryStalled trips.
    auto sustained_cfg = make_cfg();
    auto sustained_hw = std::make_shared<TestHardware>(sustained_cfg);
    OcppAdapter sustained_adapter(sustained_cfg, sustained_hw);
    seed_session(sustained_adapter, 1);

    auto sustained_st = make_status(370.0, 384.0, 30.0);
    sustained_st.relay_closed = true;
    sustained_st.cp_state = 'B';
    sustained_st.last_target_update = std::chrono::steady_clock::now();
    sustained_hw->set_status_override(1, sustained_st);
    OcppAdapter::TestHook::apply_power_plan(sustained_adapter);
    const auto sustained_cmd_before = sustained_hw->last_power_command(1);
    if (!sustained_cmd_before.has_value() || sustained_cmd_before->current_limit_a < 0.5) {
        std::cerr << "Power delivery stall test failed: expected temporary hold during CP=B transition\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1700));
    sustained_st.last_telemetry = std::chrono::steady_clock::now();
    sustained_st.last_target_update = std::chrono::steady_clock::now(); // emulate continuously refreshed stale target metadata
    sustained_hw->set_status_override(1, sustained_st);
    OcppAdapter::TestHook::apply_power_plan(sustained_adapter);
    const auto sustained_cmd_after = sustained_hw->last_power_command(1);
    if (!sustained_cmd_after.has_value() || sustained_cmd_after->current_limit_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: expected 0A after sustained CP=B despite target refresh\n";
        return 1;
    }
    const auto sustained_mreq_after = OcppAdapter::TestHook::last_module_command_for_slot(sustained_adapter, 1);
    if (!sustained_mreq_after.has_value() || sustained_mreq_after->current_a > 1e-6) {
        std::cerr << "Power delivery stall test failed: module current did not gate after sustained CP=B\n";
        return 1;
    }
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(sustained_adapter));
        if (OcppAdapter::TestHook::sessions(sustained_adapter).find(1) ==
            OcppAdapter::TestHook::sessions(sustained_adapter).end()) {
            std::cerr << "Power delivery stall test failed: session faulted during sustained CP=B bounded hold\n";
            return 1;
        }
    }

    // Near-edge decay: if current drops shortly after GC-open timeout threshold, do not immediately fault.
    auto gc_cfg = make_cfg();
    auto gc_hw = std::make_shared<TestHardware>(gc_cfg);
    OcppAdapter gc_adapter(gc_cfg, gc_hw);
    seed_session(gc_adapter, 1);

    auto gc_st = make_status(369.0, 384.0, 41.0);
    gc_st.cp_state = 'B';
    gc_st.hlc_stage = 10;
    gc_st.hlc_power_ready = true;
    gc_st.relay_closed = true;
    gc_st.present_current_a = 41.0;
    gc_st.last_telemetry = std::chrono::steady_clock::now();
    gc_hw->set_status_override(1, gc_st);
    OcppAdapter::TestHook::apply_power_plan(gc_adapter); // arm GC-open timeout tracking

    std::this_thread::sleep_for(std::chrono::milliseconds(3650));
    gc_st.last_telemetry = std::chrono::steady_clock::now();
    gc_hw->set_status_override(1, gc_st);
    OcppAdapter::TestHook::apply_power_plan(gc_adapter); // cross threshold but should remain in confirmation window
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(gc_adapter));
        if (OcppAdapter::TestHook::sessions(gc_adapter).find(1) ==
            OcppAdapter::TestHook::sessions(gc_adapter).end()) {
            std::cerr << "Power delivery stall test failed: session faulted at first GC-open timeout edge\n";
            return 1;
        }
    }

    gc_st.present_current_a = 0.0;
    gc_st.last_telemetry = std::chrono::steady_clock::now();
    gc_hw->set_status_override(1, gc_st);
    OcppAdapter::TestHook::apply_power_plan(gc_adapter);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(gc_adapter));
        if (OcppAdapter::TestHook::sessions(gc_adapter).find(1) ==
            OcppAdapter::TestHook::sessions(gc_adapter).end()) {
            std::cerr << "Power delivery stall test failed: session faulted even after current decayed to 0A\n";
            return 1;
        }
    }

    std::cout << "power_delivery_stall_tests passed\n";
    return 0;
}
