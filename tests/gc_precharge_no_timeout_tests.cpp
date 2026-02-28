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
    cfg.charge_point_id = "gc-precharge-no-timeout-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    cfg.require_auth_for_precharge = false;
    cfg.precharge_voltage_tolerance_v = 50.0;
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.switch_stable_time_ms = 0;
    cfg.max_modules_per_gun = 1;
    cfg.precharge_timeout_ms = 2000;
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static void seed_session(OcppAdapter& adapter, int connector, const std::string& session_id = "sess") {
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = session_id;
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
    st.hlc_stage = 8; // HLC_WAIT_PRECHARGE (Basic PLC enum)
    st.hlc_precharge_active = true;
    st.hlc_charge_complete = false;
    st.hlc_power_ready = false;
    st.authorization_granted = true;
    st.lock_engaged = true;
    st.module_healthy_mask = 0x01;
    st.module_fault_mask = 0x00;
    st.target_voltage_v = target_v;
    st.target_current_a = target_i;
    st.last_target_update = std::chrono::steady_clock::now();
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

    auto st = make_status(0.0, 400.0, 5.0);
    hw->set_status_override(1, st);
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto cmd0 = hw->last_power_command(1);
    if (!cmd0.has_value()) {
        std::cerr << "gc_precharge_no_timeout_tests failed: missing power command\n";
        return 1;
    }
    if (cmd0->module_count <= 0) {
        std::cerr << "gc_precharge_no_timeout_tests failed: expected module assignment in precharge\n";
        return 1;
    }
    if (cmd0->voltage_set_v <= 50.0) {
        std::cerr << "gc_precharge_no_timeout_tests failed: invalid precharge voltage command V_set="
                  << cmd0->voltage_set_v << "\n";
        return 1;
    }
    if (cmd0->gc_closed) {
        std::cerr << "gc_precharge_no_timeout_tests failed: expected GC gating before precharge convergence\n";
        return 1;
    }
    if (cmd0->current_limit_a > cfg.precharge_max_current_a + 1e-6) {
        std::cerr << "gc_precharge_no_timeout_tests failed: expected precharge current clamp\n";
        return 1;
    }

    // Simulate normal precharge voltage rise; session must remain active (no false timeout).
    for (double v : {40.0, 100.0, 180.0, 260.0, 340.0, 395.0}) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        st.present_voltage_v = v;
        st.last_target_update = std::chrono::steady_clock::now();
        st.last_telemetry = std::chrono::steady_clock::now();
        hw->set_status_override(1, st);
        OcppAdapter::TestHook::apply_power_plan(adapter);
    }

    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        if (OcppAdapter::TestHook::sessions(adapter).find(1) == OcppAdapter::TestHook::sessions(adapter).end()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: session cleared during healthy precharge ramp\n";
            return 1;
        }
        // End this scenario before running additional adapters to avoid background timeout interactions.
        OcppAdapter::TestHook::sessions(adapter).erase(1);
    }

    // Brief precharge-current transient above nominal 2A should not immediately fault the session.
    auto hw_over_i_short = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_over_i_short(cfg, hw_over_i_short);
    seed_session(adapter_over_i_short, 1, "sess-over-i-short");
    auto over_i_short = make_status(398.0, 400.0, 5.0);
    over_i_short.relay_closed = true;
    over_i_short.present_current_a = 2.8;
    over_i_short.last_target_update = std::chrono::steady_clock::now();
    over_i_short.last_telemetry = std::chrono::steady_clock::now();
    hw_over_i_short->set_status_override(1, over_i_short);
    OcppAdapter::TestHook::apply_power_plan(adapter_over_i_short);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    over_i_short.last_target_update = std::chrono::steady_clock::now();
    over_i_short.last_telemetry = std::chrono::steady_clock::now();
    hw_over_i_short->set_status_override(1, over_i_short);
    OcppAdapter::TestHook::apply_power_plan(adapter_over_i_short);
    over_i_short.present_current_a = 0.0;
    over_i_short.last_target_update = std::chrono::steady_clock::now();
    over_i_short.last_telemetry = std::chrono::steady_clock::now();
    hw_over_i_short->set_status_override(1, over_i_short);
    OcppAdapter::TestHook::apply_power_plan(adapter_over_i_short);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_over_i_short));
        if (OcppAdapter::TestHook::sessions(adapter_over_i_short).find(1) ==
            OcppAdapter::TestHook::sessions(adapter_over_i_short).end()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: brief precharge overcurrent transient latched fault\n";
            return 1;
        }
    }

    // Precharge with no-energy gating (e.g. auth-pending transition) must still keep a valid non-zero
    // voltage command while current remains clamped. Regresses "module stuck around ~200V, V_set=0" failures.
    auto hw_pending_auth = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_pending_auth(cfg, hw_pending_auth);
    seed_session(adapter_pending_auth, 1, "sess-pending-auth");
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_pending_auth));
        auto& sessions = OcppAdapter::TestHook::sessions(adapter_pending_auth);
        auto it = sessions.find(1);
        if (it != sessions.end()) {
            it->second.authorized = false;
        }
    }
    auto pending_auth = make_status(/*present_v=*/200.0, /*target_v=*/355.0, /*target_i=*/1.2);
    pending_auth.authorization_granted = false;
    pending_auth.relay_closed = false;
    const int module_slot_id = cfg.slots.empty() ? 1 : cfg.slots.front().id;
    auto verify_precharge_nonzero_voltage = [&](const char* phase) -> bool {
        const auto cmd = hw_pending_auth->last_power_command(1);
        if (!cmd.has_value()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: missing power command (" << phase << ")\n";
            return false;
        }
        if (cmd->module_count > 0 && cmd->voltage_set_v <= 50.0) {
            std::cerr << "gc_precharge_no_timeout_tests failed: zero/low precharge voltage cmd (" << phase
                      << ") V_set=" << cmd->voltage_set_v << " module_count=" << cmd->module_count << "\n";
            return false;
        }
        if (cmd->current_limit_a > cfg.precharge_max_current_a + 1e-6) {
            std::cerr << "gc_precharge_no_timeout_tests failed: precharge current limit exceeded (" << phase
                      << ") I_lim=" << cmd->current_limit_a << "\n";
            return false;
        }
        const auto mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter_pending_auth, module_slot_id);
        if (!mreq.has_value()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: missing module command (" << phase << ")\n";
            return false;
        }
        if (mreq->enable && mreq->voltage_v <= 50.0) {
            std::cerr << "gc_precharge_no_timeout_tests failed: module voltage collapsed in precharge (" << phase
                      << ") V_mod=" << mreq->voltage_v << "\n";
            return false;
        }
        return true;
    };
    for (int i = 0; i < 6; ++i) {
        pending_auth.last_target_update = std::chrono::steady_clock::now();
        pending_auth.last_telemetry = std::chrono::steady_clock::now();
        hw_pending_auth->set_status_override(1, pending_auth);
        OcppAdapter::TestHook::apply_power_plan(adapter_pending_auth);
        if (!verify_precharge_nonzero_voltage("auth_pending")) {
            return 1;
        }
    }
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_pending_auth));
        auto& sessions = OcppAdapter::TestHook::sessions(adapter_pending_auth);
        auto it = sessions.find(1);
        if (it != sessions.end()) {
            it->second.authorized = true;
        }
    }
    pending_auth.authorization_granted = true;
    for (int i = 0; i < 6; ++i) {
        pending_auth.last_target_update = std::chrono::steady_clock::now();
        pending_auth.last_telemetry = std::chrono::steady_clock::now();
        hw_pending_auth->set_status_override(1, pending_auth);
        OcppAdapter::TestHook::apply_power_plan(adapter_pending_auth);
        if (!verify_precharge_nonzero_voltage("auth_granted")) {
            return 1;
        }
    }

    // Stale precharge-active on non-precharge stage (e.g., after TCP peer close) must not keep warmup/energy alive.
    auto hw_stale = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_stale(cfg, hw_stale);
    seed_session(adapter_stale, 1, "sess-stale");
    auto stale = make_status(354.0, 384.0, 40.0);
    stale.cp_state = 'B';
    stale.hlc_stage = 2; // early handshake stage (not precharge)
    stale.hlc_precharge_active = true; // stale flag glitch
    stale.hlc_power_ready = false;
    stale.relay_closed = false;
    stale.last_target_update = std::chrono::steady_clock::now();
    stale.last_telemetry = std::chrono::steady_clock::now();
    hw_stale->set_status_override(1, stale);
    OcppAdapter::TestHook::apply_power_plan(adapter_stale);
    const auto stale_cmd = hw_stale->last_power_command(1);
    if (!stale_cmd.has_value()) {
        std::cerr << "gc_precharge_no_timeout_tests failed: missing stale-stage power command\n";
        return 1;
    }
    if (stale_cmd->gc_closed || stale_cmd->mc_closed || stale_cmd->module_count != 0 ||
        stale_cmd->current_limit_a > 1e-6 || stale_cmd->power_kw > 1e-6) {
        std::cerr << "gc_precharge_no_timeout_tests failed: stale precharge flag kept output active\n";
        return 1;
    }
    const auto stale_mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter_stale, 1);
    if (!stale_mreq.has_value()) {
        std::cerr << "gc_precharge_no_timeout_tests failed: missing stale-stage module command\n";
        return 1;
    }
    if (stale_mreq->enable || stale_mreq->voltage_v > 1.0 || stale_mreq->current_a > 1e-6 ||
        stale_mreq->power_kw > 1e-6) {
        std::cerr << "gc_precharge_no_timeout_tests failed: stale precharge flag kept module warmup enabled\n";
        return 1;
    }

    // Explicit non-power HLC stage with fresh EV targets must not be treated as power delivery.
    auto hw_stage = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_stage(cfg, hw_stage);
    seed_session(adapter_stage, 1, "sess-stage");
    auto stage = make_status(354.0, 384.0, 40.0);
    stage.cp_state = 'C';
    stage.hlc_stage = 2; // early handshake stage (not precharge/power)
    stage.hlc_precharge_active = false;
    stage.hlc_power_ready = false;
    stage.relay_closed = false;
    stage.last_target_update = std::chrono::steady_clock::now();
    stage.last_telemetry = std::chrono::steady_clock::now();
    hw_stage->set_status_override(1, stage);
    OcppAdapter::TestHook::apply_power_plan(adapter_stage);
    const auto stage_cmd = hw_stage->last_power_command(1);
    if (!stage_cmd.has_value()) {
        std::cerr << "gc_precharge_no_timeout_tests failed: missing stage-fallback power command\n";
        return 1;
    }
    if (stage_cmd->gc_closed || stage_cmd->mc_closed || stage_cmd->module_count != 0 ||
        stage_cmd->current_limit_a > 1e-6 || stage_cmd->power_kw > 1e-6) {
        std::cerr << "gc_precharge_no_timeout_tests failed: early HLC stage was treated as power delivery\n";
        return 1;
    }
    const auto stage_mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter_stage, 1);
    if (!stage_mreq.has_value()) {
        std::cerr << "gc_precharge_no_timeout_tests failed: missing stage-fallback module command\n";
        return 1;
    }
    if (stage_mreq->enable || stage_mreq->voltage_v > 1.0 || stage_mreq->current_a > 1e-6 ||
        stage_mreq->power_kw > 1e-6) {
        std::cerr << "gc_precharge_no_timeout_tests failed: early HLC stage kept module warmup enabled\n";
        return 1;
    }

    // CP=B handshake window with high present voltage must not trip StuckVoltage before precharge begins.
    auto hw_cp_b = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_cp_b(cfg, hw_cp_b);
    seed_session(adapter_cp_b, 1, "sess-cp-b");
    auto cp_b = make_status(355.0, 355.0, 2.0);
    cp_b.cp_state = 'B';
    cp_b.hlc_stage = 2; // handshake, not precharge
    cp_b.hlc_precharge_active = false;
    cp_b.hlc_power_ready = false;
    cp_b.relay_closed = false;
    cp_b.target_voltage_v.reset();
    cp_b.target_current_a.reset();
    cp_b.last_target_update = std::chrono::steady_clock::time_point{};
    cp_b.last_telemetry = std::chrono::steady_clock::now();
    hw_cp_b->set_status_override(1, cp_b);
    OcppAdapter::TestHook::set_stuck_output_voltage_since(adapter_cp_b, 1,
                                                          std::chrono::steady_clock::now() -
                                                              std::chrono::seconds(13));
    OcppAdapter::TestHook::apply_power_plan(adapter_cp_b);
    if (OcppAdapter::TestHook::local_hw_disabled(adapter_cp_b, 1)) {
        std::cerr << "gc_precharge_no_timeout_tests failed: CP=B handshake tripped local HW disable\n";
        return 1;
    }
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_cp_b));
        if (OcppAdapter::TestHook::sessions(adapter_cp_b).find(1) ==
            OcppAdapter::TestHook::sessions(adapter_cp_b).end()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: CP=B handshake cleared session\n";
            return 1;
        }
    }
    cp_b.cp_state = 'C';
    cp_b.hlc_stage = 8;
    cp_b.hlc_precharge_active = true;
    cp_b.target_voltage_v = 355.0;
    cp_b.target_current_a = 2.0;
    cp_b.last_target_update = std::chrono::steady_clock::now();
    cp_b.last_telemetry = std::chrono::steady_clock::now();
    hw_cp_b->set_status_override(1, cp_b);
    OcppAdapter::TestHook::apply_power_plan(adapter_cp_b);
    if (OcppAdapter::TestHook::local_hw_disabled(adapter_cp_b, 1)) {
        std::cerr << "gc_precharge_no_timeout_tests failed: precharge transition latched StuckVoltage\n";
        return 1;
    }
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_cp_b));
        if (OcppAdapter::TestHook::sessions(adapter_cp_b).find(1) ==
            OcppAdapter::TestHook::sessions(adapter_cp_b).end()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: precharge transition cleared session\n";
            return 1;
        }
    }

    // Separate scenario: voltage never rises to target => precharge timeout fault should clear the session.
    auto hw_stuck = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_stuck(cfg, hw_stuck);
    seed_session(adapter_stuck, 1);

    auto stuck = make_status(0.0, 400.0, 5.0);
    hw_stuck->set_status_override(1, stuck);
    OcppAdapter::TestHook::apply_power_plan(adapter_stuck);
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    stuck.last_target_update = std::chrono::steady_clock::now();
    stuck.last_telemetry = std::chrono::steady_clock::now();
    hw_stuck->set_status_override(1, stuck);
    OcppAdapter::TestHook::apply_power_plan(adapter_stuck);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_stuck));
        if (OcppAdapter::TestHook::sessions(adapter_stuck).find(1) !=
            OcppAdapter::TestHook::sessions(adapter_stuck).end()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: expected precharge timeout to clear session\n";
            return 1;
        }
    }

    // Session-epoch reset: stale GC close timeout state must not carry over to a fresh session.
    auto hw_epoch = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter_epoch(cfg, hw_epoch);
    seed_session(adapter_epoch, 1, "sess-old");

    auto blocked = make_status(/*present_v=*/120.0, /*target_v=*/400.0, /*target_i=*/5.0);
    hw_epoch->set_status_override(1, blocked);
    OcppAdapter::TestHook::apply_power_plan(adapter_epoch); // starts GC close timeout tracking
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));

    // New session on the same connector while still plugged in.
    seed_session(adapter_epoch, 1, "sess-new");
    auto fresh = make_status(/*present_v=*/0.0, /*target_v=*/400.0, /*target_i=*/5.0);
    fresh.last_target_update = std::chrono::steady_clock::now();
    fresh.last_telemetry = std::chrono::steady_clock::now();
    hw_epoch->set_status_override(1, fresh);
    OcppAdapter::TestHook::apply_power_plan(adapter_epoch);
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter_epoch));
        if (OcppAdapter::TestHook::sessions(adapter_epoch).find(1) ==
            OcppAdapter::TestHook::sessions(adapter_epoch).end()) {
            std::cerr << "gc_precharge_no_timeout_tests failed: stale GC close timeout carried into new session\n";
            return 1;
        }
    }

    std::cout << "gc_precharge_no_timeout_tests passed\n";
    return 0;
}
