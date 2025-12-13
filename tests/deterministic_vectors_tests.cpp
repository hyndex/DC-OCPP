#define private public
#define protected public

#include "ocpp_adapter.hpp"
#include "hardware_sim.hpp"
#include "power_manager.hpp"

#undef private
#undef protected

#include <cassert>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "deterministic-tests";
    cfg.connectors = {ConnectorConfig{.id = 1}, ConnectorConfig{.id = 2}};
    cfg.meter_sample_interval_s = 1;
    cfg.auth_wait_timeout_s = 10;
    return cfg;
}

static PlannerConfig make_planner_cfg() {
    PlannerConfig pcfg{};
    pcfg.module_power_kw = 30.0;
    pcfg.grid_limit_kw = 60.0;
    pcfg.default_voltage_v = 800.0;
    pcfg.max_modules_per_gun = 2;
    pcfg.min_modules_per_active_gun = 1;
    pcfg.max_island_radius = 1;
    pcfg.allow_cross_slot_islands = false;
    return pcfg;
}

static std::vector<Slot> make_slots() {
    Slot s1{};
    s1.id = 1;
    s1.gun_id = 1;
    s1.gc_id = "GC_1";
    s1.mc_id = "MC_1";
    s1.modules = {"M1_0", "M1_1"};
    Slot s2{};
    s2.id = 2;
    s2.gun_id = 2;
    s2.gc_id = "GC_2";
    s2.mc_id = "MC_2";
    s2.modules = {"M2_0", "M2_1"};
    return {s1, s2};
}

int main() {
    const auto now = std::chrono::steady_clock::now();
    auto cfg = make_cfg();
    auto hw = std::make_shared<SimulatedHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // RemoteStart rejection when no connector is plugged.
    auto rs_none = adapter.evaluate_remote_start_acceptance("TOKEN", {});
    assert(rs_none == ocpp::v16::RemoteStartStopStatus::Rejected);

    // Plug in connector 1 and ensure acceptance.
    hw->set_plugged_in(1, true, true);
    auto rs_plugged = adapter.evaluate_remote_start_acceptance("TOKEN", {1});
    assert(rs_plugged == ocpp::v16::RemoteStartStopStatus::Accepted);

    // Comm fault should force rejection even if plugged.
    SimulatedHardware::FaultOverride fault{};
    fault.comm_fault = true;
    hw->set_fault_override(1, fault);
    auto rs_commfault = adapter.evaluate_remote_start_acceptance("TOKEN", {1});
    assert(rs_commfault == ocpp::v16::RemoteStartStopStatus::Rejected);
    hw->clear_fault_override(1);

    // Autocharge pending should not authorize; RFID afterwards should grant.
    AuthToken autochg;
    autochg.id_token = "AUTO";
    autochg.source = AuthTokenSource::Autocharge;
    autochg.prevalidated = false;
    autochg.connector_hint = 1;
    autochg.received_at = now;
    adapter.ingest_auth_tokens({autochg}, now);
    OcppAdapter::ActiveSession sess{};
    sess.session_id = "sess";
    sess.connected_at = now;
    auto pending_auto = adapter.pop_next_pending_token(1, now + std::chrono::seconds(1));
    assert(pending_auto.has_value());
    auto state_auto = adapter.try_authorize_with_token(1, sess, *pending_auto);
    assert(state_auto == AuthorizationState::Pending);
    assert(!sess.authorized);
    AuthToken rfid;
    rfid.id_token = "RFIDOK";
    rfid.source = AuthTokenSource::RFID;
    rfid.prevalidated = true;
    rfid.connector_hint = 1;
    rfid.received_at = now + std::chrono::seconds(2);
    adapter.ingest_auth_tokens({rfid}, now + std::chrono::seconds(2));
    auto pending_rfid = adapter.pop_next_pending_token(1, now + std::chrono::seconds(2));
    assert(pending_rfid.has_value());
    auto state_rfid = adapter.try_authorize_with_token(1, sess, *pending_rfid);
    assert(state_rfid == AuthorizationState::Granted);
    assert(sess.authorized);

    // Module dropout planning: no healthy modules => zero assignment.
    PlannerConfig pcfg = make_planner_cfg();
    PowerManager pm(pcfg);
    auto slots = make_slots();
    pm.set_slots(slots);
    std::vector<ModuleState> modules;
    for (const auto& s : slots) {
        for (const auto& mid : s.modules) {
            ModuleState m{};
            m.id = mid;
            m.slot_id = s.id;
            m.mn_id = "MN_" + mid;
            m.healthy = false; // simulate dropout
            modules.push_back(m);
        }
    }
    pm.update_modules(modules);
    std::vector<GunState> guns;
    GunState g1{};
    g1.id = 1;
    g1.slot_id = 1;
    g1.gc_id = "GC_1";
    g1.gun_power_limit_kw = 60.0;
    g1.gun_current_limit_a = 200.0;
    g1.safety_ok = true;
    g1.ev_session_active = true;
    g1.ev_req_power_kw = 60.0;
    g1.ev_req_voltage_v = 800.0;
    guns.push_back(g1);
    pm.update_guns(guns);
    auto plan = pm.compute_plan();
    assert(!plan.guns.empty());
    assert(plan.guns.front().modules_assigned == 0);

    std::cout << "Deterministic vectors passed\n";
    return 0;
}
