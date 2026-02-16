// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <cassert>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "status-transition-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static GunStatus base_status() {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.relay_closed = true;
    st.authorization_granted = true;
    st.hlc_stage = 9;
    st.hlc_power_ready = true;
    st.lock_engaged = true;
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Case 0: CP=B noise without plug/session evidence should stay Available.
    GunStatus st = base_status();
    st.plugged_in = false;
    st.cp_state = 'B';
    st.relay_closed = false;
    st.authorization_granted = false;
    st.hlc_stage = 0;
    st.hlc_power_ready = false;
    st.hlc_precharge_active = false;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  false, // has_session
                                                  false, // tx_started
                                                  false, // authorized
                                                  false, // fault_active
                                                  false, // disabled
                                                  false, // post_stop_plugged
                                                  false, // seamless_retry_active
                                                  false  // suppress_available_event
    );
    auto state = OcppAdapter::TestHook::connector_state(adapter, 1);
    if (state != ConnectorState::Available) {
        std::cerr << "status_transition_tests failed: expected Available for CP=B noise without plug/session evidence\n";
        return 1;
    }

    // Case 1: constrained but relay closed should remain Charging (avoid false SuspendedEVSE).
    st = base_status();
    OcppAdapter::TestHook::power_constrained(adapter)[1] = true;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  true,  // has_session
                                                  true,  // tx_started
                                                  true,  // authorized
                                                  false, // fault_active
                                                  false, // disabled
                                                  false, // post_stop_plugged
                                                  false, // seamless_retry_active
                                                  false  // suppress_available_event
    );
    state = OcppAdapter::TestHook::connector_state(adapter, 1);
    if (state != ConnectorState::Charging) {
        std::cerr << "status_transition_tests failed: expected Charging under constraint with relay closed\n";
        return 1;
    }

    // Case 2: CP unknown, no power-stage evidence: keep Preparing (avoid false SuspendedEV).
    st = base_status();
    st.relay_closed = false;
    st.cp_state = 'U';
    st.hlc_stage = 0;
    st.hlc_power_ready = false;
    st.target_current_a.reset();
    OcppAdapter::TestHook::power_constrained(adapter)[1] = false;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  true, true, true, false, false, false, false, false);
    state = OcppAdapter::TestHook::connector_state(adapter, 1);
    if (state != ConnectorState::Preparing) {
        std::cerr << "status_transition_tests failed: expected Preparing when CP unknown and no power-stage evidence\n";
        return 1;
    }

    // Case 3: CP unknown but HLC power stage reached: treat as SuspendedEVSE.
    st.hlc_stage = 9;
    st.hlc_power_ready = true;
    st.target_current_a = 10.0;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  true, true, true, false, false, false, false, false);
    state = OcppAdapter::TestHook::connector_state(adapter, 1);
    if (state != ConnectorState::SuspendedEVSE) {
        std::cerr << "status_transition_tests failed: expected SuspendedEVSE when HLC power stage reached\n";
        return 1;
    }

    // Case 4: HLC precharge phase should remain Preparing even after tx_started.
    st = base_status();
    st.relay_closed = false;
    st.cp_state = 'C';
    st.hlc_stage = 1;
    st.hlc_power_ready = false;
    st.hlc_precharge_active = true;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  true, true, true, false, false, false, false, false);
    state = OcppAdapter::TestHook::connector_state(adapter, 1);
    if (state != ConnectorState::Preparing) {
        std::cerr << "status_transition_tests failed: expected Preparing during precharge\n";
        return 1;
    }

    std::cout << "status_transition_tests passed\n";
    return 0;
}
