// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "charge-complete-finishing-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static GunStatus base_charging_status() {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'C';
    st.relay_closed = true;
    st.authorization_granted = true;
    st.hlc_stage = 9;
    st.hlc_power_ready = true;
    st.hlc_precharge_active = false;
    st.hlc_charge_complete = false;
    st.lock_engaged = true;
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Charging -> Finishing when the EV signals charge complete.
    GunStatus st = base_charging_status();
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
    if (OcppAdapter::TestHook::connector_state(adapter, 1) != ConnectorState::Charging) {
        std::cerr << "charge_complete_finishing_available_tests failed: expected Charging\n";
        return 1;
    }

    st.hlc_charge_complete = true;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  true, true, true, false, false, false, false, false);
    if (OcppAdapter::TestHook::connector_state(adapter, 1) != ConnectorState::Finishing) {
        std::cerr << "charge_complete_finishing_available_tests failed: expected Finishing on charge_complete\n";
        return 1;
    }

    // After the transaction ends but the vehicle remains present, keep Finishing (avoid spurious Available).
    st.cp_state = 'B';
    st.relay_closed = false;
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  false, // has_session
                                                  false, // tx_started
                                                  false, // authorized
                                                  false, // fault_active
                                                  false, // disabled
                                                  true,  // post_stop_plugged
                                                  false, // seamless_retry_active
                                                  false  // suppress_available_event
    );
    if (OcppAdapter::TestHook::connector_state(adapter, 1) != ConnectorState::Finishing) {
        std::cerr << "charge_complete_finishing_available_tests failed: expected Finishing while still plugged\n";
        return 1;
    }

    // Once the vehicle is removed, return to Available.
    st.plugged_in = false;
    st.cp_state = 'A';
    st.hlc_charge_complete = false; // no finishing hint
    OcppAdapter::TestHook::update_connector_state(adapter, 1, st,
                                                  false, false, false, false, false, false, false, false);
    if (OcppAdapter::TestHook::connector_state(adapter, 1) != ConnectorState::Available) {
        std::cerr << "charge_complete_finishing_available_tests failed: expected Available after unplug\n";
        return 1;
    }

    std::cout << "charge_complete_finishing_available_tests passed\n";
    return 0;
}

