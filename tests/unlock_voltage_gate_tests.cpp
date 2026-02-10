// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "unlock-gate-test";
    ConnectorConfig c{};
    c.id = 1;
    c.require_lock = true;
    cfg.connectors = {c};
    cfg.unlock_voltage_threshold_v = 60.0;
    cfg.max_modules_per_gun = 1;
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static GunStatus make_status(double present_v, bool relay_closed) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'B';
    st.lock_engaged = true;
    st.relay_closed = relay_closed;
    st.present_voltage_v = present_v;
    st.present_current_a = 0.0;
    st.last_telemetry = std::chrono::steady_clock::now();
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Case 0: block unlock when relay is closed.
    hw->set_status_override(1, make_status(0.0, true));
    if (OcppAdapter::TestHook::unlock_connector(adapter, 1) != ocpp::v16::UnlockStatus::UnlockFailed) {
        std::cerr << "Unlock gate test failed: expected UnlockFailed when relay_closed\n";
        return 1;
    }

    // Case 1: block unlock when voltage is >= 60V.
    hw->set_status_override(1, make_status(120.0, false));
    if (OcppAdapter::TestHook::unlock_connector(adapter, 1) != ocpp::v16::UnlockStatus::UnlockFailed) {
        std::cerr << "Unlock gate test failed: expected UnlockFailed when V>=60V\n";
        return 1;
    }

    // Case 2: allow unlock when voltage < 60V and relay is open.
    hw->set_status_override(1, make_status(50.0, false));
    if (OcppAdapter::TestHook::unlock_connector(adapter, 1) != ocpp::v16::UnlockStatus::Unlocked) {
        std::cerr << "Unlock gate test failed: expected Unlocked when V<60V and relay open\n";
        return 1;
    }

    std::cout << "unlock_voltage_gate_tests passed\n";
    return 0;
}

