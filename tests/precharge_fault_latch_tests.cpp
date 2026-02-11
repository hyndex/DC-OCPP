// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "precharge-fault-latch-tests";
    cfg.connectors = {ConnectorConfig{.id = 1, .require_lock = false}};
    populate_minimal_slots(cfg);
    return cfg;
}

static GunStatus make_status(bool plugged_in) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = plugged_in;
    st.cp_state = plugged_in ? 'C' : 'A';
    st.lock_engaged = true;
    st.last_telemetry = std::chrono::steady_clock::now();
    return st;
}

int main() {
    const auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    const char* reasons[] = {"PrechargeOverCurrent", "PrechargeVoltageOvershoot"};
    for (const auto* reason : reasons) {
        OcppAdapter::TestHook::mark_local_hw_disable(adapter, 1, reason);
        OcppAdapter::TestHook::maybe_reenable_local_hw(adapter, 1, make_status(true), false, false, false);
        if (!OcppAdapter::TestHook::local_hw_disabled(adapter, 1)) {
            std::cerr << "precharge_fault_latch_tests failed: reason " << reason
                      << " re-enabled while EV remained plugged\n";
            return 1;
        }
        OcppAdapter::TestHook::maybe_reenable_local_hw(adapter, 1, make_status(false), false, false, false);
        if (OcppAdapter::TestHook::local_hw_disabled(adapter, 1)) {
            std::cerr << "precharge_fault_latch_tests failed: reason " << reason
                      << " did not clear after unplug\n";
            return 1;
        }
    }

    std::cout << "precharge_fault_latch_tests passed\n";
    return 0;
}
