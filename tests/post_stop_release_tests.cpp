// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "post-stop-release-test";
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
    st.relay_closed = false;
    st.authorization_granted = true;
    st.lock_engaged = true;
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    auto now = std::chrono::steady_clock::now();
    GunStatus st = base_status();

    auto& pending = OcppAdapter::TestHook::pending_session_stop(adapter);
    auto& post_plugged = OcppAdapter::TestHook::post_stop_plugged(adapter);
    auto& post_time = OcppAdapter::TestHook::post_stop_time(adapter);
    auto& faulted = OcppAdapter::TestHook::connector_faulted(adapter);

    // Case 1: hold expired while still plugged -> release pending session stop and clear post-stop hold.
    pending[1] = "sess-expired";
    post_plugged[1] = true;
    post_time[1] = now - std::chrono::milliseconds(6000);
    faulted[1] = false;

    bool post_stop = false;
    bool pending_flag = false;
    std::optional<std::string> pending_id;
    OcppAdapter::TestHook::process_post_stop_state(adapter, 1, st, now, &post_stop, &pending_flag, &pending_id);

    if (!pending_id || *pending_id != "sess-expired") {
        std::cerr << "post_stop_release_tests failed: expected pending session stop to release after hold\n";
        return 1;
    }
    if (pending_flag) {
        std::cerr << "post_stop_release_tests failed: pending flag should be false after release\n";
        return 1;
    }
    if (post_stop) {
        std::cerr << "post_stop_release_tests failed: post-stop hold should clear after release\n";
        return 1;
    }

    // Case 2: hold not expired -> keep pending session stop and hold.
    pending[1] = "sess-active";
    post_plugged[1] = true;
    post_time[1] = now - std::chrono::milliseconds(1000);
    pending_id.reset();
    post_stop = false;
    pending_flag = false;

    OcppAdapter::TestHook::process_post_stop_state(adapter, 1, st, now, &post_stop, &pending_flag, &pending_id);

    if (pending_id) {
        std::cerr << "post_stop_release_tests failed: did not expect release before hold expiry\n";
        return 1;
    }
    if (!pending_flag) {
        std::cerr << "post_stop_release_tests failed: pending flag should remain true before hold expiry\n";
        return 1;
    }
    if (!post_stop) {
        std::cerr << "post_stop_release_tests failed: post-stop hold should remain before expiry\n";
        return 1;
    }

    std::cout << "post_stop_release_tests passed\n";
    return 0;
}
