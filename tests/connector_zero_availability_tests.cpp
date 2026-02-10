// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <cassert>
#include <iostream>
#include <mutex>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "connector-zero-availability-test";
    cfg.connectors = {ConnectorConfig{.id = 1}, ConnectorConfig{.id = 2}};
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static OcppAdapter::TestHook::ActiveSession make_session(const std::string& sid) {
    OcppAdapter::TestHook::ActiveSession s{};
    s.session_id = sid;
    s.id_token = "TAG";
    s.connected_at = std::chrono::steady_clock::now();
    s.pending_started = std::chrono::steady_clock::now();
    s.last_seen_plugged = std::chrono::steady_clock::now();
    s.transaction_started = true;
    s.authorized = true;
    s.ev_connected = true;
    return s;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Baseline: both connectors enabled state should be "not disabled".
    assert(!OcppAdapter::TestHook::evse_disabled(adapter, 1));
    assert(!OcppAdapter::TestHook::evse_disabled(adapter, 2));

    // connectorId=0 should enable all connectors.
    assert(OcppAdapter::TestHook::enable_evse(adapter, 0));
    assert(hw->get_status(1).relay_closed);
    assert(hw->get_status(2).relay_closed);
    assert(!OcppAdapter::TestHook::evse_disabled(adapter, 1));
    assert(!OcppAdapter::TestHook::evse_disabled(adapter, 2));

    // Seed sessions so disable path exercises finish_transaction for all connectors.
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        OcppAdapter::TestHook::sessions(adapter)[1] = make_session("sess-1");
        OcppAdapter::TestHook::sessions(adapter)[2] = make_session("sess-2");
    }

    // connectorId=0 should disable all connectors and end any active sessions.
    assert(OcppAdapter::TestHook::disable_evse(adapter, 0));
    assert(!hw->get_status(1).relay_closed);
    assert(!hw->get_status(2).relay_closed);
    assert(OcppAdapter::TestHook::evse_disabled(adapter, 1));
    assert(OcppAdapter::TestHook::evse_disabled(adapter, 2));
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        assert(OcppAdapter::TestHook::sessions(adapter).empty());
    }

    std::cout << "connector_zero_availability_tests passed\n";
    return 0;
}

