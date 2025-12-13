#define private public
#define protected public

#include "ocpp_adapter.hpp"
#include "hardware_sim.hpp"

#undef private
#undef protected

#include <cassert>
#include <iostream>

using namespace charger;

static ChargerConfig make_basic_config() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "auth-flow-test";
    cfg.connectors = {ConnectorConfig{.id = 1}, ConnectorConfig{.id = 2}};
    cfg.meter_sample_interval_s = 1;
    return cfg;
}

int main() {
    const auto now = std::chrono::steady_clock::now();
    auto cfg = make_basic_config();
    auto hw = std::make_shared<SimulatedHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Plug-first then late RemoteStart (prevalidated)
    adapter.record_presence_state(1, true, now);
    AuthToken remote1;
    remote1.id_token = "REMOTE1";
    remote1.source = AuthTokenSource::RemoteStart;
    remote1.connector_hint = 1;
    remote1.prevalidated = true;
    remote1.received_at = now;
    adapter.ingest_auth_tokens({remote1}, now);
    OcppAdapter::ActiveSession sess1{};
    sess1.session_id = "s1";
    sess1.connected_at = now;
    auto pending1 = adapter.pop_next_pending_token(1, now);
    assert(pending1.has_value());
    assert(adapter.try_authorize_with_token(1, sess1, *pending1) == AuthorizationState::Granted);
    assert(sess1.authorized);
    assert(sess1.id_token.value() == "REMOTE1");

    // RemoteStart first, plug later on hinted connector
    AuthToken remote2;
    remote2.id_token = "REMOTE2";
    remote2.source = AuthTokenSource::RemoteStart;
    remote2.connector_hint = 2;
    remote2.prevalidated = true;
    remote2.received_at = now;
    adapter.ingest_auth_tokens({remote2}, now);
    adapter.record_presence_state(2, true, now + std::chrono::seconds(1));
    OcppAdapter::ActiveSession sess2{};
    sess2.session_id = "s2";
    sess2.connected_at = now + std::chrono::seconds(1);
    auto pending2 = adapter.pop_next_pending_token(2, now + std::chrono::seconds(2));
    assert(pending2.has_value());
    assert(pending2->token.id_token == "REMOTE2");
    assert(adapter.try_authorize_with_token(2, sess2, *pending2) == AuthorizationState::Granted);

    // Autocharge rejected (not prevalidated) then RFID succeeds
    AuthToken autochg;
    autochg.id_token = "AUTOCHG1234567890ABCDEF";
    autochg.source = AuthTokenSource::Autocharge;
    autochg.prevalidated = false;
    autochg.connector_hint = 1;
    autochg.received_at = now;
    adapter.ingest_auth_tokens({autochg}, now);
    OcppAdapter::ActiveSession sess3{};
    sess3.session_id = "s3";
    sess3.connected_at = now;
    auto first = adapter.pop_next_pending_token(1, now + std::chrono::seconds(1));
    assert(first.has_value());
    // Autocharge not prevalidated and no CSMS => reject
    auto state_auto = adapter.try_authorize_with_token(1, sess3, *first);
    assert(state_auto == AuthorizationState::Pending);
    assert(!sess3.authorized);
    AuthToken rfid;
    rfid.id_token = "RFIDTAG";
    rfid.source = AuthTokenSource::RFID;
    rfid.prevalidated = true;
    rfid.connector_hint = 1;
    rfid.received_at = now + std::chrono::seconds(2);
    adapter.ingest_auth_tokens({rfid}, now + std::chrono::seconds(2));
    auto second = adapter.pop_next_pending_token(1, now + std::chrono::seconds(2));
    assert(second.has_value());
    assert(second->token.source == AuthTokenSource::RFID);
    assert(adapter.try_authorize_with_token(1, sess3, *second) == AuthorizationState::Granted);
    assert(sess3.authorized);

    // Timeout handling drops expired tokens
    AuthToken expired;
    expired.id_token = "EXPIRED";
    expired.source = AuthTokenSource::RFID;
    expired.prevalidated = true;
    expired.connector_hint = 1;
    expired.received_at = now - std::chrono::seconds(cfg.auth_wait_timeout_s + 5);
    adapter.ingest_auth_tokens({expired}, now);
    auto none = adapter.pop_next_pending_token(1, now);
    assert(!none.has_value());

    std::cout << "Auth flow tests passed\n";
    return 0;
}
