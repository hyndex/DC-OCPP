#define private public
#define protected public

#include "ocpp_adapter.hpp"
#include "hardware_sim.hpp"

#undef private
#undef protected

#include <cassert>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>

using namespace charger;

static ChargerConfig make_cfg(const std::filesystem::path& db_path) {
    ChargerConfig cfg{};
    cfg.charge_point_id = "persist-test";
    cfg.database_dir = db_path;
    cfg.connectors = {ConnectorConfig{.id = 1}, ConnectorConfig{.id = 2}};
    cfg.auth_wait_timeout_s = 60;
    cfg.meter_sample_interval_s = 1;
    return cfg;
}

int main() {
    namespace fs = std::filesystem;
    const auto temp_dir = fs::temp_directory_path() / "ocpp_pending_tokens_test";
    fs::remove_all(temp_dir);
    fs::create_directories(temp_dir);

    auto cfg = make_cfg(temp_dir);
    auto hw = std::make_shared<SimulatedHardware>(cfg);
    OcppAdapter adapter(cfg, hw);
    const auto now = std::chrono::steady_clock::now();

    // Inject two tokens and ensure they persist
    AuthToken t1;
    t1.id_token = "PERSIST1";
    t1.connector_hint = 1;
    t1.prevalidated = true;
    t1.source = AuthTokenSource::RemoteStart;
    t1.received_at = now;

    AuthToken t2;
    t2.id_token = "PERSIST2";
    t2.connector_hint = 2;
    t2.prevalidated = true;
    t2.source = AuthTokenSource::RFID;
    t2.received_at = now;

    adapter.ingest_auth_tokens({t1, t2}, now);
    adapter.persist_pending_tokens(); // force flush to disk

    // Reload adapter from disk; tokens should still be available.
    auto hw2 = std::make_shared<SimulatedHardware>(cfg);
    OcppAdapter adapter2(cfg, hw2);
    const auto load_time = now + std::chrono::seconds(5);
    auto pt1 = adapter2.pop_next_pending_token(1, load_time);
    assert(pt1.has_value());
    assert(pt1->token.id_token == "PERSIST1");
    auto pt2 = adapter2.pop_next_pending_token(2, load_time);
    assert(pt2.has_value());
    assert(pt2->token.id_token == "PERSIST2");

    // Expired tokens should be dropped on load
    nlohmann::json root;
    root["tokens"] = nlohmann::json::array();
    const auto expired_epoch = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch() - std::chrono::seconds(10)).count();
    nlohmann::json expired;
    expired["connector"] = 1;
    expired["idToken"] = "EXPIRED";
    expired["source"] = "rfid";
    expired["connectorHint"] = 1;
    expired["prevalidated"] = true;
    expired["receivedAt"] = expired_epoch - 60;
    expired["expiresAt"] = expired_epoch;
    root["tokens"].push_back(expired);
    {
        std::ofstream out(cfg.database_dir / "pending_tokens.json");
        out << root.dump(2);
    }
    OcppAdapter adapter3(cfg, hw2);
    auto expired_tok = adapter3.pop_next_pending_token(1, load_time);
    assert(!expired_tok.has_value());

    std::cout << "Pending token persistence tests passed\n";
    return 0;
}
