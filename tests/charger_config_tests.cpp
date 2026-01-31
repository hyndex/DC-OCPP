#include "charger_config.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

using namespace charger;
namespace fs = std::filesystem;

namespace {

fs::path make_temp_dir() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    auto dir = fs::temp_directory_path() / ("dc_ocpp_config_tests_" + std::to_string(now));
    fs::create_directories(dir);
    return dir;
}

fs::path write_file(const fs::path& dir, const std::string& name, const std::string& content) {
    fs::create_directories(dir);
    const auto path = dir / name;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

} // namespace

int main() {
    // Valid tie-mode config parses and backfills missing MN IDs.
    {
        const auto dir = make_temp_dir();
        const auto path = write_file(
            dir, "charger.json",
            R"JSON(
{
  "chargePoint": {
    "id": "cfg-test",
    "vendor": "TestVendor",
    "model": "TestModel",
    "firmwareVersion": "0.0.0",
    "canInterface": "can0",
    "centralSystemURI": "ws://localhost"
  },
  "ocpp": {
    "Core": { "NumberOfConnectors": 2 }
  },
  "connectors": [
    { "id": 1, "plcId": 0 },
    { "id": 2, "plcId": 1 }
  ],
  "plc": {
    "relayMode": "ties",
    "moduleRelaysEnabled": true,
    "gunRelayOwnedByPlc": false,
    "relayFeedbackAvailable": false
  },
  "planner": {
    "allowCrossSlotIslands": true,
    "tieCloseMaxDeltaV": 15,
    "switchMaxCurrentA": 1.5,
    "switchStableTimeMs": 100
  },
  "slots": [
    { "id": 1, "gunId": 1, "gc": "GC_1", "mc": "MC_1", "cw": 2, "ccw": 2,
      "modules": [ { "id": "M1_0" }, { "id": "M1_1" } ] },
    { "id": 2, "gunId": 2, "gc": "GC_2", "mc": "MC_2", "cw": 1, "ccw": 1,
      "modules": [ { "id": "M2_0" }, { "id": "M2_1" } ] }
  ]
}
)JSON");

        const auto cfg = load_charger_config(path);
        assert(cfg.plc_relay_mode == PlcRelayMode::Ties);
        assert(cfg.tie_close_max_delta_v == 15.0);
        assert(cfg.switch_max_current_a == 1.5);
        assert(cfg.switch_stable_time_ms == 100);
        assert(cfg.slots.size() == 2);
        std::set<std::string> mn_ids;
        for (const auto& slot : cfg.slots) {
            for (const auto& mod : slot.modules) {
                assert(!mod.id.empty());
                assert(!mod.mn_id.empty());
                mn_ids.insert(mod.mn_id);
            }
        }
        assert(mn_ids.size() == 4);
    }

    // CW/CCW consistency validation.
    {
        const auto dir = make_temp_dir();
        const auto path = write_file(
            dir, "charger.json",
            R"JSON(
{
  "chargePoint": { "id": "cfg-test", "centralSystemURI": "ws://localhost" },
  "ocpp": { "Core": { "NumberOfConnectors": 2 } },
  "connectors": [ { "id": 1, "plcId": 0 }, { "id": 2, "plcId": 1 } ],
  "slots": [
    { "id": 1, "gunId": 1, "cw": 2, "ccw": 0, "gc": "GC_1", "mc": "MC_1" },
    { "id": 2, "gunId": 2, "cw": 0, "ccw": 0, "gc": "GC_2", "mc": "MC_2" }
  ]
}
)JSON");
        bool threw = false;
        try {
            (void)load_charger_config(path);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        assert(threw);
    }

    // Duplicate contactor IDs.
    {
        const auto dir = make_temp_dir();
        const auto path = write_file(
            dir, "charger.json",
            R"JSON(
{
  "chargePoint": { "id": "cfg-test", "centralSystemURI": "ws://localhost" },
  "ocpp": { "Core": { "NumberOfConnectors": 2 } },
  "connectors": [ { "id": 1, "plcId": 0 }, { "id": 2, "plcId": 1 } ],
  "slots": [
    { "id": 1, "gunId": 1, "cw": 2, "ccw": 2, "gc": "GC_X", "mc": "MC_1" },
    { "id": 2, "gunId": 2, "cw": 1, "ccw": 1, "gc": "GC_X", "mc": "MC_2" }
  ]
}
)JSON");
        bool threw = false;
        try {
            (void)load_charger_config(path);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        assert(threw);
    }

    std::cout << "charger_config_tests passed\n";
    return 0;
}
