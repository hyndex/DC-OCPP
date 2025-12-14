// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"
#include "can_plc.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {
struct Frame {
    uint32_t id{0};
    std::vector<uint8_t> data;
};

std::vector<Frame> load_frames(const std::string& path) {
    std::vector<Frame> frames;
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open frames file: " + path);
    }
    json j = json::parse(f);
    if (!j.is_array()) {
        throw std::runtime_error("Frames JSON must be an array");
    }
    for (const auto& item : j) {
        Frame frame;
        if (item.contains("id")) {
            const auto id_str = item["id"].get<std::string>();
            frame.id = std::stoul(id_str, nullptr, 16);
        } else {
            throw std::runtime_error("Frame missing id");
        }
        if (item.contains("data") && item["data"].is_array()) {
            for (const auto& b : item["data"]) {
                frame.data.push_back(static_cast<uint8_t>(b.get<int>() & 0xFF));
            }
        }
        frames.push_back(frame);
    }
    return frames;
}
} // namespace

int main(int argc, char* argv[]) {
    std::string config_path = "configs/charger.json";
    std::string frames_path;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_path = argv[++i];
        } else if ((arg == "-f" || arg == "--frames") && i + 1 < argc) {
            frames_path = argv[++i];
        }
    }
    if (frames_path.empty()) {
        std::cerr << "Usage: can_conformance_harness --frames frames.json [--config configs/charger.json]\n";
        return 1;
    }
    charger::ChargerConfig cfg = charger::load_charger_config(config_path);
    // Force PLC socket closed; we only replay frames.
    charger::PlcHardware plc(cfg, /*open_can=*/false);

    auto frames = load_frames(frames_path);
    for (const auto& f : frames) {
        plc.ingest_can_frame(f.id, f.data);
    }

    // Dump basic status per connector.
    for (const auto& c : cfg.connectors) {
        const auto st = plc.get_status(c.id);
        std::cout << "Connector " << c.id << " relay_closed=" << st.relay_closed
                  << " safety_ok=" << st.safety_ok
                  << " comm_fault=" << st.comm_fault
                  << " earth_fault=" << st.earth_fault
                  << " estop=" << st.estop
                  << " gc_welded=" << st.gc_welded
                  << " mc_welded=" << st.mc_welded
                  << " module_healthy_mask=0x" << std::hex << static_cast<int>(st.module_healthy_mask)
                  << " module_fault_mask=0x" << static_cast<int>(st.module_fault_mask) << std::dec
                  << " v=" << st.present_voltage_v.value_or(0.0)
                  << " i=" << st.present_current_a.value_or(0.0)
                  << " p=" << st.present_power_w.value_or(0.0)
                  << "\n";
    }
    return 0;
}
