// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"
#include "can_plc.hpp"
#include "hardware_sim.hpp"
#include "ocpp_adapter.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>

namespace {
std::atomic<bool> keep_running{true};

void handle_signal(int) {
    keep_running = false;
}

std::string parse_config_path(int argc, char* argv[]) {
    std::string path = "configs/charger.json";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
            path = argv[i + 1];
        }
    }
    return path;
}
} // namespace

int main(int argc, char* argv[]) {
    const auto config_path = parse_config_path(argc, argv);

    charger::ChargerConfig cfg;
    try {
        cfg = charger::load_charger_config(config_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return 1;
    }

    std::shared_ptr<charger::HardwareInterface> hardware;
    bool plc_available = false;
    try {
        if (cfg.use_plc) {
            hardware = std::make_shared<charger::PlcHardware>(cfg);
            plc_available = true;
        }
    } catch (const std::exception& e) {
        std::cerr << "PLC hardware init failed (" << e.what() << "), falling back to simulation." << std::endl;
    }
    if (!hardware) {
        hardware = std::make_shared<charger::SimulatedHardware>(cfg);
    }
    cfg.plc_backend_available = plc_available;
    charger::OcppAdapter adapter(cfg, hardware);

    if (!adapter.start()) {
        std::cerr << "Failed to start OCPP adapter" << std::endl;
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    adapter.stop();
    return 0;
}
