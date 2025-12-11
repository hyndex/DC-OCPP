// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace charger {

namespace fs = std::filesystem;

struct ConnectorConfig {
    int id{1};
    int plc_id{0};                 // PLC node id (low nibble in command IDs)
    std::string can_interface;     // Optional CAN iface override per connector (e.g. "can0")
    double max_current_a{0};
    double max_power_w{0};
    int meter_sample_interval_s{0};
    std::string label;
};

struct SecurityConfig {
    fs::path csms_ca_bundle;
    fs::path mo_ca_bundle;
    fs::path v2g_ca_bundle;
    fs::path client_cert_dir;
    fs::path client_key_dir;
    fs::path secc_cert_dir;
    fs::path secc_key_dir;
};

struct ChargerConfig {
    std::string charge_point_id;
    std::string vendor;
    std::string model;
    std::string firmware_version;
    std::string central_system_uri;
    std::string can_interface; // Default CAN interface for PLC nodes (e.g. "can0")
    bool use_plc{false};

    fs::path ocpp_config;
    fs::path share_path;
    fs::path user_config;
    fs::path database_dir;
    fs::path sql_migrations;
    fs::path message_log_path;
    fs::path logging_config;

    SecurityConfig security;
    int meter_sample_interval_s{30};
    std::vector<ConnectorConfig> connectors;
};

/// \brief Load charger.json and populate a ChargerConfig with absolute paths.
ChargerConfig load_charger_config(const fs::path& config_path);

/// \brief Load the base OCPP config JSON and patch dynamic values (ids, URI, connector count).
std::string load_and_patch_ocpp_config(const ChargerConfig& cfg);

} // namespace charger
