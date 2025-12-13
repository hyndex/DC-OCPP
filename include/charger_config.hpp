// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
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
    double max_voltage_v{0};
    double min_voltage_v{0};
    int meter_sample_interval_s{0};
    std::string label;
    bool require_lock{true};
    int lock_input_switch{3};       // Which PLC switch input indicates lock engaged (1-4)
    std::string meter_source{"plc"}; // "plc" (default) or "shunt"
    double meter_scale{1.0};        // Calibration multiplier for meter/shunt readings
    double meter_offset_wh{0.0};    // Calibration offset applied to imported energy
};

struct ModuleConfig {
    std::string id;
    std::string mn_id;
};

struct SlotMapping {
    int id{0};
    int gun_id{0};
    std::string gc_id;
    std::string mc_id;
    int cw_id{0};
    int ccw_id{0};
    std::vector<ModuleConfig> modules;
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
    bool simulation_mode{false}; // If true, suppress comm faults and run purely simulated hardware
    bool plc_backend_available{false}; // Set at runtime when PLC backend actually initialized
    bool plc_use_crc8{false};
    bool require_https_uploads{true};
    double module_power_kw{30.0};
    double grid_limit_kw{1000.0};
    double default_voltage_v{800.0};
    bool allow_cross_slot_islands{false};
    int max_modules_per_gun{2};
    int min_modules_per_active_gun{1};
    int max_island_radius{6};
    int min_module_hold_ms{1000};
    int min_mc_hold_ms{1000};
    int min_gc_hold_ms{500};
    double mc_open_current_a{1.0};
    double gc_open_current_a{1.0};
    std::size_t upload_max_bytes{100 * 1024 * 1024}; // 100 MB safety cap
    int upload_connect_timeout_s{10};
    int upload_transfer_timeout_s{60};
    bool upload_allow_file_targets{true};
    double precharge_voltage_tolerance_v{50.0};
    int precharge_timeout_ms{2000};
    int auth_wait_timeout_s{1800};
    int power_request_timeout_s{60};
    int evse_limit_ack_timeout_ms{1500};
    int telemetry_timeout_ms{2000};
    int minimum_status_duration_s{0};
    int meter_keepalive_s{300};
    std::string ocpp_config_inline; // Preferred inline OCPP base config JSON (single source)

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
    std::vector<SlotMapping> slots; // optional explicit topology map for ring/modules
};

/// \brief Load charger.json and populate a ChargerConfig with absolute paths.
ChargerConfig load_charger_config(const fs::path& config_path);

/// \brief Load the base OCPP config JSON and patch dynamic values (ids, URI, connector count).
std::string load_and_patch_ocpp_config(const ChargerConfig& cfg);

} // namespace charger
