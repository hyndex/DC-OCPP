// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace charger {

namespace fs = std::filesystem;

enum class PlcRelayMode {
    Ties // Relay bits: GC + tieCW + tieCCW (bus sectionalizers)
};

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
    std::string meter_source{"plc"}; // "plc" (default), "module", or "shunt"
    double meter_scale{1.0};        // Calibration multiplier for meter/shunt readings
    double meter_offset_wh{0.0};    // Calibration offset applied to imported energy
};

struct ModuleConfig {
    std::string id;
    std::string mn_id;
    std::string type;           // e.g. "maxwell-mxr", "maxwell", "maxwell-max"
    std::string can_interface;  // optional override for module CAN bus
    int address{-1};            // module address on the bus (0-63 for Maxwell)
    int group{0};               // optional module group number
    int monitor_address{1};     // optional monitor address (ENR/UUGreen)
    int production_day{0};      // optional production day (ENR/UUGreen)
    int serial_low{0};          // optional serial low part (ENR/UUGreen)
    int source_address{0xA0};   // optional source address (Tonhe J1939)
    int input_mode{-1};         // optional input mode (1=AC,2=DC,3=3-phase AC); -1 disables
    int hi_lo_mode{-1};         // optional hi/lo/auto voltage mode (1=high,2=low,3=auto)
    int silent_mode{-1};        // optional silent mode (UUGreen: 0-2)
    double rated_power_kw{0.0}; // optional per-module power rating
    double rated_current_a{0.0}; // optional per-module current rating
    int poll_interval_ms{500};
    int cmd_interval_ms{500};
    int poll_budget_fps{0};      // 0 disables budget; otherwise caps low-priority poll reads per CAN interface
    int telemetry_stale_ms{0};   // override stale telemetry threshold (0 = auto)
    bool broadcast{false};
    bool probe_on_startup{true};
    bool readback_limits{false};
    bool send_output_current{false}; // send 0x001B output current command
    bool send_output_power{false};   // send 0x0020 output power command
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
    fs::path mf_ca_bundle;
    fs::path v2g_ca_bundle;
    fs::path client_cert_dir;
    fs::path client_key_dir;
    fs::path secc_cert_dir;
    fs::path secc_key_dir;
};

struct FirmwareUpdateConfig {
    bool enabled{false};
    bool allow_unsigned{false};
    fs::path staging_dir;
    std::string systemd_service_name;
    fs::path target_binary_path;
    int max_wait_seconds{900};
};

struct CanTrafficConfig {
    double max_total_kbps_per_interface{20.0};
    int window_ms{10000};
    int bits_per_frame_estimate{150};
    int over_cap_debounce_ms{5000};
    bool enforce{true};
};

struct ChargerConfig {
    std::string charge_point_id;
    std::string vendor;
    std::string model;
    std::string firmware_version;
    std::string charge_point_serial_number;
    std::string meter_serial_number;
    std::string meter_type;
    std::string iccid;
    std::string imsi;
    std::string imei;
    std::string apn;
    std::string central_system_uri;
    std::string can_interface; // Default CAN interface for PLC nodes (e.g. "can0")
    bool use_plc{false};
    bool plc_backend_available{false}; // Set at runtime when PLC backend actually initialized
    bool plc_use_crc8{true};
    bool plc_owns_gun_relay{false}; // When true, controller will not command GC relay; PLC owns it
    bool plc_module_relays_enabled{true}; // Drive PLC auxiliary relays as tie/island contactors
    bool plc_relay3_enabled{false}; // Deprecated: single-island-contactor mode never drives Relay3
    PlcRelayMode plc_relay_mode{PlcRelayMode::Ties}; // Split charging: relay bits [1..2] drive tie contactors
    bool plc_relay_feedback{false}; // Commanded-state model; AUX relay feedback is not used
    std::string autocharge_id_source{"evmac"}; // "evmac", "evccid", or "emaid"
    bool require_https_uploads{true};
    double module_power_kw{30.0};
    double grid_limit_kw{1000.0};
    double default_voltage_v{800.0};
    // True split charging across neighboring cabinets/modules in the ring topology.
    // When enabled, the planner can expand a gun's island beyond its home slot.
    bool allow_cross_slot_islands{true};
    int max_modules_per_gun{2};
    int min_modules_per_active_gun{1};
    int max_island_radius{6};
    int min_module_hold_ms{1000};
    int min_mc_hold_ms{1000};
    int min_gc_hold_ms{500};
    double mc_open_current_a{1.0};
    double gc_open_current_a{1.0};
    double ramp_step_a{10.0};
    double planner_voltage_margin_v{2.0};
    double planner_current_margin_a{0.2};
    double planner_voltage_guard_band_v{10.0};
    double planner_ramp_up_min_a_per_s{20.0};
    double planner_ramp_up_max_a_per_s{250.0};
    double planner_ramp_down_min_a_per_s{100.0};
    double planner_ramp_down_max_a_per_s{300.0};
    double planner_ramp_down_emergency_a_per_s{350.0};
    double planner_ramp_jerk_a_per_s2{2000.0};
    double planner_ramp_response_s{0.45};
    double planner_capture_current_a{0.2};
    double planner_capture_rate_a_per_s{1.5};
    double tie_close_max_delta_v{20.0};
    double switch_max_current_a{2.0};
    int switch_stable_time_ms{200};
    // When true, do not energize DC output (precharge/warmup) until OCPP authorization is granted.
    // This may delay or prevent ISO15118 (HLC) progression on some vehicles.
    bool require_auth_for_precharge{false};
    std::size_t upload_max_bytes{100 * 1024 * 1024}; // 100 MB safety cap
    int upload_connect_timeout_s{10};
    int upload_transfer_timeout_s{60};
    bool upload_allow_file_targets{true};
    // CCS/DC: during precharge, EVSE must limit current to <= 2 A (IEC 61851-23 / CharIN guidance).
    double precharge_max_current_a{2.0};
    // CCS/DC: EV closes its internal disconnecting device when ΔV < 20 V; use this as the default close tolerance.
    double precharge_voltage_tolerance_v{20.0};
    int precharge_timeout_ms{2000};
    // CCS/DC: unlock only when HV is discharged below 60 V (or stored energy <= 0.2 J; voltage check used here).
    double unlock_voltage_threshold_v{60.0};
    int module_health_grace_ms{2000};
    int auth_wait_timeout_s{1800};
    int auth_denied_hold_s{5};
    int hlc_auth_timeout_s{150};
    int pnc_block_ttl_s{1200};
    int power_request_timeout_s{60};
    // Grace window before force-stopping long CP=B + no-output sessions while still plugged in.
    // 0 disables this stop condition (vehicle-friendly default is to wait for EV/unplug events).
    int suspended_no_output_stop_s{600};
    // Allow more time for PLCs that acknowledge EVSE limits slowly during precharge/auth phases.
    int evse_limit_ack_timeout_ms{5000};
    int telemetry_timeout_ms{2000};
    int plc_present_warn_ms{1000};
    int plc_limits_warn_ms{1500};
    int minimum_status_duration_s{0};
    int meter_keepalive_s{300};
    bool free_mode{false};
    // Lab-only: bypass certain safety checks/faults. Must be false in production.
    bool lab_bypass{false};
    std::string default_tag;
    std::string ocpp_config_inline; // Preferred inline OCPP base config JSON (single source)

    fs::path ocpp_config;
    fs::path share_path;
    fs::path user_config;
    fs::path database_dir;
    fs::path sql_migrations;
    fs::path message_log_path;
    fs::path logging_config;

    SecurityConfig security;
    FirmwareUpdateConfig firmware_update;
    CanTrafficConfig can_traffic;
    int meter_sample_interval_s{30};
    std::vector<ConnectorConfig> connectors;
    std::vector<SlotMapping> slots; // optional explicit topology map for ring/modules
};

/// \brief Load charger.json and populate a ChargerConfig with absolute paths.
ChargerConfig load_charger_config(const fs::path& config_path);

/// \brief Load the base OCPP config JSON and patch dynamic values (ids, URI, connector count).
std::string load_and_patch_ocpp_config(const ChargerConfig& cfg);

} // namespace charger
