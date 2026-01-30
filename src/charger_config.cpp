// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"

#include <fstream>
#include <stdexcept>
#include <set>
#include <algorithm>
#include <cctype>
#include <limits>
#include <type_traits>
#include <cstddef>

#include <nlohmann/json.hpp>

namespace charger {

namespace {
fs::path make_absolute(const fs::path& base, const fs::path& relative_or_absolute) {
    if (relative_or_absolute.is_absolute()) {
        return relative_or_absolute;
    }
    return fs::weakly_canonical(base / relative_or_absolute);
}

void ensure_parent_dir(const fs::path& file_path) {
    const auto parent = file_path.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent);
    }
}

std::string trim_copy(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::string normalize_autocharge_source(std::string s) {
    s = trim_copy(std::move(s));
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (s == "mac") {
        return "evmac";
    }
    if (s == "evcc") {
        return "evccid";
    }
    if (s == "ema") {
        return "emaid";
    }
    if (s != "evmac" && s != "evccid" && s != "emaid") {
        return "evmac";
    }
    return s;
}

ConnectorConfig parse_connector(const nlohmann::json& connector_json, int default_interval) {
    ConnectorConfig connector;
    connector.id = connector_json.value("id", 1);
    connector.label = connector_json.value("label", "");
    connector.plc_id = connector_json.value("plcId", connector.id - 1);
    if (connector.plc_id < 0 || connector.plc_id > 15) {
        throw std::runtime_error("Connector plcId must be 0..15 (got " + std::to_string(connector.plc_id) +
                                 ") for connector id=" + std::to_string(connector.id));
    }
    connector.can_interface = connector_json.value("canInterface", "");
    connector.max_current_a = connector_json.value("maxCurrentA", 0.0);
    connector.max_power_w = connector_json.value("maxPowerW", 0.0);
    connector.max_voltage_v = connector_json.value("maxVoltageV", 0.0);
    connector.min_voltage_v = connector_json.value("minVoltageV", 0.0);
    connector.meter_sample_interval_s = connector_json.value("meterSampleIntervalSeconds", default_interval);
    connector.require_lock = connector_json.value("requireLock", true);
    connector.lock_input_switch = connector_json.value("lockInputSwitch", 3);
    connector.meter_source = connector_json.value("meterSource", "plc");
    std::transform(connector.meter_source.begin(), connector.meter_source.end(), connector.meter_source.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    connector.meter_scale = connector_json.value("meterScale", 1.0);
    connector.meter_offset_wh = connector_json.value("meterOffsetWh", 0.0);
    return connector;
}

SlotMapping parse_slot_mapping(const nlohmann::json& slot_json, int idx_fallback) {
    SlotMapping slot;
    slot.id = slot_json.value("id", idx_fallback);
    slot.gun_id = slot_json.value("gunId", slot.id);
    slot.gc_id = slot_json.value("gc", "GC_" + std::to_string(slot.id));
    slot.mc_id = slot_json.value("mc", "MC_" + std::to_string(slot.id));
    slot.cw_id = slot_json.value("cw", slot.id % 12 + 1);
    slot.ccw_id = slot_json.value("ccw", slot.id == 1 ? 12 : slot.id - 1);
    if (slot_json.contains("modules") && slot_json["modules"].is_array()) {
        for (const auto& m : slot_json["modules"]) {
            ModuleConfig mc;
            mc.id = m.value("id", "");
            mc.mn_id = m.value("mn", "");
            mc.type = m.value("type", "");
            mc.can_interface = m.value("canInterface", "");
            mc.address = m.value("address", -1);
            mc.group = m.value("group", 0);
            mc.monitor_address = m.value("monitorAddress", 1);
            mc.production_day = m.value("productionDay", 0);
            mc.serial_low = m.value("serialLow", 0);
            mc.source_address = m.value("sourceAddress", 0xA0);
            mc.input_mode = m.value("inputMode", -1);
            mc.hi_lo_mode = m.value("hiLoMode", -1);
            mc.silent_mode = m.value("silentMode", -1);
            mc.rated_power_kw = m.value("ratedPowerKW", 0.0);
            mc.rated_current_a = m.value("ratedCurrentA", 0.0);
            mc.poll_interval_ms = m.value("pollMs", 500);
            mc.cmd_interval_ms = m.value("cmdIntervalMs", 500);
            mc.telemetry_stale_ms = m.value("telemetryStaleMs", 0);
            mc.broadcast = m.value("broadcast", false);
            mc.probe_on_startup = m.value("probeOnStartup", true);
            mc.readback_limits = m.value("readbackLimits", false);
            mc.send_output_current = m.value("sendOutputCurrent", false);
            mc.send_output_power = m.value("sendOutputPower", false);
            slot.modules.push_back(mc);
        }
    }
    if (slot.modules.empty()) {
        ModuleConfig m0{"M" + std::to_string(slot.id) + "_0", "MN_" + std::to_string(slot.id) + "_0"};
        ModuleConfig m1{"M" + std::to_string(slot.id) + "_1", "MN_" + std::to_string(slot.id) + "_1"};
        slot.modules = {m0, m1};
    }
    return slot;
}

nlohmann::json load_ocpp_base_config(const ChargerConfig& cfg) {
    if (!cfg.ocpp_config_inline.empty()) {
        return nlohmann::json::parse(cfg.ocpp_config_inline);
    }
    if (!cfg.ocpp_config.empty()) {
        if (!fs::exists(cfg.ocpp_config)) {
            throw std::runtime_error("OCPP base config missing: " + cfg.ocpp_config.string());
        }
        std::ifstream file(cfg.ocpp_config);
        return nlohmann::json::parse(file);
    }
    throw std::runtime_error("OCPP base config missing: provide an inline 'ocpp' block in charger.json");
}
} // namespace

ChargerConfig load_charger_config(const fs::path& config_path) {
    if (!fs::exists(config_path)) {
        throw std::runtime_error("Config file not found: " + config_path.string());
    }

    std::ifstream file(config_path);
    const auto json = nlohmann::json::parse(file);
    const auto base_dir = config_path.parent_path().empty() ? fs::current_path() : config_path.parent_path();

    ChargerConfig cfg{};
    const auto cp = json.value("chargePoint", nlohmann::json::object());
    cfg.charge_point_id = cp.value("id", "chargepoint-1");
    cfg.vendor = cp.value("vendor", "UnknownVendor");
    cfg.model = cp.value("model", "UnknownModel");
    cfg.firmware_version = cp.value("firmwareVersion", "0.0.0");
    cfg.charge_point_serial_number = cp.value("chargePointSerialNumber", cp.value("serialNumber", ""));
    cfg.meter_serial_number = cp.value("meterSerialNumber", "");
    cfg.meter_type = cp.value("meterType", "");
    cfg.iccid = cp.value("iccid", json.value("ICCID", ""));
    cfg.imsi = cp.value("imsi", json.value("IMSI", ""));
    cfg.imei = cp.value("imei", json.value("IMEI", ""));
    cfg.apn = cp.value("apn", json.value("APN", ""));

    if (cfg.charge_point_serial_number.empty() && !cfg.imei.empty()) {
        cfg.charge_point_serial_number = cfg.imei;
    }

    const auto endpoint_override = trim_copy(cp.value("ocppEndpointToBackend", json.value("OCPPEndpointToBackend", "")));
    if (!endpoint_override.empty()) {
        std::string endpoint = endpoint_override;
        while (!endpoint.empty() && endpoint.back() == '/') endpoint.pop_back();
        const auto scheme_pos = endpoint.find("://");
        const std::size_t after_scheme = scheme_pos == std::string::npos ? 0 : scheme_pos + 3;
        const auto last_slash = endpoint.find_last_of('/');
        if (last_slash == std::string::npos || last_slash < after_scheme || last_slash + 1 >= endpoint.size()) {
            throw std::runtime_error("Invalid OCPPEndpointToBackend: " + endpoint_override);
        }
        cfg.charge_point_id = endpoint.substr(last_slash + 1);
        cfg.central_system_uri = endpoint.substr(0, last_slash);
    } else {
        cfg.central_system_uri = cp.value("centralSystemURI", "");
    }
    cfg.can_interface = cp.value("canInterface", "can0");
    const auto plc_cfg = json.value("plc", nlohmann::json::object());
    cfg.use_plc = plc_cfg.value("enabled", true);
    cfg.use_plc = cp.value("usePlc", cfg.use_plc);
    cfg.use_plc = cp.value("usePLC", cfg.use_plc);
    cfg.plc_use_crc8 = plc_cfg.value("useCRC8", true);
    cfg.plc_owns_gun_relay = plc_cfg.value("gunRelayOwnedByPlc", false);
    cfg.plc_module_relays_enabled = plc_cfg.value("moduleRelaysEnabled", true);
    cfg.plc_three_relay_mode = plc_cfg.value("threeRelayMode", false);
    cfg.plc_relay_feedback = plc_cfg.value("relayFeedbackAvailable", true);
    cfg.autocharge_id_source = normalize_autocharge_source(
        plc_cfg.value("autochargeIdSource", cfg.autocharge_id_source));
    cfg.require_https_uploads = plc_cfg.value("requireHttpsUploads", true);
    const auto uploads = json.value("uploads", nlohmann::json::object());
    cfg.upload_max_bytes = uploads.value("maxBytes", cfg.upload_max_bytes);
    cfg.upload_connect_timeout_s = uploads.value("connectTimeoutSeconds", cfg.upload_connect_timeout_s);
    cfg.upload_transfer_timeout_s = uploads.value("transferTimeoutSeconds", cfg.upload_transfer_timeout_s);
    cfg.upload_allow_file_targets = uploads.value("allowFileTargets", cfg.upload_allow_file_targets);
    const auto site_limits = json.value("siteLimits", nlohmann::json::object());
    cfg.module_power_kw = json.value("modulePowerKW", 30.0);
    cfg.grid_limit_kw = json.value("gridLimitKW", site_limits.value("gridPowerLimitKW", 1000.0));
    cfg.default_voltage_v = json.value("defaultVoltageV", site_limits.value("defaultVoltageV", 800.0));
    const auto planner = json.value("planner", nlohmann::json::object());
    auto planner_value = [&](const char* key, auto default_value) {
        using T = std::decay_t<decltype(default_value)>;
        if (planner.contains(key)) {
            return planner.value(key, default_value);
        }
        return json.value<T>(key, default_value);
    };
    const auto timeouts = json.value("timeouts", nlohmann::json::object());
    cfg.auth_wait_timeout_s = timeouts.value("authorizationSeconds", cfg.auth_wait_timeout_s);
    cfg.power_request_timeout_s = timeouts.value("powerRequestSeconds", cfg.power_request_timeout_s);
    cfg.evse_limit_ack_timeout_ms = timeouts.value("evseLimitAckMs", cfg.evse_limit_ack_timeout_ms);
    cfg.telemetry_timeout_ms = timeouts.value("telemetryTimeoutMs", cfg.telemetry_timeout_ms);
    cfg.plc_present_warn_ms = timeouts.value("plcPresentWarnMs", cfg.plc_present_warn_ms);
    cfg.plc_limits_warn_ms = timeouts.value("plcLimitsWarnMs", cfg.plc_limits_warn_ms);
    cfg.allow_cross_slot_islands = planner_value("allowCrossSlotIslands", cfg.allow_cross_slot_islands);
    cfg.max_modules_per_gun = planner_value("maxModulesPerGun", cfg.max_modules_per_gun);
    cfg.min_modules_per_active_gun = planner_value("minModulesPerActiveGun", cfg.min_modules_per_active_gun);
    cfg.max_island_radius = planner_value("maxIslandRadius", cfg.max_island_radius);
    cfg.min_module_hold_ms = planner_value("minModuleHoldMs", cfg.min_module_hold_ms);
    cfg.min_mc_hold_ms = planner_value("minMcHoldMs", cfg.min_mc_hold_ms);
    cfg.min_gc_hold_ms = planner_value("minGcHoldMs", cfg.min_gc_hold_ms);
    cfg.mc_open_current_a = planner_value("mcOpenCurrentA", cfg.mc_open_current_a);
    cfg.gc_open_current_a = planner_value("gcOpenCurrentA", cfg.gc_open_current_a);
    cfg.precharge_voltage_tolerance_v = planner_value("prechargeVoltageToleranceV", cfg.precharge_voltage_tolerance_v);
    cfg.precharge_timeout_ms = planner_value("prechargeTimeoutMs", cfg.precharge_timeout_ms);
    if (cfg.module_power_kw <= 0.0) {
        cfg.module_power_kw = 30.0;
    }
    if (cfg.grid_limit_kw <= 0.0) {
        cfg.grid_limit_kw = std::numeric_limits<double>::max();
    }
    if (cfg.default_voltage_v <= 0.0) {
        cfg.default_voltage_v = 800.0;
    }
    // authorizationSeconds == 0 disables the timeout (wait indefinitely for authorization).
    // Keep the legacy default (1800s) only when the value is negative/invalid.
    if (cfg.auth_wait_timeout_s < 0) {
        cfg.auth_wait_timeout_s = 1800;
    }
    // powerRequestSeconds == 0 disables the timeout (wait indefinitely for EV power delivery).
    // Keep the legacy default (60s) only when the value is negative/invalid.
    if (cfg.power_request_timeout_s < 0) {
        cfg.power_request_timeout_s = 60;
    }
    if (cfg.evse_limit_ack_timeout_ms <= 0) {
        cfg.evse_limit_ack_timeout_ms = 1500;
    }
    if (cfg.telemetry_timeout_ms <= 0) {
        cfg.telemetry_timeout_ms = 2000;
    }
    if (cfg.max_modules_per_gun > 2) {
        throw std::runtime_error("maxModulesPerGun > 2 is not supported by this build (PLC relay mask and telemetry "
                                 "arrays are sized for 2 modules per gun)");
    }
    if (cfg.plc_three_relay_mode) {
        cfg.allow_cross_slot_islands = false;
        cfg.max_modules_per_gun = std::min(cfg.max_modules_per_gun, 2);
        cfg.max_island_radius = 1;
    }

    if (json.contains("ocpp") && json["ocpp"].is_object()) {
        cfg.ocpp_config_inline = json["ocpp"].dump();
        const auto& ocpp_obj = json["ocpp"];
        if (ocpp_obj.contains("Core") && ocpp_obj["Core"].is_object()) {
            const auto& core = ocpp_obj["Core"];
            cfg.meter_sample_interval_s = core.value("MeterValueSampleInterval", cfg.meter_sample_interval_s);
            cfg.minimum_status_duration_s = core.value("MinimumStatusDuration", cfg.minimum_status_duration_s);
        }
    }

    const auto ocpp_config_path = json.value("ocppConfig", "");
    if (!ocpp_config_path.empty()) {
        cfg.ocpp_config = make_absolute(base_dir, ocpp_config_path);
    }
    cfg.share_path = make_absolute(base_dir, json.value("sharePath", "libocpp/config/v16"));
    cfg.user_config = make_absolute(base_dir, json.value("userConfig", "data/user_config.json"));
    cfg.database_dir = make_absolute(base_dir, json.value("databaseDir", "data/db"));
    cfg.sql_migrations = make_absolute(base_dir, json.value("sqlMigrationsPath", "libocpp/config/v16/core_migrations"));
    cfg.message_log_path = make_absolute(base_dir, json.value("messageLogPath", "logs"));
    cfg.logging_config = make_absolute(base_dir, json.value("loggingConfig", "libocpp/config/logging.ini"));

    cfg.meter_sample_interval_s = json.value("meterSampleIntervalSeconds", cfg.meter_sample_interval_s);
    cfg.meter_keepalive_s = json.value("meterKeepAliveSeconds", cfg.meter_keepalive_s);
    cfg.minimum_status_duration_s = json.value("minimumStatusDurationSeconds", cfg.minimum_status_duration_s);
    cfg.module_health_grace_ms = json.value("moduleHealthGraceMs", cfg.module_health_grace_ms);

    const auto controller = json.value("controller", nlohmann::json::object());
    cfg.free_mode = controller.value("freeMode", json.value("FreeMode", cfg.free_mode));
    cfg.default_tag = controller.value("defaultTag", json.value("DefaultTag", cfg.default_tag));

    const auto security = json.value("security", nlohmann::json::object());
    cfg.security.csms_ca_bundle = make_absolute(base_dir, security.value("csmsCaBundle", "data/certs/ca/csms/CSMS_ROOT_CA.pem"));
    cfg.security.mo_ca_bundle = make_absolute(base_dir, security.value("moCaBundle", "data/certs/ca/mo/MO_ROOT_CA.pem"));
    cfg.security.v2g_ca_bundle = make_absolute(base_dir, security.value("v2gCaBundle", "data/certs/ca/v2g/V2G_ROOT_CA.pem"));
    cfg.security.client_cert_dir = make_absolute(base_dir, security.value("clientCertDir", "data/certs/client/csms"));
    cfg.security.client_key_dir = make_absolute(base_dir, security.value("clientKeyDir", "data/certs/client/csms"));
    cfg.security.secc_cert_dir = make_absolute(base_dir, security.value("seccCertDir", "data/certs/client/cso"));
    cfg.security.secc_key_dir = make_absolute(base_dir, security.value("seccKeyDir", "data/certs/client/cso"));

    if (json.contains("connectors") && json["connectors"].is_array()) {
        for (const auto& connector_json : json["connectors"]) {
            cfg.connectors.push_back(parse_connector(connector_json, cfg.meter_sample_interval_s));
        }
    }
    if (json.contains("slots") && json["slots"].is_array()) {
        int idx = 1;
        for (const auto& slot_json : json["slots"]) {
            cfg.slots.push_back(parse_slot_mapping(slot_json, idx++));
        }
    }

    for (auto& c : cfg.connectors) {
        if (c.can_interface.empty()) {
            c.can_interface = cfg.can_interface;
        }
        if (c.lock_input_switch < 1 || c.lock_input_switch > 4) {
            c.lock_input_switch = 3;
        }
        if (c.meter_scale <= 0.0) {
            c.meter_scale = 1.0;
        }
        if (c.meter_source != "plc" && c.meter_source != "shunt" && c.meter_source != "module") {
            c.meter_source = "plc";
        }
        if (c.min_voltage_v < 0.0) {
            c.min_voltage_v = 0.0;
        }
        if (c.max_voltage_v > 0.0 && c.min_voltage_v > c.max_voltage_v) {
            c.min_voltage_v = c.max_voltage_v * 0.5;
        }
    }

    if (cfg.connectors.empty()) {
        // Provide at least connector 1 to keep libocpp happy
        cfg.connectors.push_back(ConnectorConfig{});
    }

    if (cfg.slots.empty()) {
        for (std::size_t i = 0; i < cfg.connectors.size(); ++i) {
            const auto& conn = cfg.connectors[i];
            SlotMapping sm;
            sm.id = conn.id;
            sm.gun_id = conn.id;
            sm.gc_id = "GC_" + std::to_string(conn.id);
            sm.mc_id = "MC_" + std::to_string(conn.id);
            sm.cw_id = cfg.connectors[(i + 1) % cfg.connectors.size()].id;
            sm.ccw_id = cfg.connectors[(i + cfg.connectors.size() - 1) % cfg.connectors.size()].id;
            ModuleConfig m0{"M" + std::to_string(conn.id) + "_0", "MN_" + std::to_string(conn.id) + "_0"};
            ModuleConfig m1{"M" + std::to_string(conn.id) + "_1", "MN_" + std::to_string(conn.id) + "_1"};
            sm.modules = {m0, m1};
            cfg.slots.push_back(sm);
        }
    }

    for (const auto& slot : cfg.slots) {
        if (slot.modules.size() > 2) {
            throw std::runtime_error("Slot id=" + std::to_string(slot.id) + " has " +
                                     std::to_string(slot.modules.size()) +
                                     " modules; this build supports at most 2 modules per slot");
        }
    }

    // Normalize and validate module metadata for plugin drivers.
    {
        std::set<std::tuple<std::string, int, int>> seen_addresses;
        for (auto& slot : cfg.slots) {
            for (auto& m : slot.modules) {
                if (m.can_interface.empty()) {
                    m.can_interface = cfg.can_interface;
                }
                const bool is_rectifier = (m.type == "maxwell-enr" || m.type == "enr" ||
                                           m.type == "uugreen" || m.type == "uugreenpower");
                const bool is_tonhe = (m.type == "tonhe");
                if (m.group < 0) {
                    m.group = 0;
                }
                if (is_rectifier) {
                    if (m.group > 0x0F) {
                        m.group = 0x0F;
                    }
                } else if (m.group > 60) {
                    m.group = 60;
                }
                if (m.monitor_address < 0) m.monitor_address = 1;
                if (m.monitor_address > 0x0F) m.monitor_address = 0x0F;
                if (m.production_day < 0) m.production_day = 0;
                if (m.production_day > 31) m.production_day = 0;
                if (m.serial_low < 0) m.serial_low = 0;
                if (m.serial_low > 0x1FF) m.serial_low = 0x1FF;
                if (m.source_address < 0) m.source_address = 0xA0;
                if (m.source_address > 0xFF) m.source_address = 0xA0;
                if (m.input_mode < -1 || (m.input_mode > 0 && m.input_mode != 1 && m.input_mode != 2 && m.input_mode != 3)) {
                    m.input_mode = -1;
                }
                if (is_tonhe && m.input_mode == 3) {
                    // Tonhe supports only AC/DC; map 3-phase to AC.
                    m.input_mode = 1;
                }
                if (m.hi_lo_mode < -1 || (m.hi_lo_mode > 0 && m.hi_lo_mode != 1 && m.hi_lo_mode != 2 && m.hi_lo_mode != 3)) {
                    m.hi_lo_mode = -1;
                }
                if (m.silent_mode < -1 || m.silent_mode > 2) {
                    m.silent_mode = -1;
                }
                if (!m.type.empty()) {
                    std::transform(m.type.begin(), m.type.end(), m.type.begin(),
                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    if (is_tonhe && m.broadcast) {
                        throw std::runtime_error("Module " + m.id + " uses broadcast mode, which is not supported for Tonhe");
                    }
                    if (!m.broadcast) {
                        if (is_rectifier) {
                            if (m.address < 1 || m.address > 0x7F) {
                                throw std::runtime_error("Module " + m.id + " has invalid address (expected 1-127)");
                            }
                        } else if (is_tonhe) {
                            if (m.address < 1 || m.address > 0xFF) {
                                throw std::runtime_error("Module " + m.id + " has invalid address (expected 1-255)");
                            }
                        } else {
                            if (m.address < 0 || m.address > 63) {
                                throw std::runtime_error("Module " + m.id + " has invalid address (expected 0-63)");
                            }
                        }
                    } else {
                        if (is_rectifier) {
                            if (m.address < 0) m.address = 0;
                            if (m.address > 0x7F) m.address = 0x7F;
                        } else {
                            // Broadcast allows 0xFE (group broadcast) or 0xFF (global)
                            if (m.address < 0) m.address = 0xFE;
                        }
                    }
                    if (m.rated_power_kw <= 0.0) {
                        m.rated_power_kw = cfg.module_power_kw;
                    }
                    if (m.rated_current_a < 0.0) {
                        m.rated_current_a = 0.0;
                    }
                    m.poll_interval_ms = std::max(100, m.poll_interval_ms);
                    m.cmd_interval_ms = std::max(100, m.cmd_interval_ms);
                    if (m.telemetry_stale_ms < 0) {
                        m.telemetry_stale_ms = 0;
                    }
                    const auto key = std::make_tuple(m.can_interface, m.group, m.address);
                    if (!m.broadcast) {
                        if (!seen_addresses.insert(key).second) {
                            throw std::runtime_error("Duplicate module CAN address " + std::to_string(m.address) +
                                                     " on interface " + m.can_interface + " group " +
                                                     std::to_string(m.group));
                        }
                    }
                }
            }
        }
    }

    // Prepare filesystem locations early so libocpp can open them
    ensure_parent_dir(cfg.user_config);
    if (!fs::exists(cfg.user_config)) {
        std::ofstream out(cfg.user_config);
        out << "{}";
    }
    fs::create_directories(cfg.database_dir);
    fs::create_directories(cfg.message_log_path);
    fs::create_directories(cfg.security.client_cert_dir);
    fs::create_directories(cfg.security.client_key_dir);
    fs::create_directories(cfg.security.secc_cert_dir);
    fs::create_directories(cfg.security.secc_key_dir);
    fs::create_directories(cfg.security.csms_ca_bundle.parent_path());
    fs::create_directories(cfg.security.mo_ca_bundle.parent_path());
    fs::create_directories(cfg.security.v2g_ca_bundle.parent_path());

    return cfg;
}

std::string load_and_patch_ocpp_config(const ChargerConfig& cfg) {
    auto json = load_ocpp_base_config(cfg);

    json["Internal"]["ChargePointId"] = cfg.charge_point_id;
    json["Internal"]["ChargeBoxSerialNumber"] = cfg.charge_point_id;
    json["Internal"]["ChargePointModel"] = cfg.model;
    json["Internal"]["ChargePointVendor"] = cfg.vendor;
    json["Internal"]["FirmwareVersion"] = cfg.firmware_version;
    if (!cfg.charge_point_serial_number.empty()) {
        json["Internal"]["ChargePointSerialNumber"] = cfg.charge_point_serial_number;
    }
    if (!cfg.meter_serial_number.empty()) {
        json["Internal"]["MeterSerialNumber"] = cfg.meter_serial_number;
    }
    if (!cfg.meter_type.empty()) {
        json["Internal"]["MeterType"] = cfg.meter_type;
    }
    if (!cfg.iccid.empty()) {
        json["Internal"]["ICCID"] = cfg.iccid;
    }
    if (!cfg.imsi.empty()) {
        json["Internal"]["IMSI"] = cfg.imsi;
    }
    if (!cfg.central_system_uri.empty()) {
        json["Internal"]["CentralSystemURI"] = cfg.central_system_uri;
    }

    const auto connector_count = static_cast<int>(cfg.connectors.size());
    json["Core"]["NumberOfConnectors"] = connector_count;
    // Keep websocket alive by default: set a ping interval only if the user didn't configure one.
    // Note: a configured value of 0 explicitly disables websocket pings (valid in libocpp).
    if (json.contains("Core") && json["Core"].is_object() && !json["Core"].contains("WebSocketPingInterval")) {
        json["Core"]["WebSocketPingInterval"] = 10;
    }
    if (cfg.minimum_status_duration_s > 0) {
        json["Core"]["MinimumStatusDuration"] = cfg.minimum_status_duration_s;
    }
    if (cfg.meter_sample_interval_s > 0) {
        json["Core"]["MeterValueSampleInterval"] = cfg.meter_sample_interval_s;
    }

    return json.dump();
}

} // namespace charger
