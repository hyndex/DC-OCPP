// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"

#include <fstream>
#include <stdexcept>
#include <set>
#include <algorithm>
#include <cctype>
#include <limits>

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

ConnectorConfig parse_connector(const nlohmann::json& connector_json, int default_interval) {
    ConnectorConfig connector;
    connector.id = connector_json.value("id", 1);
    connector.label = connector_json.value("label", "");
    connector.plc_id = connector_json.value("plcId", connector.id - 1);
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
    cfg.central_system_uri = cp.value("centralSystemURI", "");
    cfg.use_plc = cp.value("usePLC", false);
    cfg.can_interface = cp.value("canInterface", "can0");
    const auto site_limits = json.value("siteLimits", nlohmann::json::object());
    cfg.module_power_kw = json.value("modulePowerKW", 30.0);
    cfg.grid_limit_kw = json.value("gridLimitKW", site_limits.value("gridPowerLimitKW", 1000.0));
    cfg.default_voltage_v = json.value("defaultVoltageV", site_limits.value("defaultVoltageV", 800.0));
    if (cfg.module_power_kw <= 0.0) {
        cfg.module_power_kw = 30.0;
    }
    if (cfg.grid_limit_kw <= 0.0) {
        cfg.grid_limit_kw = std::numeric_limits<double>::max();
    }
    if (cfg.default_voltage_v <= 0.0) {
        cfg.default_voltage_v = 800.0;
    }

    cfg.ocpp_config = make_absolute(base_dir, json.value("ocppConfig", "configs/ocpp16-config.json"));
    cfg.share_path = make_absolute(base_dir, json.value("sharePath", "libocpp/config/v16"));
    cfg.user_config = make_absolute(base_dir, json.value("userConfig", "data/user_config.json"));
    cfg.database_dir = make_absolute(base_dir, json.value("databaseDir", "data/db"));
    cfg.sql_migrations = make_absolute(base_dir, json.value("sqlMigrationsPath", "libocpp/config/v16/core_migrations"));
    cfg.message_log_path = make_absolute(base_dir, json.value("messageLogPath", "logs"));
    cfg.logging_config = make_absolute(base_dir, json.value("loggingConfig", "libocpp/config/logging.ini"));

    cfg.meter_sample_interval_s = json.value("meterSampleIntervalSeconds", 30);

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
        if (c.meter_source != "plc" && c.meter_source != "shunt") {
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

    // PLC validation: unique PLC IDs and single CAN interface (current driver uses one socket)
    if (cfg.use_plc) {
        std::set<int> plc_ids;
        std::set<std::string> ifaces;
        for (const auto& c : cfg.connectors) {
            if (plc_ids.count(c.plc_id)) {
                throw std::runtime_error("Duplicate PLC id " + std::to_string(c.plc_id) +
                                         " across connectors. Use unique plcId per connector.");
            }
            plc_ids.insert(c.plc_id);
            ifaces.insert(c.can_interface);
        }
        if (ifaces.size() > 1) {
            throw std::runtime_error("PLC config invalid: multiple CAN interfaces defined. Current host driver opens a "
                                     "single SocketCAN interface; set the same canInterface for all connectors.");
        }
        if (!ifaces.empty()) {
            cfg.can_interface = *ifaces.begin();
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
    if (!fs::exists(cfg.ocpp_config)) {
        throw std::runtime_error("OCPP base config missing: " + cfg.ocpp_config.string());
    }
    std::ifstream file(cfg.ocpp_config);
    auto json = nlohmann::json::parse(file);

    json["Internal"]["ChargePointId"] = cfg.charge_point_id;
    json["Internal"]["ChargeBoxSerialNumber"] = cfg.charge_point_id;
    json["Internal"]["ChargePointModel"] = cfg.model;
    json["Internal"]["ChargePointVendor"] = cfg.vendor;
    json["Internal"]["FirmwareVersion"] = cfg.firmware_version;
    if (!cfg.central_system_uri.empty()) {
        json["Internal"]["CentralSystemURI"] = cfg.central_system_uri;
    }

    const auto connector_count = static_cast<int>(cfg.connectors.size());
    json["Core"]["NumberOfConnectors"] = connector_count;

    return json.dump();
}

} // namespace charger
