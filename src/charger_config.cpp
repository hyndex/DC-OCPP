// SPDX-License-Identifier: Apache-2.0
#include "charger_config.hpp"

#include <fstream>
#include <stdexcept>
#include <set>

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
    connector.meter_sample_interval_s = connector_json.value("meterSampleIntervalSeconds", default_interval);
    return connector;
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

    for (auto& c : cfg.connectors) {
        if (c.can_interface.empty()) {
            c.can_interface = cfg.can_interface;
        }
    }

    if (cfg.connectors.empty()) {
        // Provide at least connector 1 to keep libocpp happy
        cfg.connectors.push_back(ConnectorConfig{});
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
