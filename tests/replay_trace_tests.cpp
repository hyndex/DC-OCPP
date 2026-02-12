// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

using namespace charger;
namespace fs = std::filesystem;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "replay-trace-test";
    cfg.module_power_kw = 30.0;
    cfg.grid_limit_kw = 60.0;
    cfg.default_voltage_v = 800.0;
    cfg.allow_cross_slot_islands = true;
    cfg.max_modules_per_gun = 2;
    cfg.min_modules_per_active_gun = 1;
    cfg.precharge_max_current_a = 2.0;
    cfg.precharge_voltage_tolerance_v = 20.0;
    cfg.switch_max_current_a = 2.0;
    cfg.tie_close_max_delta_v = 20.0;
    cfg.switch_stable_time_ms = 0;
    cfg.min_module_hold_ms = 0;
    cfg.min_mc_hold_ms = 0;
    cfg.min_gc_hold_ms = 0;
    cfg.use_plc = true;
    cfg.plc_owns_gun_relay = false;
    cfg.plc_module_relays_enabled = true;
    cfg.plc_relay_feedback = false;
    cfg.plc_relay_mode = PlcRelayMode::Ties;
    cfg.database_dir.clear(); // keep persistence off for this unit test

    cfg.connectors = {
        ConnectorConfig{.id = 1,
                        .max_current_a = 200,
                        .max_power_w = 60000,
                        .max_voltage_v = 1000,
                        .min_voltage_v = 200,
                        .require_lock = false,
                        .meter_source = "module"},
    };
    populate_minimal_slots(cfg);
    return cfg;
}

static void seed_session(OcppAdapter& adapter, int connector) {
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = "sess-" + std::to_string(connector);
    session.id_token = "TAG";
    session.authorized = true;
    session.ev_connected = true;
    session.transaction_started = true;
    session.connected_at = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        OcppAdapter::TestHook::sessions(adapter)[connector] = session;
    }
}

static std::optional<std::string> read_file(const fs::path& p) {
    std::ifstream in(p);
    if (!in) {
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

static GunStatus parse_status(const nlohmann::json& j) {
    GunStatus st{};
    auto set_opt_double = [&](std::optional<double>& field, const char* key) {
        if (!j.contains(key)) return;
        if (j[key].is_null()) {
            field.reset();
            return;
        }
        field = j[key].get<double>();
    };

    if (j.contains("safety_ok")) st.safety_ok = j["safety_ok"].get<bool>();
    if (j.contains("plugged_in")) st.plugged_in = j["plugged_in"].get<bool>();
    if (j.contains("relay_closed")) st.relay_closed = j["relay_closed"].get<bool>();
    if (j.contains("authorization_granted")) st.authorization_granted = j["authorization_granted"].get<bool>();
    if (j.contains("lock_engaged")) st.lock_engaged = j["lock_engaged"].get<bool>();

    if (j.contains("cp_state")) {
        const auto s = j["cp_state"].get<std::string>();
        st.cp_state = !s.empty() ? s.front() : 'U';
    }
    if (j.contains("hlc_stage")) st.hlc_stage = static_cast<uint8_t>(j["hlc_stage"].get<int>());
    if (j.contains("hlc_cable_check_ok")) st.hlc_cable_check_ok = j["hlc_cable_check_ok"].get<bool>();
    if (j.contains("hlc_precharge_active")) st.hlc_precharge_active = j["hlc_precharge_active"].get<bool>();
    if (j.contains("hlc_power_ready")) st.hlc_power_ready = j["hlc_power_ready"].get<bool>();
    if (j.contains("hlc_charge_complete")) st.hlc_charge_complete = j["hlc_charge_complete"].get<bool>();

    set_opt_double(st.present_voltage_v, "present_voltage_v");
    set_opt_double(st.present_current_a, "present_current_a");
    set_opt_double(st.present_power_w, "present_power_w");
    set_opt_double(st.target_voltage_v, "target_voltage_v");
    set_opt_double(st.target_current_a, "target_current_a");

    if (j.contains("module_healthy_mask")) st.module_healthy_mask = static_cast<uint8_t>(j["module_healthy_mask"].get<int>());
    if (j.contains("module_fault_mask")) st.module_fault_mask = static_cast<uint8_t>(j["module_fault_mask"].get<int>());

    st.last_telemetry = std::chrono::steady_clock::now();
    return st;
}

static std::optional<std::string> check_expectations(const nlohmann::json& expect, const PowerCommand& cmd) {
    auto maybe_double = [&](const char* key) -> std::optional<double> {
        if (!expect.contains(key)) return std::nullopt;
        return expect[key].get<double>();
    };
    auto maybe_int = [&](const char* key) -> std::optional<int> {
        if (!expect.contains(key)) return std::nullopt;
        return expect[key].get<int>();
    };
    auto maybe_bool = [&](const char* key) -> std::optional<bool> {
        if (!expect.contains(key)) return std::nullopt;
        return expect[key].get<bool>();
    };

    const double eps = 1e-6;
    if (auto v = maybe_bool("gc_closed")) {
        if (cmd.gc_closed != *v) {
            return std::string("gc_closed mismatch");
        }
    }
    if (auto v = maybe_int("module_count")) {
        if (cmd.module_count != *v) {
            return std::string("module_count mismatch");
        }
    }
    if (auto v = maybe_double("max_current_limit_a")) {
        if (cmd.current_limit_a > *v + eps) {
            return std::string("current_limit_a above max");
        }
    }
    if (auto v = maybe_double("min_current_limit_a")) {
        if (cmd.current_limit_a + eps < *v) {
            return std::string("current_limit_a below min");
        }
    }
    if (auto v = maybe_double("max_voltage_set_v")) {
        if (cmd.voltage_set_v > *v + eps) {
            return std::string("voltage_set_v above max");
        }
    }
    if (auto v = maybe_double("min_voltage_set_v")) {
        if (cmd.voltage_set_v + eps < *v) {
            return std::string("voltage_set_v below min");
        }
    }
    return std::nullopt;
}

int main() {
    const fs::path trace_path = fs::path(__FILE__).parent_path() / "data" / "replay_ccs_basic_trace.json";
    const auto txt = read_file(trace_path);
    if (!txt) {
        std::cerr << "replay_trace_tests failed: failed to read trace file: " << trace_path << "\n";
        return 1;
    }
    nlohmann::json trace;
    try {
        trace = nlohmann::json::parse(*txt);
    } catch (const std::exception& e) {
        std::cerr << "replay_trace_tests failed: failed to parse trace JSON: " << e.what() << "\n";
        return 1;
    }
    if (!trace.is_array()) {
        std::cerr << "replay_trace_tests failed: trace root must be an array\n";
        return 1;
    }

    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);
    seed_session(adapter, 1);

    for (const auto& step : trace) {
        const std::string name = step.value("name", "<unnamed>");
        if (!step.contains("gun") || !step.contains("expect")) {
            std::cerr << "replay_trace_tests failed: step missing gun/expect: " << name << "\n";
            return 1;
        }
        GunStatus st = parse_status(step["gun"]);
        const int ticks = std::max(1, step.value("ticks", 1));
        for (int t = 0; t < ticks; ++t) {
            st.last_telemetry = std::chrono::steady_clock::now();
            hw->set_status_override(1, st);
            OcppAdapter::TestHook::apply_power_plan(adapter);
        }
        const auto cmd = hw->last_power_command(1);
        if (!cmd.has_value()) {
            std::cerr << "replay_trace_tests failed: missing power command at step " << name << "\n";
            return 1;
        }
        if (auto err = check_expectations(step["expect"], *cmd)) {
            std::cerr << "replay_trace_tests failed: " << *err << " at step " << name
                      << " cmd(module_count=" << cmd->module_count
                      << " gc_closed=" << (cmd->gc_closed ? "1" : "0")
                      << " V_set=" << cmd->voltage_set_v
                      << " I_lim=" << cmd->current_limit_a << ")\n";
            return 1;
        }
    }

    std::cout << "replay_trace_tests passed\n";
    return 0;
}
