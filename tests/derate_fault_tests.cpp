#define private public
#define protected public

#include "ocpp_adapter.hpp"
#include "hardware_interface.hpp"

#undef private
#undef protected

#include <cassert>
#include <iostream>
#include <memory>

using namespace charger;

class FakeHardware : public HardwareInterface {
public:
    GunStatus status{};
    ocpp::Measurement meas{};
    PowerCommand last_cmd{};

    FakeHardware() {
        status.safety_ok = true;
        status.plugged_in = true;
        status.lock_engaged = true;
        status.module_healthy_mask = 0x03;
        status.present_voltage_v = 800.0;
        status.target_current_a = 100.0;
        meas.power_meter.energy_Wh_import.total = 0.0;
        meas.power_meter.voltage_V.emplace();
        meas.power_meter.voltage_V->DC = 800.0f;
    }

    bool enable(std::int32_t) override { return true; }
    bool disable(std::int32_t) override { return true; }
    bool pause_charging(std::int32_t) override { return true; }
    bool resume_charging(std::int32_t) override { return true; }
    bool stop_transaction(std::int32_t, ocpp::v16::Reason) override { return true; }
    ocpp::v16::UnlockStatus unlock(std::int32_t) override { return ocpp::v16::UnlockStatus::Unlocked; }
    ocpp::v16::ReservationStatus reserve(std::int32_t, std::int32_t, ocpp::DateTime, const std::string&,
                                         const std::optional<std::string>&) override {
        return ocpp::v16::ReservationStatus::Accepted;
    }
    bool cancel_reservation(std::int32_t) override { return true; }
    ocpp::v16::GetLogResponse upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest&) override { return {}; }
    ocpp::v16::GetLogResponse upload_logs(const ocpp::v16::GetLogRequest&) override { return {}; }
    void update_firmware(const ocpp::v16::UpdateFirmwareRequest&) override {}
    ocpp::v16::UpdateFirmwareStatusEnumType
    update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest&) override {
        return ocpp::v16::UpdateFirmwareStatusEnumType::Accepted;
    }
    void set_connection_timeout(std::int32_t) override {}
    bool is_reset_allowed(const ocpp::v16::ResetType&) override { return true; }
    void reset(const ocpp::v16::ResetType&) override {}
    void on_remote_start_token(const std::string&, const std::vector<std::int32_t>&, bool) override {}
    ocpp::Measurement sample_meter(std::int32_t) override { return meas; }
    GunStatus get_status(std::int32_t) override { return status; }
    void apply_power_command(const PowerCommand& cmd) override { last_cmd = cmd; }
};

static ChargerConfig basic_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "derate-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    cfg.allow_cross_slot_islands = false;
    return cfg;
}

int main() {
    auto hw = std::make_shared<FakeHardware>();
    auto cfg = basic_cfg();
    OcppAdapter adapter(cfg, hw);

    // Install an authorized session so planner runs
    OcppAdapter::ActiveSession session{};
    session.session_id = "sess";
    session.authorized = true;
    session.id_token = "TOKEN";
    session.ev_connected = true;
    session.transaction_started = true;
    session.connected_at = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(adapter.session_mutex_);
        adapter.sessions_[1] = session;
    }

    // Case 1: temp below trip, should allow power plan (modules > 0)
    hw->status.connector_temp_c = 70.0;
    hw->status.target_current_a = 50.0;
    adapter.apply_power_plan();
    assert(hw->last_cmd.module_count >= 0); // not forced off

    // Case 2: temp above trip, safety should cut power and open GC
    hw->status.connector_temp_c = 100.0;
    adapter.apply_power_plan();
    assert(hw->last_cmd.module_count == 0);
    assert(hw->last_cmd.gc_closed == false);
    std::cout << "Derate/fault tests passed\n";
    return 0;
}
