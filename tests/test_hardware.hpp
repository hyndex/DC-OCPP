// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"
#include "hardware_interface.hpp"

#include <map>
#include <mutex>

namespace charger {

class TestHardware final : public HardwareInterface {
public:
    explicit TestHardware(const ChargerConfig& cfg) {
        for (const auto& c : cfg.connectors) {
            GunStatus st{};
            st.safety_ok = true;
            st.lock_engaged = true;
            st.cp_state = 'A';
            status_[c.id] = st;
            energy_wh_[c.id] = 0.0;
        }
    }

    void set_status_override(std::int32_t connector, const GunStatus& st) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_[connector] = st;
    }

    bool enable(std::int32_t connector) override {
        std::lock_guard<std::mutex> lock(mutex_);
        status_[connector].relay_closed = true;
        return true;
    }

    bool disable(std::int32_t connector) override {
        std::lock_guard<std::mutex> lock(mutex_);
        status_[connector].relay_closed = false;
        return true;
    }

    bool pause_charging(std::int32_t connector) override { return disable(connector); }
    bool resume_charging(std::int32_t connector) override { return enable(connector); }

    bool stop_transaction(std::int32_t connector, ocpp::v16::Reason) override { return disable(connector); }

    ocpp::v16::UnlockStatus unlock(std::int32_t) override {
        return ocpp::v16::UnlockStatus::Unlocked;
    }

    ocpp::v16::ReservationStatus reserve(std::int32_t, std::int32_t, ocpp::DateTime,
                                         const std::string&, const std::optional<std::string>&) override {
        return ocpp::v16::ReservationStatus::Accepted;
    }

    bool cancel_reservation(std::int32_t) override { return true; }

    ocpp::v16::GetLogResponse upload_diagnostics(const ocpp::v16::GetDiagnosticsRequest&) override {
        ocpp::v16::GetLogResponse resp{};
        resp.status = ocpp::v16::LogStatusEnumType::Rejected;
        return resp;
    }

    ocpp::v16::GetLogResponse upload_logs(const ocpp::v16::GetLogRequest&) override {
        ocpp::v16::GetLogResponse resp{};
        resp.status = ocpp::v16::LogStatusEnumType::Rejected;
        return resp;
    }

    bool update_firmware(const ocpp::v16::UpdateFirmwareRequest&) override { return false; }

    ocpp::v16::UpdateFirmwareStatusEnumType
    update_firmware_signed(const ocpp::v16::SignedUpdateFirmwareRequest&) override {
        return ocpp::v16::UpdateFirmwareStatusEnumType::Rejected;
    }

    void set_connection_timeout(std::int32_t) override {}
    bool is_reset_allowed(const ocpp::v16::ResetType&) override { return true; }
    void reset(const ocpp::v16::ResetType&) override {}

    void on_remote_start_token(const std::string&, const std::vector<std::int32_t>&, bool) override {}

    ocpp::Measurement sample_meter(std::int32_t connector) override {
        ocpp::Measurement m{};
        std::lock_guard<std::mutex> lock(mutex_);
        m.power_meter.energy_Wh_import.total = energy_wh_[connector];
        return m;
    }

    GunStatus get_status(std::int32_t connector) override {
        std::lock_guard<std::mutex> lock(mutex_);
        return status_[connector];
    }

    void set_authorization_state(std::int32_t connector, AuthorizationState state) override {
        std::lock_guard<std::mutex> lock(mutex_);
        status_[connector].authorization_granted = (state == AuthorizationState::Granted);
    }

    void apply_power_command(const PowerCommand&) override {}
    void apply_power_allocation(std::int32_t, int) override {}
    void set_evse_limits(std::int32_t, const EvseLimits&) override {}
    void publish_evse_present(std::int32_t, double, double, double, bool, bool) override {}
    void publish_fault_state(std::int32_t, uint8_t) override {}
    void clear_faults(std::int32_t) override {}

private:
    std::mutex mutex_;
    std::map<std::int32_t, GunStatus> status_;
    std::map<std::int32_t, double> energy_wh_;
};

} // namespace charger
