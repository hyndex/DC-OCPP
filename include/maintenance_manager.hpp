// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "charger_config.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <ocpp/common/evse_security.hpp>
#include <ocpp/common/types.hpp>
#include <ocpp/v16/messages/GetDiagnostics.hpp>
#include <ocpp/v16/messages/GetLog.hpp>
#include <ocpp/v16/messages/SignedUpdateFirmware.hpp>
#include <ocpp/v16/messages/UpdateFirmware.hpp>
#include <ocpp/v16/ocpp_enums.hpp>

namespace charger {

// Asynchronous maintenance tasks triggered via OCPP:
// - Diagnostics/Log uploads (GetDiagnostics / GetLog)
// - Firmware updates (UpdateFirmware / SignedUpdateFirmware)
//
// This manager is intentionally independent of the PLC backend; it operates on local files + HTTP(S)/FTP endpoints.
class MaintenanceManager {
public:
    struct Callbacks {
        // For GetDiagnostics: request_id is -1.
        std::function<void(int request_id, const std::string& status)> on_log_status;
        // For UpdateFirmware/SignedUpdateFirmware: request_id is -1 (v16 API in this repo).
        std::function<void(int request_id, ocpp::FirmwareStatusNotification status)> on_firmware_status;
        // Return true if any connector has an active OCPP transaction (used to gate firmware installation).
        std::function<bool()> any_active_transaction;
        // Optional override for restarting the service (used by unit tests). Return 0 on success.
        std::function<int(const std::string& systemd_service_name)> restart_service;
        // Optional override for terminating the process after install (used by unit tests).
        std::function<void(int exit_code)> exit_process;
    };

    MaintenanceManager(ChargerConfig cfg, std::shared_ptr<ocpp::EvseSecurity> evse_security, Callbacks callbacks);
    ~MaintenanceManager();

    MaintenanceManager(const MaintenanceManager&) = delete;
    MaintenanceManager& operator=(const MaintenanceManager&) = delete;

    ocpp::v16::GetLogResponse handle_get_diagnostics(const ocpp::v16::GetDiagnosticsRequest& request);
    ocpp::v16::GetLogResponse handle_get_log(const ocpp::v16::GetLogRequest& request);

    void handle_update_firmware(const ocpp::v16::UpdateFirmwareRequest& request);
    ocpp::v16::UpdateFirmwareStatusEnumType handle_signed_update_firmware(const ocpp::v16::SignedUpdateFirmwareRequest& request);

    // Test/ops helper: block until the job queue is drained (or timeout).
    bool wait_for_idle(std::chrono::milliseconds timeout);

    void shutdown();

private:
    struct Job {
        std::chrono::steady_clock::time_point run_at;
        std::function<void()> fn;
    };

    struct JobEarlier {
        bool operator()(const Job& a, const Job& b) const {
            return a.run_at > b.run_at; // min-heap by run_at
        }
    };

    void worker_loop();
    void enqueue_job(std::chrono::steady_clock::time_point run_at, std::function<void()> fn);

    ChargerConfig cfg_;
    std::shared_ptr<ocpp::EvseSecurity> evse_security_;
    Callbacks callbacks_;

    std::atomic<bool> stopping_{false};
    std::thread worker_;
    std::mutex job_mutex_;
    std::condition_variable job_cv_;
    std::condition_variable idle_cv_;
    std::priority_queue<Job, std::vector<Job>, JobEarlier> jobs_;
    int active_jobs_{0};

    std::mutex fw_mutex_;
    bool fw_job_active_{false};

    fs::path uploads_dir_;
    fs::path fw_state_path_;
};

} // namespace charger
