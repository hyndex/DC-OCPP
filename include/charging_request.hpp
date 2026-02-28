// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "hardware_interface.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace charger {

enum class EvRequestInvalidReason : std::uint8_t {
    None = 0,
    NotPlugged,
    CommFault,
    CpFault,
    ChargeComplete,
    LockNotEngaged,
    CpNotReady,
    NotInPowerPhase,
    MissingTargetCurrent,
    MissingTargetVoltage,
    StaleTarget,
};

struct EvPowerRequest {
    bool valid{false};
    bool cp_power_ready{false};
    bool hlc_power_phase{false};
    bool hlc_precharge_phase{false};
    bool target_recent{false};
    bool target_current_present{false};
    bool target_voltage_present{false};
    double target_current_a{0.0};
    double target_voltage_v{0.0};
    EvRequestInvalidReason invalid_reason{EvRequestInvalidReason::None};
};

inline bool measurement_source_is_control_trusted(MeasurementSource src) {
    return src == MeasurementSource::Meter || src == MeasurementSource::Module ||
           src == MeasurementSource::PlcPresent;
}

inline EvPowerRequest derive_ev_power_request(const GunStatus& status, std::chrono::steady_clock::time_point now,
                                              std::chrono::milliseconds target_fresh_window, bool lock_required,
                                              std::uint8_t hlc_min_power_stage = 9,
                                              double min_current_request_a = 0.1,
                                              double min_voltage_request_v = 10.0) {
    EvPowerRequest out{};
    out.cp_power_ready = (status.cp_state == 'C' || status.cp_state == 'D');
    out.hlc_power_phase =
        status.hlc_power_ready || (status.hlc_stage >= hlc_min_power_stage && !status.hlc_charge_complete);
    out.hlc_precharge_phase =
        !status.hlc_charge_complete && status.hlc_stage > 0 && status.hlc_stage < hlc_min_power_stage;
    out.target_current_present = status.target_current_a && status.target_current_a.value() > min_current_request_a;
    out.target_voltage_present = status.target_voltage_v && status.target_voltage_v.value() > min_voltage_request_v;
    out.target_current_a = out.target_current_present ? status.target_current_a.value() : 0.0;
    out.target_voltage_v = out.target_voltage_present ? status.target_voltage_v.value() : 0.0;
    out.target_recent = status.last_target_update.time_since_epoch().count() != 0 &&
                        (now - status.last_target_update) <= target_fresh_window;

    if (!status.plugged_in) {
        out.invalid_reason = EvRequestInvalidReason::NotPlugged;
        return out;
    }
    if (status.comm_fault) {
        out.invalid_reason = EvRequestInvalidReason::CommFault;
        return out;
    }
    if (status.cp_fault) {
        out.invalid_reason = EvRequestInvalidReason::CpFault;
        return out;
    }
    if (status.hlc_charge_complete) {
        out.invalid_reason = EvRequestInvalidReason::ChargeComplete;
        return out;
    }
    if (lock_required && !status.lock_engaged) {
        out.invalid_reason = EvRequestInvalidReason::LockNotEngaged;
        return out;
    }
    if (!out.cp_power_ready) {
        out.invalid_reason = EvRequestInvalidReason::CpNotReady;
        return out;
    }
    if (!out.hlc_power_phase) {
        out.invalid_reason = EvRequestInvalidReason::NotInPowerPhase;
        return out;
    }
    if (!out.target_current_present) {
        out.invalid_reason = EvRequestInvalidReason::MissingTargetCurrent;
        return out;
    }
    if (!out.target_voltage_present) {
        out.invalid_reason = EvRequestInvalidReason::MissingTargetVoltage;
        return out;
    }
    if (!out.target_recent) {
        out.invalid_reason = EvRequestInvalidReason::StaleTarget;
        return out;
    }

    out.valid = true;
    out.invalid_reason = EvRequestInvalidReason::None;
    return out;
}

} // namespace charger
