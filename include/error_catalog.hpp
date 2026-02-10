// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include <ocpp/v16/ocpp_enums.hpp>

namespace charger::errors {

struct ErrorDescriptor {
    ocpp::v16::ChargePointErrorCode ocpp_code;
    bool is_fault;
    const char* info;
    const char* vendor_code;
};

enum class ErrorKey {
    CpFault,
    EmergencyStop,
    PlcCommFault,
    TelemetryStale,
    EarthFault,
    SafetyTrip,
    IsolationFault,
    OverTemp,
    OverCurrent,
    GcWelded,
    McWelded,
    ModulesUnavailable,
    ModuleDegraded,
    LockFault,
    MeterStale,
    GlobalFault
};

inline ErrorDescriptor descriptor(ErrorKey key) {
    switch (key) {
    case ErrorKey::CpFault:
        return {ocpp::v16::ChargePointErrorCode::OtherError, true, "CPFault", "CP_FAULT"};
    case ErrorKey::EmergencyStop:
        return {ocpp::v16::ChargePointErrorCode::OtherError, true, "EmergencyStop", "ESTOP"};
    case ErrorKey::PlcCommFault:
        return {ocpp::v16::ChargePointErrorCode::InternalError, true, "PlcCommFault", "PLC_COMM"};
    case ErrorKey::TelemetryStale:
        return {ocpp::v16::ChargePointErrorCode::InternalError, true, "TelemetryStale", "TELEM_STALE"};
    case ErrorKey::EarthFault:
        return {ocpp::v16::ChargePointErrorCode::GroundFailure, true, "EarthFault", "EARTH"};
    case ErrorKey::SafetyTrip:
        return {ocpp::v16::ChargePointErrorCode::OtherError, true, "SafetyTrip", "SAFETY"};
    case ErrorKey::IsolationFault:
        return {ocpp::v16::ChargePointErrorCode::GroundFailure, true, "IsolationFault", "ISO"};
    case ErrorKey::OverTemp:
        return {ocpp::v16::ChargePointErrorCode::HighTemperature, true, "OverTemperature", "OVERTEMP"};
    case ErrorKey::OverCurrent:
        return {ocpp::v16::ChargePointErrorCode::OverCurrentFailure, true, "OverCurrent", "OVERCURRENT"};
    case ErrorKey::GcWelded:
        return {ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true, "GCWelded", "GC_WELDED"};
    case ErrorKey::McWelded:
        return {ocpp::v16::ChargePointErrorCode::PowerSwitchFailure, true, "MCWelded", "MC_WELDED"};
    case ErrorKey::ModulesUnavailable:
        return {ocpp::v16::ChargePointErrorCode::OtherError, true, "ModulesUnavailable", "MODULES"};
    case ErrorKey::ModuleDegraded:
        return {ocpp::v16::ChargePointErrorCode::OtherError, false, "ModulesDegraded", "MODULE_DEGRADED"};
    case ErrorKey::LockFault:
        return {ocpp::v16::ChargePointErrorCode::ConnectorLockFailure, true, "LockFault", "LOCK"};
    case ErrorKey::MeterStale:
        return {ocpp::v16::ChargePointErrorCode::PowerMeterFailure, false, "MeterStale", "METER_STALE"};
    case ErrorKey::GlobalFault:
    default:
        return {ocpp::v16::ChargePointErrorCode::OtherError, true, "GlobalFault", "FAULT"};
    }
}

inline ErrorDescriptor comm_descriptor(bool telemetry_stale) {
    return telemetry_stale ? descriptor(ErrorKey::TelemetryStale) : descriptor(ErrorKey::PlcCommFault);
}

inline ErrorDescriptor global_fault_descriptor(const std::string& reason) {
    if (reason == "estop") return descriptor(ErrorKey::EmergencyStop);
    if (reason == "earth") return descriptor(ErrorKey::EarthFault);
    if (reason == "safety") return descriptor(ErrorKey::SafetyTrip);
    return descriptor(ErrorKey::GlobalFault);
}

inline ocpp::v16::ChargePointErrorCode local_fault_error_code(const std::string& reason) {
    if (reason == "GCWelded" || reason == "MCWelded" ||
        reason == "GCOpenTimeout" || reason == "GCCloseTimeout" ||
        reason == "MCOpenTimeout" || reason == "PowerDeliveryStalled" ||
        reason == "StuckVoltage" || reason == "StuckCurrent") {
        return ocpp::v16::ChargePointErrorCode::PowerSwitchFailure;
    }
    if (reason == "Isolation") {
        return ocpp::v16::ChargePointErrorCode::GroundFailure;
    }
    if (reason == "Overtemp") {
        return ocpp::v16::ChargePointErrorCode::HighTemperature;
    }
    if (reason == "Overcurrent") {
        return ocpp::v16::ChargePointErrorCode::OverCurrentFailure;
    }
    if (reason == "CommFault") {
        return ocpp::v16::ChargePointErrorCode::InternalError;
    }
    return ocpp::v16::ChargePointErrorCode::OtherError;
}

} // namespace charger::errors
