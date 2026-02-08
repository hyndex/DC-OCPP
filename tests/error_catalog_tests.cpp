// SPDX-License-Identifier: Apache-2.0
#include "error_catalog.hpp"

#include <iostream>

using namespace charger::errors;

int main() {
    auto comm_desc = comm_descriptor(true);
    if (std::string(comm_desc.vendor_code) != "TELEM_STALE") {
        std::cerr << "error_catalog_tests failed: expected telemetry vendor code TELEM_STALE\n";
        return 1;
    }

    auto gc_desc = descriptor(ErrorKey::GcWelded);
    if (gc_desc.ocpp_code != ocpp::v16::ChargePointErrorCode::PowerSwitchFailure) {
        std::cerr << "error_catalog_tests failed: GC welded should map to PowerSwitchFailure\n";
        return 1;
    }

    auto overtemp = local_fault_error_code("Overtemp");
    if (overtemp != ocpp::v16::ChargePointErrorCode::HighTemperature) {
        std::cerr << "error_catalog_tests failed: Overtemp should map to HighTemperature\n";
        return 1;
    }

    auto iso = local_fault_error_code("Isolation");
    if (iso != ocpp::v16::ChargePointErrorCode::GroundFailure) {
        std::cerr << "error_catalog_tests failed: Isolation should map to GroundFailure\n";
        return 1;
    }

    std::cout << "error_catalog_tests passed\n";
    return 0;
}
