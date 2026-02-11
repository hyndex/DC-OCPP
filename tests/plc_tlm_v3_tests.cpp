#include "can_contract.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

using namespace charger;

int main() {
    // Round-trip decode/encode with CRC enabled.
    const auto data = can_contract::build_plc_tlm_v3(
        /*cp_state_enum=*/3,   // C
        /*hlc_stage=*/12,
        /*fault_reason=*/9,
        /*relay_state_mask=*/0x05,
        /*relay_fault_mask=*/0x02,
        /*safety_ok=*/true,
        /*estop=*/false,
        /*earth_fault=*/true,
        /*comm_fault=*/true,
        /*lock_engaged=*/true,
        /*cable_checked=*/false,
        /*precharge_active=*/true,
        /*charge_complete=*/false,
        /*limits_rx_count_lsb=*/0xA7,
        /*ev_target_voltage_1v=*/800,
        /*ev_target_current_0p5a=*/321, // 160.5 A
        /*use_crc8=*/true);
    const auto tlm = can_contract::decode_plc_tlm_v3(data.data(), /*use_crc8=*/true);
    assert(tlm.crc_ok);
    assert(tlm.cp_state == 'C');
    assert(tlm.hlc_stage == 12);
    assert(tlm.fault_reason == 9);
    assert(tlm.relay_state_mask == 0x05);
    assert(tlm.relay_fault_mask == 0x02);
    assert(tlm.safety_ok);
    assert(!tlm.estop);
    assert(tlm.earth_fault);
    assert(tlm.comm_fault);
    assert(tlm.lock_engaged);
    assert(!tlm.cable_checked);
    assert(tlm.precharge_active);
    assert(!tlm.charge_complete);
    assert(tlm.limits_rx_count_lsb == 0xA7);
    assert(tlm.ev_target_voltage_v == 800.0);
    assert(tlm.ev_target_current_a == 160.5);

    // CRC failure should be detected when enabled.
    auto bad = data;
    bad[0] ^= 0x01;
    const auto tlm_bad = can_contract::decode_plc_tlm_v3(bad.data(), /*use_crc8=*/true);
    assert(!tlm_bad.crc_ok);

    // LSB counter extension should be wrap-safe.
    bool changed = false;
    const uint32_t extended = can_contract::extend_counter_lsb(0x000001FEu, 0x01u, &changed);
    assert(changed);
    assert(extended == 0x00000201u);
    const uint32_t same = can_contract::extend_counter_lsb(extended, 0x01u, &changed);
    assert(!changed);
    assert(same == extended);

    std::cout << "plc_tlm_v3_tests passed\n";
    return 0;
}

