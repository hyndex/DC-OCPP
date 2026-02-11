#include "can_contract.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

using namespace charger;

namespace {

uint64_t unpack_u56(const std::array<uint8_t, 8>& data) {
    uint64_t packed = 0;
    for (int i = 0; i < 7; ++i) {
        packed |= static_cast<uint64_t>(data[static_cast<std::size_t>(i)]) << (i * 8);
    }
    return packed;
}

bool nearly_equal(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

} // namespace

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
    assert(nearly_equal(tlm.ev_target_voltage_v, 800.0));
    assert(nearly_equal(tlm.ev_target_current_a, 160.5));

    // CRC failure should be detected when enabled.
    auto bad = data;
    bad[0] ^= 0x01;
    const auto tlm_bad = can_contract::decode_plc_tlm_v3(bad.data(), /*use_crc8=*/true);
    assert(!tlm_bad.crc_ok);

    // Field widths should clamp/mask as expected.
    const auto clamped = can_contract::build_plc_tlm_v3(
        /*cp_state_enum=*/0xFF,
        /*hlc_stage=*/0xFF,
        /*fault_reason=*/0xFF,
        /*relay_state_mask=*/0xFF,
        /*relay_fault_mask=*/0xFF,
        /*safety_ok=*/true,
        /*estop=*/true,
        /*earth_fault=*/true,
        /*comm_fault=*/true,
        /*lock_engaged=*/true,
        /*cable_checked=*/true,
        /*precharge_active=*/true,
        /*charge_complete=*/true,
        /*limits_rx_count_lsb=*/0xFF,
        /*ev_target_voltage_1v=*/4095,
        /*ev_target_current_0p5a=*/4095,
        /*use_crc8=*/true);
    const auto tlm_clamped = can_contract::decode_plc_tlm_v3(clamped.data(), /*use_crc8=*/true);
    assert(tlm_clamped.crc_ok);
    assert(tlm_clamped.cp_state == 'U'); // 0x7 maps to reserved/unknown
    assert(tlm_clamped.hlc_stage == 63);
    assert(tlm_clamped.fault_reason == 15);
    assert(tlm_clamped.relay_state_mask == 0x07);
    assert(tlm_clamped.relay_fault_mask == 0x07);
    assert(nearly_equal(tlm_clamped.ev_target_voltage_v, 1023.0));
    assert(nearly_equal(tlm_clamped.ev_target_current_a, 511.5));

    // CRC disabled mode should not fail decode.
    auto no_crc = can_contract::build_plc_tlm_v3(
        /*cp_state_enum=*/2,
        /*hlc_stage=*/1,
        /*fault_reason=*/0,
        /*relay_state_mask=*/0,
        /*relay_fault_mask=*/0,
        /*safety_ok=*/true,
        /*estop=*/false,
        /*earth_fault=*/false,
        /*comm_fault=*/false,
        /*lock_engaged=*/false,
        /*cable_checked=*/false,
        /*precharge_active=*/false,
        /*charge_complete=*/false,
        /*limits_rx_count_lsb=*/1,
        /*ev_target_voltage_1v=*/400,
        /*ev_target_current_0p5a=*/80,
        /*use_crc8=*/false);
    const auto tlm_no_crc = can_contract::decode_plc_tlm_v3(no_crc.data(), /*use_crc8=*/false);
    assert(tlm_no_crc.crc_ok);
    assert(tlm_no_crc.cp_state == 'B');
    assert(nearly_equal(tlm_no_crc.ev_target_voltage_v, 400.0));
    assert(nearly_equal(tlm_no_crc.ev_target_current_a, 40.0));

    // LSB counter extension should be wrap-safe.
    bool changed = false;
    const uint32_t extended = can_contract::extend_counter_lsb(0x000001FEu, 0x01u, &changed);
    assert(changed);
    assert(extended == 0x00000201u);
    const uint32_t same = can_contract::extend_counter_lsb(extended, 0x01u, &changed);
    assert(!changed);
    assert(same == extended);
    const uint32_t wrap = can_contract::extend_counter_lsb(0x0000FFFEu, 0x03u, &changed);
    assert(changed);
    assert(wrap == 0x00010003u);

    // Validate EVSE_FAST bit layout and CRC.
    const auto fast = can_contract::build_evse_fast(
        /*present_v_0p5=*/1234,
        /*present_i_0p2=*/321,
        /*present_p_0p5=*/111,
        /*output_enabled=*/true,
        /*regulating=*/false,
        /*fault_bits=*/0x2A,
        /*relay_cmd_mask=*/0x05,
        /*relay_enable_mask=*/0x07,
        /*sys_enable=*/true,
        /*force_off=*/false,
        /*clear_faults=*/true,
        /*seq=*/0x5C,
        /*use_crc8=*/true);
    assert(fast[7] == can_contract::crc8_07(fast.data(), 7));
    const uint64_t fast_packed = unpack_u56(fast);
    assert(((fast_packed >> 0) & 0x07FFu) == 1234u);
    assert(((fast_packed >> 11) & 0x07FFu) == 321u);
    assert(((fast_packed >> 22) & 0x01FFu) == 111u);
    assert(((fast_packed >> 31) & 0x3Fu) == 0x2Au);
    assert(((fast_packed >> 37) & 0x1u) == 1u);
    assert(((fast_packed >> 38) & 0x1u) == 0u);
    assert(((fast_packed >> 39) & 0x7u) == 0x5u);
    assert(((fast_packed >> 42) & 0x1u) == 1u);
    assert(((fast_packed >> 43) & 0x1u) == 0u);
    assert(((fast_packed >> 44) & 0xFFu) == 0x5Cu);
    assert(((fast_packed >> 52) & 0x7u) == 0x7u);
    assert(((fast_packed >> 55) & 0x1u) == 1u);

    // Validate EVSE_SLOW bit layout including protocol nibble.
    const auto slow = can_contract::build_evse_slow(
        /*max_v_0p5=*/1400,
        /*max_i_0p2=*/400,
        /*max_p_0p5=*/350,
        /*auth_granted=*/true,
        /*auth_pending=*/false,
        /*hlc_enable=*/true,
        /*pnc_blocked=*/false,
        /*lock_cmd=*/true,
        /*use_crc8=*/true);
    assert(slow[7] == can_contract::crc8_07(slow.data(), 7));
    const uint64_t slow_packed = unpack_u56(slow);
    assert(((slow_packed >> 0) & 0x07FFu) == 1400u);
    assert(((slow_packed >> 11) & 0x07FFu) == 400u);
    assert(((slow_packed >> 22) & 0x01FFu) == 350u);
    assert(((slow_packed >> 31) & 0x1u) == 1u);
    assert(((slow_packed >> 32) & 0x1u) == 0u);
    assert(((slow_packed >> 33) & 0x1u) == 1u);
    assert(((slow_packed >> 34) & 0x1u) == 0u);
    assert(((slow_packed >> 35) & 0x1u) == 1u);
    assert(((slow_packed >> 36) & 0x0Fu) == can_contract::PROTOCOL_VERSION);

    std::cout << "plc_tlm_v3_tests passed\n";
    return 0;
}
