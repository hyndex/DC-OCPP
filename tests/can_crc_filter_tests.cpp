// SPDX-License-Identifier: Apache-2.0
#include "can_plc.hpp"

#include <cassert>
#include <iostream>

using namespace charger;

int main() {
    // CRC8 known vectors (poly 0x07, init 0x00)
    {
        const uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE};
        assert(plc_crc8(data, sizeof(data)) == 0xD1);
    }
    {
        const uint8_t data[] = {0, 0, 0, 0, 0, 0, 0};
        assert(plc_crc8(data, sizeof(data)) == 0x00);
    }
    {
        const uint8_t data[] = {1, 2, 3, 4, 5, 6, 7};
        assert(plc_crc8(data, sizeof(data)) == 0xD8);
    }

    // Filter spec should include extended flag and low nibble PLC id.
    constexpr uint32_t base = 0x160;
    constexpr int plc_id = 3;
    const auto spec = make_plc_rx_filter(base, plc_id);
    const uint32_t expected_id = (base & PLC_TX_MASK) | static_cast<uint32_t>(plc_id) | CAN_EFF_FLAG;
    const uint32_t expected_mask = (PLC_TX_MASK | 0x0F | CAN_EFF_FLAG) & (CAN_EFF_MASK | CAN_EFF_FLAG);
    assert(spec.id == expected_id);
    assert(spec.mask == expected_mask);
    assert((spec.id & CAN_EFF_FLAG) != 0);
    std::cout << "can_crc_filter_tests passed\n";
    return 0;
}
