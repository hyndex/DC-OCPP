// SPDX-License-Identifier: Apache-2.0
#include "can_plc.hpp"

#include <cassert>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>
#include <iostream>

using namespace charger;

namespace {

ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.plc_use_crc8 = true;
    cfg.plc_module_relays_enabled = true;
    cfg.plc_owns_gun_relay = true;
    ConnectorConfig c{};
    c.id = 1;
    c.plc_id = 2;
    c.require_lock = false;
    cfg.connectors.push_back(c);
    return cfg;
}

uint8_t crc_for(std::initializer_list<uint8_t> bytes) {
    std::vector<uint8_t> buf(bytes);
    return plc_crc8(buf.data(), buf.size());
}

} // namespace

int main() {
    auto cfg = make_cfg();
    const uint32_t plc_id = static_cast<uint32_t>(cfg.connectors.at(0).plc_id & 0x0F);

    // Golden CRC vectors for controller->PLC frames (bytes0..6 -> crc in byte7).
    {
        assert(crc_for({0x09, 0x12, 0x07, 0x00, 0x00, 0x00, 0x00}) == 0xD0); // RELAY_CONTROL
        assert(crc_for({0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00}) == 0x56); // GCMC_CMD mirror
        assert(crc_for({0x15, 0x00, 0x88, 0x13, 0x00, 0x00, 0x00}) == 0xAA); // CONFIG_CMD
        assert(crc_for({0x64, 0x00, 0x2C, 0x01, 0x32, 0x00, 0x00}) == 0x42); // EVSE_LIMITS
        assert(crc_for({0xE8, 0x03, 0xFA, 0x00, 0x2C, 0x01, 0x03}) == 0x8E); // EVSE_PRESENT
    }

    // GCMC status ACK tracking must not clear awaiting_ack on stale status frames.
    {
        PlcHardware plc(cfg, false);
        PlcHardware::TestHook::set_ack_state(plc, 1, true, 0x0B, 1);

        uint8_t stale[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x0A, 0x0A, 0x00};
        stale[7] = plc_crc8(stale, 7);
        const uint32_t id = 0x150 | plc_id;
        plc.ingest_can_frame(id, stale, sizeof(stale));
        assert(PlcHardware::TestHook::awaiting_ack(plc, 1));
        assert(PlcHardware::TestHook::expected_cmd_seq(plc, 1) == 0x0B);
        assert(PlcHardware::TestHook::status(plc, 1).last_cmd_seq_applied == 0x0A);

        uint8_t ack[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x0B, 0x0B, 0x00};
        ack[7] = plc_crc8(ack, 7);
        plc.ingest_can_frame(id, ack, sizeof(ack));
        assert(!PlcHardware::TestHook::awaiting_ack(plc, 1));
        assert(PlcHardware::TestHook::retry_count(plc, 1) == 0);
        assert(PlcHardware::TestHook::status(plc, 1).last_cmd_seq_applied == 0x0B);
        assert(PlcHardware::TestHook::expected_cmd_seq(plc, 1) == 0x0B);
    }

    // CRC/DLC enforcement for CRC-protected frames (len != 8 must fault).
    {
        PlcHardware plc(cfg, false);
        uint8_t relay_status_bad[7] = {0x03, 0x12, 0x10, 0x00, 0x2A, 0x00, 0x00}; // missing CRC byte
        const uint32_t id = 0x160 | plc_id;
        plc.ingest_can_frame(id, relay_status_bad, sizeof(relay_status_bad));
        assert(PlcHardware::TestHook::crc_mode_mismatch(plc, 1));
        assert(PlcHardware::TestHook::status(plc, 1).safety.comm_fault);
    }

    // Golden PLC->controller parsing vectors (RELAY_STATUS, SAFETY_STATUS, GCMC_STATUS, CONFIG_ACK).
    {
        PlcHardware plc(cfg, false);
        PlcHardware::TestHook::set_protocol_version_ok(plc, 1, true); // avoid handshake retries in test environment
        uint8_t relay_status[8] = {0x03, 0x12, 0x10, 0x00, 0x2A, 0x00, 0x00, 0x5C};
        uint8_t safety_status[8] = {0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xB1};
        uint8_t gcmc_status[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x12, 0x12, 0x6B};
        uint8_t cfg_ack[8] = {0x5A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0xA9};
        plc.ingest_can_frame(0x160 | plc_id, relay_status, sizeof(relay_status));
        plc.ingest_can_frame(0x190 | plc_id, safety_status, sizeof(safety_status));
        plc.ingest_can_frame(0x150 | plc_id, gcmc_status, sizeof(gcmc_status));
        plc.ingest_can_frame(0x1A0 | plc_id, cfg_ack, sizeof(cfg_ack));
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        const auto st = plc.get_status(1);
        assert(st.relay_closed);
        assert(st.safety_ok);
        assert(!st.comm_fault);
        const auto internal = PlcHardware::TestHook::status(plc, 1);
        assert(internal.limit_ack_count == 1);
        assert(internal.last_cmd_seq_applied == 0x12);
        assert(internal.last_cmd_seq_received == 0x12);
    }

    // ESTOP is asserted if either ESTOP_LATCHED (bit3) or ESTOP_INPUT (bit7) is set.
    {
        PlcHardware plc(cfg, false);
        PlcHardware::TestHook::set_protocol_version_ok(plc, 1, true);

        {
            uint8_t safety_status[8] = {static_cast<uint8_t>(0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            safety_status[7] = plc_crc8(safety_status, 7);
            plc.ingest_can_frame(0x190 | plc_id, safety_status, sizeof(safety_status));
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            const auto st = plc.get_status(1);
            assert(st.estop);
        }
        {
            uint8_t safety_status[8] = {static_cast<uint8_t>(0x08), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            safety_status[7] = plc_crc8(safety_status, 7);
            plc.ingest_can_frame(0x190 | plc_id, safety_status, sizeof(safety_status));
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            const auto st = plc.get_status(1);
            assert(st.estop);
        }
    }

    // FaultReason mapping: ESTOP_INPUT_FAULT must set estop, not earth_fault.
    {
        PlcHardware plc(cfg, false);
        PlcHardware::TestHook::set_protocol_version_ok(plc, 1, true);

        uint8_t relay_status[8] = {0x01, 0x12, 0x10, 0x0C, 0x00, 0x00, 0x00, 0x00}; // fault_reason=12
        relay_status[7] = plc_crc8(relay_status, 7);
        plc.ingest_can_frame(0x160 | plc_id, relay_status, sizeof(relay_status));

        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        const auto st = plc.get_status(1);
        assert(st.estop);
        assert(!st.earth_fault);
    }

    std::cout << "can_protocol_vectors_tests passed\n";
    return 0;
}
