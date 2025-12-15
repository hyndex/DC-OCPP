// SPDX-License-Identifier: Apache-2.0
#define private public
#define protected public
#include "can_plc.hpp"
#undef private
#undef protected

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
        auto* node = plc.find_node(1);
        assert(node != nullptr);
        node->awaiting_ack = true;
        node->expected_cmd_seq = 0x0B;
        node->retry_count = 1;

        uint8_t stale[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x0A, 0x0A, 0x00};
        stale[7] = plc_crc8(stale, 7);
        const uint32_t id = 0x150 | (node->cfg.plc_id & 0x0F);
        plc.ingest_can_frame(id, stale, sizeof(stale));
        assert(node->awaiting_ack);
        assert(node->expected_cmd_seq == 0x0B);
        assert(node->status.last_cmd_seq_applied == 0x0A);

        uint8_t ack[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x0B, 0x0B, 0x00};
        ack[7] = plc_crc8(ack, 7);
        plc.ingest_can_frame(id, ack, sizeof(ack));
        assert(!node->awaiting_ack);
        assert(node->retry_count == 0);
        assert(node->status.last_cmd_seq_applied == 0x0B);
        assert(node->expected_cmd_seq == 0x0B);
    }

    // CRC/DLC enforcement for CRC-protected frames (len != 8 must fault).
    {
        PlcHardware plc(cfg, false);
        auto* node = plc.find_node(1);
        assert(node != nullptr);
        uint8_t relay_status_bad[7] = {0x03, 0x12, 0x10, 0x00, 0x2A, 0x00, 0x00}; // missing CRC byte
        const uint32_t id = 0x160 | (node->cfg.plc_id & 0x0F);
        plc.ingest_can_frame(id, relay_status_bad, sizeof(relay_status_bad));
        assert(node->crc_mode_mismatch);
        assert(node->status.safety.comm_fault);
    }

    // Golden PLC->controller parsing vectors (RELAY_STATUS, SAFETY_STATUS, GCMC_STATUS, CONFIG_ACK).
    {
        PlcHardware plc(cfg, false);
        auto* node = plc.find_node(1);
        assert(node != nullptr);
        node->protocol_version_ok = true; // avoid handshake retries in test environment
        const uint32_t plc_id = node->cfg.plc_id & 0x0F;
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
        assert(node->status.limit_ack_count == 1);
        assert(node->status.last_cmd_seq_applied == 0x12);
        assert(node->status.last_cmd_seq_received == 0x12);
    }

    std::cout << "can_protocol_vectors_tests passed\n";
    return 0;
}
