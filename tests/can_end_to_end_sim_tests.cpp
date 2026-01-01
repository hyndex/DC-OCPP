// SPDX-License-Identifier: Apache-2.0
// End-to-end CAN simulation between controller driver (PlcHardware) and a lightweight PLC
// model to verify CRC alignment and message semantics in both directions.
#include "can_plc.hpp"

#include "can_contract.hpp"
#include "crc8.h"

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

using namespace charger;

namespace {

struct Frame {
    uint32_t id{0};
    std::array<uint8_t, 8> data{};
};

// CAN ID helpers (PLC-aware IDs include low-nibble plc_id per DBC).
constexpr uint32_t relay_control_id(int plc_id) { return 0x0300 | ((0x4u << 4) | (plc_id & 0x0F)); }
constexpr uint32_t gcmc_cmd_id(int plc_id) { return 0x0300 | ((0x9u << 4) | (plc_id & 0x0F)); }
constexpr uint32_t config_cmd_id(int plc_id) { return 0x0300 | ((0x8u << 4) | (plc_id & 0x0F)); }
constexpr uint32_t evse_limits_id(int plc_id) { return 0x0300 | ((0x0u << 4) | (plc_id & 0x0F)); }
constexpr uint32_t evse_present_id(int plc_id) { return 0x0300 | ((0x1u << 4) | (plc_id & 0x0F)); }
constexpr uint32_t relay_status_id(int plc_id) { return 0x0160 | (plc_id & 0x0F); }
constexpr uint32_t safety_status_id(int plc_id) { return 0x0190 | (plc_id & 0x0F); }
constexpr uint32_t gcmc_status_id(int plc_id) { return 0x0150 | (plc_id & 0x0F); }
constexpr uint32_t config_ack_id(int plc_id) { return 0x01A0 | (plc_id & 0x0F); }
constexpr uint32_t evdc_targets_id(int plc_id) { return 0x0210 | (plc_id & 0x0F); }

uint16_t to_deciv(double value) {
    if (value <= 0.0) return 0;
    const double scaled = value * 10.0;
    if (scaled >= 65535.0) return 65535;
    return static_cast<uint16_t>(scaled + 0.5);
}

uint16_t to_deciamp(double value) {
    if (value <= 0.0) return 0;
    const double scaled = value * 10.0;
    if (scaled >= 65535.0) return 65535;
    return static_cast<uint16_t>(scaled + 0.5);
}

void assert_crc_match(const std::array<uint8_t, 8>& data) {
    const uint8_t a = plc_crc8(data.data(), 7);
    const uint8_t b = crc8_07(data.data(), 7);
    assert(a == b);
}

// Minimal PLC model that consumes controller TX frames and emits PLC TX frames with CRC8 applied.
class FakePlc {
public:
    explicit FakePlc(int plc_id) : plc_id_(plc_id) {}

    void rx(const Frame& f) {
        if (needs_crc(f.id)) {
            const uint8_t expected = crc8_07(f.data.data(), 7);
            assert(expected == f.data[7]);
        }
        if (f.id == relay_control_id(plc_id_)) {
            handle_relay_control(f.data);
        } else if (f.id == gcmc_cmd_id(plc_id_)) {
            handle_gcmc_cmd(f.data);
        } else if (f.id == evse_limits_id(plc_id_)) {
            handle_evse_limits(f.data);
        } else if (f.id == evse_present_id(plc_id_)) {
            handle_evse_present(f.data);
        } else if (f.id == config_cmd_id(plc_id_)) {
            handle_config_cmd(f.data);
        }
    }

    std::vector<Frame> drain_tx() {
        auto out = tx_;
        tx_.clear();
        return out;
    }

private:
    int plc_id_{0};
    uint8_t last_cmd_seq_{0};
    uint8_t relay_cmd_mask_{0};
    uint8_t relay_state_mask_{0};
    bool sys_enable_{false};
    bool force_all_off_{false};
    bool safety_ok_{true};
    bool earth_fault_{false};
    bool crc_fault_{false};
    uint16_t uptime_s_{11};
    uint32_t limit_ack_count_{0};
    double present_voltage_v_{0.0};
    double present_current_a_{0.0};

    bool needs_crc(uint32_t id) const {
        return id == relay_control_id(plc_id_) || id == gcmc_cmd_id(plc_id_) ||
               id == config_cmd_id(plc_id_) || id == evse_limits_id(plc_id_) ||
               id == evse_present_id(plc_id_);
    }

    void push_frame(uint32_t id, const std::array<uint8_t, 8>& data) {
        Frame f{};
        f.id = id;
        f.data = data;
        tx_.push_back(f);
    }

    void emit_status(uint8_t fault_reason = 0, bool comm_fault = false) {
        std::array<uint8_t, 8> relay_status{};
        relay_status.fill(0);
        relay_status[0] = relay_state_mask_ & 0x07;
        if (!safety_ok_) relay_status[0] |= 0x40;
        if (comm_fault || crc_fault_) relay_status[0] |= 0x80;
        relay_status[1] = last_cmd_seq_;
        uint8_t safety_bits = 0;
        if (safety_ok_) safety_bits |= 0x10;
        if (earth_fault_) safety_bits |= 0x20;
        relay_status[2] = safety_bits;
        relay_status[3] = fault_reason;
        relay_status[4] = static_cast<uint8_t>(uptime_s_ & 0xFF);
        relay_status[5] = static_cast<uint8_t>((uptime_s_ >> 8) & 0xFF);
        relay_status[6] = 0;
        relay_status[7] = crc8_07(relay_status.data(), 7);
        assert_crc_match(relay_status);
        push_frame(relay_status_id(plc_id_), relay_status);

        std::array<uint8_t, 8> safety_status{};
        safety_status.fill(0);
        if (safety_ok_) safety_status[0] |= 0x10;
        if (earth_fault_) safety_status[0] |= 0x20;
        safety_status[7] = crc8_07(safety_status.data(), 7);
        assert_crc_match(safety_status);
        push_frame(safety_status_id(plc_id_), safety_status);

        std::array<uint8_t, 8> gcmc{};
        gcmc.fill(0);
        gcmc[0] = relay_state_mask_ & 0x07; // command bits seen
        gcmc[1] = relay_state_mask_ & 0x07; // feedback bits
        gcmc[2] = fault_reason;
        gcmc[3] = (comm_fault || crc_fault_) ? 1 : 0;
        gcmc[4] = safety_ok_ ? 0 : 1;
        gcmc[5] = last_cmd_seq_;
        gcmc[6] = last_cmd_seq_;
        gcmc[7] = crc8_07(gcmc.data(), 7);
        assert_crc_match(gcmc);
        push_frame(gcmc_status_id(plc_id_), gcmc);
    }

    void send_config_ack(uint8_t param_id, uint8_t status, uint32_t value) {
        std::array<uint8_t, 8> ack{};
        ack.fill(0);
        ack[0] = param_id;
        ack[1] = status;
        ack[2] = static_cast<uint8_t>(value & 0xFF);
        ack[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
        ack[4] = static_cast<uint8_t>((value >> 16) & 0xFF);
        ack[5] = static_cast<uint8_t>((value >> 24) & 0xFF);
        ack[6] = static_cast<uint8_t>(plc_id_ & 0x0F);
        ack[7] = crc8_07(ack.data(), 7);
        assert_crc_match(ack);
        push_frame(config_ack_id(plc_id_), ack);
    }

    void send_limit_ack() {
        send_config_ack(can_contract::kConfigParamEvseLimitAck, 0, limit_ack_count_);
    }

    void send_evdc_targets() {
        std::array<uint8_t, 8> payload{};
        payload.fill(0);
        const uint16_t tgt_v = to_deciv(present_voltage_v_);
        const uint16_t tgt_i = to_deciamp(present_current_a_);
        payload[0] = static_cast<uint8_t>(tgt_v & 0xFF);
        payload[1] = static_cast<uint8_t>((tgt_v >> 8) & 0xFF);
        payload[2] = static_cast<uint8_t>(tgt_i & 0xFF);
        payload[3] = static_cast<uint8_t>((tgt_i >> 8) & 0xFF);
        payload[4] = static_cast<uint8_t>(tgt_v & 0xFF); // mirror as present V
        payload[5] = static_cast<uint8_t>((tgt_v >> 8) & 0xFF);
        payload[6] = static_cast<uint8_t>(tgt_i & 0xFF); // mirror as present I
        payload[7] = static_cast<uint8_t>((tgt_i >> 8) & 0xFF);
        push_frame(evdc_targets_id(plc_id_), payload);
    }

    void handle_relay_control(const std::array<uint8_t, 8>& data) {
        relay_cmd_mask_ = data[0] & 0x07;
        sys_enable_ = (data[0] & 0x08) != 0;
        force_all_off_ = (data[0] & 0x10) != 0;
        last_cmd_seq_ = data[1];
        const bool close = sys_enable_ && !force_all_off_ && relay_cmd_mask_;
        relay_state_mask_ = close ? static_cast<uint8_t>(relay_cmd_mask_ & 0x07) : 0;
        safety_ok_ = close;
        crc_fault_ = false;
        emit_status();
    }

    void handle_gcmc_cmd(const std::array<uint8_t, 8>& data) {
        relay_cmd_mask_ = data[0] & 0x07;
        last_cmd_seq_ = data[1];
        const bool close = relay_cmd_mask_ != 0 && !force_all_off_;
        relay_state_mask_ = close ? static_cast<uint8_t>(relay_cmd_mask_ & 0x07) : 0;
        safety_ok_ = close;
        emit_status();
    }

    void handle_evse_limits(const std::array<uint8_t, 8>& data) {
        max_voltage_deciv_ = static_cast<uint16_t>(data[0] | (data[1] << 8));
        max_current_deciv_ = static_cast<uint16_t>(data[2] | (data[3] << 8));
        max_power_decik_ = static_cast<uint16_t>(data[4] | (data[5] << 8));
        limit_ack_count_++;
        send_limit_ack();
    }

    void handle_evse_present(const std::array<uint8_t, 8>& data) {
        const uint16_t v = static_cast<uint16_t>(data[0] | (data[1] << 8));
        const uint16_t i = static_cast<uint16_t>(data[2] | (data[3] << 8));
        present_voltage_v_ = v * 0.1;
        present_current_a_ = i * 0.1;
        send_evdc_targets();
    }

    void handle_config_cmd(const std::array<uint8_t, 8>& data) {
        const uint8_t param = data[0];
        const uint32_t value = static_cast<uint32_t>(data[2]) |
                               (static_cast<uint32_t>(data[3]) << 8) |
                               (static_cast<uint32_t>(data[4]) << 16) |
                               (static_cast<uint32_t>(data[5]) << 24);
        uint8_t status = 0;
        uint32_t ack_value = value;
        if (param == can_contract::kConfigParamProtoVersion) {
            ack_value = can_contract::kProtoVersion;
        }
        send_config_ack(param, status, ack_value);
    }

    uint16_t max_voltage_deciv_{0};
    uint16_t max_current_deciv_{0};
    uint16_t max_power_decik_{0};
    std::vector<Frame> tx_;
};

Frame make_evse_limits_frame(int plc_id, double v, double i, double p_kw) {
    Frame f{};
    f.id = evse_limits_id(plc_id);
    f.data.fill(0);
    const uint16_t v_deciv = to_deciv(v);
    const uint16_t i_deciv = to_deciv(i);
    const uint16_t p_decik = to_deciv(p_kw);
    f.data[0] = static_cast<uint8_t>(v_deciv & 0xFF);
    f.data[1] = static_cast<uint8_t>((v_deciv >> 8) & 0xFF);
    f.data[2] = static_cast<uint8_t>(i_deciv & 0xFF);
    f.data[3] = static_cast<uint8_t>((i_deciv >> 8) & 0xFF);
    f.data[4] = static_cast<uint8_t>(p_decik & 0xFF);
    f.data[5] = static_cast<uint8_t>((p_decik >> 8) & 0xFF);
    f.data[6] = 0;
    f.data[7] = crc8_07(f.data.data(), 7);
    assert_crc_match(f.data);
    return f;
}

Frame make_evse_present_frame(int plc_id, double v, double i, double p_kw, bool output_enabled, bool regulating) {
    Frame f{};
    f.id = evse_present_id(plc_id);
    f.data.fill(0);
    const uint16_t v_deciv = to_deciv(v);
    const uint16_t i_deciv = to_deciv(i);
    const uint16_t p_decik = to_deciv(p_kw);
    f.data[0] = static_cast<uint8_t>(v_deciv & 0xFF);
    f.data[1] = static_cast<uint8_t>((v_deciv >> 8) & 0xFF);
    f.data[2] = static_cast<uint8_t>(i_deciv & 0xFF);
    f.data[3] = static_cast<uint8_t>((i_deciv >> 8) & 0xFF);
    f.data[4] = static_cast<uint8_t>(p_decik & 0xFF);
    f.data[5] = static_cast<uint8_t>((p_decik >> 8) & 0xFF);
    uint8_t flags = 0;
    if (output_enabled) flags |= 0x01;
    if (regulating) flags |= 0x02;
    f.data[6] = flags;
    f.data[7] = crc8_07(f.data.data(), 7);
    assert_crc_match(f.data);
    return f;
}

Frame make_relay_control_frame(int plc_id, uint8_t relay_mask, uint8_t seq, bool sys_enable, bool force_all_off) {
    Frame f{};
    f.id = relay_control_id(plc_id);
    f.data.fill(0);
    f.data[0] = relay_mask & 0x07;
    if (sys_enable) f.data[0] |= 0x08;
    if (force_all_off) f.data[0] |= 0x10;
    f.data[1] = seq;
    f.data[2] = relay_mask & 0x07;
    f.data[3] = 0; // steady mode
    f.data[4] = 0; // pulse ms L
    f.data[5] = 0; // pulse ms H
    f.data[6] = 0;
    f.data[7] = crc8_07(f.data.data(), 7);
    assert_crc_match(f.data);
    return f;
}

} // namespace

int main() {
    // Verify CRC implementations are identical between controller and PLC code.
    {
        std::array<uint8_t, 8> sample{{0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0x00}};
        sample[7] = crc8_07(sample.data(), 7);
        assert_crc_match(sample);
    }

    const int plc_id = 2;
    ChargerConfig cfg{};
    cfg.plc_use_crc8 = true;
    cfg.plc_module_relays_enabled = true;
    cfg.plc_owns_gun_relay = true;
    ConnectorConfig c{};
    c.id = 1;
    c.plc_id = plc_id;
    c.require_lock = false;
    cfg.connectors.push_back(c);

    FakePlc sim(plc_id);
    PlcHardware controller(cfg, /*open_can=*/false);

    auto deliver = [&](const Frame& f) {
        sim.rx(f);
        for (const auto& tx : sim.drain_tx()) {
            controller.ingest_can_frame(tx.id, tx.data.data(), tx.data.size());
        }
    };

    const auto limits = make_evse_limits_frame(plc_id, 920.0, 300.0, 220.0);
    deliver(limits); // triggers ConfigAck with CRC8

    const auto present = make_evse_present_frame(plc_id, 400.0, 80.0, 32.0, true, true);
    deliver(present); // triggers EVDC_Targets mirror

    const uint8_t relay_seq = 0x22;
    const auto relay_ctrl = make_relay_control_frame(plc_id, /*relay_mask=*/0x07, relay_seq, /*sys_enable=*/true,
                                                     /*force_all_off=*/false);
    deliver(relay_ctrl); // triggers RelayStatus/SafetyStatus/GCMC_Status with CRC8

    // Allow safety debounce in PlcHardware to apply pending safety state.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    {
        // Validate controller view of the PLC after the simulated exchange.
        const auto st = controller.get_status(c.id);
        assert(st.relay_closed);
        assert(st.safety_ok);
        assert(!st.comm_fault);
        assert(st.present_voltage_v.has_value());
        assert(st.present_voltage_v.value() > 390.0);
        assert(st.present_current_a.has_value());
        assert(st.present_current_a.value() > 70.0);
    }
    {
        const auto internal = PlcHardware::TestHook::status(controller, c.id);
        assert(internal.limit_ack_count == 1);
        assert(internal.last_cmd_seq_applied == relay_seq);
        assert(!PlcHardware::TestHook::crc_mode_mismatch(controller, c.id));
        assert(!internal.safety.comm_fault);
    }

    std::cout << "can_end_to_end_sim_tests passed\n";
    return 0;
}
