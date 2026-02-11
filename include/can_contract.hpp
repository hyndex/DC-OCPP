// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace charger::can_contract {

// Contract: low-bandwidth PLC<->Controller CAN protocol v3.
//
// Notes:
// - EVSE control frames (EVSE_FAST/EVSE_SLOW) reuse the legacy on-wire format but are treated as the v3 contract.
// - PLC telemetry is consolidated into a single PLC_TLM_V3 frame.
constexpr uint8_t PROTOCOL_VERSION = 3;

// CAN ID bases (29-bit extended IDs).
constexpr uint32_t RX_BASE_ID = 0x00000300u;
constexpr uint8_t RX_TYPE_SHIFT = 4;

constexpr uint32_t TX_BASE_SYS = 0x00000100u;
constexpr uint32_t TX_BASE_EVDC = 0x00000200u;
constexpr uint32_t TX_BASE_CP = 0x00000400u;
constexpr uint32_t TX_BASE_BOOT = 0x00090000u;

// BootConfig feature flags.
constexpr uint8_t FEATURE_RELAYS = 1u << 0;
constexpr uint8_t FEATURE_SAFETY = 1u << 1;
constexpr uint8_t FEATURE_METER = 1u << 2;
constexpr uint8_t FEATURE_RFID = 1u << 3;

inline uint32_t rx_id(uint8_t type_nibble, uint8_t plc_id) {
    return RX_BASE_ID | (static_cast<uint32_t>(type_nibble & 0x0Fu) << RX_TYPE_SHIFT) |
           static_cast<uint32_t>(plc_id & 0x0Fu);
}

inline uint32_t tx_id(uint32_t base_aligned, uint8_t type_nibble, uint8_t plc_id) {
    return base_aligned | (static_cast<uint32_t>(type_nibble & 0x0Fu) << 4) |
           static_cast<uint32_t>(plc_id & 0x0Fu);
}

// Controller -> PLC (RX on PLC)
inline uint32_t evse_fast_id(uint8_t plc_id) { return rx_id(0x2u, plc_id); }
inline uint32_t evse_slow_id(uint8_t plc_id) { return rx_id(0x3u, plc_id); }

// PLC -> Controller (TX on PLC)
inline uint32_t energy_meter_id(uint8_t plc_id) { return tx_id(TX_BASE_SYS, 0x7u, plc_id); }
inline uint32_t rfid_event_id(uint8_t plc_id) { return tx_id(TX_BASE_SYS, 0x8u, plc_id); }

inline uint32_t evccid_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x8u, plc_id); }
inline uint32_t evemaid0_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x6u, plc_id); }
inline uint32_t evemaid1_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x7u, plc_id); }
inline uint32_t evmac_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x4u, plc_id); }

inline uint32_t plc_tlm_v3_id(uint8_t plc_id) { return tx_id(TX_BASE_CP, 0x6u, plc_id); }
inline uint32_t boot_config_id(uint8_t plc_id) { return TX_BASE_BOOT | static_cast<uint32_t>(plc_id & 0x0Fu); }

inline uint8_t crc8_07(const uint8_t* data, std::size_t len) {
    uint8_t crc = 0x00;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>(((crc << 1) ^ 0x07u) & 0xFFu);
            } else {
                crc = static_cast<uint8_t>((crc << 1) & 0xFFu);
            }
        }
    }
    return crc;
}

struct RelayStatus {
    bool relay[3]{false, false, false};
    bool relay_fault[3]{false, false, false};
    bool safety_active{false};
    bool comm_fault{false};
    bool sw_active[4]{false, false, false, false}; // sw1..sw4
    bool estop_latched{false};
    bool safety_ok{false};
    bool earth_fault{false};
    bool estop_input{false};
    uint8_t fault_reason{0};
    uint16_t uptime_s{0};
    uint8_t tec{0};
    uint8_t rec{0};
    bool crc_ok{true};
};

struct SafetyStatus {
    bool sw_active[4]{false, false, false, false};
    bool estop_latched{false};
    bool safety_ok{false};
    bool earth_fault{false};
    bool estop_input{false};
    uint8_t policy{0};
    bool crc_ok{true};
};

struct MeterReading {
    bool meter_ok{false};
    bool comm_error{false};
    bool stale{true};
    bool overrange{false};
    bool fallback_active{false};
    double voltage_v{0.0};
    double current_a{0.0};
    double power_kw{0.0};
    double import_energy_kwh{0.0};
    double freq_hz{0.0};
    std::uint64_t seq{0};
};

struct BootConfig {
    uint8_t fw_major{0};
    uint8_t fw_minor{0};
    uint8_t fw_patch{0};
    uint8_t feature_flags{0};
    uint8_t can_bitrate_kbps{0};
};

struct RfidEventSegment {
    uint8_t event_type{0};
    uint8_t uid_len{0};
    uint8_t event_id{0};
    uint8_t seg_idx{0};
    uint8_t seg_cnt{0};
    std::array<uint8_t, 5> payload{};
};

struct IdentitySegment {
    uint8_t len{0};
    uint8_t seg_cnt{0};
    uint8_t seg_idx{0};
    std::array<uint8_t, 5> payload{};
};

struct PlcTlmV3 {
    char cp_state{'U'};
    uint8_t hlc_stage{0};
    uint8_t fault_reason{0};
    uint8_t relay_state_mask{0};
    uint8_t relay_fault_mask{0};
    bool safety_ok{true};
    bool estop{false};
    bool earth_fault{false};
    bool comm_fault{false};
    bool lock_engaged{false};
    bool cable_checked{false};
    bool precharge_active{false};
    bool charge_complete{false};
    uint8_t limits_rx_count_lsb{0};
    double ev_target_voltage_v{0.0};
    double ev_target_current_a{0.0};
    bool crc_ok{true};
};

inline uint32_t extend_counter_lsb(uint32_t current, uint8_t new_lsb, bool* changed = nullptr) {
    const uint8_t last_lsb = static_cast<uint8_t>(current & 0xFFu);
    const uint8_t delta = static_cast<uint8_t>(new_lsb - last_lsb); // wrap-safe
    if (delta == 0) {
        if (changed) *changed = false;
        return current;
    }
    if (changed) *changed = true;
    return current + static_cast<uint32_t>(delta);
}

inline std::array<uint8_t, 8> build_plc_tlm_v3(uint8_t cp_state_enum,
                                               uint8_t hlc_stage,
                                               uint8_t fault_reason,
                                               uint8_t relay_state_mask,
                                               uint8_t relay_fault_mask,
                                               bool safety_ok,
                                               bool estop,
                                               bool earth_fault,
                                               bool comm_fault,
                                               bool lock_engaged,
                                               bool cable_checked,
                                               bool precharge_active,
                                               bool charge_complete,
                                               uint8_t limits_rx_count_lsb,
                                               uint16_t ev_target_voltage_1v,
                                               uint16_t ev_target_current_0p5a,
                                               bool use_crc8) {
    std::array<uint8_t, 8> data{};
    uint64_t packed = 0;
    packed |= static_cast<uint64_t>(cp_state_enum & 0x07u) << 0;
    packed |= static_cast<uint64_t>(hlc_stage & 0x3Fu) << 3;
    packed |= static_cast<uint64_t>(fault_reason & 0x0Fu) << 9;
    packed |= static_cast<uint64_t>(relay_state_mask & 0x07u) << 13;
    packed |= static_cast<uint64_t>(relay_fault_mask & 0x07u) << 16;
    packed |= static_cast<uint64_t>(safety_ok ? 1u : 0u) << 19;
    packed |= static_cast<uint64_t>(estop ? 1u : 0u) << 20;
    packed |= static_cast<uint64_t>(earth_fault ? 1u : 0u) << 21;
    packed |= static_cast<uint64_t>(comm_fault ? 1u : 0u) << 22;
    packed |= static_cast<uint64_t>(lock_engaged ? 1u : 0u) << 23;
    packed |= static_cast<uint64_t>(cable_checked ? 1u : 0u) << 24;
    packed |= static_cast<uint64_t>(precharge_active ? 1u : 0u) << 25;
    packed |= static_cast<uint64_t>(charge_complete ? 1u : 0u) << 26;
    packed |= static_cast<uint64_t>(limits_rx_count_lsb) << 27;
    packed |= static_cast<uint64_t>(ev_target_voltage_1v & 0x03FFu) << 35;
    packed |= static_cast<uint64_t>(ev_target_current_0p5a & 0x03FFu) << 45;
    for (int i = 0; i < 7; ++i) {
        data[i] = static_cast<uint8_t>((packed >> (i * 8)) & 0xFFu);
    }
    data[7] = use_crc8 ? crc8_07(data.data(), 7) : 0;
    return data;
}

inline PlcTlmV3 decode_plc_tlm_v3(const uint8_t in[8], bool use_crc8) {
    PlcTlmV3 st{};
    if (use_crc8) {
        st.crc_ok = (crc8_07(in, 7) == in[7]);
    }
    uint64_t packed = 0;
    for (int i = 0; i < 7; ++i) {
        packed |= (static_cast<uint64_t>(in[i]) << (8 * i));
    }
    const uint8_t cp_enum = static_cast<uint8_t>((packed >> 0) & 0x07u);
    switch (cp_enum) {
        case 1: st.cp_state = 'A'; break;
        case 2: st.cp_state = 'B'; break;
        case 3: st.cp_state = 'C'; break;
        case 4: st.cp_state = 'D'; break;
        case 5: st.cp_state = 'E'; break;
        case 6: st.cp_state = 'F'; break;
        default: st.cp_state = 'U'; break;
    }
    st.hlc_stage = static_cast<uint8_t>((packed >> 3) & 0x3Fu);
    st.fault_reason = static_cast<uint8_t>((packed >> 9) & 0x0Fu);
    st.relay_state_mask = static_cast<uint8_t>((packed >> 13) & 0x07u);
    st.relay_fault_mask = static_cast<uint8_t>((packed >> 16) & 0x07u);
    st.safety_ok = ((packed >> 19) & 0x01u) != 0;
    st.estop = ((packed >> 20) & 0x01u) != 0;
    st.earth_fault = ((packed >> 21) & 0x01u) != 0;
    st.comm_fault = ((packed >> 22) & 0x01u) != 0;
    st.lock_engaged = ((packed >> 23) & 0x01u) != 0;
    st.cable_checked = ((packed >> 24) & 0x01u) != 0;
    st.precharge_active = ((packed >> 25) & 0x01u) != 0;
    st.charge_complete = ((packed >> 26) & 0x01u) != 0;
    st.limits_rx_count_lsb = static_cast<uint8_t>((packed >> 27) & 0xFFu);
    const uint16_t v_1v = static_cast<uint16_t>((packed >> 35) & 0x03FFu);
    const uint16_t i_0p5a = static_cast<uint16_t>((packed >> 45) & 0x03FFu);
    st.ev_target_voltage_v = static_cast<double>(v_1v);
    st.ev_target_current_a = static_cast<double>(i_0p5a) * 0.5;
    return st;
}

inline std::array<uint8_t, 8> build_evse_fast(uint16_t present_v_0p5,
                                              uint16_t present_i_0p2,
                                              uint16_t present_p_0p5,
                                              bool output_enabled,
                                              bool regulating,
                                              uint8_t fault_bits,
                                              uint8_t relay_cmd_mask,
                                              uint8_t relay_enable_mask,
                                              bool sys_enable,
                                              bool force_off,
                                              bool clear_faults,
                                              uint8_t seq,
                                              bool use_crc8) {
    std::array<uint8_t, 8> data{};
    uint64_t packed = 0;
    packed |= static_cast<uint64_t>(present_v_0p5 & 0x07FFu) << 0;
    packed |= static_cast<uint64_t>(present_i_0p2 & 0x07FFu) << 11;
    packed |= static_cast<uint64_t>(present_p_0p5 & 0x01FFu) << 22;
    packed |= static_cast<uint64_t>(fault_bits & 0x3Fu) << 31;
    packed |= static_cast<uint64_t>(output_enabled ? 1u : 0u) << 37;
    packed |= static_cast<uint64_t>(regulating ? 1u : 0u) << 38;
    packed |= static_cast<uint64_t>(relay_cmd_mask & 0x07u) << 39;
    packed |= static_cast<uint64_t>(sys_enable ? 1u : 0u) << 42;
    packed |= static_cast<uint64_t>(force_off ? 1u : 0u) << 43;
    packed |= static_cast<uint64_t>(seq) << 44;
    packed |= static_cast<uint64_t>(relay_enable_mask & 0x07u) << 52;
    packed |= static_cast<uint64_t>(clear_faults ? 1u : 0u) << 55;
    for (int i = 0; i < 7; ++i) {
        data[i] = static_cast<uint8_t>((packed >> (i * 8)) & 0xFFu);
    }
    data[7] = use_crc8 ? crc8_07(data.data(), 7) : 0;
    return data;
}

inline std::array<uint8_t, 8> build_evse_slow(uint16_t max_v_0p5,
                                              uint16_t max_i_0p2,
                                              uint16_t max_p_0p5,
                                              bool auth_granted,
                                              bool auth_pending,
                                              bool hlc_enable,
                                              bool pnc_blocked,
                                              bool lock_cmd,
                                              bool use_crc8) {
    std::array<uint8_t, 8> data{};
    uint64_t packed = 0;
    packed |= static_cast<uint64_t>(max_v_0p5 & 0x07FFu) << 0;
    packed |= static_cast<uint64_t>(max_i_0p2 & 0x07FFu) << 11;
    packed |= static_cast<uint64_t>(max_p_0p5 & 0x01FFu) << 22;
    packed |= static_cast<uint64_t>(auth_granted ? 1u : 0u) << 31;
    packed |= static_cast<uint64_t>(auth_pending ? 1u : 0u) << 32;
    packed |= static_cast<uint64_t>(hlc_enable ? 1u : 0u) << 33;
    packed |= static_cast<uint64_t>(pnc_blocked ? 1u : 0u) << 34;
    packed |= static_cast<uint64_t>(lock_cmd ? 1u : 0u) << 35;
    packed |= static_cast<uint64_t>(PROTOCOL_VERSION & 0x0Fu) << 36;
    for (int i = 0; i < 7; ++i) {
        data[i] = static_cast<uint8_t>((packed >> (i * 8)) & 0xFFu);
    }
    data[7] = use_crc8 ? crc8_07(data.data(), 7) : 0;
    return data;
}

inline MeterReading decode_energy_meter(const uint8_t in[8], std::uint64_t seq) {
    MeterReading m{};
    const auto mux = in[0];
    const bool meter_ok = (in[1] & 0x01u) != 0;
    const bool comm_error = (in[1] & 0x02u) != 0;
    const bool stale = (in[1] & 0x04u) != 0;
    const bool overrange = (in[1] & 0x08u) != 0;
    const bool fallback = (in[1] & 0x10u) != 0;
    m.meter_ok = meter_ok;
    m.comm_error = comm_error;
    m.stale = stale;
    m.overrange = overrange;
    m.fallback_active = fallback;
    m.seq = seq;
    if (mux == 0) {
        const uint16_t v0p1 = static_cast<uint16_t>(in[2]) | (static_cast<uint16_t>(in[3]) << 8);
        const uint16_t i0p01_u = static_cast<uint16_t>(in[4]) | (static_cast<uint16_t>(in[5]) << 8);
        const uint16_t p0p01kw_u = static_cast<uint16_t>(in[6]) | (static_cast<uint16_t>(in[7]) << 8);
        const std::int16_t i0p01 = static_cast<std::int16_t>(i0p01_u);
        const std::int16_t p0p01kw = static_cast<std::int16_t>(p0p01kw_u);
        m.voltage_v = static_cast<double>(v0p1) / 10.0;
        m.current_a = static_cast<double>(i0p01) / 100.0;
        m.power_kw = static_cast<double>(p0p01kw) / 100.0;
    } else if (mux == 1) {
        const uint32_t e0p1kwh = static_cast<uint32_t>(in[2]) | (static_cast<uint32_t>(in[3]) << 8) |
                                 (static_cast<uint32_t>(in[4]) << 16) | (static_cast<uint32_t>(in[5]) << 24);
        const uint16_t f0p01hz = static_cast<uint16_t>(in[6]) | (static_cast<uint16_t>(in[7]) << 8);
        m.import_energy_kwh = static_cast<double>(e0p1kwh) / 10.0;
        m.freq_hz = static_cast<double>(f0p01hz) / 100.0;
    }
    return m;
}

inline BootConfig decode_boot_config(const uint8_t in[8]) {
    BootConfig cfg{};
    cfg.fw_major = in[0];
    cfg.fw_minor = in[1];
    cfg.fw_patch = in[2];
    cfg.feature_flags = in[3];
    cfg.can_bitrate_kbps = in[4];
    return cfg;
}

inline RfidEventSegment decode_rfid_event(const uint8_t in[8]) {
    RfidEventSegment seg{};
    seg.uid_len = static_cast<uint8_t>(in[0] & 0x0Fu);
    seg.event_type = static_cast<uint8_t>((in[0] >> 4) & 0x0Fu);
    seg.event_id = in[1];
    seg.seg_idx = static_cast<uint8_t>(in[2] & 0x0Fu);
    seg.seg_cnt = static_cast<uint8_t>((in[2] >> 4) & 0x0Fu);
    seg.payload = {in[3], in[4], in[5], in[6], in[7]};
    return seg;
}

inline IdentitySegment decode_identity_segment(const uint8_t in[8]) {
    IdentitySegment seg{};
    seg.len = in[0];
    seg.seg_cnt = in[1];
    seg.seg_idx = in[2];
    seg.payload = {in[3], in[4], in[5], in[6], in[7]};
    return seg;
}

} // namespace charger::can_contract

