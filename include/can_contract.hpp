// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

namespace charger::can_contract {

constexpr uint32_t RX_BASE_ID = 0x00000300u;
constexpr uint8_t RX_TYPE_SHIFT = 4;

constexpr uint32_t TX_BASE_SYS = 0x00000100u;
constexpr uint32_t TX_BASE_EVDC = 0x00000200u;
constexpr uint32_t TX_BASE_CP = 0x00000400u;
constexpr uint32_t TX_BASE_BOOT = 0x00090000u;

// Config parameter ids (must match Basic firmware)
constexpr uint8_t PARAM_AUTH_STATE = 20;
constexpr uint8_t PARAM_AUTH_PENDING = 21;
constexpr uint8_t PARAM_HLC_ENABLE = 22;
constexpr uint8_t PARAM_PNC_BLOCKED = 23;
constexpr uint8_t PARAM_LOCK_CMD = 30;
constexpr uint8_t PARAM_EVSE_LIMIT_ACK = 90;
constexpr uint8_t PARAM_PROTO_VERSION = 91;
constexpr uint8_t PROTOCOL_VERSION = 2;

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

inline uint32_t config_cmd_id(uint8_t plc_id) { return rx_id(0x8u, plc_id); }
inline uint32_t evse_fast_v2_id(uint8_t plc_id) { return rx_id(0x2u, plc_id); }
inline uint32_t evse_slow_v2_id(uint8_t plc_id) { return rx_id(0x3u, plc_id); }

inline uint32_t energy_meter_id(uint8_t plc_id) { return tx_id(TX_BASE_SYS, 0x7u, plc_id); }
inline uint32_t rfid_event_id(uint8_t plc_id) { return tx_id(TX_BASE_SYS, 0x8u, plc_id); }
inline uint32_t config_ack_id(uint8_t plc_id) { return tx_id(TX_BASE_SYS, 0xAu, plc_id); }
inline uint32_t evdc_max_limits_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x0u, plc_id); }
inline uint32_t evdc_targets_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x1u, plc_id); }
inline uint32_t ev_status_display_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x2u, plc_id); }
inline uint32_t evdc_energy_limits_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x3u, plc_id); }
inline uint32_t evac_chg_ctrl_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x5u, plc_id); }
inline uint32_t evccid_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x8u, plc_id); }
inline uint32_t evemaid0_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x6u, plc_id); }
inline uint32_t evemaid1_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x7u, plc_id); }
inline uint32_t evmac_id(uint8_t plc_id) { return tx_id(TX_BASE_EVDC, 0x4u, plc_id); }
inline uint32_t plc_state_v2_id(uint8_t plc_id) { return tx_id(TX_BASE_CP, 0x4u, plc_id); }
inline uint32_t plc_status_v2_id(uint8_t plc_id) { return tx_id(TX_BASE_CP, 0x5u, plc_id); }
inline uint32_t boot_config_id(uint8_t plc_id) { return TX_BASE_BOOT | static_cast<uint32_t>(plc_id & 0x0Fu); }

constexpr std::array<const char*, 13> FAULT_REASONS = {{
    "OK",
    "SAFETY_SW1",
    "SAFETY_SW2",
    "SAFETY_SW3",
    "ESTOP_LATCHED_FAULT",
    "RELAYCTRL_TIMEOUT",
    "CAN_BUS_OFF",
    "REMOTE_FORCE_OFF",
    "CRC_FAIL",
    "INTERNAL_ERROR",
    "EARTH_FAULT",
    "SAFETY_SW4",
    "ESTOP_INPUT_FAULT",
}};

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

struct ConfigAck {
    uint8_t param_id{0};
    uint8_t status{0};
    uint32_t value{0};
    uint8_t plc_id{0};
    bool crc_ok{true};
};

struct BootConfig {
    uint8_t fw_major{0};
    uint8_t fw_minor{0};
    uint8_t fw_patch{0};
    uint8_t feature_flags{0};
    uint8_t can_bitrate_kbps{0};
};

struct EvdcTargets {
    double target_v{0.0};
    double target_a{0.0};
    double present_v{0.0};
    double present_a{0.0};
};

struct EvdcLimits {
    double max_voltage_v{0.0};
    double max_current_a{0.0};
    double max_power_kw{0.0};
};

struct EvacChgCtrl {
    uint8_t duty_pct{0};
    char cp_state{'U'};
    double target_v{0.0};
    double target_a{0.0};
    double present_a{0.0};
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

struct PlcStateV2 {
    char cp_state{'U'};
    uint8_t duty_pct{0};
    uint8_t hlc_stage{0};
    bool auth_granted{false};
    bool auth_pending{false};
    bool lock_engaged{false};
    bool charge_complete{false};
    bool precharge_active{false};
    bool cable_checked{false};
    uint32_t state_seq{0};
    bool crc_ok{true};
};

struct PlcStatusV2 {
    RelayStatus relay{};
    SafetyStatus safety{};
    uint8_t policy{0};
    bool crc_ok{true};
};

inline std::array<uint8_t, 8> build_config_cmd(uint8_t param_id, uint8_t op, uint32_t value, bool use_crc8) {
    std::array<uint8_t, 8> data{};
    data[0] = param_id;
    data[1] = op;
    data[2] = static_cast<uint8_t>(value & 0xFFu);
    data[3] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    data[4] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    data[5] = static_cast<uint8_t>((value >> 24) & 0xFFu);
    data[6] = 0;
    data[7] = use_crc8 ? crc8_07(data.data(), 7) : 0;
    return data;
}

inline std::array<uint8_t, 8> build_evse_fast_v2(uint16_t present_v_0p5,
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

inline std::array<uint8_t, 8> build_evse_slow_v2(uint16_t max_v_0p5,
                                                  uint16_t max_i_0p2,
                                                  uint16_t max_p_0p5,
                                                  bool auth_granted,
                                                  bool auth_pending,
                                                  bool hlc_enable,
                                                  bool pnc_blocked,
                                                  bool lock_cmd,
                                                  uint8_t proto_version,
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
    packed |= static_cast<uint64_t>(proto_version & 0x0Fu) << 36;
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

inline ConfigAck decode_config_ack(const uint8_t in[8], bool use_crc8) {
    ConfigAck ack{};
    ack.param_id = in[0];
    ack.status = in[1];
    ack.value = static_cast<uint32_t>(in[2]) | (static_cast<uint32_t>(in[3]) << 8) |
                (static_cast<uint32_t>(in[4]) << 16) | (static_cast<uint32_t>(in[5]) << 24);
    ack.plc_id = static_cast<uint8_t>(in[6] & 0x0Fu);
    if (use_crc8) {
        ack.crc_ok = crc8_07(in, 7) == in[7];
    }
    return ack;
}

inline PlcStateV2 decode_plc_state_v2(const uint8_t in[8], bool use_crc8) {
    PlcStateV2 st{};
    if (use_crc8) {
        st.crc_ok = (crc8_07(in, 7) == in[7]);
    }
    st.cp_state = static_cast<char>(in[0]);
    st.duty_pct = in[1];
    st.hlc_stage = in[2];
    const uint8_t flags = in[3];
    st.auth_granted = (flags & 0x01u) != 0;
    st.auth_pending = (flags & 0x02u) != 0;
    st.lock_engaged = (flags & 0x04u) != 0;
    st.charge_complete = (flags & 0x08u) != 0;
    st.precharge_active = (flags & 0x10u) != 0;
    st.cable_checked = (flags & 0x20u) != 0;
    st.state_seq = static_cast<uint32_t>(in[4]) | (static_cast<uint32_t>(in[5]) << 8) |
                   (static_cast<uint32_t>(in[6]) << 16);
    return st;
}

inline PlcStatusV2 decode_plc_status_v2(const uint8_t in[8], bool use_crc8) {
    PlcStatusV2 st{};
    if (use_crc8) {
        st.crc_ok = (crc8_07(in, 7) == in[7]);
    }
    const uint8_t b0 = in[0];
    st.relay.relay[0] = (b0 & 0x01u) != 0;
    st.relay.relay[1] = (b0 & 0x02u) != 0;
    st.relay.relay[2] = (b0 & 0x04u) != 0;
    st.relay.relay_fault[0] = (b0 & 0x08u) != 0;
    st.relay.relay_fault[1] = (b0 & 0x10u) != 0;
    st.relay.relay_fault[2] = (b0 & 0x20u) != 0;
    st.relay.safety_active = (b0 & 0x40u) != 0;
    st.relay.comm_fault = (b0 & 0x80u) != 0;

    const uint8_t b1 = in[1];
    st.safety.sw_active[0] = (b1 & 0x01u) != 0;
    st.safety.sw_active[1] = (b1 & 0x02u) != 0;
    st.safety.sw_active[2] = (b1 & 0x04u) != 0;
    st.safety.estop_latched = (b1 & 0x08u) != 0;
    st.safety.safety_ok = (b1 & 0x10u) != 0;
    st.safety.earth_fault = (b1 & 0x20u) != 0;
    st.safety.sw_active[3] = (b1 & 0x40u) != 0;
    st.safety.estop_input = (b1 & 0x80u) != 0;

    st.relay.fault_reason = in[2];
    st.relay.tec = static_cast<uint8_t>((in[3] >> 4) & 0x0Fu);
    st.relay.rec = static_cast<uint8_t>(in[3] & 0x0Fu);
    st.relay.uptime_s = static_cast<uint16_t>(in[4]) | (static_cast<uint16_t>(in[5]) << 8);
    st.safety.policy = in[6];
    st.relay.crc_ok = st.crc_ok;
    st.safety.crc_ok = st.crc_ok;
    return st;
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

inline EvdcTargets decode_evdc_targets(const uint8_t in[8]) {
    EvdcTargets t{};
    const uint16_t tgt_v_0p1 = static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
    const uint16_t tgt_i_0p1 = static_cast<uint16_t>(in[2]) | (static_cast<uint16_t>(in[3]) << 8);
    const uint16_t pres_v_0p1 = static_cast<uint16_t>(in[4]) | (static_cast<uint16_t>(in[5]) << 8);
    const uint16_t pres_i_0p1 = static_cast<uint16_t>(in[6]) | (static_cast<uint16_t>(in[7]) << 8);
    t.target_v = static_cast<double>(tgt_v_0p1) / 10.0;
    t.target_a = static_cast<double>(tgt_i_0p1) / 10.0;
    t.present_v = static_cast<double>(pres_v_0p1) / 10.0;
    t.present_a = static_cast<double>(pres_i_0p1) / 10.0;
    return t;
}

inline EvdcLimits decode_evdc_limits(const uint8_t in[8]) {
    EvdcLimits l{};
    const uint16_t max_v_0p1 = static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
    const uint16_t max_i_0p1 = static_cast<uint16_t>(in[2]) | (static_cast<uint16_t>(in[3]) << 8);
    const uint16_t max_p_0p1k = static_cast<uint16_t>(in[4]) | (static_cast<uint16_t>(in[5]) << 8);
    l.max_voltage_v = static_cast<double>(max_v_0p1) / 10.0;
    l.max_current_a = static_cast<double>(max_i_0p1) / 10.0;
    l.max_power_kw = static_cast<double>(max_p_0p1k) / 10.0;
    return l;
}

inline EvacChgCtrl decode_evac_chg_ctrl(const uint8_t in[8]) {
    EvacChgCtrl c{};
    c.duty_pct = in[0];
    c.cp_state = static_cast<char>(in[1]);
    const uint16_t tgt_v_0p1 = static_cast<uint16_t>(in[2]) | (static_cast<uint16_t>(in[3]) << 8);
    const uint16_t tgt_i_0p1 = static_cast<uint16_t>(in[4]) | (static_cast<uint16_t>(in[5]) << 8);
    const uint16_t pres_i_0p1 = static_cast<uint16_t>(in[6]) | (static_cast<uint16_t>(in[7]) << 8);
    c.target_v = static_cast<double>(tgt_v_0p1) / 10.0;
    c.target_a = static_cast<double>(tgt_i_0p1) / 10.0;
    c.present_a = static_cast<double>(pres_i_0p1) / 10.0;
    return c;
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
