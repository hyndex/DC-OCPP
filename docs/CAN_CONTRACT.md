# PLC ↔ Controller CAN Contract (Authoritative Runtime Expectations)

This document captures the production contract between the PLC firmware (ISO/DIN/V2G stack) and the controller (OCPP + power/module orchestration). It reflects the current implementation and MUST be honored in production deployments.

## ID map (extended IDs, low nibble = plc_id)

- Protocol v2 (compact)
  - Controller→PLC
    - `0x0300|(0x2<<4)|plc_id` EVSE_FAST_V2: present V/I/P (0.5 V/0.2 A/0.5 kW) + output/regulating +
      fault bits + relay cmd bits + sys_enable/force_off + seq + enable_mask + clear_faults.
    - `0x0300|(0x3<<4)|plc_id` EVSE_SLOW_V2: max V/I/P (0.5 V/0.2 A/0.5 kW) + auth/pending + hlc_enable +
      pnc_blocked + lock_cmd + proto_version.
  - PLC→Controller
    - `0x0400|(0x4<<4)|plc_id` PLC_STATE_V2: cp_state + duty_pct + hlc_stage + auth/lock/precharge flags.
    - `0x0400|(0x5<<4)|plc_id` PLC_STATUS_V2: relay state/fault bits + safety bits + fault_reason + tec/rec + uptime +
      safety policy.
    - `0x0200|(0x0<<4)|plc_id` EVDC_MAX_LIMITS: EVSE max V/I/P from HLC (0.1V/0.1A/0.1kW).
    - `0x0200|(0x1<<4)|plc_id` EVDC_TARGETS: EV target V/I + EVSE present V/I (0.1V/0.1A).
    - `0x0200|(0x2<<4)|plc_id` EV_STATUS_DISPLAY: present V/I + CP/HLC summary.
    - `0x0200|(0x3<<4)|plc_id` EVDC_ENERGY_LIMITS: EVSE max V/I/P (0.1V/0.1A/0.1kW).
    - `0x0200|(0x5<<4)|plc_id` EVAC_CHG_CTRL: CP duty/state + setpoint/present I.
    - `0x0100|(0x7<<4)|plc_id` ENERGY_METER: V/I/P/Energy.
    - `0x0100|(0xA<<4)|plc_id` CONFIG_ACK: ACK for EVSE limit/config params (param=90 EVSE_LIMIT_ACK).
    - `0x0100|(0x8<<4)|plc_id` RFID_EVENT: segmented UID events.
    - `0x0200|(0x6<<4)|plc_id` EMAID0 segments.
    - `0x0200|(0x7<<4)|plc_id` EMAID1 segments.
    - `0x0200|(0x8<<4)|plc_id` EVCCID segments.
    - `0x0200|(0x4<<4)|plc_id` EVMAC segments (optional).
    - `0x090000|plc_id` BOOTCONFIG: firmware + feature flags.
    - `0x0100|(0x3<<4)|plc_id` HW_STATUS, `0x0100|(0xB<<4)|plc_id` DEBUGINFO,
      `0x0400|(0x0<<4)|plc_id` RTTLOG, `0x0400|(0x2<<4)|plc_id` RTEVLOG,
      `0x0100|(0x1<<4)|plc_id` SOFTWAREINFO, `0x0100|(0xC<<4)|plc_id` GUN_TEMP (if enabled),
      `0x0100|(0x2<<4)|plc_id` ERRORCODES, `0x0100|(0x4<<4)|plc_id` SLACINFO (debug/placeholder).
  - Controller→PLC
    - `0x0300|(0x8<<4)|plc_id` CONFIG_CMD: PROTO_VERSION query (op=1); PLC returns CONFIG_ACK value `2`.

## Ownership rules

- **Gun contactor (GC) ownership is PLC-only when `plc.gunRelayOwnedByPlc=true` (default).**
  - Controller will not set GC bits in EVSE_FAST_V2 when this flag is true.
  - PLC firmware must ignore/override any GC command bits from controller when it owns GC.
- Auxiliary relays (RLY2/RLY3) are used as KM_A/KM_B bus sectionalizers when `plc.moduleRelaysEnabled=true`
  (module‑level ring cuts; bit0=KM_A, bit1=KM_B).
- `CLEAR_FAULTS` in EVSE_FAST_V2 is asserted only on explicit operator/service request (no auto-clear on any fault).

## Fault propagation (controller → PLC)

- Controller computes a 6-bit fault mask each planner tick and packs into EVSE_FAST_V2 bits 31..36:
  - bit0 (LSB in mask) = general fault present (safety not OK or meter/CP issues)
  - bit1 = communication fault (PLC/controller path)
  - bit2 = isolation/earth/E-Stop fault
  - bit3 = thermal fault
  - bit4 = overcurrent fault
  - bit5 = weld detection (GC or MC)
- PLC firmware should:
  - Decode these bits and map into ISO/DIN ResponseCode and DC_EVSEStatusCode (NotReady/Fault/WeldDetected/IsolationMonitoringActive).
  - Prefer controller-provided weld/isolation bits over internal stubs.

## Cadence and freshness requirements

- EVSE_FAST_V2: send every 100 ms nominal; **warn at > presentWarnMs (default 1000 ms)**, controller will flag comm
  fault and constrain power on staleness.
- EVSE_SLOW_V2: send every 1 s nominal and whenever derates/config change; **warn at > limitsWarnMs (default 1500 ms)**.
- PLC must ACK EVSE limits via CONFIG_ACK param 90. Controller will constrain power if ACK is stale (`evseLimitAckTimeoutMs`).
- PROTO_VERSION (param 91) handshake is mandatory; controller queries (op=1) and expects CONFIG_ACK value `2`
  (mismatch treated as a comm fault).
- CRC8 (poly 0x07, init 0x00) is required on ConfigCmd/ConfigAck and EVSE_FAST_V2/EVSE_SLOW_V2/PLC_STATE_V2/
  PLC_STATUS_V2; frames that carry CRC8 must use DLC=8.
- Authorization state should be refreshed by controller every ~1 s via EVSE_SLOW_V2.

## Identity / Autocharge

- PLC emits EVCCID/EMAID/EVMAC in segmented frames (`0x260/0x270/0x280/0x240`). Controller reconstructs segments by plc_id and exposes tokens to OCPP authorization.

## Expected V2G/ISO/DIN behavior (summary)

- PLC owns V2G stage truth, isolation check, precharge, PowerDelivery contactor sequencing.
- Controller publishes EVSE limits + present + faults; PLC must use these as authoritative for DC_EVSEStatus (NotReady vs Fault vs Welding) and ResponseCode selection.
- If fault bits indicate weld/iso/comm/thermal/overcurrent, PLC should:
  - Refuse/abort PowerDelivery/CurrentDemand with appropriate ResponseCode (e.g., `FAILED_IsolationMonitoringActive`, `FAILED_PowerDeliveryNotApplied`, `FAILED_WeldingDetectionFailed`).
  - Set DC_EVSEStatusCode to Faulted/IsolationMonitoringActive/WeldDetected accordingly.
- Suggested ISO/DIN ResponseCode mapping (consume fault bits in EVSE_FAST_V2):
  - Isolation/earth/E-Stop (bit2): `FAILED_IsolationMonitoringActive` + `EVSE_IsolationMonitoringActive`
  - Weld (bit5): `FAILED_WeldingDetectionFailed` + `EVSE_EmergencyShutdown`
  - Overcurrent (bit4): `FAILED_PowerDeliveryNotApplied` + `EVSE_EmergencyShutdown`
  - Thermal (bit3): `FAILED_PowerDeliveryNotApplied` + `EVSE_EmergencyShutdown`
  - Comm (bit1): `FAILED_PowerDeliveryNotApplied` + `EVSE_NotReady`
  - General fault (bit0): if none of the above, use `FAILED_PowerDeliveryNotApplied` + `EVSE_EmergencyShutdown`
  - Otherwise when ready: `EVSE_Ready`
- TLS/Contract policy: if TLS stack absent, PLC should reject/omit Contract certificate handling and respond `tls=false` in SDP; SupportedAppProtocol should not advertise Contract unless TLS is available and validated.

## Logging/metrics (controller side)

- `present_stale_events` / `limit_stale_events` / `auth_push_count` exposed in GunStatus for monitoring.
- Controller constrains power or trips comm fault when cadence warnings are observed.
