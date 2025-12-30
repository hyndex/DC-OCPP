# PLC ↔ Controller CAN Contract (Authoritative Runtime Expectations)

This document captures the production contract between the PLC firmware (ISO/DIN/V2G stack) and the controller (OCPP + power/module orchestration). It reflects the current implementation and MUST be honored in production deployments.

## ID map (extended IDs, low nibble = plc_id)

- PLC→Controller
  - `0x0100|(0x0<<4)|plc_id` CHARGEINFO: HLC stage/flags (auth pending/granted, lock, contactor state)
  - `0x0200|(0x1<<4)|plc_id` EVDC_TARGETS: EV target V/I + EVSE present V/I (0.1V/0.1A)
  - `0x0200|(0x0<<4)|plc_id` EVDC_MAX_LIMITS: EV max limits (EV-requested)
  - `0x0200|(0x3<<4)|plc_id` EVDC_ENERGY_LIMITS: EV energy limits (if used)
  - `0x0200|(0x5<<4)|plc_id` EVAC_CTRL: SLAC/SDP control
  - `0x0200|(0x6<<4)|plc_id` EMAID0 segments
  - `0x0200|(0x7<<4)|plc_id` EMAID1 segments
  - `0x0200|(0x8<<4)|plc_id` EVCCID segments
  - `0x0200|(0x4<<4)|plc_id` EVMAC segments (optional)
  - `0x0400|(0x1<<4)|plc_id` CHARGING_SESSION: HLC session flags
  - `0x0400|(0x3<<4)|plc_id` CP_LEVELS: CP state/duty/voltages
  - `0x0100|(0x6<<4)|plc_id` RELAY_STATUS: relay feedback + safety bits
  - `0x0100|(0x9<<4)|plc_id` SAFETY_STATUS: e-stop/earth/comm faults
  - `0x0100|(0x7<<4)|plc_id` ENERGY_METER: V/I/P/Energy
  - `0x0100|(0xA<<4)|plc_id` CONFIG_ACK: ACK for EVSE limit/config params (param=90 EVSE_LIMIT_ACK)
  - `0x0100|(0xB<<4)|plc_id` DEBUG_INFO: diag counters

- Controller→PLC
  - `0x0300|(0x0<<4)|plc_id` EVSE_DC_MAX_LIMITS_CMD: EVSE capability (0.1V/0.1A/0.1kW). **Cadence: ≥1 Hz**
  - `0x0300|(0x1<<4)|plc_id` EVSE_DC_PRESENT_CMD (EVSE_DC_REG_LIMITS): EVSE present V/I/P + flags. **Cadence: ≥10 Hz**
    - byte6 bits: b0=output_enabled, b1=regulating, b2..b7=fault_bits[0..5] (see below)
  - `0x0300|(0x8<<4)|plc_id` CONFIG_CMD: runtime config (auth pending/granted etc.)
    - param 91: PROTO_VERSION (value must equal `1`); PLC returns NOT_ALLOWED on mismatch.
    - param 20: AUTH_STATE (0/1 = denied/granted)
    - param 21: AUTH_PENDING (0/1)
    - param 30: LOCK_CMD (0=unlock, 1=lock)
  - `0x0300|(0x4<<4)|plc_id` RELAY_CMD: module/gun relay drive (used only when controller owns GC)
  - `0x0300|(0x9<<4)|plc_id` GCMC_CMD: mirrored relay command (legacy)

## Ownership rules

- **Gun contactor (GC) ownership is PLC-only when `plc.gunRelayOwnedByPlc=true` (default).**
  - Controller will not set GC bits in `RELAY_CMD` when this flag is true.
  - PLC firmware must ignore/override any GC command bits from controller when it owns GC.
- Module relays (RLY2/RLY3) may be driven by PLC when `plc.moduleRelaysEnabled=true` else controller/hardware driver manages modules directly.
- When both RELAY_CMD and mirrored GCMC_CMD are present, PLC gives priority to RELAY_CMD updates received within 50 ms; GCMC_CMD is used only as a legacy fallback when no fresh RELAY_CMD is seen.

## Fault propagation (controller → PLC)

- Controller computes a 6-bit fault mask each planner tick and packs into EVSE_PRESENT byte6 bits 2..7:
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

- EVSE_DC_PRESENT_CMD (`0x310+id`): send every 100 ms nominal; **warn at > presentWarnMs (default 1000 ms)**, controller will flag comm fault and constrain power on staleness.
- EVSE_DC_MAX_LIMITS_CMD (`0x300+id`): send every 1 s nominal and whenever derates change; **warn at > limitsWarnMs (default 1500 ms)**, controller will constrain power on staleness.
- PLC must ACK EVSE limits via CONFIG_ACK param 90. Controller will constrain power if ACK is stale (`evseLimitAckTimeoutMs`).
- PROTO_VERSION (param 91) handshake is mandatory; the controller expects value `1` and treats mismatches as a comm fault.
- CRC8 (poly 0x07, init 0x00) is required on RelayControl/RelayStatus/SafetyStatus/GCMC_Command/GCMC_Status/ConfigCmd/ConfigAck **and** EVSE_DC_MAX_LIMITS/EVSE_DC_PRESENT; frames that carry CRC8 must use DLC=8.
- Authorization state must be refreshed by controller every ~1 s; PLC must treat AUTH_PENDING as valid until controller clears it.

## Identity / Autocharge

- PLC emits EVCCID/EMAID/EVMAC in segmented frames (`0x260/0x270/0x280/0x240`). Controller reconstructs segments by plc_id and exposes tokens to OCPP authorization.

## Expected V2G/ISO/DIN behavior (summary)

- PLC owns V2G stage truth, isolation check, precharge, PowerDelivery contactor sequencing.
- Controller publishes EVSE limits + present + faults; PLC must use these as authoritative for DC_EVSEStatus (NotReady vs Fault vs Welding) and ResponseCode selection.
- If fault bits indicate weld/iso/comm/thermal/overcurrent, PLC should:
  - Refuse/abort PowerDelivery/CurrentDemand with appropriate ResponseCode (e.g., `FAILED_IsolationMonitoringActive`, `FAILED_PowerDeliveryNotApplied`, `FAILED_WeldingDetectionFailed`).
  - Set DC_EVSEStatusCode to Faulted/IsolationMonitoringActive/WeldDetected accordingly.
- Suggested ISO/DIN ResponseCode mapping (consume fault bits in EVSE_PRESENT):
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
