# PLC ↔ Controller CAN Contract (Protocol v3)

This document captures the **production** CAN contract between the PLC firmware (ISO15118/DIN stack) and the
controller (DC-OCPP + power/module orchestration).

## ID map (extended IDs, low nibble = plc_id)

- Controller → PLC
  - `0x0300|(0x2<<4)|plc_id` `EVSE_FAST`
    - Present V/I/P (0.5 V / 0.2 A / 0.5 kW), output/regulating, fault bits, relay cmd bits, sys_enable/force_off,
      seq, enable_mask, clear_faults.
    - Byte7: CRC8.
  - `0x0300|(0x3<<4)|plc_id` `EVSE_SLOW`
    - Max V/I/P (0.5 V / 0.2 A / 0.5 kW), auth/pending, hlc_enable, pnc_blocked, lock_cmd, proto_version (=3).
    - Byte7: CRC8.

- PLC → Controller
  - `0x0400|(0x6<<4)|plc_id` `PLC_TLM_V3`
    - Combined CP/HLC/relay/safety flags + EV targets + `limits_rx_count_lsb`.
    - Byte7: CRC8.
  - `0x0100|(0x7<<4)|plc_id` `ENERGY_METER` (if enabled)
  - `0x0100|(0x8<<4)|plc_id` `RFID_EVENT` (segmented UID events, if enabled)
  - `0x0200|(0x6<<4)|plc_id` `EMAID0` segments (optional)
  - `0x0200|(0x7<<4)|plc_id` `EMAID1` segments (optional)
  - `0x0200|(0x8<<4)|plc_id` `EVCCID` segments (optional)
  - `0x0200|(0x4<<4)|plc_id` `EVMAC` segments (optional)
  - `0x090000|plc_id` `BOOTCONFIG` (firmware + feature flags)

## CRC8

- CRC8 polynomial `0x07`, init `0x00`, over bytes `[0..6]`, stored in byte7.
- CRC8 is **mandatory** on: `EVSE_FAST`, `EVSE_SLOW`, `PLC_TLM_V3` (DLC=8).

## Ownership rules

- **Gun contactor (GC) ownership is PLC-only when `plc.gunRelayOwnedByPlc=true` (default: false).**
  - Controller will not set GC bits in `EVSE_FAST` when this flag is true.
  - PLC firmware must ignore/override any GC command bits from controller when it owns GC.
- Auxiliary relays (RLY2/RLY3) are used as KM_A/KM_B bus sectionalizers when `plc.moduleRelaysEnabled=true`
  (module-level ring cuts; bit0=KM_A, bit1=KM_B).
- `CLEAR_FAULTS` in `EVSE_FAST` is asserted only on explicit operator/service request (no auto-clear on any fault).

## Fault propagation (controller → PLC)

Controller packs a 6-bit fault mask into `EVSE_FAST` bits 31..36 each planner tick:
- bit0: general fault present
- bit1: communication fault
- bit2: isolation/earth/E-Stop fault
- bit3: thermal fault
- bit4: overcurrent fault
- bit5: weld detection (GC or MC)

PLC should map these into ISO/DIN ResponseCode + DC_EVSEStatusCode.

## Cadence and freshness requirements

- `EVSE_FAST`: controller keepalive cadence is mode-dependent, clamped to `<= 500 ms`.
  - Active (HLC stage > 0 or relay urgent/dirty): `300 ms`
  - Idle: `500 ms`
- `EVSE_SLOW`: controller keepalive cadence: `1500 ms`.
- `PLC_TLM_V3`: PLC telemetry cadence:
  - Active (HLC stage > 0 or CP connected): `300 ms`
  - Idle: `1000 ms`
- EVSE limit ACK watchdog:
  - PLC reports `limits_rx_count_lsb` in `PLC_TLM_V3`.
  - Controller reconstructs a wrap-safe counter and uses it as the EVSE limits ACK watchdog.

## Module CAN bandwidth policy

- This repo enforces a production module traffic budget in addition to PLC protocol cadence:
  - Target: `<20 kbps` total CAN load per interface in active steady-state.
  - Rolling window defaults: `windowMs=10000`, `bitsPerFrameEstimate=150`, `overCapDebounceMs=5000`.
  - Per-interface module budget is computed from configured cap minus PLC reserve:
    - `module_budget_kbps = maxTotalKbpsPerInterface - (1.5 * plc_count_on_iface)`.
- Traffic classes:
  - `SafetyUrgent`: allowed even during cap events (fail-safe shutdown path).
  - `Control`: subject to bandwidth governor.
  - `Telemetry`: subject to bandwidth governor and poll budget.
- Over-cap behavior:
  - If observed interface load remains above cap beyond debounce, controller latches `ModuleCanOverload`,
    blocks non-urgent module traffic, and drives safe derate/disable behavior.
- Protocol references used for module-side assumptions:
  - `docs/modules/CAN Communication Protocol - Maxwell_V1.50.pdf`
  - `docs/modules/ENR series CAN Comunication Protocol S0 (1).pdf`
  - `docs/modules/UUGreenPower CAN Protocol (36.2 Version)Reference Guide.pdf`
  - `docs/modules/TonHe CAN communication between charging module and monitor TONHE V1.2.pdf`
- Tonhe caveat:
  - Tonhe periodic/status uplinks include non-host-controllable inbound traffic (nominal ~500 ms plus triggers).
    If this inbound load alone keeps interface occupancy above cap beyond debounce, the controller latches
    `ModuleCanOverload` and holds modules in safe derate/disable state.

## Identity / Autocharge

PLC may emit EVCCID/EMAID/EVMAC in segmented frames. Controller reconstructs segments by `plc_id` and exposes tokens
to OCPP authorization.
