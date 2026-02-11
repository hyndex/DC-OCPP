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

## Identity / Autocharge

PLC may emit EVCCID/EMAID/EVMAC in segmented frames. Controller reconstructs segments by `plc_id` and exposes tokens
to OCPP authorization.

