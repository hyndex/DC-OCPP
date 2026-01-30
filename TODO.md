# Production Hardening TODO

This list tracks the end-to-end work required to align the controller and PLC (Ref/Basic) with the CAN contract, V2G flows, and production safety requirements. Update statuses as work completes.

## Stage 0 — Contract Alignment + Safety Gates (Controller)
- [x] Add CAN contract decoding for ChargeInfo/CP_Voltage_Levels/ChargingSession/RFID/EVCCID/EMAID/EVMAC/BootConfig.
- [x] Decode EVDC_Targets and use EV setpoints for power planning (avoid treating EVSE limits as targets).
- [x] Stop overwriting EVSE-present telemetry with setpoints; prefer fresh EVSE present, fallback to EV present.
- [x] Add PLC lock command support (PARAM_LOCK_CMD) for UnlockConnector + session relock.
- [x] Track per-PLC CP state, HLC stage/flags, lock feedback, and boot feature flags with freshness gating.
- [x] Implement periodic RelayControl keepalive (<=100 ms) and map relay ownership via `gunRelayOwnedByPlc`/`moduleRelaysEnabled`.
- [x] Emit auth tokens from RFID/EVCCID/EMAID/EVMAC frames with de-dup and connector hints.
- [x] Clear PLC auth pending when authorization is denied/unknown.
- [x] Record EVSE limit ACKs and surface them in `GunStatus` for safety watchdogs.
- [x] Meter fallback integration in PLC backend (energy integration + meter availability detection).

## Stage 1 — PLC Firmware Readiness (Ref/Basic)
- [x] Use CAN EVSE present/regulating as PSU readiness to unblock HLC PowerDelivery when hardware PSU is external.
- [x] Propagate CAN fault bits into PSU fault status (avoid false-ready on faults).
- [x] Validate protocol version handshake via ConfigCmd/ConfigAck (PARAM_PROTO_VERSION).
- [x] Wire module telemetry as the authoritative meter source (controller-side) and verify EnergyMeterData accuracy.

## Stage 2 — Security + Auth (Controller/OCPP)
- [x] Require valid CA bundles by default; allow insecure only via explicit opt-in flag/env var.
- [ ] (Deferred) Implement ISO15118/PnC DataTransfer handling if/when Plug-and-Charge is required.
- [x] Add configurable Autocharge id source selection (EVCCID/EMAID/EVMAC).
- [ ] Confirm idToken mapping strategy for Autocharge (EVCCID/EMAID/EVMAC) with CSMS.

## Stage 3 — System Tests + HIL
- [ ] Run CAN protocol tests (CRC + vector + end-to-end) against updated controller + PLC. (blocked: needs hardware)
- [ ] Execute HIL plan (`tests/HIL_PLAN.md`) with PLC hardware and CSMS. (blocked: needs hardware)
- [ ] Execute soak plan (`docs/soak_test_plan.md`) and record stability/telemetry results. (blocked: needs hardware)
- [ ] Verify security profile compliance (TLS, cert provisioning, revoke/renewal flows). (blocked: needs hardware/certs)

## Stage 4 — 3‑Relay PLC Hardware Alignment (GC + 2 Modules Only)
- [x] Add config flag to force 3‑relay mode and disable cross‑slot islands.
- [x] Ensure MC contactor logic is bypassed when only GC + 2 module relays exist.
- [x] Enforce adjacency-only module selection (per‑slot mask) with warmup support.
- [ ] Validate relay behavior on PLC with real hardware (GC + MN0 + MN1 only). (blocked: needs hardware)

## Stage 5 — Split‑Bus / ZC Topology Support (Future)
- [ ] Add JSON schema for bus segments + ZC contactors + module‑segment mapping.
- [ ] Extend PLC relay protocol to multi‑frame relay masks (ZC1..ZC2N).
- [ ] Implement segment‑graph planner and safe ZC sequencing.
- [ ] Add bus‑segment voltage sampling and mismatch protection.
