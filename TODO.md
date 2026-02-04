# TODO — Pre‑HLC/HLC Alignment & PnC/EIM Hardening

## Stage 1 — Analysis & Target State Machine
- [x] Map current PLC PWM/SLAC/HLC and host auth flows vs. required pre‑HLC/HLC timing rules.
- [x] Identify gaps: unconditional 5% PWM, no preauth gating, no PnC block, long/implicit auth waits.
- [x] Define target state machine + timing budgets (preauth indefinite, HLC auth ~150s).

## Stage 2 — CAN Contract & Interface Plumbing
- [x] Add CAN config params: `HLC_ENABLE` (digital comm request) and `PNC_BLOCKED` (force EIM).
- [x] Wire params through PLC config manager + CAN message handling (query/set + CRC logging).
- [x] Extend host hardware interface + PlcCanHardware to send new params reliably.

## Stage 3 — PLC Firmware Behavior
- [x] Gate CP PWM/SLAC/HLC on `HLC_ENABLE` (new PREAUTH mode, stop HLC when disabled).
- [x] Enforce total auth‑pending budget (~150s) in HLC (timeout → abort/fallback).
- [x] Use runtime PnC gate in ServiceDiscovery (suppress Contract when blocked).
- [x] Update PLC readme/config docs for new params + behaviors.

## Stage 4 — Host/OCPP Flow Control
- [x] Add per‑connector HLC control state (digital enable, PnC block TTL, last autocharge ID).
- [x] Implement PnC → EIM fallback (autocharge reject/timeout drops digital + blocks PnC).
- [x] Ensure EIM mode waits pre‑HLC and only starts digital after authorization.
- [x] Handle edge cases: remote start before plug, unplug resets block, post‑stop plug hold.

## Stage 5 — Config, Tests, Validation
- [x] Add charger.json settings for HLC auth timeout + PnC block TTL (and defaults).
- [x] Update config examples with HLC auth timeout + PnC block TTL.
- [x] Add/adjust unit tests for HLC gate + PnC fallback behavior.
- [x] Run targeted tests (auth_flow, pending_token_persistence, deterministic_vectors, limit_fallback, derate_fault, hlc_fallback).
