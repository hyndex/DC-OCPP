# TODO - Module Control Alignment (EVerest-Style CCCV)

## Stage 1 - Baseline Audit
- [x] Map end-to-end module control path (`OcppAdapter -> PowerModuleController -> driver CAN`).
- [x] Identify non-aligned control paths (mixed knobs, command thrash, strict fault escalations).
- [x] Capture current timing/cadence sources (planner loop + module keepalive + polling).

## Stage 2 - Control-Mode Simplification (Maxwell)
- [x] Enforce CCCV-only runtime control: `0x0021` voltage + `0x0022` current-limit-point + `0x0030` start/stop.
- [x] Remove runtime writes to `0x0023` overvoltage point.
- [x] Remove runtime writes to `0x001B` absolute current command.
- [x] Remove runtime writes to `0x0020` power command.
- [x] Keep command update behavior event-driven with bounded keepalive refresh.

## Stage 2b - Control-Path Parity (Tonhe + UUGreen/ENR)
- [x] Align OFF command behavior to edge + bounded OFF keepalive (remove disable churn).
- [x] Align UUGreen/ENR command sequencing with demand-up/down ordering (V/I limit ordering).
- [x] Confirm Tonhe scaling against protocol docs (0.1V/bit, 0.01A/bit).
- [x] Verify no mixed power-path command knobs are used in runtime for these drivers.

## Stage 3 - Telemetry Robustness
- [x] Keep Maxwell read validation strict (`status == 0xF0` only).
- [x] Ensure invalid/non-F0 samples are dropped and not mapped to 0 V / 0 A telemetry.
- [x] Keep slot-current aggregation resilient when one module sample is late.
- [x] Reduce false hard-faulting from transient current-imbalance status bit.

## Stage 4 - Integration Guardrail Cleanup
- [x] Ignore deprecated `sendOutputCurrent/sendOutputPower` knobs at runtime.
- [x] Emit explicit warning when deprecated knobs are present in config.
- [x] Re-check for leftover dead paths and remove if safe.

## Stage 5 - Validation
- [x] Compile `dc_ocpp` and fix any regressions.
- [ ] Run targeted session validation with connected EV (no full test suite).
  Notes: process restarted with fresh logging and CAN capture; runtime is stable. A full live charging cycle
  (precharge -> current demand) still requires active EV trigger in this run window.
- [x] Verify command cadence and polling behavior from logs.

## Stage 6 - Finalization
- [ ] Summarize exact code deltas with file/line references.
- [ ] Provide end-to-end gap report vs requested playbook.
- [ ] List known limitations / residual risks and next hardening steps.
