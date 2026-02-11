# Power Delivery + Contactor Sequencing Refactor TODO

Status legend: `[ ]` pending, `[~]` in progress, `[x]` done

## Stage 1 - Baseline Alignment and Gap Mapping
- [x] Compare requested CCS/HLC flow against current `OcppAdapter::apply_power_plan()` behavior.
- [x] Identify blocking mismatch: GC was forced open during precharge in legacy sequencing.
- [x] Identify required sequencing updates: safe pre-close arm at ~0V, then precharge ramp with GC closed.
- [x] Identify required telemetry strategy update: avoid broadcast reads for module telemetry.

Dependencies:
- None.

## Stage 2 - Core Sequencing Refactor (OcppAdapter)
- [x] Add explicit per-connector precharge sequencing state for arm-at-0V clamp, safe-close hold, and ramped precharge.
- [x] Enforce GC close only after safe arm conditions are satisfied (no-feedback topology).
- [x] Remove legacy behavior that blocked GC close throughout precharge.
- [x] Keep precharge current limited to `precharge_max_current_a` while ramping to EV precharge target voltage.
- [x] Ensure precharge -> power transition does not drop voltage to 0V.
- [x] Ensure current ramps from precharge clamp to EV demand in power phase.

Dependencies:
- Stage 1 complete.

## Stage 3 - Fault/Timeout Hardening
- [x] Add precharge overcurrent detection (`I_OUT > precharge clamp + tolerance`).
- [x] Add precharge voltage overshoot detection during precharge ramp.
- [x] Add precharge transition timeout (EV never leaves precharge after stable hold).
- [x] Preserve safe-open behavior (avoid opening GC until near-zero current in normal stop path).

Dependencies:
- Stage 2 complete.

## Stage 4 - Module Telemetry Transport Compliance
- [x] Update Maxwell module driver to avoid broadcast telemetry reads.
- [x] Update rectifier module driver to avoid broadcast telemetry reads.
- [x] Keep control-write behavior unchanged except where safety required updates.

Dependencies:
- None.

## Stage 5 - Test Refactor and Coverage
- [x] Update tests that assumed `GC open during precharge`.
- [x] Add/adjust tests for safe close in precharge, `<=2A` clamp, no voltage drop at precharge->power transition, and staged current ramp-up.
- [x] Add/adjust tests for precharge fault/timeout behavior.
- [x] Run targeted tests and broader regression tests.

Dependencies:
- Stages 2-4 complete.

## Stage 6 - Design-Guide Blocker Remediation (v7 doc cross-check)
- [x] Latch `PrechargeOverCurrent` and `PrechargeVoltageOvershoot` local faults until unplug.
- [x] Harden Maxwell ratio fallback: unknown rated current at low voltage now forces safe `I_limit` ratio (`0`) instead of max ratio.
- [x] Normalize/derive `ratedCurrentA` for `maxwell-mxr`/`maxwell`/`maxwell-max` modules in config load path.
- [x] Harden PLC CP interlock gating so relay-close is blocked unless CP is fresh and in power-allowed state (`C`/`D`).
- [x] Make relay-open transitions urgent whenever commands move from ON->OFF, not only force-off transitions.

Dependencies:
- Stage 3 complete.

## Stage 7 - Documentation + HIL Plan Alignment
- [x] Align `docs/hil_validation_runbook.md` with implemented precharge contactor sequencing.
- [x] Align `tests/HIL_PLAN.md` expectations for precharge closure and transition behavior.
- [x] Explicitly document CP-loss mitigation timing targets from design guide in HIL guidance.

Dependencies:
- Stage 6 complete.

## Stage 8 - Final Verification and Closeout
- [x] Rebuild and run full unit/integration test suite (`ctest --output-on-failure`).
- [x] Verify no regressions in touched paths (`ocpp_adapter`, `plc_can`, `power_module_controller`, config parser, docs).
- [x] Publish final blocker status and residual-risk notes (if any) with file-level references.

Dependencies:
- Stages 1-7 complete.
