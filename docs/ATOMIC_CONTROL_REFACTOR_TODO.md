# Atomic Control + Relay Ops Hardened Refactor TODO

Status legend: `[ ]` pending, `[~]` in progress, `[x]` done

## Stage 1 - Baseline + Gap Mapping
- [x] Capture latest failures from field logs (precharge stuck near measured voltage, session drop).
- [x] Compare current flow with split-ring rules (`owner[]`, tie truth rule, one-EV-per-island invariant).
- [x] Identify conflicting control overrides in `OcppAdapter::apply_power_plan()`.
- [x] Identify stale-target fallback paths that can self-confirm wrong voltage.

## Stage 2 - Single Source of Truth (Planner/Gating)
- [x] Introduce atomic output decision pipeline with explicit priority order:
  - `HardOff` (fault/disable),
  - `NoEnergy` (hold/gating),
  - `EnergyDelivery` (normal dispatch).
- [x] Remove late conflicting command overrides by centralizing command finalization.
- [x] Keep module mask/module count/GC/MC outputs coherent under a single decision branch.

## Stage 3 - EV Target Freshness + Voltage Continuity
- [x] Harden PLC target freshness handling for active HLC stages even when `ev_target_recent` jitters.
- [x] Store/hold latest valid EV target voltage for bounded continuity during telemetry jitter.
- [x] Remove precharge transition checks that fallback to dispatch voltage when EV target is stale.

## Stage 4 - Relay Ops Cleanup
- [x] Keep relay sequencing deterministic with one final command path per connector.
- [x] Enforce CP not-ready/unknown current cut after bounded debounce in power phase.
- [x] Preserve no-feedback welded-suspect guard behavior without duplicate state mutation paths.
- [ ] Remove remaining redundant local booleans and duplicate relay mask fallback branches.

## Stage 5 - Safety Priority Verification
- [x] Ensure safety/fault/disable always dominate over no-energy and normal energy-delivery paths.
- [x] Ensure no-energy pause clamps current/power to 0 without silently re-enabling later in same tick.
- [x] Ensure precharge path never bypasses current limit and timeout guards.
- [x] Add explicit unit test for `HardOff > NoEnergy > EnergyDelivery` precedence.

## Stage 6 - Output Goal Tests (End-to-End)
- [x] Rebuild `dc_ocpp` and test binaries after refactor.
- [x] Update deterministic unit tests to encode explicit output goals (precharge clamp, post-precharge hold, CP gate behavior).
- [x] Run focused test set:
  - `gc_precharge_no_timeout_tests`
  - `precharge_current_clamp_tests`
  - `post_precharge_hold_tests`
  - `power_delivery_stall_tests`
- [x] Run broader regression (`ctest --output-on-failure`) and document residual risks.

## Stage 7 - Closeout
- [ ] Summarize final behavior with file/line references.
- [ ] Record remaining non-blocking cleanup items for subsequent commit.

## Stage 8 - Live Session Root-Cause Hardening
- [x] Confirm from CAN traces that module current telemetry drops to `0A` while voltage setpoint is maintained (not a CAN parser artifact).
- [x] Gate planner stall-fault path on real voltage underdelivery (avoid no-energy recovery on EV-side current pauses with healthy EVSE voltage).
- [x] Tighten Basic non-precharge voltage headroom to sub-volt cap so EV target voltage is tracked closely during CurrentDemand.
- [ ] Flash updated Basic + DC-OCPP binaries and validate with fresh end-to-end logging session.

## Stage 9 - Passive-Tie + Ring-Break Determinism
- [x] Fix one-sided tie telemetry mirroring so passive/no-telemetry edges can still close safely (prevents `modules>0` with `mask=0` deadlock on single-module islands).
- [x] Harden forced single-owner ring-break selection to avoid active gun home segment where possible.
- [x] Add planner unit assertion for ring-break placement away from active home segment in full-island mode.
- [ ] Validate in live session that power transition no longer flips `MC=C -> O` with `mask=0x0` on active connector.

## Stage 10 - Second Verification Pass (Peer-Close + Local-Only MC)
- [x] Block power-phase hold extension when PLC stage has already fallen below precharge (`stage < PRECHARGE`).
- [x] Reject power-delivery inference on explicit pre-power HLC stages (targets-only fallback allowed only when stage unknown).
- [x] Remove local-only pass-through slot expansion so home MC stays open unless additional cabinets are actually borrowed.
- [x] Replace stale `hlc_precharge_active` usage in PLC status relevance/idle checks with stage-derived precharge truth.
- [x] Add persistent underdelivery clamp: derate commanded + advertised EVSE current to measured-deliverable range with recovery hysteresis.
- [ ] Re-run full regression (`ctest --output-on-failure`) and confirm all suites green after this pass.
