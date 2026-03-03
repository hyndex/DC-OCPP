# Production Readiness Tracker (DC-OCPP)

Last updated: 2026-02-28

This is the implementation tracker for the agreed hardening plan.  
Status legend: `[x] done`, `[~] in progress / partial`, `[ ] pending`.

## Stage 0: Instrumentation and Observability
- [x] Canonical EV-request logging in adapter/PLC paths.
- [x] Delivery-loss detection logs (`target_I`, `offered_I`, `measured_I`, timing).
- [x] Module capability/log export (`0x0040` flags + `0x0003` effects).
- [x] Stop-origin summaries (`planner_*`, fault reasons, HLC/CP context).
- [~] Add dedicated metric stream export (currently log-based; no metrics backend integration yet).

## Stage 1: Canonical EV Request Truth
- [x] Shared `derive_ev_power_request()` in `include/charging_request.hpp`.
- [x] Adapter uses canonical request for power-delivery gating.
- [x] PLC relay gating uses canonical request semantics.
- [x] Status mapping uses canonical request as a key input.

## Stage 2: Fast Delivery-Loss Handling
- [x] Detect underdelivery with 300 ms class window (`deliveryLossDetectMs`).
- [x] Clamp to no-energy mode immediately on confirmed loss.
- [x] Escalation window (`deliveryLossEscalationMs`) and logs.
- [~] Dedicated standalone `delivery_supervisor.*` module (logic currently inline in adapter).
- [~] Explicit on-demand module read trigger hook on loss (`0x0040/0x0003`) not yet split as dedicated API.

## Stage 3: Dynamic Module Capability Planning
- [x] Parse/export capability flags (`module_off`, `power_limited`, `temp_derated`, `ac_limited`).
- [x] Limit-point readback (`0x0003`) support with current capability derivation.
- [x] Planner uses dynamic capability (not only healthy-count static math).
- [x] Snapshot propagation: module capability fields flow into adapter/planner decisions.
- [x] Removed startup deadlock: `module_off` no longer treated as "unavailable" for module assignment/planner health.

## Stage 4: Stale-Target and Hold Policy
- [x] Hold defaults reduced to production values (`300 ms` class).
- [x] Request-loss debounce reduced to `300 ms`.
- [x] Target freshness hold reduced to `300 ms`.
- [x] Cached target power cleared on invalid contexts (unplug, CP-not-ready outside precharge, stage exit, stale invalid).
- [x] Delivery-loss path clears continuity hold and cached EV target power.

## Stage 5: Measurement Provenance Policy
- [x] `MeasurementSource` end-to-end (`Meter`, `Module`, `PlcPresent`, `EstimatedTarget`).
- [x] `measurement_source_is_control_trusted()` guard in control path.
- [x] Estimated target fallback remains display/fallback only; not trusted for protection logic.

## Stage 6: Planner and Ramp Stability
- [x] Measured-aware ramp anchor (`i_meas_a`) in planner.
- [x] Emergency drop mode on delivery loss/weld/safety.
- [x] Module capability current/power clamps included in dispatch path.

## Stage 7: Precharge Robustness (Latest Blocker)
- [x] Suppress module output drive while GC is open in explicit precharge.
- [x] Keep GC-open precharge in safe no-energy arm state.
- [x] Precharge ramp above arm voltage only with fresh EV target (or bounded short hold).
- [x] GC close gating now prefers fresh module voltage in precharge (even when connector meter source is not `module`).
- [x] Precharge present-vs-target mismatch can switch to module telemetry before GC closes (no relay-closed prerequisite).
- [x] Precharge-phase detection now accepts `hlc_precharge_active` at stage>=authorization when target is fresh and CP is C/D (prevents late module assignment).
- [x] Removed synthetic `800V` present-voltage fallback during precharge when no trusted/fresh measurement is available.
- [x] GC close gating now marks `V_meas_src=unknown` when precharge has no trusted present/module voltage (prevents false `present=800V` gating path).
- [x] Added targeted regression for auth-pending/zero-current precharge: `V_set` and module voltage must stay non-zero while module remains assigned.
- [x] Regression coverage: targeted precharge/stall/split tests passing.
- [~] Live 5-min and 30-min HIL/session validation with fresh captures after latest planner/auth fixes.

## Stage 10: Session-Start Guard Simplification (Latest)
- [x] EVMAC autocharge now grants locally when CSMS is offline (removes pending/retry startup deadlock).
- [x] Preserved online authorization path and EIM fallback for non-EVMAC flows.
- [ ] Re-validate with fresh live session capture that auth latency and precharge-to-delivery transition are stable.

## Stage 8: Config Baseline Hardening
- [x] Runtime defaults: `pollMs=200`, `cmdIntervalMs=200`, `readbackLimits=true`, `sendOutputCurrent=true`, `sendOutputPower=false`.
- [x] Deployed `configs/charger.json` updated to `cmdIntervalMs=200`.
- [~] Example configs still include conservative `1000 ms` values (kept for legacy examples; not production baseline).

## Stage 9: Documentation and Release Artifacts
- [x] Production hardening document updated (`docs/production_hardening_v2.md`).
- [x] Module API compliance matrix updated (`docs/modules/API_COMPLIANCE_MATRIX.md`).
- [~] Full state/timing diagrams and failure matrix are partially documented across runbooks; consolidate into single release note pending.
