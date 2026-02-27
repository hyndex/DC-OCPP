# Module Control Stability Hardening TODO

Last updated: 2026-02-27

## Stage 0 - Baseline and Evidence
- [x] Identify charge-drop root causes in module CAN governor, dispatch split logic, and OCPP underdelivery handling.
- [x] Map high-risk stop/session-drop paths across `power_module_controller.cpp`, `ocpp_adapter.cpp`, and `plc_can.cpp`.
- [x] Correlate with captured field run `live_autocharge_20260227T083043Z` and isolate first trigger:
      `target_gap` -> continuity hold -> hold expiry -> planner `I_lim=0A`.
- [ ] Correlate with fresh post-fix field logs/pcap from real-vehicle sessions (pending active charge run).

Dependencies:
- Access to latest `dc_ocpp` runtime logs.
- CAN trace from module bus during a dip event.

## Stage 1 - CAN Overload Recovery (Controller)
- [x] Add overload unlatch hysteresis (`overCapClearRatio`, `overCapClearHoldMs`) in `CanTrafficGovernor`.
- [x] Keep control dispatch active during overload (remove overload-as-hard-disable in module apply path).
- [x] Fix current/power per-module split to use actual active module count.
- [x] Keep overload as telemetry/diagnostic signal instead of forcing `fault_mask` in snapshot.
- [x] Add overload-aware telemetry polling downshift (poll budget reduction per iface while latched).

Dependencies:
- Config parser support for new clear-policy fields.

## Stage 2 - Module Driver Robustness
- [x] Stop converting startup-off timeout into immediate hard fault (keep recovery retries active).
- [x] Guard Maxwell 0x0020 output-power command for sub-floor ratios (avoid invalid low-ratio writes).
- [x] Increase default telemetry stale tolerance to reduce false dropouts under transient bus pressure.

Dependencies:
- Verify module response behavior with real Maxwell firmware under low-power/precharge setpoints.

## Stage 3 - OCPP/Planner Fault-Coupling Controls
- [x] Remove command-mutating underdelivery clamp (diagnostic-only underdelivery tracking/logging retained).
- [x] Remove `module_can_overload` from hard local fault and general fault synthesis.
- [x] Keep capability advertisement driven by planner/module availability, not measured-current ratcheting.

Dependencies:
- Validate no regressions in precharge/current-demand transitions.

## Stage 3.5 - Root-Cause Request Continuity (Current Collapse Origin)
- [x] Replace ad-hoc `last_nonzero_req_kw_seen_` timing map with explicit per-connector continuity state
      (`power_request_continuity_`) carrying last non-zero request, hold reason, hold start, and release logs.
- [x] Extend request generation to keep non-zero `req_kw` during active CP=C/D HLC power context if
      session/target metadata temporarily gaps and no explicit 0A target or safety fault exists.
- [x] Extend session-active gating so planner allocation does not drop modules during valid continuity hold windows.
- [x] Keep explicit-stop behavior intact: disable continuity hold on explicit 0A target, CP-not-ready, pause/disable, or faults.

Dependencies:
- Long-gap runtime validation with real EV to tune `HLC_REQ_KW_CONTINUITY_HOLD_MS` from field evidence.
- Correlation report generation confirming continuity hold only triggers for telemetry/session gaps (not real EV stop).

## Stage 3.6 - PLC Target Freshness Root Fix (Origin Elimination)
- [x] Add per-connector cached EV target data structure in `plc_can` state (`last_valid_ev_target_*` + timestamp).
- [x] Persist last valid target on every valid telemetry frame independent of `ev_target_recent`.
- [x] Expose cached targets as synthetic-fresh during active HLC power stage (bounded by 70 s lease) so planner input does not decay to stale-null mid-session.
- [x] Reuse cached target in PLC relay gating paths (`apply_power_command` / `apply_power_allocation`) to avoid false `ev_requesting=0` decisions from transient freshness drops.
- [ ] Validate with real-vehicle run that no `target_gap -> continuity hold expired -> I_lim=0A` sequence occurs.

Dependencies:
- Live connector-2 session with continuous CurrentDemand traffic while forcing PLC target freshness flaps.
- Correlated CAN + PLC + DC-OCPP logs proving sustained non-zero module commands through flap window.

## Stage 3.7 - Near-Target Voltage Stability (Root Control Law)
- [x] Add dynamic final-voltage margin policy in planner for active HLC power phase:
      ramp headroom between 4.0% and 3.5% of EV target voltage.
- [x] Keep precharge/non-HLC phases on fixed absolute voltage margin (no regression risk to precharge sequencing).
- [x] Wire new planner config keys (`finalVoltageMarginLowPct`, `finalVoltageMarginHighPct`) through config load and planner initialization.
- [x] Add unit tests for planner dynamic margin behavior and config normalization.
- [ ] Validate with real EV that near-target oscillation is removed and no late-session current collapse occurs.

Dependencies:
- Fresh connector-2 session logs with target voltage near BMS ceiling and per-cycle planner dispatch traces.

## Stage 4 - Configuration and Test Coverage
- [x] Extend `CanTrafficConfig` and `ModuleCanTrafficPolicy` with clear-policy fields.
- [x] Parse and validate new fields in `charger_config.cpp`.
- [x] Add/extend unit tests in `tests/charger_config_tests.cpp` for new config behavior.
- [ ] Add targeted unit tests for `CanTrafficGovernor` latch/unlatch transitions (not present yet).
- [x] Update targeted regression tests for underdelivery diagnostics (no command clamp behavior asserted).

Dependencies:
- Introduce test seam for governor state transitions.

## Stage 5 - Integration, Vehicle Validation, and Release Gate
- [x] Build and run focused unit/integration tests in this repo after patch set.
- [x] Start fresh controller runtime capture with synchronized timestamps in all streams:
      `live_autocharge_20260227T090624Z` (`PLC_TIMESTAMP_LINES=1`, CAN + PLC0 + PLC1 + DC-OCPP).
- [ ] Flash PLC/controller stack via `EVSE/scripts` flow and collect fresh logs.
- [ ] Execute real-vehicle HLC session validation on connector 2 / plc0 with dip-fault scenarios.
- [ ] Verify no session flaps across SLAC/ND/TCP/HLC handoffs in combined system logs.
- [ ] Prepare production rollout checklist and fallback toggles.

Dependencies:
- Physical EV + charger hardware access.
- Permission to run flash/start scripts on target bench.
