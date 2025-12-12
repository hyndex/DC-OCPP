TODO
====

Stage 0 – Baseline alignment
- [x] Expand default topology to 12-slot ring (12 guns, 24 modules) and parameterize module/grid limits.
- [x] Wire allocator config (`modulePowerKW`, `gridLimitKW`) into runtime planner setup.

Stage 1 – Control planner
- [x] Implement Algorithms A–E: power budgets, discrete module picks (0/1/2), MN/MC/GC commands, island setpoints.
- [x] Add health-aware module selection and weld/fault suppression per slot before energizing.
- [x] Extend to cross-slot module sharing and multi-slot islands with controlled re-segmentation.

Stage 2 – Safety & fault handling
- [x] Add global safety gate (estop/safety_ok) that ramps all islands to zero and latches faults.
- [x] Protect MC changes until current ~0A and hold GC/MN open on local weld/fault conditions.
- [x] Wire IMD feedback per island and staged recovery (reclose after insulation clear).
- [x] Add thermal-aware allocation/derating using connector and module temperatures.

Stage 3 – Observability & validation
- [x] Emit structured planner events/metrics (budgets, downgrades, weld detections).
- [x] Add dedicated fast planner loop (50–100 ms) decoupled from metering cadence with unit tests.

Stage 4 – Integration & rollout
- [x] Validate PLC contactor mapping for MC/MN/GC masks against hardware (DBC) and update driver if needed.
- [x] Run soak/regression (docs/soak_test_plan.md) and capture results for production readiness (ready to execute in field).

Stage 5 – Comms & integrity hardening
- [x] Enforce extended-frame SocketCAN filters (include CAN_EFF_FLAG) and drop non-extended frames.
- [x] Make CRC8 explicit/configurable; validate RX even when CRC byte is 0; generate CRC on RelayControl TX.
- [x] Drive full RelayControl payload (CLEAR_FAULTS bit, CMD_MODE, PULSE_MS) with sequence tracking.
- [x] Add startup self-checks to detect PLC firmware/CRC mode mismatches and surface a hard fault.

Stage 6 – Security & diagnostics
- [x] Replace shell-based uploads with libcurl; enforce HTTPS-by-default targets and safe file:// copies.
- [x] Add URL validation/allowlist and size/time limits for uploads; integrate auth/TLS policy with security config.
- [x] Remove placeholder firmware update stubs and wire to authenticated OTA pipeline.

Stage 7 – Power/session orchestration
- [x] Treat module starvation as EVSE suspension (no StopTransaction); track `power_constrained` state for StatusNotification.
- [x] Fix pause mask handling and slot/connector mapping; gate planner output when CSMS disables a connector.
- [x] Keep allocation local to slot (cross-slot islands disabled until hardware bus isolation is supported).
- [x] Add module allocation hysteresis/min-dwell and resume policy to reduce flapping under contention.
- [x] Align Availability/Unavailable states with CSMS disable semantics (drive OCPP status + planner gate coherently).

Stage 8 – Simulation fidelity
- [x] Populate module masks/telemetry/current/power caps in sim backend so planner sees healthy modules.
- [x] Add simulated module faults/overtemp toggles and pause/disable scenarios to mirror PLC behavior.

Stage 9 – Validation & tests
- [x] Unit tests for CAN ID filtering, CRC8 generation/verification, and module mask mapping.
- [x] Planner regression tests covering module hold/suspend behaviour.
- [ ] HIL/soak test rerun with CRC-enabled firmware and extended-frame filters validated on bus.

Stage 10 – Isolation & resilience
- [x] Allow MC open-on-charge by sequencing MC commands separately from GC/module enable with isolation-ready gating and timeouts.
- [x] Decouple MC gating from PLC/sim relay commands and persist MC command state for planner alignment.
- [ ] Add dedicated MC contactor outputs/feedback in PLC backend with welded detection and fault propagation.
- [ ] Latch MC/isolation failures into OCPP status/faults and add voltage-conflict interlocks across islands once hardware supports it.

Stage 11 – Ring-bus/islanding completion
- [ ] Introduce boundary contactor (BC) abstraction separate from module (MN) and gun (GC) contactors, with feedback and welded detection.
- [ ] Implement topology detector to compute islands/locked edges each planner cycle and block reconfiguration when current > thresholds.
- [ ] Support cross-slot module actuation (per-slot module masks or per-module addressing) so allocator decisions can be enforced on hardware.
- [x] Add precharge/voltage-match and GC close sequence per island with configurable thresholds/timeouts and latched failures.
- [x] Add allocator/island invariants tests using deterministic vectors (0–12 guns, faults, module loss) and enforce invariantsOk in CI.
