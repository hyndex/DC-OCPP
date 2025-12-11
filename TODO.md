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
