# Production Readiness Audit (2026-03-02)

## Scope
- Branch/worktree: `DC-OCPP-Split` (`/home/jpi/Desktop/EVSE/DC-OCPP`)
- Focus: end-to-end DC-OCPP integration path (OCPP adapter, planner handoff, module controller/drivers, PLC CAN, config/docs alignment).
- Constraint applied: keep module bridge simple (no added guard-heavy logic).

## Validation Executed
- Build: `cmake --build build -j6` (pass).
- Full tests: `ctest --test-dir build --output-on-failure` (31/31 pass).
- Additional focused regressions already re-run after module-control simplification:
  - `gc_precharge_no_timeout_tests`
  - `cable_check_no_hv_tests`
  - `precharge_default_module_policy_sim_tests`

## Confirmed Findings

### Medium
1. Production sign-off cannot be claimed yet due to open release-gate items that are explicitly still pending in repo tracking docs.
   - Evidence:
     - `docs/TODO_PRODUCTION_READINESS_TRACKER.md:13` (metrics backend still partial)
     - `docs/TODO_PRODUCTION_READINESS_TRACKER.md:25-27` (delivery supervisor split/API pending)
     - `docs/TODO_PRODUCTION_READINESS_TRACKER.md:63` (HIL validation pending)
     - `docs/TODO_PRODUCTION_READINESS_TRACKER.md:68` (fresh live session re-validation pending)
     - `docs/TODO_PRODUCTION_READINESS_TRACKER.md:73` (example configs still legacy conservative)
     - `docs/TODO_PRODUCTION_READINESS_TRACKER.md:78` (doc consolidation pending)
     - `docs/module_control_stability_todo.md:10,59,71,80,90-93` (field validation/release-gate tasks pending)
   - Impact: software passes repo tests, but production-readiness claim remains blocked by unexecuted field/HIL release gates.

### Low
1. Historical hardening docs include completed tasks for now-removed power-command controls (`0x0020` path) and can be misread as current runtime behavior.
   - Evidence: `docs/module_control_stability_todo.md:28`
   - Impact: operator/engineering confusion risk, not a runtime blocker.

## Changes Applied During This Audit
- Runtime module output control kept V/I-only.
- Runtime power/ceiling control path removed from Maxwell control (`0x0020`, `0x0023` not used).
- Module bridge simplified (removed controller per-slot command cache short-circuit).
- Default current-control mode switched to absolute command path (`0x001B`) unless explicitly overridden:
  - `include/charger_config.hpp`
  - `src/charger_config.cpp`
  - `include/power_module_controller.hpp`
  - `include/power_manager.hpp`
- Docs aligned to current runtime behavior:
  - `docs/modules/API_COMPLIANCE_MATRIX.md`
  - `README.md`
  - `docs/module_command_telemetry_compare_split_vs_before_refactor.md`
  - `docs/TODO_PRODUCTION_READINESS_TRACKER.md`
  - `scripts/maxwell_test.py` defaults/documentation updated for absolute-current default.

## Conclusion
- No confirmed code/runtime blockers were found in this repository pass (build + 31/31 tests green).
- Production-ready/no-bug-left cannot be honestly certified yet because documented field/HIL/release-gate tasks are still pending.
