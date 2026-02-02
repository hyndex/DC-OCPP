Split Charging Hardening TODO

Stage 0 — Audit + Alignment
- [done] Compare current split-charging flow vs spec (global plan, topology switching, open-loop safety, module-missing behavior)
- [done] Enumerate required code/test/doc changes and track in stages below

Stage 1 — Planner/Topology Behavior
- [done] Single active gun => one island that spans all slots (full ring), allocate all healthy modules
- [done] Allow empty/pass-through slots in island expansion (do not block topology when a module is missing)
- [done] Avoid opening MC boundaries when island already spans the full ring
- [done] Update PowerManager unit tests for single-gun/full-ring and empty-slot pass-through

Stage 2 — Slot/Relay Mapping
- [done] Always build two KM slots per PLC (KM_A/KM_B) even if a module is missing
- [done] Ensure relay-mask mapping uses KM_A/KM_B slot indices consistently
- [done] Keep module specs only for present modules; missing module => no module driver but bus slot remains
- [done] Update docs to reflect module-missing behavior and KM slot mapping

Stage 3 — Switching Safety + Open-Loop Enforcement
- [done] Gate MC switching using island-level telemetry (current + ΔV + stable time)
- [done] Block tie close if it would merge islands with multiple GC-closed requests
- [done] Freeze module output for islands involved in MC transitions (quiesce before switching)
- [done] Gate GC close on ΔV match (use precharge tolerance) and stable current
- [done] Treat telemetry-missing/stale as “no switch” and keep last safe topology
- [done] Fall back to gun telemetry for island gating when module telemetry is absent
- [done] Keep pass-through slots (no module + no gun) closed to avoid unmeasured islands
- [done] Add regression tests: tie close blocked by ΔV, blocked by current, telemetry invalid => no switch

Stage 4 — Verification
- [done] Run updated unit tests (power_manager_tests, charger_config_tests, new gating tests)
- [done] Re-scan for split-charging edge cases or regressions; update TODO accordingly
