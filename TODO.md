# TODO

## Stage 1 - Analysis and alignment
- [x] Parse serial logs for precharge/power-delivery failures and confirm contactor never closes
- [x] Map PLC/host responsibilities and current GC gating paths (ocpp_adapter/plc_can/config)
- [x] Identify deadlock paths: GC close gating depends on telemetry completeness; no close-timeout safeguard

## Stage 2 - Controller logic fixes
- [x] Treat HLC power-delivery stage as authoritative even if precharge flag is stuck
- [x] Gate GC closure to PowerDelivery phase while allowing fallback close when telemetry is incomplete but voltage/current are within tolerance
- [x] Track GC close attempts; add timeout that fails safe with a clear fault reason and session stop
- [x] Add diagnostic logs for GC close gating/timeout decisions

## Stage 3 - Robustness and edge handling
- [x] Add/verify watchdog for prolonged EV power request with relay closed but zero current (avoid infinite SuspendedEVSE)
- [x] Ensure OCPP fault mapping includes GC close timeout reason and emits correct error code

## Stage 4 - Tests
- [x] Add unit test: Precharge stage must not close GC even when modules are assigned
- [x] Add unit test: GC close fallback when telemetry is incomplete but HLC power stage + voltage tolerance are satisfied
- [x] Add unit test: GC close timeout triggers a local fault when closure cannot be achieved
- [x] Add unit test: Power delivery stall triggers local fault when current stays near zero

## Stage 5 - Validation
- [x] Run unit tests including GC close + power delivery stall coverage
- [ ] Spot-check with log-derived scenario (precharge -> PowerDelivery -> CurrentDemand) to confirm contactor closes

## Stage 6 - Dynamic split charging alignment
- [x] Review planner vs spec: full-island behavior, module sizing, priority semantics, hysteresis direction
- [x] Update module sizing to ceil(request/module_power) with max-modules caps
- [x] Gate single-gun full-island to only when all modules are needed and no reserved slots
- [x] Fix priority weighting/downgrade ordering to favor lower numeric priority
- [x] Apply hysteresis only on module drops when previous modules remain available
- [x] Fix tie-gating for telemetry-less pass-through islands (mirror telemetry from either side) so ties can close/open
- [x] Relax tie-close current gating to allow boosting into a loaded island when the other side is quiet (precharge/warmup island)
- [x] Allow planner to open pass-through MC boundaries (do not force pass-through slots Closed) so multi-gun splits can form

## Stage 7 - Coverage and documentation
- [x] Refresh power_manager tests for new single-gun behavior and hysteresis
- [x] Add tests for ceil allocation, reserved-slot blocking, priority-driven allocation
- [x] Update split-charging design doc to match priority weighting and ceil sizing
- [x] Add end-to-end simulation coverage for 2-gun contention rebalancing (single EV gets 2 modules -> split 1+1 on second EV)
- [x] Add end-to-end simulation coverage for default-module precharge policy (precharge uses home module only; boost module added only in power phase)
- [ ] Add integration/soak coverage for multi-gun contention rebalancing (HIL/soak, real PLC timing)

## Stage 8 - Validation
- [x] Run `tests/power_manager_tests` (and other affected unit tests)
- [ ] Spot-check planner output for single-gun low-demand and reserved-slot scenarios

## Stage 9 - OCPP adapter hardening (status, tokens, availability)
- [x] Fix telemetry mismatch counter race (per-connector lock)
- [x] Fix last_module_alloc_ locking (plan_mutex_ only)
- [x] Correct charge-complete and safety stop reasons (Other vs EmergencyStop)
- [x] Handle connectorId=0 for enable/disable/pause/resume/stop/unlock; avoid duplicate status events

## Stage 10 - Power plan correctness
- [x] Compute module availability per island/connector (no global mask)
- [x] Fix fallback module mask using module_slot_index mapping
- [x] Prevent fallback from overriding module health/fault flags

## Stage 11 - Transactions and persistence
- [x] Capture meterStart at transaction start
- [x] Use double for meterStop through libocpp interface
- [x] Remove file I/O under session_mutex_ and write pending token file atomically
- [x] Skip persisting pending tokens when auth timeout disabled (avoid stale tokens)

## Stage 12 - Security and error lifecycle
- [x] Add mfCaBundle support in config and EVSE security
- [x] Fix resolve_bundle_path fallback behavior (empty when missing)
- [x] Track/clear local faults via sync_ocpp_error

## Stage 13 - Validation
- [x] Build + run impacted unit tests (auth_flow, pending_token_persistence, power_manager)
- [ ] Validate OCPP status sequence for charge complete -> Finishing -> Available
- [ ] Verify connectorId=0 ChangeAvailability flows on multi-connector config

## Stage 14 - CCS2 DC Control Recipe Alignment (EVSE-side)
- [x] Align HLC phase detection: distinguish CableCheck vs PreCharge vs PowerDelivery (avoid treating all stage<9 as "precharge")
- [x] Enforce PreCharge limits: clamp output current to <= 2 A during precharge; track EV requested V/I
- [x] Clamp precharge planning budget to <= 2 A so extra modules are not allocated/energized during precharge
- [x] Harden stop sequencing: prevent “held-closed” GC states from continuing to allow current (force 0A via planner gating + PLC open-guard)
- [x] Enforce unlock safety: block UnlockConnector unless V_out < 60 V (fail-safe on missing telemetry)
- [x] Implement telemetry sanity checks for stuck/weld/backfeed using available V/I measurements (no aux feedback)
- [x] Implement CP-lost / hot-unplug emergency shutdown targets (<5 A in 30 ms, <60 V in 100 ms) where feasible
- [x] Update configs/docs to reflect CCS thresholds (20 V precharge tolerance, 2 A precharge current, 1 A stop, 60 V unlock)
- [x] Remove unsafe “lab bypass” behavior from default control path; keep behind `controller.labBypass`
- [x] Respect `connectors[].meterSource` when choosing present V/I vs module telemetry for planning and switching gates

## Stage 15 - Tests
- [x] Add unit test: precharge current clamp applies even if EV requests >2 A
- [x] Add unit test: unlock blocked when voltage >= 60 V
- [x] Add regression test(s): CableCheck stage must not energize modules/raise HV setpoints

## Stage 16 - Validation
- [x] Run full unit test suite (`ctest` or individual binaries) and fix regressions
- [ ] Spot-check with representative logs/HIL: CableCheck -> PreCharge -> PowerDelivery -> CurrentDemand -> Stop -> Unlock
