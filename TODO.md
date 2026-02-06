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
