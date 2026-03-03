# Simple Router Alignment (DC-OCPP-Split)

## Goal
Replace planner/control over-stabilization with a direct vehicle-target routing model:
- consume latest PLC/HLC target voltage/current,
- compute request power from that target,
- route to allocated modules,
- keep only minimal short-lived target caching for frame jitter.

## Everest Reference Used
Reference source:
- `ref/everest-core/modules/EVSE/EvseManager/EvseManager.cpp`

Relevant behavior:
- `subscribe_dc_ev_target_voltage_current(...)` updates latest EV targets.
- `apply_new_target_voltage_current()` sends targets directly:
  - `powersupply_DC_set(latest_target_voltage, latest_target_current)`.

## What Changed in DC-OCPP

### 1) Planner kept as simple router
- `src/power_manager.cpp` now routes per active gun from home-slot modules only.
- Removed complex budget/island/ramp/hysteresis strategy from planner implementation.

### 2) Target request path simplified in adapter
- `src/ocpp_adapter.cpp` (`apply_power_plan()`):
  - removed request continuity state machine,
  - removed cached `last_ev_target_power_kw_` replay,
  - now uses live target V/I with only short V/I hold window (`HLC_TARGET_VOLTAGE_HOLD_MS`, `HLC_TARGET_CURRENT_HOLD_MS`) for brief telemetry gaps,
  - computes `req_kw = V * I / 1000` directly,
  - retains precharge current clamp and low-current precharge keepalive behavior.

### 3) Stall recovery made diagnostic-only
- Removed auto no-energy recovery windows and retry escalation state.
- Stall detection now logs sustained underdelivery but does not inject pause/recovery control cycles.

### 4) Module command policy remains V/I only
- Adapter enforces `sendOutputPower` ignored at runtime (warn + force off).
- Module command emission remains voltage/current setpoints only.

## Net Runtime Behavior
- Fewer adapter-side control interventions.
- More direct PLC target to module command routing.
- Minimal cache retained only to bridge very short target-frame jitter.
- OCPP pause still forces no-energy mode; stall logic no longer forces pause/recovery windows.

## Basic Safety Coverage Check (Everest vs DC-OCPP)

### Implemented in DC-OCPP controller path
- E-stop / safety / earth fault trip and disable path.
- CP fault handling with debounce/grace then disable.
- Isolation fault, weld fault, overcurrent, overtemperature fault propagation and stop.
- Precharge safety controls:
  - precharge current clamp,
  - precharge timeout,
  - precharge overshoot / overcurrent fault checks.
- Stuck-output detection (unexpected voltage/current while output expected off).
- OCPP fault sync and hardware fault-state publishing.

### Where Everest has deeper direct logic
- Full cable-check orchestration and IMD self-test/wait loops are explicit in Everest `EvseManager`.
- In this stack, those are primarily owned by PLC/HLC + hardware side; controller consumes resulting status/fault signals.

### Important guard
- Keep `controller.labBypass=false` in production (default is false). Enabling it bypasses multiple safety faults by design.
