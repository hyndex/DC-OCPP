# Production Hardening V2 (Canonical Request / Delivery Loss / Dynamic Capability)

## Scope
This document defines the current production behavior added in Stage 18:
- Canonical EV request semantics shared across PLC and adapter.
- Fast delivery-loss clamp path.
- Dynamic module capability propagation from module telemetry to planner dispatch.
- Measurement provenance policy for control decisions.
- Precharge sequencing guard to avoid GC-open low-voltage module command rejects.

## Canonical EV Request
Source: `include/charging_request.hpp`

`derive_ev_power_request()` is the single request-truth function used by:
- `src/ocpp_adapter.cpp` (`power_delivery_requested`, contactor/request gating, state mapping).
- `src/plc_can.cpp` (relay gating in `apply_power_command` and `apply_power_allocation`).

Request validity requires:
- plugged-in
- no comm fault
- no CP fault
- not charge-complete
- lock engaged (if lock required)
- CP power-ready (`C`/`D`)
- HLC power phase
- target current present and above threshold
- target voltage present and sane
- target timestamp fresh

## Delivery-Loss Fast Path
Source: `src/ocpp_adapter.cpp` (underdelivery block in power command path).

Current behavior:
- Detect underdelivery when:
  - power-delivery context active
  - target fresh
  - offered current >= 5 A
  - measured current < `max(0.5 A, 0.2 * offered)`
  - persists for `deliveryLossDetectMs` (default 300 ms)
- On trigger:
  - set command current to `0 A`
  - set command power to `0 kW`
  - switch to no-energy pause mode
  - clear continuity hold and cached EV target power
  - log escalation if sustained beyond `deliveryLossEscalationMs`

## Precharge Sequencing Guard (MXR Compatibility)
Source: `src/ocpp_adapter.cpp`

Behavior:
- During explicit precharge with GC open, module output drive is suppressed.
- GC-open precharge command stays in safe no-energy arm state.
- Precharge voltage ramp is allowed only with fresh EV target voltage (or short target hold cache).
- This avoids low-voltage `0x0021` writes that can be rejected (`0xF2`) by some module firmware, which can otherwise
  trigger false precharge failure paths.

## Dynamic Module Capability Policy
Sources:
- `src/power_module_controller.cpp`
- `src/ocpp_adapter.cpp`
- `src/power_manager.cpp`

Exported capability fields:
- `available_current_a`
- `available_power_kw`
- `module_off`
- `module_power_limited`
- `module_temp_derated`
- `module_ac_limited`
- `limit_fresh`

Telemetry mapping:
- 0x0040 bit 22: module off
- 0x0040 bit 23: power limit
- 0x0040 bit 24: temperature derating
- 0x0040 bit 25: AC limit
- 0x0003: current limit point (limit scale)

Planner behavior:
- Total power budget uses summed available power, not only healthy-module count.
- Module allocation uses available-module count (`healthy && !off && !severe_fault`).
- Dispatch current clamps against available current when present.

## Measurement Provenance Policy
Sources:
- `include/hardware_interface.hpp`
- `src/plc_can.cpp`
- `src/ocpp_adapter.cpp`

`MeasurementSource` is attached to present V/I/P:
- `Meter`
- `Module`
- `PlcPresent`
- `EstimatedTarget`
- `Unknown`

Control policy:
- Estimated target is treated as non-authoritative for protection/supervision.
- Adapter control path uses measured-source policy (`measurementSourcePolicy`).

## Config Baseline (Stage 18)
Source: `include/charger_config.hpp`, `src/charger_config.cpp`

Defaults:
- `pollMs = 200`
- `cmdIntervalMs = 200`
- `readbackLimits = true`
- `sendOutputPower = false` (unchanged)

New knobs:
- `hlcTargetHoldMs` (default 300)
- `requestLossDebounceMs` (default 300)
- `targetFreshHoldMs` (default 300)
- `deliveryLossDetectMs` (default 300)
- `deliveryLossRecoveryMs` (default 500)
- `deliveryLossEscalationMs` (default 1800)
- `activeCurrentHoldMs` (default 250)
- `measurementSourcePolicy` (default `strictMeasured`)

## Known Open Items
- Dedicated delivery supervisor module (`delivery_supervisor.*`) is not split out yet (logic is inline in adapter).
- Immediate on-demand module read trigger hooks (0x0040/0x0003) are not yet explicit.
- Full HIL/soak validation matrix remains pending.
