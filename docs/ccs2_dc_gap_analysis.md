# CCS2 DC Gap Analysis (controller + PLC)

Updated: 2026-02-10

Context: align the controller (`src/ocpp_adapter.cpp`, `include/*`) and PLC firmware (`Ref/Basic/*`) with the CCS2 DC
workflow (ISO 15118-2 / DIN 70121: CP/PP, SLAC/SDP, CPD, CableCheck, PreCharge, PowerDelivery, CurrentDemand, stop).

## Architecture snapshot (current repo)
- Controller control loop tick: 50 ms (see `control_tick` in `src/ocpp_adapter.cpp`).
- PLC is authoritative for HLC stage truth, isolation state, and EV request values (targets/limits frames).
- Controller is authoritative for:
  - Power planning (module allocation / split charging)
  - EVSE limit envelopes pushed to PLC
  - Tie/GC commands when `plc.gunRelayOwnedByPlc=false`
- Telemetry sources are configurable via `connectors[].meterSource`:
  - Prefer module telemetry when available; fall back to PLC/meter/shunt as configured.
- Switching gates enforce "no opening under load" on island boundary changes:
  - Controller ramps current down to `<= switchMaxCurrentA` for `switchStableTimeMs` before re-segmenting islands.

## Implemented / aligned (no longer gaps)
- **CableCheck vs PreCharge vs PowerDelivery phase detection** is hardened (CableCheck does not energize HV).
- **PreCharge current clamp** is enforced (`<= prechargeMaxCurrentA`, default 2 A) and voltage is tracked to the EV request.
- **PreCharge close tolerance** uses `prechargeVoltageToleranceV` (default 20 V) for safe GC/tie decisions.
- **Stop + unlock safety** is enforced:
  - Current is reduced to ~0 A before opening contactors.
  - Unlock is gated by `unlockVoltageThresholdV` (default 60 V).
- **Measured vs reported V/I/P consistency** is checked and trips a local fault if mismatches persist (protects against
  stale/bad telemetry paths).
- **Split charging** supports cross-slot islands and dynamic re-partitioning (e.g., 2->1+1 modules across two guns),
  gated by the switching thresholds above.
- **ISO 15118 contract certificate (PnC) lifecycle via OCPP** is intentionally disabled in this release (Autocharge is used
  instead). OCPP "PnC" feature profile is removed from the default configs to avoid advertising unsupported behavior.

## Remaining items (require HIL / PLC validation or larger architectural change)
- **CurrentDemand update cadence**: the controller dispatch loop runs at 50 ms. If certification targets require reacting to
  faster EV CurrentDemandReq cycles (e.g., <20 ms), the PLC must own the fast loop or a faster deterministic controller
  path must be introduced. This is a performance/certification topic, not a functional blocker for typical deployments.
- **CPD completeness**: validate in HIL that the PLC always populates mandatory EVSEMaxPowerLimit/limit fields for the
  protocol variants you ship (DIN/ISO), and that the controller's max envelopes match hardware limits before CPD.
- **Authorization "pending" semantics**: validate in HIL that controller `AuthorizationState::Pending` maps to the correct
  ISO 15118/DIN "Ongoing" behavior on the PLC side for your EV population.
