- Stage 0 – Safety hardening (P0)
  - [x] Add config to hand PLC gun relay ownership (no controller-driven GC closes; safe opens only).
  - [x] Refresh PLC auth state periodically (Pending/Granted re-push every ~1s to avoid 15118 timeout).
  - [x] Propagate EVSE fault bits toward PLC via EVSE_PRESENT flags; compute mask every planner tick.
  - [ ] Wire PLC firmware to consume fault bits and ownership mode (map to ResponseCode/EVSEStatus).
  - [x] Validate EVSE_PRESENT (>=10 Hz) and EVSE_LIMITS (>=1 Hz) cadence on hardware with alerts on staleness.

- Stage 1 – Feature completeness (P1)
  - [x] Enforce single GC authority in controller/PLC boundary (controller suppresses GC when PLC owns it; relay conflicts halt session).
  - [ ] Map module/controller faults into ISO/DIN ResponseCode and DC_EVSEStatus (include weld/isolation/derate) in PLC firmware. *Blocked: requires PLC firmware source changes.*
  - [ ] Reconcile TLS/Contract advertising with stack availability (disable Contract when TLS absent or add stack). *Blocked: requires PLC TLS stack or policy change in PLC code.*

- Stage 2 – Testing & docs (P2)
  - [x] Document CAN contract (IDs, payload bits incl. new fault flags, expected publish rates).
  - [x] Add counters/metrics for auth refresh pushes, relay command conflicts, stale present/limit telemetry.
  - [ ] Run end-to-end sims/hardware tests: long-latency auth, mid-charge fault propagation, weld detection.
