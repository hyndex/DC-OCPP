CCS2 DC hardening TODO
======================

Stage 0 — baseline and instrumentation
- [x] Capture repo-specific CCS2 DC gap analysis (`docs/ccs2_dc_gap_analysis.md`).
- [ ] Add CAN/HLC telemetry probes to log EVDC_Targets/EVAC cadence and CurrentDemand timing against planner loop.
- [ ] Build a minimal CAN replay harness to exercise PLC frames against the controller (plug/SLAC/CPD/CurrentDemand) for regression.

Stage 1 — P0 interop/safety
- [ ] Provide a fast CurrentDemand path (PLC-owned loop or <20 ms host loop) that applies envelopes locally and uses measured V/I in responses.
- [ ] Guarantee CPD carries EVSEMaxPowerLimit (DIN70121) and valid limits even when schedules are late; seed default `EvseLimits` before CPD.
- [ ] Enforce AuthorizationRes ongoing/finished behavior: map `AuthorizationState::Pending/Granted` to EVSEProcessing with a watchdog per mode.
- [x] Add measured V/I consistency checks between PLC CurrentDemandRes payload and power hardware telemetry; derate/stop on mismatch.
- [x] Add TTLs on envelopes/telemetry (EVDC_* heartbeats + limit ACK) that trigger controlled stop/derate when expired (configurable ACK timeout + telemetry watchdog).

Stage 2 — P1 certification/completeness
- [ ] Align precharge timing/voltage tolerance with ISO15118/DIN expectations; make per-gun configurable and validated against PLC HLC timings.
- [ ] Extend OCPP fault mapping to cover PLC-specific errors (CRC mode mismatch, comm loss, base EVSE frame stubs) with tests.
- [ ] Implement PnC/contract flows: EXI pass-through PLC→controller, OCPP `Get15118EVCertificate`/Authorize DataTransfer handling, TLS policy.

Stage 3 — P2 enhancements
- [ ] Support schedule renegotiation/tariff/value-added services and bridge to PLC messaging.
- [ ] Extend CAN contract for cross-slot/island contactors or constrain planner to current two-relay PLC interface.
- [ ] Build automated CCS2 DC HIL cases (CableCheck, PreCharge, CurrentDemand stress, weld detection) and integrate into CI.
