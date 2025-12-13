# Production hardening plan (multi-stage)

## Stage 1 — Controller correctness & timing
- [x] Decouple control/session cadence from meter upload interval to avoid 30s transaction start/stop latency.
- [x] Report Preparing/SuspendedEVSE on physical plug-in or CSMS disable even before a transaction exists.
- [x] Persist and replay pending RemoteStart/RFID tokens across process restarts.
- [x] Add <1s status/fault publish loop separate from metering to tighten OCPP event delivery.
- [x] Deterministic persistence/regression tests for auth token lifecycle and restart recovery.

## Stage 2 — PLC firmware alignment
- [x] Drive ISO15118/SECC auth gating from config param 20 (`auth_granted`) so PLC can signal Ongoing/Accepted (gates relay enable, exported via chargeinfo flags).
- [x] Implement lock actuator control + feedback sanity (config param 30, debounced feedback, chargeinfo flag).
- [x] Surface SLAC/HLC state, EV MAC, and auth request events over CAN for host coordination (EVMAC segmentation, auth pending/lock/auth flags in chargeinfo/session).
- [x] Add precharge/open-under-load sequencing with contactor feedback hooks and weld detection coverage (open-under-load deferral with timeout and fault flagging).
- [x] Integrate meter fallback/validation into shared_state so host can differentiate PLC vs shunt readings.

## Stage 3 — Resource allocation & safety envelopes
- [x] Cross-connector module sharing/ring topology planner validation with hardware capability flags (hardware capability gate in adapter).
- [x] Thermal/current derate regression tests: connector and module ramps, shutdown thresholds, recovery (derate_fault_tests).
- [ ] Global fault latch/clear UX: structured error reasons, auto-clear rules, and CSMS notifications.

## Stage 4 — Ring bus & islanding (hardware dependent)
- [x] Expand PLC command set to address GC/MC/MN independently per segment with feedback (new GCMC RX/TX CAN frames mapping to relays with command/feedback masks and faults).
- [x] Implement island manager (segment discovery, isolation, rejoin) with validation harness.
- [x] Power budget fairness tests for N-gun / 2N-module configurations under faults and contention.
