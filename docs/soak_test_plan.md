Long-Haul Soak Test Plan (24h+)
===============================

Goal: validate memory/thread stability, comms recovery, and metering monotonicity over extended runs
with multiple EVs/connectors.

Prereqs
-------
- Build `dc_ocpp` in Release mode and configure two connectors in `configs/charger.json`.
- Ensure logs are persisted (`logs/`), and a CSMS endpoint is reachable (SteVe or production backend).
- Populate certificates if testing TLS (see `docs/security_pki.md`).

Execution
---------
1) Start the charger with verbose logging:
   ```
   ./build/dc_ocpp --config configs/charger.json
   ```
2) Connect two EVs (or EV simulators) and alternate plug/unplug every 2–4 hours.
3) Drive at least three remote start/stop cycles per connector and one reset (Soft/Hard) during the window.
4) Let each charging session run for ≥2 hours with at least one paused/resume cycle to exercise
   the zero-amp pause path.
5) If PLC is enabled, pull the CAN cable briefly (<3 s) once to confirm comm fault debounce/recovery.

Monitoring
----------
- Watch `logs/` for `comm_fault`, `cp_fault`, `meter_stale`, and weld/lock errors. None should persist
  after recovery.
- Track process RSS and thread count every hour; memory growth should plateau (<5% drift).
- Validate OCPP transactions on the backend: energy counters must be monotonic and match meter totals
  within calibration tolerances.

Post-run validation
-------------------
- Trigger an OCPP `GetDiagnostics` (and optionally `GetLog`) via the CSMS and verify:
  - Charger responds `Accepted` immediately (non-blocking handler).
  - Status notifications progress `Uploading -> Uploaded` (or `UploadFailed` with a clear reason).
  - The uploaded bundle excludes secrets (no `*.pem`, `*.key`, or cert directories).
- Optional (maintenance validation): trigger `SignedUpdateFirmware` using a test artifact and verify
  notifications progress `Downloading -> Downloaded -> Installing -> Installed`, followed by a clean restart.
- Inspect the uploaded bundle for repeated timeouts/retries; investigate anomalies before release.
