# HIL / Ops Validation Runbook (TLS + Maintenance + CCS2 DC)

This runbook is intended for production validation on real hardware (PLC + modules + guns) with a real CSMS.
It complements unit/simulation coverage by validating timing, contactor behavior, and network security end-to-end.

## Preconditions
- A charger build deployed on the target device (Linux recommended).
- PLC + CAN wiring verified and stable at the required bitrate.
- Real EV(s) or an ISO15118/DIN-capable EV simulator.
- A CSMS endpoint reachable over TLS (`wss://...`) with OCPP 1.6.

## 1) TLS Provisioning (SecurityProfile 3)
1. Configure `configs/charger.json`:
   - `chargePoint.centralSystemURI`: `wss://host:port/path` or scheme-less `host:port/path` (port 443 typical).
   - `security.csmsCaBundle`: CSMS CA bundle PEM.
   - `security.clientCertDir` / `security.clientKeyDir`: station TLS client cert/key directories.
2. Provision station client cert/key (naming is strict):
   - `${clientCertDir}/${charge_point_id}_cert.pem`
   - `${clientKeyDir}/${charge_point_id}_key.pem`
3. Start `dc_ocpp` and confirm:
   - Successful BootNotification.
   - Websocket connection is `wss` and remains stable across reconnects.
4. Negative checks:
   - Set `centralSystemURI` to `ws://...` with SecurityProfile 2/3 and confirm startup rejects with a clear error.
   - Remove client cert/key for profile 3 and confirm startup fails fast with a clear error.

## 2) Uploads Validation (GetDiagnostics / GetLog)
1. Ensure `uploads.requireHttpsUploads=true` (default) in `configs/charger.json`.
2. Trigger from CSMS:
   - `GetDiagnostics` and/or `GetLog`.
3. Confirm charger behavior:
   - Immediate `Accepted` response (handler is non-blocking).
   - Status notifications progress `Uploading -> Uploaded` (or `UploadFailed` with reason).
4. Confirm content policy:
   - Uploaded bundle must not contain private keys or cert bundles (`*.key`, `*.pem`, `*.p12`, `*.pfx`).

## 3) Firmware Update Validation (SignedUpdateFirmware)
1. Provision MF CA bundle:
   - `security.mfCaBundle` must contain the CA chain that issued the firmware signing certificate.
2. Configure firmware update policy:
   - `firmwareUpdate.enabled=true`
   - `firmwareUpdate.allowUnsigned=false` (recommended)
   - `firmwareUpdate.targetBinaryPath` points to the on-device `dc_ocpp` binary
   - `firmwareUpdate.systemdServiceName` matches the installed systemd unit
3. Trigger from CSMS:
   - `SignedUpdateFirmware` with:
     - `location` (HTTPS strongly recommended)
     - `signingCertificate` that chains to `mfCaBundle`
     - `signature` for the downloaded artifact
4. Confirm charger behavior:
   - Status notifications: `Downloading -> Downloaded -> Installing -> Installed`.
   - Persistent state file is created during update:
     - `${databaseDir}/fw_update_state.json`
   - State file is cleared on success.
   - Service restarts and comes back online on the updated binary.
5. Safety gates:
   - While transactions are active, installation is delayed up to `firmwareUpdate.maxWaitSeconds` then fails safe.
   - On restart failure, the charger rolls back using the `.bak` copy.

## 4) CCS2 DC Charging Flow Validation (Telemetry-Only Safe Mode)
Run one full DC session end-to-end:
1. CableCheck:
   - Confirm no HV energization (modules remain disabled / output remains safe).
2. PreCharge:
   - Confirm current is clamped to `<= prechargeMaxCurrentA` (default 2 A).
   - Confirm only the default/home module is energized during precharge.
3. PowerDelivery + CurrentDemand:
   - Confirm GC closes only in power phase (not in CableCheck).
   - Confirm dynamic module boost works:
     - Single EV demanding 60 kW: both modules allocated.
     - Second EV plugs: system safely re-segments to 1 module per gun without dropping EV1 GC.
4. Stop + Unlock:
   - Confirm current ramps to near 0 before opening contactors.
   - Confirm UnlockConnector is rejected until output voltage is below `unlockVoltageThresholdV` (default 60 V).

## 5) Failure Injection (Must Pass)
- CP loss / hot unplug while charging:
  - Current collapses quickly and output discharges.
- Stuck output / suspected weld (telemetry-only):
  - If commanded OPEN but voltage/current does not collapse, session faults safe and remains locked until service action.
- Network loss:
  - CSMS reconnect logic does not deadlock; active sessions follow local safety policy.

