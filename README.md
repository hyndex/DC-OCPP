# Joulepoint DC OCPP Controller (PLC/CAN)

This repo contains the DC charger controller that integrates libocpp (OCPP 1.6) with a PLC front-end over SocketCAN. It manages connector state, authorization, sessions, and module power planning for split-charging topologies.

Highlights
- OCPP 1.6 adapter (libocpp submodule, currently pinned to v0.31.1).
- OCPP over TLS (SecurityProfile 2 and 3) with a file-based EVSE security backend (CA bundles + client cert/key).
- `dc_ocpp` runs only with the PLC CAN backend (no simulated hardware path in the current binary).
- Multi-connector support; each connector maps to a PLC node via `plcId`.
- Split charging with KM_A/KM_B tie contactors and module sharing.
- Module CAN drivers: `maxwell-mxr`, `maxwell`, `maxwell-max`, `maxwell-enr`, `enr`, `uugreen`, `uugreenpower`, `tonhe`.
- Single source of truth config in `configs/charger.json` (inline `ocpp` or `ocppConfig` base file).
- CSMS maintenance: async diagnostics/log uploads (GetDiagnostics/GetLog) and firmware updates (UpdateFirmware/SignedUpdateFirmware).

Architecture (high level)
- Vehicle <-> PLC (IEC 61851 / SLAC / ISO 15118 / DIN stack)
- PLC <-> controller over CAN (SocketCAN, 29-bit extended IDs, 125 kbps)
- Controller handles OCPP, authorization, sessions, power planning, and module dispatch
- Power modules talk over CAN (module drivers live in `src/power_module_controller.cpp`)

Quick start
1) Init submodules:
```bash
git submodule update --init --recursive
```
2) Build:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target dc_ocpp -j
```
3) Edit `configs/charger.json` (see config section below).
4) Run:
```bash
./build/dc_ocpp -c configs/charger.json
```

Notes
- `dc_ocpp` exits if `chargePoint.usePLC` is false.
- The CAN interface must exist (e.g., `can0`) or the PLC backend will fail to start.
- The shipped `configs/ocpp/v16/config.json` defaults to `SecurityProfile=2` (TLS with server authentication only).
  If you need mTLS, use `configs/ocpp/v16/config-mtls.json` and provision the station client cert/key as described in
  `docs/security_pki.md`.

Build prerequisites
- CMake >= 3.16 and a C++17 compiler
- Boost: log, log_setup, filesystem, thread, regex, date_time
- OpenSSL, libcurl, SQLite3
- libwebsockets

CMake fetches several deps automatically (everest-cmake, everest-log, everest-timer, everest-evse_security, everest-sqlite, nlohmann-json, json-schema-validator, date) when not available locally.

Helper script
```bash
./scripts/build_dc_ocpp.sh
```

Run helpers
- `./scripts/dc_ocpp_run.sh` (uses `DC_OCPP_BIN` and `DC_OCPP_CONFIG` if set)
- `./scripts/install_systemd_service.sh` (Linux systemd unit using `dc_ocpp_run.sh`)

Config reference (charger.json)
Paths are resolved relative to the config file location. The loader creates missing directories and initializes `userConfig` if needed. Values persisted via OCPP ChangeConfiguration are stored in `userConfig` and merged into the base config on startup.

Top-level sections
- `chargePoint`
  - `id`, `vendor`, `model`, `firmwareVersion`
  - `centralSystemURI` or `ocppEndpointToBackend` (the latter overrides ID + URI from the URL path)
  - `usePLC`/`usePlc` (must be true for `dc_ocpp`), `canInterface` (default CAN interface)
  - Optional identity fields: `chargePointSerialNumber`, `meterSerialNumber`, `meterType`, `iccid`, `imsi`, `imei`, `apn`
- `firmwareUpdate`
  - `enabled`, `allowUnsigned` (unsigned updates are disabled by default), `stagingDir`
  - `systemdServiceName`, `targetBinaryPath`, `maxWaitSeconds`
- `controller`
  - `freeMode` (auto-authorize on plug-in)
  - `defaultTag` (token used when `freeMode` is true)
  - `labBypass` (DANGEROUS: bypass certain safety faults; must be false in production)
- `connectors[]`
  - `id` (1..N, unique), `plcId` (0..15, unique; default `id-1`)
  - `label`, `canInterface` (optional override), `maxCurrentA`, `maxPowerW`, `maxVoltageV`, `minVoltageV`
  - `meterSampleIntervalSeconds`, `meterSource` (`plc` | `module` | `shunt`)
  - `meterScale`, `meterOffsetWh`, `requireLock`, `lockInputSwitch` (1..4)
- `plc`
  - `enabled` (defaults true), `useCRC8`
  - `gunRelayOwnedByPlc` (must be false for split charging), `moduleRelaysEnabled` (must be true)
  - `relayMode` (only `ties` is supported), `relayFeedbackAvailable`
  - `autochargeIdSource` (`evmac`, `evccid`, `emaid`)
  - `requireHttpsUploads` (enforces HTTPS for OCPP uploads; PLC backend rejects uploads by default)
- `slots[]` (optional explicit topology; auto-generated if omitted)
  - `id`, `gunId`, `gc`, `mc`, `cw`, `ccw`
  - `modules[]` with per-module metadata
- `planner` / `siteLimits` / top-level planner fields
  - `modulePowerKW`, `gridLimitKW`, `defaultVoltageV`
  - `allowCrossSlotIslands`, `maxModulesPerGun`, `minModulesPerActiveGun`, `maxIslandRadius`
  - `minModuleHoldMs`, `minMcHoldMs`, `minGcHoldMs`
  - `mcOpenCurrentA`, `gcOpenCurrentA`, `tieCloseMaxDeltaV`, `switchMaxCurrentA`, `switchStableTimeMs`
  - `requireAuthForPrecharge` (alias: `blockPrechargeUntilAuthorized`)
  - `prechargeMaxCurrentA` (CCS precharge clamp; default 2 A)
  - `prechargeTimeoutMs`, `prechargeVoltageToleranceV` (default 20 V)
  - `unlockVoltageThresholdV` (default 60 V)
- `timeouts`
  - `authorizationSeconds`, `hlcAuthorizationSeconds` (clamped to 150s), `pncBlockSeconds`
  - `powerRequestSeconds`, `evseLimitAckMs`, `telemetryTimeoutMs`
  - `plcPresentWarnMs`, `plcLimitsWarnMs`
- `uploads`
  - `maxBytes`, `connectTimeoutSeconds`, `transferTimeoutSeconds`, `allowFileTargets`
- `security`
  - `csmsCaBundle`, `mfCaBundle`, `moCaBundle`, `v2gCaBundle`
  - `clientCertDir`, `clientKeyDir`, `seccCertDir`, `seccKeyDir`
- `ocpp` (inline base config) or `ocppConfig` (path to base config JSON)
- `sharePath`, `sqlMigrationsPath`, `databaseDir`, `userConfig`, `messageLogPath`, `loggingConfig`
- `meterSampleIntervalSeconds`, `meterKeepAliveSeconds`, `minimumStatusDurationSeconds`, `moduleHealthGraceMs`

Module config (`slots[].modules[]`)
- `id`, `mn` (module contactor id), `type`
- `canInterface`, `address`, `group`
- Optional vendor fields: `monitorAddress`, `productionDay`, `serialLow`, `sourceAddress`,
  `inputMode`, `hiLoMode`, `silentMode`
- `ratedPowerKW`, `ratedCurrentA`, `pollMs`, `cmdIntervalMs`, `telemetryStaleMs`
- `broadcast`, `probeOnStartup`, `readbackLimits`, `sendOutputCurrent`, `sendOutputPower`

Constraints and defaults
- If `slots` is omitted, a ring is auto-generated from `connectors` order with two modules per slot.
- Each slot supports at most 2 modules in the current implementation.
- `plc.relayMode` must be `ties`, `plc.moduleRelaysEnabled` must be true, and `plc.gunRelayOwnedByPlc` must be false.
- `allowCrossSlotIslands` requires hardware support (`PlcCanHardware` supports this).
- `databaseDir` also stores pending auth tokens at `pending_tokens.json`.
- Autocharge can be toggled at runtime via OCPP ChangeConfiguration key `Custom.AutochargeEnabled`.

PLC/CAN contract
- Authoritative details: `docs/CAN_CONTRACT.md` and `Ref/Basic/docs/CAN_DBC.dbc`.
- Bus: 29-bit extended IDs, 125 kbps, CRC8 on required frames.
- Handshake: controller expects `PROTO_VERSION` (param 91) and treats mismatches as comm faults.

Raspberry Pi / Docker builds
```bash
./docker/build-rpi.sh
# 32-bit Raspberry Pi OS
PLATFORM=linux/arm/v7 ./docker/build-rpi.sh
```
Artifacts land in `build-rpi/artifacts/bin` and `build-rpi/artifacts/share/`.

Testing
- Planner tests: `cmake --build build --target power_manager_tests -j` then `./build/power_manager_tests`.
- CAN loopback sanity: `./scripts/can_loop_test.sh can0`.

Operational docs
- TLS/PKI provisioning: `docs/security_pki.md`
- Soak testing: `docs/soak_test_plan.md`
- HIL scenarios: `tests/HIL_PLAN.md`
- HIL/Ops validation runbook (TLS + maintenance + CCS2 DC): `docs/hil_validation_runbook.md`

Limitations and notes
- OCPP adapter is 1.6 only (libocpp supports 2.0.1/2.1, but this binary uses v16).
- `dc_ocpp` ships with the PLC CAN backend only. To use custom hardware, implement `HardwareInterface` and update `src/main.cpp`.
- Diagnostics/log uploads and firmware updates are handled asynchronously by the controller (independent of PLC backend).
- OCPP SecurityProfile 2 and 3 are supported (TLS). `DC_OCPP_STUB_SECURITY=1` is dev-only for SecurityProfile 0 and is ignored for TLS profiles.
