OCPP DC charger integration scaffold
====================================

This project wires up a multi‑gun DC charger against libocpp (https://github.com/EVerest/libocpp) and provides a thin orchestration layer you can extend with real hardware controls. The focus is OCPP 1.6 with N connectors (“guns”) and a small simulated hardware backend so you can exercise the protocol end‑to‑end.

Architecture at a glance
------------------------
- One controller (this repo) ↔ multiple PLC nodes, one PLC per connector/gun.
- Each PLC (Ref/Basic firmware) exposes 1 gun relay + 2 module relays; controller treats each PLC as one connector.
- Vehicle ↔ PLC (CP/SLAC/ISO15118) ↔ Controller (OCPP/auth/energy planner) ↔ Modules (via PLC relays).
- Controller owns OCPP, auth, session lifecycle, and power planning; PLC owns IEC61851/SLAC/ISO15118 timing and safety IO.
- Ring/islanding: controller planner can model multi-slot islands, but Basic PLC interface only actuates the two module relays per PLC and cannot switch MC/MN/ring contactors; cross-slot/island routing is blocked until PLC exposes those actuators.

What’s here
-----------
- `libocpp/` fetched from upstream for the OCPP 1.6/2.0.1 stack
- CMake project that builds a single binary `dc_ocpp`
- Configuration files under `configs/` with defaults for a 2‑gun DC charger
- Adapter code that:
  - boots libocpp with the provided config and schemas
  - registers all required callbacks for remote control, reservations, firmware/log uploads, etc.
  - exposes hooks to push meter values, errors, and session lifecycle from your EVSE controller
  - includes a lightweight simulated hardware implementation for local testing
  - optional CAN/PLC backend (per Ref/Basic/docs/CAN_DBC.dbc) for real guns via SocketCAN

Prerequisites
-------------
- CMake ≥ 3.16 and a C++17 compiler
- Dependencies used by libocpp (install via your package manager):
  - OpenSSL 3, libwebsockets, SQLite3, Boost (program_options, regex, thread)
  - nlohmann-json, nlohmann-json-schema-validator
  - everest-log, everest-timer, everest-evse_security, everest-sqlite
- Network access to your CSMS endpoint

Build
-----
This repository uses a pinned libocpp submodule (`v0.31.1`). After cloning, make sure submodules are initialized:
```bash
git submodule update --init --recursive
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The root CMake script fetches `everest-cmake` automatically. If your dependencies are installed in non‑standard locations, append them to `CMAKE_PREFIX_PATH`.

Optional unit tests for the planner:
```bash
cmake --build build -j --target power_manager_tests
./build/power_manager_tests
```

Run
---
```bash
./build/dc_ocpp --config configs/charger.json
```

The default config targets a SteVe instance on `ws://127.0.0.1:8180/steve/websocket/CentralSystemService/`. Update `configs/charger.json` to point at your CSMS, set the ChargePointId, and size the connector list to match your number of guns. Connector IDs must start at 1.
For PLC/CAN hardware, set `"usePLC": true`, `"canInterface": "can0"` (or your iface), and map each connector’s `plcId`. Current firmware/DBC includes `plcId` in TX IDs, so multiple PLCs can share one CAN interface as long as each has a unique `plcId`.

Project layout
--------------
- `configs/charger.json` – charger/topology settings (connectors, URIs, paths)
- `configs/ocpp16-config.json` – libocpp base config; dynamically patched with ChargePointId, URI, NumberOfConnectors
- `src/` and `include/` – adapter, config loader, simulated hardware, and the main entrypoint
- `data/` – placeholder certificate/key directories and user config file location
- `logs/` – message log output

Build prerequisites
- Boost (program_options, regex, thread) must be installed and discoverable by CMake.
- SocketCAN support is required for PLC mode (Linux).

Next steps to hook up real hardware
-----------------------------------
- Replace `SimulatedHardware` with calls into your EVSE controller (contactor control, meter reads, lock control).
- Call the `OcppAdapter` methods when real events occur:
  - `push_meter_values(connector, measurement)` on each meter sample
  - `begin_transaction(...)` / `finish_transaction(...)` when charging starts/stops
  - `report_fault(...)` / `clear_faults(...)` for error handling
- Populate the certificate bundles in `data/certs` with production CA chains and keys.
- Adjust sampling intervals and charging limits per connector in `configs/charger.json`.

PLC / CAN contract (summary)
----------------------------
Full specification lives in `Ref/Basic/docs/CAN_DBC.dbc` with narrative in `Ref/Basic/docs/can_dbc_overview.md`. Key points:

- **Bus**: 29-bit extended IDs, 125 kbps. Low nibble encodes `plcId` (0–15) on both TX and RX IDs. App CRC8 (poly 0x07, init 0x00) in byte7 when non-zero.
- **RX IDs (host → PLC)**  
  - `RelayControl`: `0x0300 | ((0x4<<4)|plcId)` — safety-critical. Signals: `RLY1_CMD`, `SYS_ENABLE`, `FORCE_ALL_OFF`, `CLEAR_FAULTS`, `CMD_SEQ`, enable mask, mode, pulse_ms, CRC. Timeout ≈300 ms forces relays off and faults.
  - `ConfigCmd`: `0x0300 | ((0x8<<4)|plcId)` — runtime config get/set. Signals: `CFG_PARAM_ID`, `CFG_OP`, `CFG_VALUE`, reserved, CRC.
  - `GCMC_Command`: `0x0300 | ((0x9<<4)|plcId)` — GC/MC/MN command mask + force-off/clear (maps onto existing relays for current hardware).
- **TX IDs (PLC → host)**  
  - `RelayStatus`: `0x0100 | ((0x6<<4)|plcId)` — relay states, switches, `SAFETY_OK`, `EARTH_FAULT`, `ESTOP_INPUT`, `FAULT_REASON`, `LAST_CMD_SEQ_APPLIED`, `COMM_FAULT`, CRC.  
  - `SafetySwitchStatus`: `0x0100 | ((0x9<<4)|plcId)` — debounced switch states, estop, safety_ok, earth_fault, mask, CRC.  
  - `EnergyMeterData`: `0x0100 | ((0x7<<4)|plcId)` — multiplexed:  
    - MUX0: flags `METER_OK/COMM_ERROR/DATA_STALE/OVERRANGE/FALLBACK_ACTIVE`, `VOLTAGE_0p1V`, signed `CURRENT_0p01A`, signed `ACTIVE_POWER_0p01kW`  
    - MUX1: same flags, `IMPORT_ENERGY_0p1kWh`, `FREQ_0p01Hz`  
  - `ConfigAck`: `0x0100 | ((0xA<<4)|plcId)` — echoes param/status/value/plcId, CRC.  
  - `GCMC_Status`: `0x0100 | ((0x5<<4)|plcId)` — GC/MC/MN command/feedback bits, fault reason, comm/safety flags, seqs.
  - Additional TX for CP/V2G telemetry (CP_Voltage_Levels, ChargingSession, RTEVLog/RTTLog, EVDC_* limits/targets) are defined in the DBC; firmware may publish them as available.
  - Identity frames now include EVCCID/EMAID0/EMAID1 **and EVMAC** (AutoCharge MAC) using segmented 5-byte payloads.
  - ChargeInfo flags include `auth_granted`, `auth_pending`, and `lock_engaged` in addition to HLC stage flags.
- **Fault semantics**: Relay timeout/bus-off/CRC fail drive `FAULT_REASON` and `COMM_FAULT`, force relays off. Estop/earth faults clear safety_ok. Remote force-off flagged separately. Missing RX beyond timeout sets comm_fault.
- **Driver behavior (current code)**:
  - SocketCAN filters per PLC ID, error-frame handling (bus-off/restart), CRC8 verification when present.
  - Safety debounced; relay commands expect `CMD_SEQ` ack with retry/backoff (3 attempts) then fault.
  - Meter staleness timeout 2 s; comm faults propagate to OCPP faults; ConfigAck logged.
  - Single CAN interface per config (one socket), multiple PLCs allowed via distinct `plcId`.

OCPP adapter behavior (current)
-------------------------------
- Per-connector state tracking drives `StatusNotification`:
  - Plug-in with/without session → Preparing; post-stop while still plugged or HLC charge complete → Finishing; Charging/SuspendedEV/Evse as appropriate.
  - Faulted on safety/estop/earth/comm/meter-stale/weld; clears to Available when healthy.
  - Seamless retry tolerance: brief CP/B1-B2 drops or quick unplug/replug within 8s keep the session and avoid Finishing.
- Auth pipeline:
  - Pending auth pushed to PLC (AuthorizationState::Pending) so PLC can reply ISO15118 AuthorizationRes=Ongoing.
  - Tokens flow from RFID/RemoteStart/Autocharge (EVCCID/EMAID/EVMAC), with 10s dedup and 24h local auth cache for offline acceptance.
  - Reservations enforced: sessions won’t start without matching idTag/parentId.
- Transaction start aligned with ISO15118 HLC `PowerDelivery` readiness, CP state, and lock feedback to prevent premature energy delivery or OCPP transaction start.
- Safety gating: all faults auto-stop transactions and disable connector; weld/isolation/lock faults mapped to precise OCPP error codes.
- Metering/control loop: 200 ms supervisor for state/fault/auth handling; meter push interval per connector; monotonic energy enforcement; meter-stale triggers fault; PLC/shunt source selectable.
- Firmware/diagnostics/log uploads: emit progress notifications (Downloading/Installing/Installed, Uploading/Uploaded); log bundling and HTTPS/file upload supported.

Configuration notes
-------------------
- `configs/charger.json` fields:
  - `chargePoint` block: `id`, `vendor`, `model`, `firmwareVersion`, `centralSystemURI`, `usePLC`, `canInterface`.
  - `plc` block: `useCRC8` (enable host-side CRC8 generation/verification on all PLC frames) and
    `requireHttpsUploads` (enforce HTTPS when pushing diagnostics/log bundles).
  - `connectors[]`: `id`, `plcId`, `label`, `maxCurrentA`, `maxPowerW`, `maxVoltageV`, optional `canInterface`,
    `meterSampleIntervalSeconds`, `requireLock`, `lockInputSwitch` (1-4 switch input for lock feedback),
    `meterSource` (`plc` or `shunt`), `meterScale`, `meterOffsetWh`, `minVoltageV`.
  - `slots[]` (optional explicit ring topology): `id`, `gunId`, `gc`, `mc`, `cw`, `ccw`, and `modules[]` each with
    `id` and `mn` contactor id. If omitted, slots/modules are auto-generated per connector.
  - `modulePowerKW` (per DC module rating), `gridLimitKW` (site-wide limit), and `defaultVoltageV` drive the power
    allocator for the 12-slot ring (2 modules/slot, 12 guns by default in config).
  - `security`: CA bundle paths and key/cert directories (ensure populated for TLS/OCPP security profiles).
- PLC constraints: unique `plcId` per connector; a single CAN interface is enforced by the host driver.

Planner/allocator overview
--------------------------
- Fast planner thread (100 ms) computes power budgets, discrete module picks, and island MC/MN/GC commands.
- Supports cross-slot borrowing to build multi-slot islands when a gun’s home slot lacks healthy modules (non-overlapping,
  contiguous expansion).
- Thermal-aware derating on connector temperature and module over-temp trip, with weld/isolation/safety gating before
  energizing.
- Logging emits per-cycle dispatch summaries for observability; unit tests for the planner live in `tests/`.

Operational playbooks
---------------------
- TLS/PKI provisioning: `docs/security_pki.md`
- Soak testing and resilience: `docs/soak_test_plan.md`
- HIL/system scenarios: `tests/HIL_PLAN.md`

Known gaps / TODO for production
--------------------------------
- PLC firmware must be validated to map AuthorizationState::Pending to ISO15118 AuthorizationRes=Ongoing on-device.
- Ring/islanding and cross-slot module routing remain blocked until PLC exposes MC/MN/ring actuators; current interface only supports two module relays per PLC.
- V2G/PnC flows are not supported in this release; Autocharge via EVCCID/EMAID/EVMAC is available.
- Run the soak test plan (`docs/soak_test_plan.md`) and HIL plan (`tests/HIL_PLAN.md`) before production rollout to validate long-haul stability.

Security and TLS
----------------
- Certificate bundles and key directories are created automatically on boot if missing.
- See `docs/security_pki.md` for provisioning TLS/PKI material for both OCPP and ISO 15118 (CSMS CA, MO/V2G CA, client certs/keys).
