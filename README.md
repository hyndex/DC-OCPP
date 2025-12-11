OCPP DC charger integration scaffold
====================================

This project wires up a multi‑gun DC charger against libocpp (https://github.com/EVerest/libocpp) and provides a thin orchestration layer you can extend with real hardware controls. The focus is OCPP 1.6 with N connectors (“guns”) and a small simulated hardware backend so you can exercise the protocol end‑to‑end.

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
- **TX IDs (PLC → host)**  
  - `RelayStatus`: `0x0100 | ((0x6<<4)|plcId)` — relay states, switches, `SAFETY_OK`, `EARTH_FAULT`, `ESTOP_INPUT`, `FAULT_REASON`, `LAST_CMD_SEQ_APPLIED`, `COMM_FAULT`, CRC.  
  - `SafetySwitchStatus`: `0x0100 | ((0x9<<4)|plcId)` — debounced switch states, estop, safety_ok, earth_fault, mask, CRC.  
  - `EnergyMeterData`: `0x0100 | ((0x7<<4)|plcId)` — multiplexed:  
    - MUX0: flags `METER_OK/COMM_ERROR/DATA_STALE/OVERRANGE`, `VOLTAGE_0p1V`, signed `CURRENT_0p01A`, signed `ACTIVE_POWER_0p01kW`  
    - MUX1: same flags, `IMPORT_ENERGY_0p1kWh`, `FREQ_0p01Hz`  
  - `ConfigAck`: `0x0100 | ((0xA<<4)|plcId)` — echoes param/status/value/plcId, CRC.  
  - Additional TX for CP/V2G telemetry (CP_Voltage_Levels, ChargingSession, RTEVLog/RTTLog, EVDC_* limits/targets) are defined in the DBC; firmware may publish them as available.
- **Fault semantics**: Relay timeout/bus-off/CRC fail drive `FAULT_REASON` and `COMM_FAULT`, force relays off. Estop/earth faults clear safety_ok. Remote force-off flagged separately. Missing RX beyond timeout sets comm_fault.
- **Driver behavior (current code)**:
  - SocketCAN filters per PLC ID, error-frame handling (bus-off/restart), CRC8 verification when present.
  - Safety debounced; relay commands expect `CMD_SEQ` ack with retry/backoff (3 attempts) then fault.
  - Meter staleness timeout 2 s; comm faults propagate to OCPP faults; ConfigAck logged.
  - Single CAN interface per config (one socket), multiple PLCs allowed via distinct `plcId`.

OCPP adapter behavior (current)
-------------------------------
- Per-connector state tracking to issue `StatusNotification` transitions via libocpp callbacks:
  - Available → Charging when a session exists and relay closed.
  - Available/Suspended → Suspended when a session exists but relay open.
  - Any → Faulted on safety/estop/earth/comm/meter-stale; clears to Available when healthy.
- Transactions gated by safety_ok/comm_ok before start; faults auto-stop transactions and disable connector.
- Firmware/diagnostics/log uploads: now emit progress notifications (Downloading/Installing/Installed, Uploading/Uploaded) via libocpp status callbacks.
- Meter loop pushes measurements each interval; faults reported as OCPP errors (power meter, comm, safety).

Configuration notes
-------------------
- `configs/charger.json` fields:
  - `chargePoint` block: `id`, `vendor`, `model`, `firmwareVersion`, `centralSystemURI`, `usePLC`, `canInterface`.
  - `connectors[]`: `id`, `plcId`, `label`, `maxCurrentA`, `maxPowerW`, optional `canInterface`, `meterSampleIntervalSeconds`.
  - `security`: CA bundle paths and key/cert directories (ensure populated for TLS/OCPP security profiles).
- PLC constraints: unique `plcId` per connector; a single CAN interface is enforced by the host driver.

Known gaps / TODO for production
--------------------------------
- Smart charging: enforce charging profiles/station and per-connector limits against PLC setpoints.
- Availability/state transitions tied to CP states (plug/unplug) are not yet implemented; only relay/safety driven.
- Offline buffering/resume, auth list/cache, TLS/cert provisioning and renewal are not yet wired.
- Firmware/diagnostics uploads are mocked for progress only; no real file transfer.
- PLC: broader seq/ack beyond relay, finer debounce tuning, loss/reorder protection, richer telemetry usage remain open.
