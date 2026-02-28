# Module API Compliance Matrix

This matrix maps `docs/modules` protocol requirements to the current runtime implementation in
`src/power_module_controller.cpp`.

## Scope
- Maxwell MXR (`maxwell-mxr` / `maxwell` / `maxwell-max`)
- ENR/UUGreen (`maxwell-enr` / `enr` / `uugreen` / `uugreenpower`)
- Tonhe (`tonhe`)

## Maxwell MXR (V1.50)
Implemented:
- Read telemetry: `0x0001` voltage, `0x0002` current, `0x0003` current-limit point, `0x0004` temp.
- Control: `0x0021` voltage set, `0x0022` current-ratio set, optional `0x001B` absolute current set,
  optional `0x0020` power-ratio set, `0x0030` start/stop, `0x0023` voltage upper limit.
- Status/capability: `0x0040` alarm/status, `0x0043` address/group, `0x004B` input mode.
- Alarm bit export: bit22 module off, bit23 power-limited, bit24 temp-derated, bit25 AC-limited.
- Dynamic capability export: `current_limit_point` + derived `current_capability_a`.
- Precharge guard: with GC open in precharge, controller suppresses module output drive to avoid sub-range
  voltage writes that MXR can reject with `0xF2`.

Primary references:
- `docs/modules/CAN Communication Protocol - Maxwell_V1.50.txt`

## ENR/UUGreen
Implemented:
- Read telemetry: command `0` voltage, `1` current, `30` inlet temp, `8` status.
- Control: command `2` voltage reference, `3` current limit, `4` on/off.
- Optional mode/config reads: `62`, `89`, `95`, `96`, `101`.
- Dynamic capability:
  - Poll/read command `104` (current capability).
  - Parse command `114` response (current + capability) as fallback.
  - Convert capability into planner-visible `available_current_a/available_power_kw`.
- Status-bit mapping (protocol-correct):
  - bit22 AC derate -> `ac_limited`
  - bit23 temperature derate -> `temp_derated`
  - bit24 PFC off -> treated as limiting (`ac_limited` + `power_limited`)
  - bit25 module off -> `module_off`

Primary references:
- `docs/modules/ENR series CAN Comunication Protocol S0 (1).txt`
- `docs/modules/UUGreenPower CAN Protocol (36.2 Version)Reference Guide.pdf`

## Tonhe
Implemented:
- Downlink: `PGN 000600h` start/stop + V/I setpoints, `PGN 00AA00h` input-mode config.
- Uplink parse: `PGN 000100h` state/voltage/current/fault, `PGN 000B00h` AC info/temp,
  `PGN 009100h` extended status/fault.
- Capability export:
  - `module_off` from state (`OFF`/`FAULT_OFF`).
  - `ac_limited` from ext bit8 (input power limit).
  - `temp_derated` from ext bit9 (power limit due to overtemperature).
  - `power_limited` derived from `ac_limited || temp_derated`.

Primary reference:
- `docs/modules/TonHe CAN communication between charging module and monitor TONHE V1.2.pdf`

## Planner/Snapshot Integration
Implemented:
- Snapshot no longer assumes Maxwell bit semantics for all protocols.
- Per-driver capability flags and dynamic current capability are propagated into:
  - `ModuleHealthSnapshot.available_current_a`
  - `ModuleHealthSnapshot.available_power_kw`
  - `ModuleHealthSnapshot.module_off/power_limited/temp_derated/ac_limited`
- Current hold windows and capability freshness are preserved with safe fallback behavior.

## Known Remaining Validation Work
- Multi-protocol hardware validation (Maxwell + ENR/UUGreen + Tonhe) with fresh logs.
- Long soak sessions under CAN burst/derate/fault transitions to finalize production sign-off.
