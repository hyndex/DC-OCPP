# DC-OCPP Split vs Before-Refactoring-Power

## Module Command + Module/Vehicle Telemetry End-to-End Comparison

- Generated (UTC): 2026-03-02T18:12:24Z
- Current branch/worktree: `DC-OCPP-Split` (`/home/jpi/Desktop/EVSE/DC-OCPP`, HEAD `b34809b`, `origin/DC-OCPP-Split` at `978e055`)
- Baseline branch/worktree: `Before-Refactoring-Power` (`/home/jpi/Desktop/EVSE/DC-OCPP_before`, HEAD `018f6cd`)
- Scope: module command path, module telemetry path, vehicle telemetry path, planner consumption, PLC/HLC relay gating, tight guards and logical-risk checks.

## Pull Confirmation

- Verified pull status at `2026-03-02T17:38:08Z`.
- `DC-OCPP-Split` pull result: up to date with `origin/DC-OCPP-Split` at `978e05572a082fc28514dc056341e5f2a44fdfd0`; local branch remains one commit ahead at `b34809b4626db54bebb579dd799c590b1fb2b923`.
- `Before-Refactoring-Power` pull result: local and origin both at `018f6cd9c7e75dcf3bfe61f10d0369d41f122aee`.

## 2026-03-03 Root-Cause Confirmation + Fix (CAN/PlcCommFault)

### Confirmed field symptom
- `PlcCommFault` is raised from repeated PLC CAN TX failures of EVSE fast-control frames:
  - `id=0x321` (PLC1) and `id=0x322` (PLC2),
  - `errno=105 (No buffer space available)`,
  - seen in `/home/jpi/Desktop/EVSE/logs/live_autocharge_20260302T183408Z/dc_ocpp.stderr.log`.

### What this means
- Module telemetry is still being read (module CAN stats and module RX traffic are present), so the failure is not "module telemetry missing".
- The failing path is PLC control TX starvation/backpressure on shared `can0`, which then escalates to communication fault handling.

### Last-commit clues (`b34809b`) that increased fragility
- Module defaults became more aggressive at runtime:
  - `pollMs/cmdIntervalMs` moved to `200/200` defaults in config parser/model.
  - `readbackLimits` default moved to `true` (extra readback traffic paths enabled by default).
- New control/fault debounce defaults were very tight (hundreds of ms), making brief disturbances more likely to trip control-loss paths.
- Underdelivery handling in adapter moved from diagnostic behavior to active `0A/0kW` clamp/pause behavior.

### Applied simplification/stability changes (local workspace)
- Restored conservative module defaults:
  - `pollMs/cmdIntervalMs` default back to `500/500`.
  - `readbackLimits` default back to `false`.
- Updated active config (`configs/charger.json`) to match:
  - `readbackLimits=false`, `pollMs=200`, `cmdIntervalMs=500`.
- Restored longer debounce/hold windows:
  - `hlcTargetHoldMs=12000`,
  - `requestLossDebounceMs=12000`,
  - `targetFreshHoldMs=12000`,
  - `deliveryLossDetectMs=2500`,
  - `deliveryLossRecoveryMs=6000`,
  - `deliveryLossEscalationMs=45000`,
  - `activeCurrentHoldMs=800`,
  - `measurementSourcePolicy=allowEstimated`.
- Reverted underdelivery behavior to diagnostic-only (no forced `0A/0kW` pause/clamp).
- Removed now-unused underdelivery confirmed-latch state from adapter.

### Build verification
- `cmake --build build --target dc_ocpp` completed successfully after these changes.

## Post-Review Simplification Applied (Local Workspace)

Following your module-control review request, local `DC-OCPP-Split` was simplified to keep the module bridge V/I-only and reduce control-path complexity:

- Module command path is now V/I-only at runtime:
  - `OcppAdapter` forces `sendOutputPower` off for module runtime state/spec propagation.
  - `ModuleCommandRequest.power_kw` is forced to `0.0` before dispatch to module controller.
  - Default current command mode is absolute (`sendOutputCurrent=true`, `0x001B`).
- Maxwell command path removed from active control:
  - Removed `0x0020` output-power writes.
  - Removed `0x0023` upper-voltage-ceiling writes/retry path.
  - Active control now stays on startup/shutdown + `0x0021` voltage + `0x0022` ratio current / `0x001B` abs current + `0x0046` input mode.
- Maxwell current command semantics aligned closer to field script behavior:
  - Ratio path allows values up to `3.5` (`0x0022`) instead of a hard `1.0` cap.
  - Absolute current (`0x001B`) uses `A * 1024` scaling with full `uint32` clamp.
  - Internal startup-load guard now keys off current demand only (power-based trigger removed).
- Module bridge caching simplified:
  - Removed controller-level per-slot request-cache suppression path.
  - Adapter -> controller forwarding is now direct per apply cycle (no command-cache short-circuit).
- Scope intentionally excluded planner logic, per your instruction.

## 0) Cross-Check Against Alternate Summary (User-Pasted)

Post-pull status:
- Commit `978e05572a082fc28514dc056341e5f2a44fdfd0` is now present locally (fetched from origin).
- Exact diff requested in your pasted summary (`018f6cd..978e055`) is reproducible.
- Local branch HEAD is `b34809b`, which is one commit ahead of `978e055` (`New Refectored Planner`), so there are two behaviors that differ between pure `978e055` and local HEAD.

Claim status matrix:

| Pasted claim | Status on exact pair `018f6cd..978e055` | Status on local HEAD `b34809b` |
|---|---|---|
| Adapter keepalive removed; dedup moved to controller | Confirmed | Confirmed |
| PLC target cache fallback added (70s) | Confirmed | Confirmed |
| CAN overload no longer immediate hard fault at connector level | Confirmed | Confirmed |
| Config diff in `configs/charger.json` (`maxCurrentA 100->200`, `minVoltageV 50->200`, `maxModulesPerGun 2->1`) | Confirmed | Confirmed |
| Planner removed capability-aware behavior entirely | Confirmed for `978e055` | Not true on local HEAD (`src/power_manager.cpp:85-88`, `790-806`, `966-980`) |
| Underdelivery became diagnostics-only (no command mutation) | Confirmed for `978e055` (`src/ocpp_adapter.cpp` has diagnostic-only underdelivery log path) | Not true on local HEAD (active delivery-loss pause to `0A/0kW`: `src/ocpp_adapter.cpp:7575-7591`, `7601-7607`, `7626-7633`) |

Interpretation:
- Your pasted summary is aligned with `Before-Refactoring-Power..978e055`.
- The two mismatches I previously called out are due to the additional local planner commit now on top of `978e055` in this workspace (`b34809b`).

## 1) End-to-End Flow (Both Branches)

1. OCPP/session/HLC state is collected in `OcppAdapter::apply_power_plan()`.
2. `GunState` is built per connector using PLC status + module snapshots.
3. `PowerManager` computes island/module allocation, relay commands, and per-gun dispatch.
4. Adapter emits `PowerCommand` to PLC hardware and `ModuleCommandRequest` to `PowerModuleController`.
5. `PowerModuleController` fans out setpoints to module drivers (Maxwell/Rectifier/Tonhe/Sim).
6. Drivers send CAN commands and poll telemetry.
7. Controller aggregates `ModuleHealthSnapshot` per slot.
8. Adapter fuses module telemetry + PLC telemetry into measured present values and module states.
9. Loop repeats.

This pipeline exists in both branches. The differences below are in what is commanded, what telemetry is trusted, and which guards are active.

## 2) Command Model: Field-by-Field (Used vs Not Used)

### 2.1 `ModuleCommandRequest` fields
Defined identically in both branches:
- Current: `include/power_module_controller.hpp:40-47`
- Before: `include/power_module_controller.hpp:44-51`

| Field | Current (Split) | Before-Refactoring-Power |
|---|---|---|
| `slot_id` | Used for slot dispatch in controller (`src/power_module_controller.cpp:2726-2729`) | Used (`src/power_module_controller.cpp:2146-2149`) |
| `mask` | Used to select active modules (`src/power_module_controller.cpp:2742-2746`, `2754-2756`) | Used (`src/power_module_controller.cpp:2155-2158`, `2166-2170`) |
| `enable` | Used to arm/disarm per-module setpoints (`src/power_module_controller.cpp:2756-2761`) | Used (`src/power_module_controller.cpp:2170-2175`) |
| `voltage_v` | Used for module voltage command path (`src/power_module_controller.cpp:2759`, Maxwell `1247-1252`, Rectifier `1539-1543`, Tonhe `2502-2515`) | Used (`src/power_module_controller.cpp:2173`, Maxwell `1039-1049`, Rectifier `1539-1543`, Tonhe `1951-1967`) |
| `current_a` | Used, divided by active modules (`src/power_module_controller.cpp:2748-2761`) | Used (`src/power_module_controller.cpp:2161-2175`) |
| `power_kw` | **Actively used** (divided and passed; Maxwell `0x0020` when enabled) (`src/power_module_controller.cpp:2750-2761`, `1266-1314`) | Carried through split, but **driver path effectively ignores power control** (no Maxwell `0x0020` send path) (`src/power_module_controller.cpp:2162-2175`, Maxwell apply `1002-1078`) |

### 2.2 Adapter-level dedup/keepalive vs controller-level dedup

- Before branch had adapter dedup+keepalive:
  - `MODULE_COMMAND_KEEPALIVE_MS(250)` (`src/ocpp_adapter.cpp:45`)
  - comparator ignores `power_kw` (`src/ocpp_adapter.cpp:261-265`)
  - resend if changed or keepalive due (`src/ocpp_adapter.cpp:7463-7473`)
  - state map in header (`include/ocpp_adapter.hpp:513-514`)

- Current branch removed adapter keepalive map and moved dedup into controller:
  - controller cache window `120ms` (`src/power_module_controller.cpp:103`)
  - cache check (`src/power_module_controller.cpp:2730-2734`)
  - request equivalence includes `power_kw` (`src/power_module_controller.cpp:161-168`)
  - no `last_module_command_sent_at_` in adapter header (only `last_module_command_by_slot_`) (`include/ocpp_adapter.hpp:530-532`)

Operational impact:
- Before: deterministic periodic keepalive from adapter even if unchanged.
- Current: dedup is tighter and local to controller; periodic control refresh is driver-managed.

### 2.3 `ModuleSpec` / module config parameter usage map

`ModuleSpec` shape is the same in both branches (`include/power_module_controller.hpp:12-38` current, `12-42` before).  
Usage differs for several control knobs.

| Parameter | Current (Split) usage | Before usage |
|---|---|---|
| `id` | Driver logs/identity (`src/power_module_controller.cpp:1177`, `1280`, `1849`) | Same role (`src/power_module_controller.cpp:1033`, `1381`) |
| `slot_id` | Slot map for request dispatch/snapshot (`2726-2729`, `2774-2777`) | Same (`2146-2149`, `2185-2188`) |
| `slot_index` | Bit selection in mask + health bit (`2742-2756`, `2819-2821`) | Same (`2155-2168`, `2219-2221`) |
| `type` | Driver selection (`2986-2999`) | Same (`2324-2337`) |
| `can_interface` | Channel creation + governor scope (`2703-2706`, `2792-2797`) | Same (`2124-2127`, `2201-2206`) |
| `address` | CAN filters/addressing in all real drivers (`2966-2980`, Maxwell/rectifier/tonhe frame decode paths) | Same |
| `group` | Rectifier frame group encoding/filter (`2191-2193`, `2277-2280`) | Same (`1696-1699`, `1782-1785`) |
| `monitor_address` | Rectifier CAN id composition/filter (`2175`, `2244-2246`) | Same (`1680`, `1749-1751`) |
| `production_day` | Rectifier identity filter (`2181`, `2247-2254`) | Same |
| `serial_low` | Rectifier identity filter (`2182`, `2250-2256`) | Same |
| `source_address` | Tonhe source in CAN ID (`2505-2506`, `2536`) | Same (`1955`, `1987`) |
| `input_mode` | Maxwell mode enforcement + Tonhe mode command + Rectifier mode writes (`1144-1169`, `2522-2546`, `2209-2215`) | Same pattern, fewer retries/guards (`978-999`, `1973-1997`, `1714-1721`) |
| `hi_lo_mode` | Rectifier mode set/read (`2212-2215`, poll task `2132-2136`) | Same (`1717-1721`, `1643-1647`) |
| `silent_mode` | Rectifier silent set/read (`2219-2223`, poll task `2138-2142`) | Same (`1724-1728`, `1649-1653`) |
| `rated_power_kw` | Power cap fallback and Maxwell power scaling (`1268-1272`, snapshot `2871-2882`) | Used for planner/model caps; no Maxwell power command scaling path |
| `rated_current_a` | Current ratio scaling + capability fallback (`1196-1206`, `2867-2875`) | Used for current scaling (`1014-1027`) |
| `poll_interval_ms` | Poll cadence + stale fallback + current hold windows (`1352-1359`, `250-257`, `2901-2903`) | Same categories without hold-window logic (`1100-1107`, `158-161`) |
| `cmd_interval_ms` | Retry/refresh and mode intervals (`1062-1065`, `133-143`, `2211`, `2526`) | Same categories, different constants (`85-88`, `1716`, `1977`) |
| `poll_budget_fps` | CanChannel token budget (`2703`, `881-897`) | Same (`2124`, `766-783`) |
| `telemetry_stale_ms` | Stale threshold (`250-257`) | Same (`158-161`) |
| `broadcast` | Direct vs broadcast addressing/probing (`1360`, `2249-2269`) | Same (`1108`, `1758-1779`) |
| `probe_on_startup` | Address/group probe scheduling (`2145-2148`) | Same (`1656-1659`) |
| `readback_limits` | **Actively used for capability/limit polling and reconciliation** (`1420`, `1754`, `2151`) | Only limit-point polling path (`1167-1169`), no rectifier capability poll |
| `send_output_current` | Chooses `0x001B` absolute current path (`1254-1263`) vs ratio (`1219-1223`) | Same switch (`963`, `1057-1064`) |
| `send_output_power` | **Active Maxwell `0x0020` command path** (`1266-1314`) | Forced off at adapter/spec layer (`src/ocpp_adapter.cpp:868-873`, `931`) |

Config default drift for these parameters:
- Current defaults: `pollMs=200`, `cmdIntervalMs=200`, `readbackLimits=true` (`src/charger_config.cpp:140-148`).
- Before defaults: `pollMs=500`, `cmdIntervalMs=500`, `readbackLimits=false` (`src/charger_config.cpp:120-128`).

## 3) Module Control Strategy Differences by Driver

### 3.1 Maxwell (MXR)

#### Before
- Control pattern: startup (`0x0030`) + voltage (`0x0021`) + current via ratio (`0x0022`) or absolute current (`0x001B`) (`src/power_module_controller.cpp:1002-1078`).
- `send_output_power` intentionally unsupported at adapter/spec stage:
  - warning + forced false in adapter (`src/ocpp_adapter.cpp:868-873`, `931`)
- Fast refresh behavior: `control_refresh_interval` clamped to <=250ms (`src/power_module_controller.cpp:85-88`).
- Off keepalive every 2s (`src/power_module_controller.cpp:74-75`).

#### Current
- `send_output_power` propagated and honored:
  - from config into module state/spec (`src/ocpp_adapter.cpp:848`, `914`)
  - writes `0x0020` output-power ratio when enabled (`src/power_module_controller.cpp:1266-1314`).
- Additional control hardening:
  - low-voltage setpoint clamp floor (`1068-1071`, `1173-1183`)
  - mode-write retry gating (`1144-1169`)
  - startup retry while OFF/unavailable status (`1185-1193`)
  - readback mismatch diagnostics (`1754-1809`)
  - startup-stalled OFF detection (diagnostic) (`1824-1835`)
- Slower default control cadence (base >=500ms, stable-tracking controlled) (`133-143`).
- Off keepalive relaxed to 10s (`89-90`).

Net effect:
- Current branch commands more parameter dimensions (adds power channel) but with more gating/diagnostics.
- Before branch is simpler and more frequent in steady command cadence.

### 3.2 Rectifier (ENR/UUGreen)

#### Before
- No capability polling path (`kPollTaskCount=7`) (`src/power_module_controller.cpp:1613-1661`).
- Parses basic telemetry only (`cmd 0/1/30/8/...`) (`1791-1824`).
- No startup-stall-to-fault promotion when module stays OFF after enable.

#### Current
- Adds capability polling (`kPollTaskCount=8`, command `104`) (`src/power_module_controller.cpp:2102-2155`).
- Parses `104` and `114` for current capability/current telemetry (`2303-2329`).
- Derate/off flags propagated from status bits (`2334-2342`).
- Startup-stall promotion to fault when load requested but module remains OFF beyond timeout (`2343-2349`).

### 3.3 Tonhe

#### Before
- Start/stop + V/I command path, no startup-stall escalation (`src/power_module_controller.cpp:1864-1938`, `2000-2018`).
- No capability/derate flags in telemetry model.

#### Current
- Same command transport, plus startup-stall promotion (`2555-2560`).
- Capability/derate/off flags propagated (`2568-2574`).

### 3.4 Simulated driver

- Before sim driver only basic telemetry fields (`src/power_module_controller.cpp:841-881`).
- Current sim driver also publishes capability flags/current capability/current limit point for planner testing (`src/power_module_controller.cpp:969-975`, `999-1013`).

## 4) Module Telemetry Model: What Changed

### 4.1 `ModuleHealthSnapshot`

- Current adds:
  - `limit_fresh`
  - `available_current_a`
  - `available_power_kw`
  - `module_off`, `module_power_limited`, `module_temp_derated`, `module_ac_limited`
  - (`include/power_module_controller.hpp:60-66`)

- Before does not have those fields (`include/power_module_controller.hpp:53-67`).

Full field map (`ModuleHealthSnapshot`):

| Field | Current | Before |
|---|---|---|
| `valid`, `health_valid` | Used | Used |
| `healthy_mask`, `fault_mask` | Used | Used |
| `temperatures_c` | Used | Used |
| `telemetry_valid`, `current_valid`, `voltage_v`, `current_a`, `power_kw` | Used | Used |
| `limit_fresh` | Used (`src/power_module_controller.cpp:2862`) | Not present |
| `available_current_a`, `available_power_kw` | Used (`2864-2885`) | Not present |
| `module_off`, `module_power_limited`, `module_temp_derated`, `module_ac_limited` | Used (`2809-2816`) | Not present |
| `can_budget_limited`, `can_overload_latched`, `can_total_kbps` | Used | Used |

### 4.2 Driver telemetry state

- Current adds capability + derate/off flags (`src/power_module_controller.cpp:223-229`).
- Before telemetry state lacks those fields (`src/power_module_controller.cpp:130-146`).

### 4.3 Snapshot aggregation behavior

- Current:
  - Aggregates capability/derate/off into slot snapshots (`src/power_module_controller.cpp:2809-2816`, `2854-2885`).
  - Uses current-hold window requiring full contributor freshness, with bounded hold-through-gap (`2891-2923`).

- Before:
  - No capability/derate/off aggregation.
  - `current_valid` accepted with partial fresh contributors (`2260-2266`).
  - If CAN overload latched, marks slot fault and skips contributor (`2222-2226`).

## 5) Adapter Telemetry Consumption Differences

### 5.1 Module telemetry into connector snapshot

- Current consumes new module capability/derate/off fields (`src/ocpp_adapter.cpp:4147-4153`, `4241-4247`, `4274-4280`).
- Before connector snapshot has no capability/derate/off fields (`src/ocpp_adapter.cpp:4097-4118`).

### 5.2 Module state writeback (`module_states_`)

- Current writes per-slot capability and limit flags:
  - `available_current_a`, `available_power_kw`, `module_off`, `power_limited`, `temp_derated`, `ac_limited`, `capability_fresh`, `severe_fault`
  - (`src/ocpp_adapter.cpp:4792-4799`)

- Before writes only temperature and health bit/fault bit outcome (`src/ocpp_adapter.cpp:4675-4684`).

`ModuleState` field-delta in planner model (`include/power_manager.hpp`):
- Current adds planner-consumed capability/fault fields (`59-67`): `available_current_a`, `available_power_kw`, `module_off`, `severe_fault`, `power_limited`, `temp_derated`, `ac_limited`, `capability_fresh`.
- Before lacks these fields (`58-61`), so planner decisions are based mainly on `healthy` + rated/config caps.

### 5.3 Measurement-source trust policy (vehicle telemetry)

- Current introduces measurement source tagging and trust policy:
  - `MeasurementSource` enum in `GunStatus` (`include/hardware_interface.hpp:33-39`, `76-78`)
  - strict/allow-estimated policy gate in adapter (`src/ocpp_adapter.cpp:4569-4575`)
  - trusted-source helper (`include/charging_request.hpp:39-42`)
- Before has no measurement source fields/policy gates.

## 6) Planner (`PowerManager`) Consumption Differences

### 6.1 Module availability

- Current `count_available_modules_in_slot()` checks:
  - healthy + no severe fault + capability-ready (`!capability_fresh || available_power_kw>0 || available_current_a>0`)
  - (`src/power_manager.cpp:78-93`, `146-148`)
- Before uses only healthy-count (`src/power_manager.cpp:68-75`).

### 6.2 System power cap basis

- Current uses telemetry-derived available power if present, fallback to config per module (`src/power_manager.cpp:790-806`).
- Before uses static module cap estimation from rated/config (`src/power_manager.cpp:87-97`, `334-335`, `814-819`).

### 6.3 Current-limit clamping by module capability

- Current clamps dispatch current by selected modules' available current (`src/power_manager.cpp:650-653`, `966-980`).
- Before has no equivalent selected-capability current clamp.

### 6.4 Delivery-loss integration

- Current includes `delivery_lost` in emergency drop and request zeroing (`src/power_manager.cpp:656-657`, `811`, `986-988`).
- Before has no `delivery_lost` signal (`src/power_manager.cpp:680-681`, `824`, `973-974`).

### 6.5 Tie-closing rule

- Current closes tie when adjacent owners are equal even if owner `0` (`src/power_manager.cpp:691-693`).
- Before required owner `>0` (`src/power_manager.cpp:715-716`).

## 7) PLC/HLC and Vehicle Telemetry Differences (Affecting Module Control)

### 7.1 EV target cache during HLC power stage

- Current adds bounded target cache fields and usage:
  - state fields (`include/plc_can.hpp:135-160`)
  - hold constant `kPowerStageTargetCacheHoldMs=70000` (`src/plc_can.cpp:50`)
  - cache used for target freshness and relay gating (`src/plc_can.cpp:1933-1944`, `2151-2171`, `2306-2326`)

- Before has no target-cache state/logic; strictly uses live target freshness (`src/plc_can.cpp:1917-1920`, `2081-2084`, `2210-2213`).

### 7.2 Present telemetry source tagging

- Current tags present V/I/P source (`Meter`, `PlcPresent`, `EstimatedTarget`, etc.) (`src/plc_can.cpp:1969-2013`).
- Before computes present values without source tagging (`src/plc_can.cpp:1921-1948`).

### 7.3 EV-request derivation for relay closing

- Current uses canonical `derive_ev_power_request()` helper in PLC relay gating (`src/plc_can.cpp:2191-2194`, `2346-2349`, helper in `include/charging_request.hpp:44-106`).
- Before uses direct threshold checks (`ev_targets_fresh && cp_power_ready && target_current > threshold`) (`src/plc_can.cpp:2100-2102`, `2229-2231`).

### 7.4 AutoCharge/PnC MAC path

- MAC-based AutoCharge extraction remains present in both branches:
  - current `evmac` handling (`src/plc_can.cpp:856-875`)
  - before equivalent (`src/plc_can.cpp:844-863`)
- No branch-level behavior change found in this specific path.

## 8) Request Continuity and HLC Stage Guarding

- Current introduces structured request continuity state machine with hold reasons (`include/ocpp_adapter.hpp:330-345`, runtime `src/ocpp_adapter.cpp:5094-5335`).
- Before used simpler timestamp hold map `last_nonzero_req_kw_seen_` (`include/ocpp_adapter.hpp:325`, runtime `src/ocpp_adapter.cpp:4958-4970`).

Precharge stage guard:
- Current allows broader plausible stage fallback when `hlc_precharge_active` (`src/ocpp_adapter.cpp:340-361`).
- Before only allowed stage `0` fallback (`src/ocpp_adapter.cpp:361-379`).

## 9) CAN Traffic Governor Differences (Over-control / Backpressure)

### 9.1 Policy fields

- Current adds overload-clear hysteresis knobs:
  - config struct: `over_cap_clear_ratio`, `over_cap_clear_hold_ms` (`include/charger_config.hpp:97-99`)
  - parse (`src/charger_config.cpp:318-321`)
  - governor logic (`src/power_module_controller.cpp:577-587`, `604-605`)

- Before has only debounce latch, no clear hysteresis knobs:
  - config struct (`include/charger_config.hpp:96-102`)
  - parse (`src/charger_config.cpp:287-289`)
  - governor logic (`src/power_module_controller.cpp:468-482`, `490-491`)

### 9.2 Overload behavior in apply/poll/snapshot

- Before:
  - apply disables module activation if interface overload latched (`src/power_module_controller.cpp:2168-2171`)
  - snapshot marks module fault on overload and skips contributor (`2222-2226`)
  - poll always iterates all modules per iface (`2286-2294`)
  - adapter escalates module CAN overload into connector general fault (`src/ocpp_adapter.cpp:4812-4814`, `5315`)

- Current:
  - apply no longer hard-disables by overload bit (`src/power_module_controller.cpp:2756-2763`)
  - overload shown as telemetry flags (`2795-2797`) instead of forced fault bit
  - poll throttles to budget=1 under overload (`2946-2957`)
  - adapter no longer includes `module_can_overload` in `general_fault` (`src/ocpp_adapter.cpp:4966-4967`)

Interpretation:
- Current shifts from hard command cut to controlled poll-rate throttling + explicit overload telemetry.

## 10) Config Default Differences That Directly Affect Module Control

- Module defaults in slot parsing:
  - `pollMs`: 200 (current) vs 500 (before) (`src/charger_config.cpp:140` vs `120`)
  - `cmdIntervalMs`: 200 vs 500 (`141` vs `121`)
  - `readbackLimits`: true vs false (`146` vs `126`)

- New current timeouts/policies affecting request continuity and telemetry trust:
  - `hlc_target_hold_ms`, `request_loss_debounce_ms`, `target_fresh_hold_ms`,
    `delivery_loss_detect_ms`, `delivery_loss_recovery_ms`, `delivery_loss_escalation_ms`,
    `active_current_hold_ms`, `measurement_source_policy`
  - declared in header (`include/charger_config.hpp:184-192`)
  - parsed/normalized (`src/charger_config.cpp:268-277`, `368-390`)

- Before branch does not parse those additional fields (`src/charger_config.cpp:236-248`, `329-334`).

## 11) Tight Guards / Potential Logical Risks Found

### High/Medium Risk Candidates

1. Long EV-target cache hold in PLC path (current)
- Evidence: `kPowerStageTargetCacheHoldMs = 70000ms` (`src/plc_can.cpp:50`), used in power-stage freshness and relay gating (`1933-1940`, `2151-2171`, `2306-2326`).
- Risk: stale target persistence during prolonged target-stream loss can keep relay/request logic permissive longer than expected.

2. Command dedup cache window at controller entry (current)
- Evidence: `MODULE_COMMAND_CACHE_WINDOW=120ms` (`src/power_module_controller.cpp:103`, `2730-2734`).
- Risk: immediate retries of identical requests can be suppressed briefly after transient send failures if upstream assumes per-tick forwarding.

3. Availability tied to capability freshness values (current)
- Evidence: module usability depends on capability fields when `capability_fresh=true` (`src/power_manager.cpp:85-88`, `146-148`, `790-797`).
- Risk: if capability telemetry temporarily reports zero while module is otherwise healthy, planner can de-allocate aggressively.

4. Tie close rule allows owner `0` equality (current)
- Evidence: tie close on equal owners without `>0` guard (`src/power_manager.cpp:691-693`), unlike before (`src/power_manager.cpp:715-716`).
- Risk: idle-adjacent cabinets may close ties depending on topology assumptions.

5. Faster default poll/cmd rates + readback enabled by default (current)
- Evidence: defaults changed to 200/200 with readback true (`src/charger_config.cpp:140-148`).
- Risk: higher CAN load if site-level `canTraffic` limits are not tuned with module count.

6. Module CAN overload no longer faults connector at adapter level (current)
- Evidence: before included `module_can_overload` in `general_fault` (`src/ocpp_adapter.cpp:4812-4814`), current does not (`src/ocpp_adapter.cpp:4966-4967`) while still collecting the signal (`4154`, `5410`).
- Risk: prolonged overload may degrade module control/telemetry quality without tripping the same connector-fault path used previously.

### Lower/Behavioral Drift (May Be Intended)

7. Before adapter keepalive removed; refresh responsibility shifted to drivers/controller
- Evidence: before keepalive resend every 250ms (`src/ocpp_adapter.cpp:45`, `7467-7473`), current removes this and uses controller dedup cache (`src/power_module_controller.cpp:2730-2734`).
- Risk: behavior depends more on driver refresh intervals; testing should confirm no starvation under constant setpoints.

8. Current underdelivery action changed from optional cap (disabled) to active delivery-loss pause
- Before path compile-time disabled (`ENABLE_MODULE_UNDERDELIVERY_CLAMP=false`) (`src/ocpp_adapter.cpp:62`, code `7066-7211`).
- Current can force `0A/0kW` and pause mode on sustained loss (`src/ocpp_adapter.cpp:7572-7650`).
- Risk: more fail-safe, but can produce harder power drop if telemetry quality is poor.

## 12) Confirmation Summary

Confirmed differences are real and implemented in code across adapter, PLC hardware layer, module controller, and power planner. The current branch is not a cosmetic refactor; it changes control semantics in these core ways:

- Power command semantics expanded (`send_output_power` now active in Maxwell).
- Module capability telemetry is now first-class and affects planning/allocation.
- Vehicle present telemetry now carries source metadata and source-trust policy.
- Request continuity and delivery-loss handling are substantially reworked.
- CAN overload handling moved toward hysteresis + throttling telemetry model.

## 13) Validation Gaps (What code reading cannot prove)

The following need HIL/bench verification for final sign-off:

1. Long target-cache hold behavior under real PLC target-loss faults.
2. No unintended command starvation with controller dedup + driver refresh.
3. Module capability zero/fresh transitions during startup and borrow/return sequences.
4. Delivery-loss pause false positives with noisy present/module current telemetry.
5. Tie owner-0 closure behavior across full ring topologies.
6. Desired overload response when module CAN stays latched above cap for extended periods.
