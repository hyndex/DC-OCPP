# Split Charging Controller Master Guide

This document is the combined master design note for building a DC charger controller for split-charging architecture.
It merges:

- the target architecture a production controller should implement,
- the behavior currently implemented in this repo,
- the gaps between the two,
- the full end-to-end flow across PLC, controller, modules, and OCPP,
- the edge cases that must be handled if the system is expected to survive field use.

This repo reference stack is:

- PLC firmware for CP, SLAC, ISO 15118 / DIN, RFID, EV identity capture, and low-level relay control,
- `dc_ocpp` controller for OCPP 1.6, sessions, authorization, power planning, fault handling, and module dispatch,
- module CAN drivers for the DC power modules.

Important current-repo reality:

- The shipped runtime config is a 2-connector / 2-slot site with 1 module per slot.
- `maxModulesPerGun=1` in the checked-in config.
- The current planner does not yet synthesize full cross-slot sharing tie commands.
- So this doc describes both:
  - what the controller should do in a finished split-charging product, and
  - what this repo actually does today.

## 1. Goal

Build a controller that can:

- manage multiple DC connectors that share a common DC ring bus,
- electrically isolate each active EV into its own island,
- allocate and deallocate power modules based on EV BMS demand, site limits, and module health,
- integrate with PLC-driven CCS/DC communication,
- support RFID, MAC-based Autocharge, OCPP RemoteStart, and local fallback modes,
- store sessions and events safely,
- rotate logs,
- handle faults deterministically,
- recover without chatter, relay abuse, or unsafe backfeed.

The core theory is simple:

- each EV requires its own independent voltage/current control,
- therefore one EV must map to one electrical island,
- modules may be shared only if they are reachable through a contiguous island path,
- contactor switching must be gated by low current, safe delta-V, and minimum hold times,
- precharge must always start from a known safe home path before any expansion into borrowed capacity.

## 2. Terminology

- Slot: one physical cabinet or section containing a gun, its local bus section, and one or more modules.
- Gun: one EV-facing charging outlet / connector.
- GC: gun contactor. This connects the island bus to the EV.
- MC: tie / island relay. This connects adjacent slot bus sections.
- MN: logical module enable path for a module. In this repo today, this is module-CAN control, not a PLC relay.
- Home slot: the slot that physically owns a connector.
- Borrowed module: a healthy module outside the connector's home slot but inside the same reachable island.
- Island: one contiguous electrical section of the ring bus assigned to one EV.
- HLC: high-level communication, meaning DIN or ISO 15118 over PLC.
- Autocharge: repo term for MAC / EVCCID / EMAID based tokenization. This is not full certificate-based Plug and Charge.

## 3. Production Design Invariants

These rules should be treated as non-negotiable if you want a controller that is field-safe.

1. Only one EV may own one electrical island at a time.
2. GC closes only after precharge and safety checks pass.
3. Tie relays open and close only under gated switching conditions.
4. Precharge starts from the home slot only.
5. Borrowed modules are added only after the home path is stable.
6. Borrowed modules are removed before the island is collapsed.
7. Faulted or telemetry-stale modules are never assigned.
8. Relay chatter is forbidden. Enforce minimum hold times and latch faults until unplug or operator clear.
9. `StartTransaction` must represent real energy delivery, not just plug-in or token acceptance.
10. Local fallback modes must be explicit and bounded. Do not hide them behind unsafe "free charge" shortcuts.

## 4. Hardware Architecture

### 4.1 General target topology

For an N-slot ring:

- each slot has one gun,
- each slot has one or more modules,
- the slots are connected by tie relays,
- the controller may partition the ring into one or more islands,
- each island has at most one closed GC.

### 4.2 Current repo relay truth

The production controller contract in this repo is:

- `Relay1` = gun contactor (`GC`)
- `Relay2` = tie / sharing relay (`MC`)
- `Relay3` = retired and should not be used by controller logic

Important nuance:

- the PLC firmware headers still label `Relay2` as CW tie and `Relay3` as CCW tie,
- but the `dc_ocpp` production contract collapses sharing onto one effective tie relay per PLC,
- therefore the controller should treat `Relay2` as the only live island relay unless the hardware contract is explicitly redesigned.

### 4.3 Current checked-in 2-PLC example

The runtime config in `configs/charger.json` maps:

| Connector | PLC | GC relay | MC relay | Home module |
|---|---|---|---|---|
| 1 | PLC1 | `Relay1 -> GC_1` | `Relay2 -> MC_1` | `MN_2_0` |
| 2 | PLC2 | `Relay1 -> GC_2` | `Relay2 -> MC_2` | `MN_1_0` |

The module IDs are swapped relative to the connector numbers. That is a config fact, not a bug in this document.

## 5. Software Architecture

### 5.1 PLC responsibilities

The PLC should own:

- CP state observation,
- CP PWM duty generation,
- SLAC handling,
- ISO 15118 / DIN state machine,
- EV target collection from HLC,
- local RFID scan capture,
- EV identity capture such as EVMAC / EVCCID / EMAID,
- low-level relay actuation interface,
- relay-safe fallback behavior if controller comms go stale.

### 5.2 Controller responsibilities

The controller should own:

- OCPP connectivity and message handling,
- connector sessions,
- token ingestion and authorization policy,
- smart charging limits,
- site-level power budgeting,
- island planning,
- module allocation and deallocation,
- fault synthesis across PLC, modules, and controller logic,
- persistence,
- diagnostics, logs, and recovery policy.

### 5.3 Module layer responsibilities

The module layer should own:

- per-module telemetry polling,
- command fan-out,
- voltage/current command application,
- capability reporting,
- vendor-specific protocol handling,
- module fault classification,
- safe disable on stale or invalid comms.

## 6. State Model

At minimum, maintain separate state for:

- connector session state,
- authorization state,
- connector electrical state,
- PLC telemetry freshness,
- module health and capability,
- island ownership,
- relay command state and last-change timestamps,
- local fault latches,
- OCPP connector state.

Recommended connector electrical state machine:

`Unplugged -> Plugged -> AuthPending -> Precharge -> GC_Close -> PowerDelivery -> Pause/NoEnergy -> Stop -> Finishing`

Fault can interrupt from any state and must force a deterministic safe path.

## 7. Authentication Modes

### 7.1 RFID

Flow:

1. EV plugs in.
2. Session is created locally.
3. PLC publishes RFID UID to controller.
4. Controller selects connector and requests OCPP `Authorize` unless token is locally prevalidated.
5. On accept, authorization becomes granted.
6. HLC continues, EV progresses to power stage.
7. Controller starts energy delivery only after electrical readiness is real.
8. A same-card re-tap during active charging may act as local stop.

Requirements:

- connector selection must not be ambiguous on multi-gun sites,
- duplicate scans need debouncing,
- the same-card stop rule must not fire before actual power delivery starts,
- denied tokens should be rate-limited to avoid HMI spam and repeated chatter.

### 7.2 MAC-based Autocharge

Flow:

1. EV plugs in.
2. Digital comms are advertised early.
3. PLC learns EV MAC / EVCCID / EMAID.
4. Controller converts that identity into an OCPP `idTag`.
5. Controller authorizes through CSMS, or applies local offline fallback if policy allows.
6. If accepted, the session continues without RFID.

Important distinction:

- In this repo, this is Autocharge / PnC-lite.
- It is not full certificate-based ISO 15118 Plug and Charge contract handling.
- Full contract-based PnC requires certificate and contract-chain handling beyond this OCPP 1.6 token mapping model.

Edge cases:

- OCPP 1.6 `idTag` length is 20 characters; raw EMAID can exceed that.
- If CSMS is offline, decide whether to:
  - accept local MAC identities,
  - queue them until backend reconnect,
  - or deny them and fall back to local RFID.
- Do not tear down HLC mid-session just because authorization is delayed for a short period.

### 7.3 OCPP RemoteStart

Flow:

1. CSMS sends `RemoteStartTransaction`.
2. Controller injects a pending token locally.
3. If the connector was specified, bind it there.
4. If not, select a connector using explicit site policy.
5. Once the EV is physically present and the session exists, consume the token.
6. Allow charging only when both auth and electrical conditions are satisfied.

Requirements:

- if `connectorId` is missing, the selection policy must be deterministic and documented,
- reservations must be checked before consuming the remote-start token,
- remote-start token should expire if the EV never plugs in.

### 7.4 Fixed Local RFID, Non-OCPP

This should be implemented as a separate local mode, not by abusing `freeMode`.

Recommended design:

- maintain a local allowlist of fixed RFID UIDs,
- match scanned UID against that allowlist,
- start a local session only on allowlist hit,
- store session locally with a `mode=local_fixed_rfid` marker,
- skip OCPP transaction creation entirely if operating in full non-OCPP mode,
- still retain the same electrical safety, power planning, and fault logic.

Do not use:

- `freeMode + defaultTag` as a substitute for fixed-card-only charging,
- PLC standalone unconditional auth for a production shared-power deployment unless the whole site is intentionally isolated from OCPP and the safety case is re-approved.

## 8. Duty Cycle and HLC Handling

Duty cycle must be treated as an HLC policy output, not as a simple auth-mode switch.

Recommended rule:

- advertise digital comms when HLC should be active,
- hold analog/preauth duty when HLC should be blocked or retried locally,
- keep the decision stable enough to avoid repeated SLAC/HLC teardown.

In this repo today:

- PLC uses `5%` duty when digital comms are enabled and local `force_preauth` is not active,
- PLC uses `100%` duty otherwise.

Practical interpretation by mode:

- MAC Autocharge: usually advertise HLC early.
- RFID: may still advertise HLC early if policy allows, but authorization gating still happens in the controller.
- RemoteStart: same electrical path as RFID after token injection.
- Local fixed RFID: should mirror RFID electrical behavior.

Never design duty-cycle policy as:

- RFID = one fixed duty,
- PnC = another fixed duty,
- RemoteStart = another fixed duty.

That model is too simplistic and causes broken retries and unstable digital sessions.

## 9. End-to-End Charging Flow

### 9.1 Plug-in to session creation

On vehicle plug-in:

1. Verify connector is not faulted, estopped, or locally disabled.
2. Verify no post-stop hold logic is blocking immediate restart.
3. Create a session shell with timestamps and connector binding.
4. Move connector state to `Preparing`.
5. Begin token and authorization handling.

### 9.2 Authorization handling

The controller should:

- keep a per-connector pending token queue,
- consume highest-priority eligible token first,
- prefer `RemoteStart` over `RFID` over `Autocharge` if multiple arrive,
- keep track of reservation constraints,
- move auth state through `Unknown -> Pending -> Granted/Denied`,
- persist pending tokens if power loss recovery matters.

### 9.3 Precharge

Precharge must:

- use exactly one healthy home-slot module,
- keep tie expansion off,
- set voltage to a safe precharge target,
- clamp current to a low configured maximum,
- verify voltage convergence before GC close,
- time out if the EV or power path does not respond correctly.

### 9.4 GC close

Only close GC when:

- HLC stage indicates precharge/power readiness,
- insulation and safety are acceptable,
- measured delta-V is within window,
- current is low enough for safe switching,
- configured stability time has been satisfied.

If GC fails to close in time, latch a fault. Do not oscillate the contactor.

### 9.5 Power delivery

After GC closes:

- move from `Preparing` to real power delivery,
- start OCPP transaction only at this point,
- drive module current and voltage according to EV targets, site limits, and module capability,
- periodically publish meter values and status.

### 9.6 Pause / no-energy

Pause should not necessarily mean a full electrical teardown.

Preferred behavior:

- preserve the session,
- preserve topology if safe and useful,
- clamp current and power to zero,
- keep connector/OCPP state consistent with paused behavior,
- decide whether HLC remains active based on pause semantics and EV expectations.

### 9.7 Stop

On stop request:

1. Ramp current to zero.
2. Wait for measured current decay below GC-open threshold.
3. Open GC.
4. Disable modules.
5. Collapse any borrowed ties only after current and delta-V gates allow.
6. End session and issue `StopTransaction` if in OCPP mode.

### 9.8 Fault stop

On fault:

- hard-off if required by safety class,
- otherwise clamp current immediately and move to safe relay-open sequence,
- publish fault to OCPP,
- latch local fault until unplug or operator clear according to severity.

## 10. Module Allocation Theory

### 10.1 Objective

Compute, for each active connector:

- requested voltage,
- requested current,
- requested power,
- allowed power after cable and site limits,
- required number of modules,
- reachable healthy modules,
- relay commands needed to form the owner island.

### 10.2 Input constraints

For each connector, limit requested output by:

- EV BMS request,
- connector current limit,
- connector power limit,
- OCPP smart-charging profile,
- site/grid limit,
- module health and capability,
- reachable island topology.

### 10.3 Allocation rules

Recommended algorithm:

1. Build the active connector set.
2. Determine each connector's effective power request.
3. Convert request into module count bounds.
4. Reserve one home module for each connector that is entering precharge or active charging.
5. Allocate additional modules only from healthy reachable slots.
6. Enforce one contiguous island per owner.
7. If a module is unreachable without conflicting island ownership, drop it.
8. If total demand exceeds site capacity, derate fairly according to policy.

### 10.4 Fairness policy

At minimum, define:

- whether fairness is equal-current, equal-power, priority-based, or reservation-based,
- whether active sessions retain a module-hold bias,
- how quickly derates and promotions are allowed.

Without a written fairness policy, field behavior becomes unpredictable.

## 11. Module Sharing Algorithm

This section describes how it should work in a finished split-sharing controller.

### 11.1 Ownership model

Assign each slot an owner:

- connector ID owning that slot's electrical island,
- or unowned if idle.

For each boundary between adjacent slots:

- close the tie relay if both sides share the same owner,
- open the tie relay if ownership differs.

If one owner spans the whole ring:

- keep one deterministic ring-break open,
- choose the break by fixed slot rule or minimum-switch-cost rule,
- document that rule so service engineers can predict behavior.

### 11.2 Add borrowed module sequence

When a connector needs extra capacity:

1. Confirm the connector is already stable on its home path.
2. Select a healthy reachable module from a slot that can join the same owner island.
3. Ramp current demand conservatively while tie is still open.
4. Verify both sides are at low current and safe delta-V.
5. Close the required tie relay.
6. Enable the borrowed module.
7. Rebalance current across active modules.

### 11.3 Remove borrowed module sequence

When demand falls or the module becomes unhealthy:

1. Reduce current contribution expected from the borrowed module.
2. Rebalance remaining modules.
3. Wait until switching conditions are safe.
4. Disable the borrowed module.
5. Open the corresponding tie relay if that boundary is no longer needed.
6. Recompute island ownership.

### 11.4 Precharge rule

Never start precharge through a borrowed slot.

The home slot must establish the first safe path. Borrowing is a post-stabilization optimization, not a prerequisite for first energization.

## 12. Current Repo Module-Sharing Reality

The current repo contains:

- topology and island-safety machinery in the controller,
- tie gating logic,
- module command fan-out,
- one-gun-per-island enforcement,
- precharge home-module forcing,
- command prioritization between hard-off, no-energy, and energy delivery.

But it does not yet fully implement the global shared-island allocator.

Current practical limitation:

- the base planner opens all tie relays and only assigns home-slot modules,
- the checked-in config limits each gun to one module,
- therefore full dynamic cross-slot sharing is not field-complete yet even though parts of the safety scaffolding already exist.

Treat this as a product gap, not a documentation bug.

## 13. Relay and Switching Policy

### 13.1 Current production truth

- `Relay1` is GC.
- `Relay2` is the live tie relay.
- `Relay3` is retired.
- `MN` is logical module enable over module CAN in the current repo.

### 13.2 Switching gates

Tie and GC switching should be allowed only when:

- measured current is below threshold,
- measured delta-V is below threshold,
- stability timer has elapsed,
- minimum relay hold time is satisfied,
- no local fault or disable is active.

### 13.3 Command priority

Final command resolution should be:

1. Hard off
2. No-energy
3. Energy delivery

This prevents one subsystem from accidentally re-enabling current while another subsystem is attempting to stop safely.

## 14. OCPP Integration

### 14.1 OCPP responsibilities

The controller should integrate:

- BootNotification,
- StatusNotification,
- Authorize,
- StartTransaction,
- StopTransaction,
- MeterValues,
- RemoteStartTransaction,
- RemoteStopTransaction,
- reservations,
- pause/resume behavior,
- ChangeConfiguration,
- diagnostics and logs,
- firmware update,
- smart charging profiles.

### 14.2 Recommended trigger points

- `Authorize`: when a non-prevalidated token is selected for a connector.
- `StartTransaction`: only when real power delivery is underway.
- `MeterValues`: while transaction is active and on keepalive/change policy.
- `StopTransaction`: after charging has actually stopped and energy is finalized.
- `StatusNotification`: from connector-state transitions, not from arbitrary intermediate booleans.

### 14.3 Smart charging interaction

The power planner must treat OCPP limits as hard caps on:

- connector power,
- connector current,
- or composite schedule limits.

Those limits should be merged into the same effective budget calculation used for BMS demand and site limit.

### 14.4 Reservations

Reservations must be checked before:

- accepting local RFID for a reserved connector,
- consuming a remote-start token,
- auto-starting any local fallback mode.

## 15. Non-OCPP Local Mode

A real non-OCPP local mode should still include:

- connector sessions,
- event logs,
- local authorization policy,
- full power planning,
- relay and module safety,
- fault recording and recovery,
- operator-visible session result history.

Recommended modes:

- `local_fixed_rfid`
- `local_service_mode`
- `ocpp_online`
- `ocpp_offline_with_local_allowlist`

Do not collapse all of these into one boolean.

## 16. Session Persistence

At minimum persist:

- pending tokens,
- session start/stop records,
- final energy and reason,
- auth source,
- fault reason if stop was abnormal,
- connector and EV identity markers as allowed by privacy policy.

Current repo status:

- session runtime is mostly in-memory,
- OCPP data is stored under `databaseDir`,
- pending auth tokens are persisted.

Recommended production persistence:

- append-only local session journal,
- crash-safe flush at transaction start and stop,
- boot-time reconciliation for incomplete sessions,
- bounded retention with export path.

## 17. Logs and Rotation

A production controller should keep separate rotating logs for:

- application controller logs,
- PLC/CAN traffic summaries,
- module CAN diagnostics,
- fault history,
- session history,
- maintenance actions such as firmware and diagnostics upload.

Recommended rotation policy:

- max file size bound,
- max file count bound,
- age bound,
- explicit compression for archived logs,
- one "live" symlink or current-file pointer for service tools.

Current repo status:

- log handling exists,
- diagnostics upload support exists,
- but the shipped runtime config is not a complete rotating-log deployment by itself.

## 18. Fault Handling

### 18.1 Fault classes

At minimum classify:

- estop / safety chain faults,
- earth / isolation faults,
- communication faults,
- module unavailable or stale telemetry,
- GC/MC close/open timeout,
- weld suspect,
- precharge timeout,
- overcurrent / overvoltage,
- temperature derate / shutdown,
- meter stale or energy inconsistency,
- controller overload such as module CAN overload.

### 18.2 Fault response rules

- Safety-critical faults: force hard-off immediately.
- Electrical sequencing faults: clamp current, open relays safely, latch until unplug.
- Telemetry freshness faults: remove the affected module or connector from service.
- Recoverable communication faults: use bounded retry windows, then escalate.

### 18.3 Recovery rules

A good recovery policy is:

- do not auto-clear every fault,
- clear only where the safety case allows,
- require unplug for most EV-path faults,
- require operator or OCPP clear for service/maintenance latches,
- re-enable HLC and power planning only after telemetry freshness is restored.

## 19. Edge Cases That Must Be Explicitly Handled

1. EV unplugs during precharge.
2. EV drops request briefly during active charging.
3. RFID scanned twice before power starts.
4. RFID scanned again after charging starts.
5. RemoteStart arrives with no `connectorId`.
6. RemoteStart token conflicts with reservation.
7. Autocharge identity arrives before session object exists.
8. Autocharge backend timeout while HLC is already active.
9. CSMS offline during MAC Autocharge.
10. EMAID too long for OCPP 1.6 `idTag`.
11. Module telemetry goes stale during power delivery.
12. Borrowed module becomes unavailable while island is active.
13. GC commanded open but current does not decay.
14. Tie commanded open but measured delta-V implies weld.
15. Meter value regresses.
16. OCPP pause while EV remains plugged and ready.
17. Power request disappears but EV is still electrically connected.
18. Site limit collapses suddenly because another dispenser starts.
19. Controller restarts mid-session.
20. PLC restarts while EV is still plugged.
21. Duplicate pending tokens across local and remote sources.
22. Post-stop plugged state causing immediate restart loops.
23. Connector fault on one slot while the neighboring slot is borrowing power.
24. Ring ownership spanning all slots and needing one deterministic break.

If these are not encoded into tests, they will become field incidents.

## 20. Recommended Hidden Handling

These behaviors should exist even if operators never see them directly:

- debounce CP and digital-presence hints so HLC does not flap,
- hold digital comms briefly across noisy telemetry gaps,
- delay transaction start until real power delivery,
- clamp `StopTransaction.meterStop` monotonic,
- rate-limit repeated auth denials,
- preserve last valid EV target briefly to survive short PLC frame gaps,
- keep relay-command hold timers independent from planning tick rate,
- record stop-origin hints before clearing session state,
- keep tie-open weld validation separate from ordinary current decay logic.

## 21. Implementation Blueprint

If you are building this controller from scratch, implement in this order:

1. Define the PLC-controller CAN contract.
2. Define module abstraction and per-vendor drivers.
3. Implement connector session and auth state machines.
4. Implement safe precharge and GC sequencing for one connector.
5. Implement local module control for home-slot-only charging.
6. Add OCPP integration and delayed `StartTransaction`.
7. Add persistence and logs.
8. Add site-level power budgeting.
9. Add island ownership model and tie gating.
10. Add cross-slot borrowing and deterministic ring-break handling.
11. Add offline/local modes.
12. Add full HIL and fault-injection coverage.

Do not start with global sharing before one-connector safety is stable.

## 22. Validation and Test Matrix

You need:

- unit tests for power planning,
- unit tests for auth/token priority,
- simulation tests for two-car sharing and precharge,
- HIL tests for relay sequencing,
- unplug-under-load tests,
- weld-suspect tests,
- CSMS offline and reconnect tests,
- log upload and firmware update tests,
- cold boot and mid-session reboot tests.

Pass criteria should include:

- no relay chatter,
- no double `StartTransaction`,
- no missing `StopTransaction`,
- monotonic energy accounting,
- deterministic connector state transitions,
- safe isolation under all single-fault cases in scope.

## 23. Current Repo File Map

Useful implementation anchors in this repo:

- `src/main.cpp`: process bootstrap and hardware/controller wiring
- `src/ocpp_adapter.cpp`: OCPP, sessions, auth, connector states, planner integration, faults
- `src/plc_can.cpp`: PLC CAN backend, RFID/MAC token ingestion, relay command transport
- `src/power_manager.cpp`: base planner
- `src/power_module_controller.cpp`: module command fan-out and telemetry
- `include/tie_gating.hpp`: tie-switch gating logic
- `configs/charger.json`: current reference deployment
- `docs/CAN_CONTRACT.md`: production PLC-controller contract
- `docs/modules/SplitCharging.md`: older split-charging design note, partly ahead of current runtime reality

## 24. Current Repo Gaps To Close

Before calling this repo a finished split-sharing product, close these gaps:

1. Implement the true global shared-island allocator that emits cross-slot tie decisions.
2. Make relay-topology docs consistent everywhere.
3. Implement a real local fixed-RFID mode instead of relying on `freeMode`.
4. Deploy proper rotating logs in the runtime configuration.
5. Define and test the deterministic ring-break policy.
6. Expand simulation and HIL coverage for borrowed-module add/remove sequences.

## 25. Summary

The correct way to think about split charging is:

- OCPP decides who is allowed to charge and what external limits apply,
- PLC decides what the EV is asking for and whether the HLC session is real,
- the controller decides which island exists and which modules are allowed to feed it,
- the module layer decides whether the requested power can actually be produced safely.

If any one of those layers is allowed to silently override the others, the site becomes difficult to reason about and unsafe to maintain.

The finished controller should therefore be:

- electrically conservative,
- transactionally accurate,
- explicit about ownership and mode,
- deterministic under failure,
- and honest about what is implemented today versus what still needs to be built.
