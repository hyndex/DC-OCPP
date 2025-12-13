# HIL / System Test Plan

## Preconditions
- Dual-gun PLC with CAN interface configured per `Ref/Basic/docs/CAN_DBC.dbc`
- Safety loop, lock feedback, contactor feedback wired
- Ability to inject estop, earth fault, isolation fault, weld faults
- ISO15118 EV simulator with SLAC

## Scenarios

1. **Auth wait / Preparing hold**
   - Plug in, do not present token for 5 min
   - Expect: OCPP Preparing, AuthorizationState Pending to PLC, no HV, no session start
   - Then present token → authorization accepted, session begins

2. **Remote start before plug**
   - Send RemoteStart for connector X
   - Plug in after 30s
   - Expect: session created when plugged, authorization honored, Preparing→Charging

3. **Reservation enforcement**
   - Create reservation with idTag A
   - Plug in with tag B → session must not start
   - Present tag A → session starts, reservation cleared

4. **Autocharge MAC/EVCCID**
   - EV provides EVCCID/MAC; authorized locally/CSMS
   - Expect: autocharge accepted, cached; next plug without CSMS still authorized from cache (within 24h)

5. **EVSE limit watchdog**
   - Begin charging; block PLC EVSE limit ACKs
   - Expect: controller stops session within 1.5s, disables power, Faulted/SuspendedEVSE

6. **HLC charge complete / Finishing**
   - Run to HLC charge_complete flag
   - Expect: Finishing status until unplug; no new session spawned while plugged

7. **Safety faults**
   - estop, earth fault, isolation fault, comm fault, overtemp, overcurrent
   - Expect: immediate stop, correct OCPP error mapping, no restart until clear

8. **Weld detection**
   - Simulate GC/MN weld (command open, feedback closed)
   - Expect: session stop, Faulted, require manual clear

9. **Meter stale fallback**
   - Block meter frames; verify fallback energy calculation active, then restored on meter recovery

10. **Seamless retry tolerance**
    - Drop ISO15118 briefly (B1/B2 toggles) without unplug; controller keeps session, resumes Charging

## CI hook suggestion
- Add job to run `ctest --output-on-failure` for controller unit tests
- Add optional HIL flag to trigger scripted versions of scenarios 1–5 using simulator APIs
