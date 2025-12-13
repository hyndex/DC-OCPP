1. A reference‑grade, end‑to‑end CCS2 DC workflow (state machine + message flow) that is *complete* for ISO 15118‑2 DC and DIN SPEC 70121 DC, including fault handling.
2. A concrete controller↔PLC contract (signals, ownership, update rates, required/optional fields, and fallback behavior) designed to satisfy CCS timing/behavior constraints.
3. A repo‑oriented gap checklist (P0/P1/P2) you can map directly onto “Basic PLC” and “DC‑OCPP/controller”. It calls out the specific failure modes that typically break real vehicles, including two well‑known CCS issues: missing `EVSEMaxPowerLimit` in CPD and “skipping” CurrentDemand processing.

If you provide either (a) the repo URLs (and confirm they are accessible without interactive login), or (b) exported source trees (zip/tar), I can convert the gap checklist into a repo/module‑exact assessment with file/function pointers.

---

## Reference architecture baseline: what “complete CCS2 DC” means

A complete CCS2 DC implementation must coordinate these layers:

* **IEC 61851 control pilot/proximity pilot (CP/PP)**: physical state machine, plug detect, safety interlocks.
* **ISO 15118‑3 SLAC (PLC link negotiation)**: data link association between EV and EVSE. EVerest separates this as a dedicated SLAC module (“EV SLAC data link negotiation according to ISO15118‑3”).
* **SDP (SECC Discovery Protocol)**: EV discovers SECC IP/port (EVSE often runs an SDP server). EVerest’s DC V2G module explicitly has an “enable_sdp_server” option.
* **ISO 15118‑2 and/or DIN SPEC 70121 message sequencing**: EXI encode/decode, V2GTP, TCP/TLS, state machine.
* **Power delivery control**: cable check (insulation), precharge, contactors, current demand loop, measurements.
* **Backend/OCPP**: authorization, transaction lifecycle, charging profiles/limits, certificate flows (PnC), error mapping.

EVerest’s reference split is instructive: `EvseManager` holds high‑level charging logic, controls SLAC + ISO15118 modules, and OCPP “uses EvseManagers to control the charging session.” ([Everest][1])
Its `EvseV2G` module contains DIN70121 + ISO15118‑2 and exposes an **extensions interface used to share data between ISO15118 and OCPP modules**.

Your architecture (controller repo for OCPP + PLC repo for V2G) is conceptually aligned with that split, but completeness depends on whether you’ve implemented the same cross‑module responsibilities and timing behavior.

---

## Deliverable 1 — End‑to‑end plan (state machine + message flow)

### A. Top‑level end‑to‑end state machine

Below is a *complete* DC CCS2 end‑to‑end state machine. I’m expressing it as one coordinated system with two sub‑FSMs:

* **PLC / SECC FSM (EV‑facing, HLC/V2G)**
* **Controller FSM (OCPP + session orchestration + power module manager)**

#### A1. PLC / SECC state machine (EV‑facing)

**S0 — Idle / Available**

* Entry: CP = A, PP = disconnected; DC output disabled; contactors open; no V2G session.
* Exit to S1 when: PP connected + CP transitions to B (vehicle present).

**S1 — EV detected / IEC61851 ready for HLC**

* PLC checks CP/PP stable; start PLC modem; prepare SLAC.
* Notify controller: `EV_PRESENT = true`, connector id.

**S2 — SLAC**

* PLC runs ISO 15118‑3 SLAC to bind EV↔EVSE link. (EVerest: “EV SLAC data link negotiation according to ISO15118‑3”.)
* Output: EV MAC / run_id (useful for AutoCharge correlation), link‑ready event.

**S3 — SDP**

* PLC provides (or hosts) SDP so EV can find SECC IP/port; then accept TCP connection.
* If TLS required (PnC / security policy), negotiate TLS (or allow/prohibit/force depending config, as in EVerest’s V2G module).

**S4 — SupportedAppProtocol**

* EV and SECC agree on protocol family/version (ISO 15118‑2 vs DIN 70121). This is the “HLC protocol selection” gate.

**S5 — SessionSetup**

* EV sends SessionSetupReq (contains EVCCID).
* PLC records EVCCID, forwards to controller (for AutoCharge mapping / session correlation).

**S6 — ServiceDiscovery + PaymentServiceSelection (+ optional ServiceDetail)**

* PLC offers DC services + payment options (EIM and/or Contract/PnC), based on controller policy/config.
* EV selects mode.

**S7 — Authorization**

* EV sends AuthorizationReq (EIM) or proceeds via PnC flow.
* PLC responds with AuthorizationRes.

  * If controller/backoffice authorization not finished, PLC must keep EV alive by responding with “processing ongoing” semantics (EV then repeats AuthorizationReq). Using `EVSEProcessing` this way is explicitly a known mechanism and is used in AuthorizationRes (and also CPD/CableCheck).
* Timeouts: enforce mode‑dependent auth timeout (EVerest exposes `auth_timeout_eim` and `auth_timeout_pnc`).

**S8 — ChargeParameterDiscovery**

* Exchange EV limits and EVSE limits + schedule.
* PLC must include all mandatory fields; for DIN 70121, `EVSEMaxPowerLimit` is mandatory—missing it is a known real‑EV abort trigger.

**S9 — CableCheck**

* PLC commands controller/power subsystem to perform insulation checks.
* Multiple CableCheckReq/Res rounds are commonly required to keep communications alive while checks run.
* PLC uses `EVSEProcessing` to signal ongoing until checks complete (supported pattern).

**S10 — PreCharge**

* EV sends PreChargeReq with target voltage/current. PLC commands power stage to ramp EVSE DC output voltage to EV battery voltage.
* Timing constraints exist in standards; one widely cited set is: pre‑charge HV bus in ~3 seconds and overall precharge process timeout ~7 seconds for ISO 15118‑2/DIN70121.
* Transition condition is commonly “|EVTargetVoltage − EVSEPresentVoltage| <= precharge_accuracy”.

**S11 — PowerDelivery (Start)**

* EV sends PowerDeliveryReq with ChargeProgress=Start.
* PLC ensures: contactors safe to close; DC output enabled; interlocks OK.
* PLC acknowledges only when power stage is ready.

**S12 — CurrentDemand loop (Charging)**

* EV cyclically sends CurrentDemandReq; PLC must process all, even at short intervals.
* PLC responds CurrentDemandRes with **present measured** voltage/current and EVSE status.
* Known failure: EV aborts if EVSE reports current/voltage values in CurrentDemandRes that do not match physically applied values.

**S13 — PowerDelivery (Stop/Pause/Renegotiate)**

* EV sends PowerDeliveryReq with ChargeProgress=Stop (or Pause/Renegotiate depending protocol).
* Controller may also request stop (RFID stop, remote stop, fault).
* PLC ramps down current, disables DC output, opens contactors.

**S14 — WeldingDetection**

* Execute welding detection per IEC 61851 practices; respond accordingly.

**S15 — SessionStop / Teardown**

* EV sends SessionStopReq; PLC responds; close TCP/TLS; stop PLC modem per policy.
* Notify controller: session ended, reason.

**SF — Faulted (global)**

* Can be entered from any state on interlock/fault.
* Actions depend on severity:

  * Immediate safe shutdown: disable DC supply, open contactors, preserve EV comm if possible to report error.
  * EVerest distinguishes “Emergency shutdown” vs “Error shutdown” and highlights differences in whether PWM stays on during HLC, but both stop DC supply and open contactor. ([Everest][1])

#### A2. Controller state machine (OCPP + orchestration)

A robust controller FSM (aligned with OCPP status model) typically looks like:

* **C0 Available** (no EV, no reservation)
* **C1 Preparing** (EV plugged / SLAC ongoing / V2G session establishing)
* **C2 Authorizing** (RFID/MAC/PnC/RemoteStart)
* **C3 Authorized** (permission granted, waiting for EV to reach power delivery start)
* **C4 Charging** (energy transfer active; OCPP transaction active)
* **C5 Finishing** (stop sequence, weld detection, session stop)
* **C6 Faulted / Inoperative** (error latched; may require service/clear)

The controller should be event‑driven by PLC session events (plug‑in, SLAC complete, V2G phase changes, power delivery start/stop, faults) and should not attempt to “drive” ISO 15118 sequencing directly—only to provide the policy inputs the PLC needs to respond correctly and to manage power hardware and OCPP.

---

### B. Message flow: EV ↔ PLC ↔ Controller ↔ Power modules

Below is the end‑to‑end sequence with explicit responsibilities and internal signals.

#### B1. Sequence diagram (simplified but complete)

```
EV/EVCC                PLC (SECC/V2G)                     Controller (OCPP/Session)           Power Modules
  |                          |                                       |                              |
  | Plug-in (CP=B)           |-- detect CP/PP ---------------------->|  EV_PRESENT event            |
  |                          |                                       |-- OCPP Status=Preparing      |
  |<-- HLC enabled (PWM) --->|                                       |                              |
  |--- SLAC msgs ----------->|-- SLAC state machine -----------------|                              |
  |<-- SLAC complete --------|-- EV_MAC/RunID ---------------------->|  (AutoCharge candidate)      |
  |--- SDP discovery --------|-- SDP server/response ----------------|                              |
  |--- TCP/TLS connect ----->|-- accept, TLS if needed --------------|                              |
  |--- supportedAppProtocol->|-- select ISO15118-2 or DIN -----------|                              |
  |--- SessionSetupReq ------|-- store EVCCID ---------------------->|  EVCCID forwarded            |
  |--- ServiceDiscoveryReq -->|-- offer services/payment ------------|<-- config/policy             |
  |--- PaymentSelectReq ----->|-- set auth mode --------------------->|                              |
  |--- AuthorizationReq ----->|-- if not authorized: EVSEProcessing=ONGOING ----------------------->|
  |                          |------------------- AUTH_REQUEST ------>|-- OCPP Authorize/RemoteStart|
  |<-- AuthorizationRes ------|-- once ok: EVSEProcessing=FINISHED ---|                              |
  |--- ChargeParamDiscovery->|-- needs EVSE limits & schedule --------|-- compute limits/schedule    |
  |<-- CPDRes (incl MaxPower)->|-- send mandatory fields -------------|                              |
  |--- CableCheckReq -------->|-- start insulation check -------------|-- command isolation monitor->|-- perform
  |<-- CableCheckRes (ONGOING)|-- loop until complete ----------------|<-- status -------------------|
  |--- PreChargeReq ----------|-- set voltage ramp target ----------->|-- optional clamp/limits ---->|-- ramp V
  |<-- PreChargeRes (Vpresent)|-- send measured present voltage ------|<-- V/I feedback -------------|
  |--- PowerDeliveryReq(Start)|-- close contactor/enable output ------|----------------------------->|-- enable
  |<-- PowerDeliveryRes ------|                                       |-- OCPP Transaction start     |
  |--- CurrentDemandReq ----->|-- compute setpoints (clamp) --------->|-- (optional ext limits) ---->|-- regulate
  |<-- CurrentDemandRes ------|-- send measured V/I (must match) -----|-- meter/telemetry to OCPP    |
  |... loop ...               |                                       |                              |
  |--- PowerDeliveryReq(Stop)->|-- ramp down/open contactor ----------|----------------------------->|-- disable
  |<-- PowerDeliveryRes ------|                                       |-- OCPP Transaction stop      |
  |--- WeldingDetectionReq --->|-- perform weld check ----------------|                              |
  |<-- WeldingDetectionRes ----|                                       |                              |
  |--- SessionStopReq -------->|-- SessionStopRes, close TCP/TLS -----|-- OCPP Status=Available      |
  | Unplug (CP=A)             |-- end                                |                              |
```

---

### C. PLC‑level V2G messaging: message/state‑by‑message responsibilities

This section answers your first evaluation item directly: **for each message/state, how PLC implements V2G messaging, how controller responds, and how both proceed**.

I’m describing the expected implementation contractually; you can compare your code against it.

#### C1. SLAC (ISO 15118‑3)

* **PLC implements**

  * Full SLAC handshake; handles timeouts/retries; exposes “link ready” + EV MAC (or similar identity).
  * Must meet timing (CharIN notes strict response timings in SLAC failure cases; e.g., delays can abort).
* **Controller responds**

  * Treats SLAC completion as “EV communication possible” event.
  * If your “MAC ID authorization” is SLAC‑MAC based, controller decides whether this MAC is authorized and sets `auth_pregranted`.
* **Proceed**

  * PLC starts/answers SDP, then TCP/TLS accept.

#### C2. SDP

* **PLC implements**

  * SDP server (typical) to announce SECC IP/port; then accept TCP.
  * If you support PnC, TLS policy must be enforced (allow/force/prohibit patterns exist; see EVerest V2G configuration fields).
* **Controller responds**

  * Provides TLS certificates / keys policy (PnC requires proper certificate mgmt).
* **Proceed**

  * supportedAppProtocol negotiation begins.

#### C3. supportedAppProtocol

* **PLC implements**

  * Select protocol family/version compatible with EV: ISO15118‑2 DC and/or DIN70121 DC.
* **Controller responds**

  * Usually none in real‑time; controller provides “supported protocol set” configuration.
* **Proceed**

  * SessionSetup.

#### C4. SessionSetup

* **PLC implements**

  * Parses EVCCID; stores session IDs; forwards EVCCID to controller as early as possible (useful for AutoCharge).
* **Controller responds**

  * Maps EVCCID → token (AutoCharge) or to diagnostic logs; may trigger pre‑authorization.
  * EVerest explicitly notes EVCCID can be converted into an AutoCharge token (EIM case). ([Everest][1])
* **Proceed**

  * ServiceDiscovery.

#### C5. ServiceDiscovery / ServiceDetail

* **PLC implements**

  * Responds with available energy transfer mode (DC), supported payment options (EIM/Contract), optional value‑added services.
* **Controller responds**

  * Supplies policy: whether Contract (PnC) enabled, whether certificate installation supported, tariffs, etc.
* **Proceed**

  * PaymentServiceSelection.

#### C6. PaymentServiceSelection + (PnC path: CertificateInstallation/Update, PaymentDetails)

* **PLC implements**

  * EIM: proceed to Authorization.
  * Contract/PnC:

    * handle certificate installation/update request messages and relay required payload to controller for CSMS interactions.
* **Controller responds**

  * Implements OCPP‑side certificate workflows. OCA documents that if EV sends CertificateInstallationReq, the charge point sends `Get15118EVCertificate` to CSMS (wrapped via OCPP DataTransfer in 1.6).
* **Proceed**

  * Authorization (PnC or EIM).

#### C7. Authorization

* **PLC implements**

  * Must respond to EV AuthorizationReq even if backend authorization not complete.
  * Use `EVSEProcessing` to indicate “ONGOING / WAITING_FOR_CUSTOMER” and avoid EV timeouts; EV repeats AuthorizationReq until FINISHED. This mechanism is explicitly used in AuthorizationRes.
  * Enforce auth timeout per mode; EVerest exposes PnC/EIM timeouts (55s PnC default; 300s EIM default).
* **Controller responds**

  * RFID: performs OCPP Authorize, sets `auth_status=Accepted/Rejected/Pending`.
  * Remote start: uses pre‑authorized state from CSMS, sets `auth_status=Accepted`.
  * “MAC ID”: uses EVCCID/SLAC MAC to authorize locally or via CSMS; sets `auth_status`.
* **Proceed**

  * When controller indicates Accepted, PLC sends AuthorizationRes FINISHED and proceeds to CPD.

#### C8. ChargeParameterDiscovery

* **PLC implements**

  * Must generate CPDRes with EVSE limits + schedule.
  * Critical: include all mandatory fields. A known real‑world failure is EV abort if CPDRes lacks `EVSEMaxPowerLimit` (DIN70121 case).
  * If controller’s smart charging schedule isn’t ready, PLC must still respond with a valid schedule (at least one schedule entry) and correct max limits.
* **Controller responds**

  * Provides:

    * hardware capabilities (max V/I/P) and current derating,
    * external limits from OCPP/energy management,
    * optional tariff/schedule.
* **Proceed**

  * CableCheck.

#### C9. CableCheck

* **PLC implements**

  * Triggers insulation monitoring & DC output test level.
  * Must keep EV comm alive; multiple CableCheckReq/Res rounds often required while the charger performs checks.
  * Uses `EVSEProcessing` ongoing until complete (supported in CableCheckRes).
* **Controller responds**

  * Commands isolation monitor and returns measured insulation status (plus thresholds).
* **Proceed**

  * On pass: PreCharge; on fail: fail response + safe shutdown.

#### C10. PreCharge

* **PLC implements**

  * Receives PreChargeReq with EV target voltage/current; commands power modules to ramp output voltage.
  * Must meet tight time behavior; one commonly referenced requirement set is 3s bus pre‑charge and ~7s overall pre‑charge timeout for ISO15118‑2/DIN70121.
  * Send PreChargeRes with present voltage; proceed once within accuracy.
* **Controller responds**

  * Supplies clamps/limits (max V/I ramp, thermal derate).
  * Provides present voltage measurement if PLC doesn’t measure directly (but then latency/consistency is critical).
* **Proceed**

  * PowerDelivery Start.

#### C11. PowerDelivery (Start/Stop)

* **PLC implements**

  * Start: close DC contactor, enable output; confirm ready.
  * Stop: ramp down, disable output, open contactor.
* **Controller responds**

  * Executes power module enable/disable; tracks transaction boundaries for OCPP.
* **Proceed**

  * Start leads to CurrentDemand loop; Stop leads to welding detection/session stop.

#### C12. CurrentDemand loop

* **PLC implements**

  * Must process *every* CurrentDemandReq from EV, even if sent in short intervals.
  * Must ensure the values it reports (present voltage/current) match what is physically applied; mismatch is a known abort trigger.
  * Generates EVSEStatus and EVSE limits; handles pause/resume semantics if supported.
* **Controller responds**

  * Provides updated limits (OCPP profile changes, thermal derating, site energy management).
  * Publishes OCPP meter values and status changes.
* **Proceed**

  * Loop until stop condition or fault.

#### C13. WeldingDetection + SessionStop

* **PLC implements**

  * Welding detection; session stop handshake; teardown.
* **Controller responds**

  * Stop transaction in OCPP; mark connector available; upload final meter values.

---

## Deliverable 2 — Clear controller ↔ PLC interface contract

The core design principle for CCS2 DC is:

* **PLC must be able to respond to EV at the EV’s message cadence** (especially CurrentDemand), without blocking on OCPP/backoffice latencies. CharIN explicitly highlights that EVSE must process CurrentDemand messages even if they arrive in short intervals.
* Therefore the controller should provide **policy inputs and envelopes**, not be in the tight loop for every EV message.

Below is a contract that meets that requirement and also cleanly splits OCPP↔V2G responsibilities.

### A. Interface overview

Use a versioned, session‑scoped API (RPC, shared memory, UART frames, etc. — transport doesn’t matter as long as latency/jitter does). Key characteristics:

* **Event channel (PLC→Controller):** state changes, EV requests, faults.
* **Control channel (Controller→PLC):** authorization result, stop commands, dynamic limits, certificate responses, backend decisions.
* **Fast telemetry channel (Power→PLC and/or Controller→PLC):** present voltage/current/energy/isolation updated at deterministic rate.

### B. Signal list (minimum viable for full CCS2 DC)

I’m listing each signal with:

* **Dir**: direction
* **Owner**: who is authoritative
* **Rate**: typical update requirement
* **Required**: required for interoperability/safety

#### B1. Session / state signals

1. `plc.v2g_phase` (enum: IDLE, SLAC, SDP, SAP, SESSION_SETUP, SERVICE_DISCOVERY, AUTH, CPD, CABLECHECK, PRECHARGE, POWERDELIVERY, CURRENTDEMAND, WELDING, SESSION_STOP, FAULT)

* Dir: PLC→Controller
* Owner: PLC
* Rate: event‑driven + heartbeat 1 Hz
* Required: Yes (controller state mapping + OCPP)

2. `plc.ev_present`, `plc.cp_state`, `plc.pp_state`

* Dir: PLC→Controller
* Owner: PLC
* Rate: event + 10 Hz during transitions
* Required: Yes (safety, OCPP status)

3. `ctrl.session_command` (enum: ENABLE, DISABLE, STOP_GRACEFUL, STOP_EMERGENCY, RESET_FAULT)

* Dir: Controller→PLC
* Owner: Controller (but PLC enforces safe stop)
* Rate: event
* Required: Yes

#### B2. Authorization contract

4. `ctrl.auth_context`

* Fields: `method`(RFID/EIM, AUTOCHARGE, REMOTE_START, PNC), `id_token`, `token_type`, `expiry`, `parent_id` (optional), `offline` flag
* Dir: Controller→PLC
* Owner: Controller
* Rate: event
* Required: Yes

5. `ctrl.auth_status` (enum: PENDING, ACCEPTED, REJECTED) + `reason`

* Dir: Controller→PLC
* Owner: Controller
* Rate: event (must arrive before PLC’s auth timeout)
* Required: Yes

6. `plc.evccid` + (optional) `plc.ev_mac`

* Dir: PLC→Controller
* Owner: PLC
* Rate: once per session
* Required: Yes if AutoCharge/MAC auth supported

**Vehicle coordination during “authorization wait”:**

* PLC uses `ctrl.auth_status=PENDING` to respond AuthorizationRes with EVSEProcessing “ONGOING / WAITING_FOR_CUSTOMER” semantics (supported mechanism).
* Controller must refresh `auth_status` before timeout; PLC enforces timeout based on mode (PnC vs EIM) — EVerest demonstrates mode‑dependent timeouts.

#### B3. Limits / schedule

7. `ctrl.evse_limits_dynamic`

* Fields: `max_voltage`, `max_current`, `max_power`, `min_current`, `ramp_rate_v`, `ramp_rate_i`, `derating_reason`, `validity_ms`
* Dir: Controller→PLC
* Owner: Controller (policy/energy management)
* Rate: 1–10 Hz (faster if doing dynamic load balancing)
* Required: Yes (for smart charging / OCPP profiles)

8. `plc.ev_limits` (from CPD)

* Fields: `ev_max_voltage`, `ev_max_current`, `ev_max_power` (if present), soc targets
* Dir: PLC→Controller
* Owner: PLC (parsed from EV)
* Rate: once + on renegotiation
* Required: Yes

9. `ctrl.schedule` (optional but recommended)

* Fields: schedule entries (start time offset + power/limit), tariff optional
* Dir: Controller→PLC
* Owner: Controller
* Rate: on CPD and on profile updates
* Required: If you claim smart charging / schedule support.

**Important interoperability constraint:**
If operating in DIN 70121 DC, the EV may abort if CPD response does not contain EVSEMaxPowerLimit; ensure the PLC always has a valid `max_power` to send, even if controller schedule is delayed.

#### B4. Power control and measurements

10. `plc.ev_request` (from PreChargeReq / CurrentDemandReq)

* Fields (DC): `target_voltage`, `target_current`, `charge_complete`, `soc`, `status_flags`
* Dir: PLC→Controller
* Owner: PLC
* Rate: every EV request (CurrentDemand cadence)
* Required: Yes

11. `plc.power_setpoint` **(preferred ownership: PLC)**

* Fields: `voltage_setpoint`, `current_setpoint`, `enable_output`
* Dir: PLC→Power module (direct) **or** PLC→Controller (if controller proxies to power modules)
* Owner: PLC (because it must keep up with CurrentDemand cadence)
* Rate: 20–100 Hz (depending EV cadence)
* Required: Yes

12. `meas.dc_present_voltage`, `meas.dc_present_current`, `meas.energy_wh`, `meas.isolation_status`

* Dir: Power subsystem → PLC (and/or Controller)
* Owner: measurement source
* Rate: at least 20–50 Hz during charging
* Required: Yes

**Consistency constraint (hard):**
CurrentDemandRes must report measured V/I that match physically applied values; EVs may interrupt the session if they don’t.

#### B5. Faults and interlocks

13. `plc.fault`

* Fields: `fault_code`, `severity`, `latched`, `diagnostic_data`
* Dir: PLC→Controller
* Owner: PLC for EV‑side electrical faults; controller for backend faults
* Rate: event + 1 Hz while active
* Required: Yes

14. `ctrl.ocpp_error_mapping` (optional, but simplifies)

* Fields: mapping table or enum translation rules
* Dir: Controller→PLC (config)
* Owner: Controller
* Rate: static
* Required: Recommended

### C. Fallback and “controller unavailable” behavior (required)

This addresses your “source of vehicle‑facing data” and “what if controller can’t provide inputs” requirements.

#### C1. What PLC sends to vehicle: source of truth

**Vehicle‑facing values should come from:**

* **Present V/I:** *measured* at DC output (or as close as possible), not inferred. (Mismatch causes EV abort risk.)
* **EVSE max limits:** `min( hardware_rating, thermal_derate, energy_manager/OCPP limit )`
* **Schedule / tariffs:** controller policy; PLC formats into protocol structures
* **EVSE status / processing:** PLC state machine

#### C2. If controller comms is lost or delayed

Define explicit rules, because EV timing windows are short (precharge is tightly bounded).

Recommended behavior:

1. **During Authorization (S7):**

* If `ctrl.auth_status` not received in time:

  * PLC ends session gracefully (AuthorizationRes finished with failure / or terminate connection according to protocol policy).
* Use mode‑dependent timeout (PnC typically shorter than EIM; EVerest exposes both).

2. **During CableCheck/PreCharge (S9/S10):**

* PLC must continue responding to EV requests (CableCheckReq/PreChargeReq) and can use “processing ongoing” semantics to keep session alive while hardware completes checks (CableCheck often needs multiple request/response rounds).
* If controller can’t supply dynamic limits:

  * PLC uses safe default envelope (hardware max with conservative derate) **but** enforces a watchdog.

3. **During CurrentDemand loop (S12):**

* PLC must not block waiting for controller each cycle; it clamps EV request using last known dynamic limits.
* Add an **external limits watchdog**:

  * If `evse_limits_dynamic.validity_ms` expires (e.g., no refresh for >1–5 s), PLC derates to 0 A and performs a controlled stop (PowerDelivery Stop).
  * This is aligned with the idea in EVerest energy management: if supplied limits time out, charging is stopped to avoid unsafe overload when connections drop. ([Everest][1])

4. **Backend/OCPP down (offline policies):**

* For Plug&Charge, offline authorization and certificate validation are policy‑controlled; OCA describes offline behavior rules for contract validation.
* For RFID/EIM, you need a clear offline policy: allow via local whitelist/cache or reject.

---

## Deliverable 3 — Gap list (by repo/module), recommended fixes, priority

Because I cannot see your repos here, the list below is structured as:

* **Requirement** (what must exist for CCS2 DC completeness)
* **Where it typically lives** (Basic PLC vs DC‑OCPP/controller)
* **How to verify in repo**
* **Failure symptoms**
* **Fix** + priority

### P0 (must fix; blocks interoperability/safety)

1. **CurrentDemand loop is not “hard real‑time” (blocking on controller/OCPP)**

* Lives: PLC (tight loop), plus interface design across both repos
* Verify:

  * PLC processes every CurrentDemandReq and responds within protocol timeouts without waiting for OCPP round trips.
* Symptom:

  * EV interrupts during charging; known issue when EVSE can’t manage high‑rate CurrentDemand updates.
* Fix:

  * Make PLC compute setpoints locally using last‑known limits; controller only updates envelopes asynchronously.

2. **Mismatch between reported CurrentDemandRes V/I and physically applied values**

* Lives: PLC + measurement path (power module integration)
* Verify:

  * Present voltage/current in responses are sourced from the same measurement used by the regulation loop (or a synchronized snapshot).
* Symptom:

  * EV interrupts during CurrentDemand phase if EVSE reports values different than physically applied values.
* Fix:

  * Tighten measurement acquisition; timestamped snapshot per response; avoid controller‑induced latency unless deterministic.

3. **Missing mandatory CPD fields (especially `EVSEMaxPowerLimit` in DIN 70121)**

* Lives: PLC message builder (+ controller provides value)
* Verify:

  * CPDRes always includes mandatory fields; for DIN70121 DC ensure EVSEMaxPowerLimit present.
* Symptom:

  * EV aborts right after ChargeParameterDiscoveryRes.
* Fix:

  * Provide fallback max power (hardware rating) in PLC; clamp with controller limits when available.

4. **CableCheck / Authorization wait does not keep comm alive (no ongoing responses)**

* Lives: PLC state machine + controller coordination
* Verify:

  * PLC uses `EVSEProcessing`‑style “ongoing” responses where appropriate (AuthorizationRes, CPDRes, CableCheckRes).
  * CableCheck supports multiple request/response rounds while tests run.
* Symptom:

  * EV times out during CableCheck or Authorization.
* Fix:

  * Implement ongoing processing responses + timers; ensure hardware tasks run asynchronously.

5. **PreCharge timing not achievable (control path too slow / wrong sequencing)**

* Lives: PLC + power module control integration
* Verify:

  * EVSE can ramp DC output to EV target in allowed time; precharge phase has tight constraints (3s HV bus pre‑charge, ~7s overall timeout are commonly referenced).
* Symptom:

  * EV aborts before PowerDelivery.
* Fix:

  * Local precharge control in PLC; avoid waiting for OCPP; implement voltage ramp with sufficient bandwidth.

6. **No deterministic safe shutdown on comm loss / external limits timeout**

* Lives: both repos (contract + watchdog)
* Verify:

  * If controller stops sending dynamic limits, PLC stops charging after a bounded time (and safely).
  * EVerest’s model: stop charging if energy limits time out. ([Everest][1])
* Fix:

  * Implement limits watchdog and safe stop procedure; clearly define “stop reasons”.

### P1 (important; affects feature completeness, certification readiness)

7. **PnC certificate flows not implemented end‑to‑end**

* Lives: PLC for ISO15118 message handling + Controller for OCPP DataTransfer certificate APIs
* Verify:

  * CertificateInstallationReq triggers OCPP `Get15118EVCertificate` workflow per OCA guidance.
  * PnC authorization uses the wrapped Authorize (DataTransfer) approach and offline behavior is defined.
* Fix:

  * Add EXI payload pass‑through + response reinjection; implement caching and timeout behavior.

8. **Authorization methods not fully coordinated with EV “authorization wait”**

* Lives: both repos
* Verify:

  * RFID, AutoCharge/MAC, RemoteStart have consistent behavior regardless of whether auth happens before or after plug‑in.
  * Withdrawal/cancel logic exists before power delivery starts (EVerest notes auth can be revoked before charging begins). ([Everest][1])
* Fix:

  * Implement explicit `auth_status` state + timers + cancellation path.

9. **OCPP ↔ V2G state and error mapping incomplete**

* Lives: controller (primarily), PLC supplies detailed fault causes
* Verify:

  * Every EVSE fault is mapped to consistent OCPP error codes and transaction stop reasons; EVSE continues to provide HLC error info where applicable.
* Fix:

  * Build a single mapping table; add test coverage with fault injection.

10. **PowerDelivery/transaction boundary ambiguous**

* Lives: controller
* Verify:

  * OCPP transaction starts at first real energy transfer (commonly after PowerDelivery Start / first CurrentDemand with current > 0), not at plug‑in.
* Fix:

  * Gate StartTransaction/TransactionEvent on PLC “power delivery active” event.

### P2 (nice‑to‑have / advanced features)

11. **Schedule renegotiation, smart charging extras, value‑added services**

* Lives: both repos
* Note:

  * Even EVerest’s feature list for its V2G module shows some items (like schedule renegotiation) not universally supported.
* Fix:

  * Add only if required by product requirements or certification targets.

---

## EVerest comparison: how your split should look if it matches best practice

EVerest’s documented coordination model (simplified):

* `EvseManager` orchestrates charging and controls SLAC + ISO15118 modules; OCPP uses EvseManager for session control/data. ([Everest][1])
* `EvSlac` handles PLC link negotiation.
* `EvseV2G` implements ISO15118‑2 + DIN70121 and has an **extensions interface** to share data with OCPP.
* Energy management can stop charging if limits time out (robustness under comm loss). ([Everest][1])

**Your architecture is “EVerest‑like” if:**

* PLC repo = SLAC + SDP + ISO15118/DIN V2G protocol state machine + EV‑facing safety‑critical sequencing
* Controller repo = OCPP + authorization + scheduling/limits + power module driver(s) + error mapping
* The PLC<->controller contract includes a dedicated “V2G↔OCPP extension bridge” for:

  * PnC authorization and certificate installation/update (EXI payload transport)
  * status mapping and session events

---

## PLC firmware completeness checklist (explicit)

A PLC firmware can be considered “complete for CCS2 DC” if it implements:

1. IEC61851 CP/PP detection and correct gating of HLC start/stop
2. SLAC end‑to‑end (timeouts, retries)
3. SDP (server) + TCP accept, TLS policy if PnC supported
4. supportedAppProtocol negotiation
5. Full ISO15118‑2 DC and/or DIN70121 DC message sequencing:

   * SessionSetup, ServiceDiscovery, PaymentServiceSelection
   * Authorization with ongoing behavior
   * ChargeParameterDiscovery with mandatory fields (DIN EVSEMaxPowerLimit)
   * CableCheck with multiple req/res rounds
   * PreCharge with timing/accuracy constraints
   * PowerDelivery start/stop
   * CurrentDemand loop (process every message; consistent measured values)
   * WeldingDetection, SessionStop
6. Fault handling:

   * emergency stop, isolation fault, overtemp, contactor failure, comm loss
   * safe shutdown sequencing (disable DC supply, open contactor, HLC error reporting where possible) ([Everest][1])

If any one of items 1–5 is missing or non‑conformant, you do not have a complete CCS2 DC end‑to‑end implementation.

---

## What I need to convert this into a “repo‑confirmed” verdict

To “confirm whether it is complete and correct” against *your* implementation (as you requested), I need direct visibility into:

* PLC repo: SLAC/SDP/V2G state machine code + interface to controller + measurement/control loops.
* Controller repo: OCPP transaction/auth/profile code + power module driver + PLC interface.

If you paste repo links (or upload exported source snapshots), I will produce:

* a state‑by‑state trace *mapped to your actual functions/modules*,
* a correctness review (timeouts, required fields, sequencing),
* and an exact gap list with file/line references.

[1]: https://everest.github.io/nightly/_included/modules_doc/EvseManager.html "5.3.6. EvseManager — EVerest  documentation"
