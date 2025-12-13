# CCS2 DC Gap Analysis (controller + PLC)

Context: align the current controller (`src/ocpp_adapter.cpp`, `include/*`) and PLC firmware (`Ref/Basic/*`) with the reference CCS2 DC workflow (ISO 15118‑2/DIN 70121, CP/PP, SLAC/SDP, CPD, CableCheck, PreCharge, CurrentDemand, WeldingDetection) and the required controller↔PLC contract.

## Architecture snapshot
- Host planner loop runs every 100 ms (`src/ocpp_adapter.cpp:318-327`), drives module/contactors and pushes EVSE limits each cycle.
- PLC contract explicitly makes the host the **single source of truth for power/module sequencing**; PLC only mirrors RelayControl masks (`Ref/Basic/docs/can_dbc_overview.md:5-8`).
- PLC publishes EV targets/present values at fixed cadences: EVDC_Targets 100 ms, EVAC_Chg_Ctrl 200 ms, EVDC_Energy_Limits/Max_Limits 1000 ms (`Ref/Basic/docs/can_dbc_overview.md:41-46,206-246`), with Base EVSE frames noted as stub placeholders (`Ref/Basic/docs/can_dbc_overview.md:53`).
- Known gaps already documented: V2G/PnC flows not supported and AuthorizationState::Pending→EVSEProcessing mapping unvalidated (`README.md:159-164`).

## P0 gaps (interop/safety blockers)
- **CurrentDemand cadence not guaranteed**: Host planner and PLC EVDC publications are ≥100 ms (`src/ocpp_adapter.cpp:318-327`, `Ref/Basic/docs/can_dbc_overview.md:41-46`), so high-rate EV CurrentDemandReq messages can be skipped/aliased before power modules are updated. With PLC confined to RelayControl (no native module modulation, `Ref/Basic/docs/can_dbc_overview.md:5-8`), real EVs that expect setpoints to track every request may abort.  
  *Fix*: Add a fast path where PLC owns the CurrentDemand loop and applies last-known envelopes locally, or introduce a deterministic <20 ms controller path that ingests EVDC_Targets/EVAC setpoints and drives power hardware at the same cadence.
- **CPD EVSEMaxPowerLimit/schedule not ensured**: Base EVSE frames are stubbed (`Ref/Basic/docs/can_dbc_overview.md:53`) and the controller never verifies that the PLC’s CPD response carries mandatory DIN70121 `EVSEMaxPowerLimit`. There is also no fallback schedule/limit injection when the controller has no profile ready.  
  *Fix*: On the PLC, always populate EVSE max V/I/P (and DIN power limit) in CPD; on the controller, seed `EvseLimits` with hardware max defaults before CPD and keep them refreshed until explicit schedules arrive.
- **Authorization “ongoing” semantics unverified**: README flags the need to validate that `AuthorizationState::Pending` maps to ISO 15118 AuthorizationRes ongoing (`README.md:159-164`). Controller writes ConfigCmd AUTH_PENDING/GRANTED (`src/can_plc.cpp:824-840`) but there is no proof the PLC replies with EVSEProcessing=Ongoing/Finished.  
  *Fix*: Implement/verify PLC HLC responses for pending/granted auth and add a watchdog that terminates the session if no backend verdict before the mode-specific timeout.
- **PnC/contract flows absent**: Explicitly not supported (`README.md:163`); ISO15118 CertificateInstallation/Get15118EVCertificate paths and TLS policy are missing.  
  *Fix*: Add EXI payload pass-through PLC→controller, trigger OCPP DataTransfer `Get15118EVCertificate`/Authorize, cache results, and enforce TLS policy knobs.
- **Measured V/I consistency not enforced**: PLC publishes HLC-present measurements (`src/can_plc.cpp:1237-1269,1293-1317`), but the controller does not cross-check against power-module telemetry before reporting to EV/OCPP. CurrentDemandRes can therefore diverge from applied values if module telemetry lags.  
  *Fix*: Source CurrentDemandRes fields from synchronized power measurements; add a tolerance check and stop/derate on mismatch.

## P1 gaps (feature completeness / certification readiness)
- **Precharge timing/config misaligned**: Controller times out precharge at 2 s with 50 V tolerance (`include/charger_config.hpp:81-82`, logic `src/ocpp_adapter.cpp:1772-1805`), tighter than the commonly allowed ~7 s window; risk of premature fault on slower EVs.  
  *Fix*: Make precharge time/voltage configurable per protocol and align with ISO15118/DIN limits, keeping PLC-level HLC timing authoritative.
- **Limit delivery watchdog limited to ACK only**: Controller stops only when EVSE limit ACK stales (>1.5 s, `src/ocpp_adapter.cpp:886-897`); there is no TTL on outbound `set_evse_limits` or on PLC heartbeat (EVDC_*).  
  *Fix*: Add validity_ms on envelopes; if PLC telemetry/ACKs or envelopes expire, ramp to 0 A and end session gracefully.
- **OCPP/EVSE fault mapping coverage**: Fault mapping relies on coarse codes; several PLC faults (CRC mode mismatch, Base EVSE frame stubs) are not mapped to specific OCPP reasons.  
  *Fix*: Extend fault map and add tests that inject PLC fault frames to verify OCPP status/Reason alignment.

## P2 gaps (enhancements)
- **Smart-charging renegotiation & tariffs**: No schedule renegotiation beyond first profile pull; no tariff/value-added service transport to PLC.  
  *Fix*: Implement CPD/Schedule renegotiation hooks and tariff pass-through if required.
- **Cross-slot/island control**: Controller planner supports islands, but PLC interface cannot actuate GC/MC/MN beyond two relays (`README.md:12`, `Ref/Basic/docs/can_dbc_overview.md:5-8`).  
  *Fix*: Extend CAN contract to expose island contactors or constrain planner accordingly.
