## Preparing + Authorization orchestration
- [x] Start physical sessions on plug-in with Preparing state while keeping energy delivery gated.
- [x] Track pending RemoteStart tokens with expiry and bind authorization when the EV plugs in (queue-based pipeline).
- [x] Accept RFID/Autocharge tokens via unified priority queue and authorize via Local/CSMS/RemoteStart prevalidation.
- [x] Gate StartTransaction/power plan on authorization with separate auth-wait vs EV power-request timeouts.
- [x] Add Preparing/Finishing connector states and avoid overwriting them with suspend statuses before power-up.

## PLC/HLC coordination (next)
- [x] Surface RFID/EVCCID identity frames from PLC driver and expose via poll_auth_tokens().
- [x] Propagate authorization grant to PLC/ISO stack once CAN interface supports it.
- [x] Add configurable auth wait/power-request timeouts to charger.json and expose via OCPP config if required.

## Simulation/validation
- [x] Allow simulator to toggle plug-in and EV power request for plug-first + delayed RemoteStart scenarios.
- [x] Inject auth tokens into simulator for RFID/Autocharge/RemoteStart paths.
- [x] Add scenario tests for plug-first + late RemoteStart, RemoteStart-first + plug later, Autocharge rejection + fallback, and timeout handling.
- [ ] Run full OCPP/ISO integration on hardware to validate Preparing/wait flows and smart-charging constraints.
