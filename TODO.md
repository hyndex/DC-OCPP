# TODO

## Stage 1 — Protocol correctness
- [x] Fix GCMC status ACK tracking (preserve expected seq; track applied/received separately).
- [x] Enforce CRC/DLC=8 for CRC-tagged frames and force CRC8 on the controller path.

## Stage 2 — PLC firmware hardening
- [x] Add PROTO_VERSION config param (91) with NOT_ALLOWED on mismatch and controller-side gating.
- [x] Split EVSE_PRESENT CRC failure accounting and include CRC totals in debug frame.
- [x] Prioritize RELAY_CMD over legacy GCMC_CMD when both arrive within the shadow window.

## Stage 3 — Validation and docs
- [x] Add CAN protocol golden-vector tests (CRC + pack/unpack coverage) and wire into CMake.
- [x] Document protocol version handshake, CRC/DLC enforcement, and relay precedence in `docs/CAN_CONTRACT.md`.
- [x] Run and record results for the new CAN protocol tests on target build/CI.
- [x] Collapse protocol constants into a single generated artifact sourced from `docs/can_contract_constants.json` to prevent drift between controller and PLC firmware.

## Stage 4 — Safety/contract alignment
- [x] Gate PLC clear-faults handling on CRC validity for RelayControl/GCMC commands.
- [x] Enable RX cadence timeouts for EVSE_DC_PRESENT and EVSE_DC_MAX_LIMITS; align EVSE_PRESENT naming.
- [x] Make controller clear-faults explicit via HardwareInterface hook; remove auto-clear on any fault.
- [x] Fix controller fault mapping (earth fault vs estop input) and estop bit interpretation.
- [x] Remove HW_CONFIG RX handling/filter from controller (Controller→PLC only).
- [x] Update CAN contract/system overview docs to reflect new timeout and clear-fault semantics.

## Stage 5 — Validation follow-through
- [x] Run controller/PLC build and CAN protocol tests (`can_crc_filter_tests`, `can_protocol_vectors_tests`, `can_end_to_end_sim_tests`).
- [ ] Exercise fault-injection checklist on hardware or HIL (CRC bad clear-fault, relay timeout, bus-off recovery).
