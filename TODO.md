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
