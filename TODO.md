# Multi-stage TODO / Progress Tracker

## Stage 1 — Authorization + Preparing alignment (P0)
- [x] Surface plug-in without session as Preparing and hold PLC in `auth_pending` until an auth decision is made.
- [x] Route all token sources through a single auth path; cache auth state per connector and notify PLC with Pending/Granted/Denied.
- [x] Enforce auth-wait timeout with clean shutdown + PLC denial to avoid silent hangs.
- [x] Add truthful RemoteStart acceptance/rejection semantics via new libOCPP hook; reject when no safe, plugged candidate connector is available.

## Stage 2 — Dynamic EVSE limits to PLC (P0)
- [x] Publish controller-computed EVSE V/I/P limits over CAN to PLC; stop clobbering PLC-provided EV request telemetry.
- [x] Teach PLC firmware to accept EVSE limit frames and feed them into HLC/ISO responses.
- [x] Add validation/diagnostic counters for EVSE limit frames (CRC + RX count) and surface via debug frame/logging.

## Stage 3 — Power/islanding robustness and observability (P1/P2)
- [x] Guard hardware mapping: PLC warns on unsupported module mask bits and enforces 2-module relay mapping; cross-slot islands auto-disabled when hardware lacks support.
- [x] Islanding safety invariants: MC/GC open-under-low-current enforcement retained; planner constrained to per-slot islands when hardware cannot cross.
- [x] Autocharge/PnC identity routing extended to EMAID/EVMAC segments; tokens feed unified auth path with fallback to other methods.
- [x] Build deterministic test vectors (sim + hardware sim) covering plug-in-before-RemoteStart, auth deny/fallback, comm loss rejection, and module dropouts.
