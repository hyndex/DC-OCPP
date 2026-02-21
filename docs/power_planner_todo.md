# Power Planner Refactor TODO

Scope: Planner-only changes (no CP logic changes).

## Stage 1 - Gap Audit
- [x] Map current planner flow end-to-end (`OcppAdapter` -> `PowerManager` -> module command split).
- [x] Identify gaps against planner guide:
  - target margins (V/I) missing at planner level
  - no voltage-guard current taper near ceiling
  - ramp uses fixed step only (no dt-aware slew/jerk model)
  - no capture-band snap at end-of-ramp (risk of endpoint dithering)
  - no explicit planner-rate configurability aligned to CCC guidance

## Stage 2 - Planner Config Surface
- [x] Add planner config knobs for:
  - voltage/current margins
  - voltage guard band
  - up/down slew limits (min/max)
  - emergency down slew
  - jerk limit
  - ramp response time constant
  - capture-band thresholds
- [x] Load and sanitize new planner knobs from config parser.
- [x] Wire new knobs from `ChargerConfig` to `PlannerConfig` in `OcppAdapter`.

## Stage 3 - Core Planner Algorithm
- [x] Add safe target shaping in `PowerManager`:
  - voltage command margin below requested ceiling
  - current margin below raw current target
  - guard-band taper as measured voltage approaches ceiling
- [x] Replace fixed-step ramp with dt-aware jerk-limited slew ramp.
- [x] Add endpoint capture band (snap + zero slew) for stable end-of-ramp.
- [x] Keep power/current command coherence after ramp shaping.

## Stage 4 - Robustness + Cleanup
- [x] Keep per-gun ramp state deterministic across active/inactive transitions.
- [x] Ensure no CP-state logic is changed by this refactor.
- [x] Preserve backward compatibility for existing configs (safe defaults).

## Stage 5 - Verification
- [x] Build `dc_ocpp` successfully.
- [x] Verify no unintended file drift outside planner-related files.
- [x] Summarize what is now aligned vs still external/runtime-dependent.
