## Module CAN Bandwidth Traceability

### Requirement
- Keep total CAN load below `20 kbps` per interface in active steady-state.
- Enforce hard-cap-first behavior.
- Latch fault and force safe derate/disable when overload cannot be controlled.
- Protocol assumptions source set:
  - `docs/modules/CAN Communication Protocol - Maxwell_V1.50.pdf`
  - `docs/modules/ENR series CAN Comunication Protocol S0 (1).pdf`
  - `docs/modules/UUGreenPower CAN Protocol (36.2 Version)Reference Guide.pdf`
  - `docs/modules/TonHe CAN communication between charging module and monitor TONHE V1.2.pdf`

### Implementation
- Config surface:
  - `include/charger_config.hpp` (`CanTrafficConfig`)
  - `src/charger_config.cpp` (`canTraffic` parsing and validation)
- Policy propagation:
  - `src/ocpp_adapter.cpp` (`ModuleCanTrafficPolicy` setup)
- Runtime governance:
  - `src/power_module_controller.cpp`
    - Rolling-window monitor and governor per CAN interface
    - TX classes: `SafetyUrgent`, `Control`, `Telemetry`
    - Poll fairness rotation per interface
    - Snapshot flags: `can_budget_limited`, `can_overload_latched`, `can_total_kbps`
- Fault lifecycle:
  - `src/ocpp_adapter.cpp` (`ModuleCanOverload` integrated into local fault path)
  - `include/error_catalog.hpp` (`ModuleCanOverload` mapping)

### Evidence Collection
- Bus capture:
  - `candump -tz -x canX > canX.log`
- Analyzer:
  - `python3 scripts/analyze_session.py --candump canX.log --window-ms 10000 --bits-per-frame 150 --max-kbps 20 --assert-cap`
- Runtime logs:
  - `Module CAN stats ...`
  - `Module CAN overload latched ...` (must be absent in normal qualification runs)

### Acceptance
- Analyzer cap assertion passes on each active CAN interface.
- No overload latch in qualified steady-state runs.
- Existing unit/regression suite passes.
