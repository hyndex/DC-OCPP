Below is a full “design doc” you can treat as the specification for your controller / OCPP back‑end logic.

I will assume the **latest architecture you described**:

* 12 guns (Gun 1…12).
* 12 slots arranged on a **closed loop (ring) DC bus**.
* Each slot has **2 modules** and **1 gun**.
* You want **dynamic power sharing across 0…N guns**, but for now:

  * **Min 1 module / gun**,
  * **Max 2 modules / gun**,
  * **Operate the minimum number of modules needed**, to save standby and switching losses.
* Each EV/gun needs its **own voltage and current**, so we **electrically isolate per‑gun islands** on the ring using contactors.

---

# 1. Goal & Scope

### 1.1 Goal

Design a **production‑ready control architecture** and **algorithm suite** for:

* A **closed loop DC bus** with 12 slots and 24 modules (2 per slot).
* **Up to 12 EVs (guns)** charging simultaneously, each with its own independent DC bus (voltage and current).
* **Dynamic power sharing** using a limited number of modules, with:

  * Min 1 module per active gun,
  * Max 2 modules per active gun,
  * Modules turned ON only when needed.
* Full **safety supervision**:

  * EV and cable ratings (power & current),
  * Over‑current/over‑voltage,
  * Insulation monitoring,
  * Contactor weld detection,
  * Thermal limits,
  * Emergency stop, etc.

The result must be suitable to implement in:

* Local controller firmware (PLC / MCU) +
* OCPP server logic (for transactions & power limits).

---

# 2. Hardware Abstraction

## 2.1 Slots, guns, modules, contactors

We standardize naming so it works in code, not just on paper.

### 2.1.1 Slots

There are 12 slots arranged on a ring:

| Slot ID | Modules (IDs) | Gun | GC contactor (NO) | Bus cut contactor (NC) | CW neighbor | CCW neighbor |
| ------: | ------------- | --- | ----------------- | ---------------------- | ----------- | ------------ |
|       1 | M1_0, M1_1    | 1   | GC_1              | MC_1                   | 2           | 12           |
|       2 | M2_0, M2_1    | 2   | GC_2              | MC_2                   | 3           | 1            |
|       3 | M3_0, M3_1    | 3   | GC_3              | MC_3                   | 4           | 2            |
|       4 | M4_0, M4_1    | 4   | GC_4              | MC_4                   | 5           | 3            |
|       5 | M5_0, M5_1    | 5   | GC_5              | MC_5                   | 6           | 4            |
|       6 | M6_0, M6_1    | 6   | GC_6              | MC_6                   | 7           | 5            |
|       7 | M7_0, M7_1    | 7   | GC_7              | MC_7                   | 8           | 6            |
|       8 | M8_0, M8_1    | 8   | GC_8              | MC_8                   | 9           | 7            |
|       9 | M9_0, M9_1    | 9   | GC_9              | MC_9                   | 10          | 8            |
|      10 | M10_0, M10_1  | 10  | GC_10             | MC_10                  | 11          | 9            |
|      11 | M11_0, M11_1  | 11  | GC_11             | MC_11                  | 12          | 10           |
|      12 | M12_0, M12_1  | 12  | GC_12             | MC_12                  | 1           | 11           |

### 2.1.2 Per‑module contactors (MN)

Each module has its own contactor MN to connect to the local bus in the island:

| Module | Slot | Module contactor |
| ------ | ---- | ---------------- |
| M1_0   | 1    | MN_1_0           |
| M1_1   | 1    | MN_1_1           |
| M2_0   | 2    | MN_2_0           |
| M2_1   | 2    | MN_2_1           |
| …      | …    | …                |
| M12_0  | 12   | MN_12_0          |
| M12_1  | 12   | MN_12_1          |

So per slot we have:

* **GC_i** – Gun contactor, normally open, closes when EV is connected and island is ready.
* **MC_i** – Bus cut contactor, normally closed, opens to split the ring into independent islands.
* **MN_i_0, MN_i_1** – Module contactors, connect modules to the island’s bus.

## 2.2 Islands (virtual DC buses)

An **island** is a contiguous run of slots on the ring where:

* All **internal MCs** are CLOSED.
* Boundary MCs on both ends are OPEN.
* The modules in those slots connect via MN to that island’s DC bus.
* At most **one GC** in that island is closed → one EV/gun per island.

Each island:

| Field     | Meaning                                                     |
| --------- | ----------------------------------------------------------- |
| island_id | integer ID                                                  |
| slot_ids  | ordered list of contiguous slots on ring (e.g. [3,4,5])     |
| modules   | modules in those slots (e.g. M3_0,M3_1,M4_0,M4_1,M5_0,M5_1) |
| gun_id    | assigned gun (or null for idle island)                      |
| V_set_v   | DC voltage setpoint for this island                         |
| P_set_kw  | island power setpoint (sum of modules for that EV)          |

---

# 3. Operating Constraints & Policy

## 3.1 Module and gun ratings

* `P_MOD_KW` – module rating (kW), e.g. 30 kW.
* `I_GUN_MAX[g]` – maximum DC current for gun g (cable / connector).
* `P_GUN_LIMIT[g] = I_GUN_MAX[g] * V_nominal[g]` – gun nameplate power limit.
* Site/grid limit: `P_GRID_LIMIT` (kW).

## 3.2 Policy: modules per gun

* **Min modules** per active gun: 1.
* **Max modules** per gun: 2.
* We operate modules in **discrete steps**:

  * 0 modules → no charging (gun can be READY but 0 power).
  * 1 module → ~ `P_MOD_KW`.
  * 2 modules → ~ `2 * P_MOD_KW` (subject to cable limit).

So per gun:

| n_modules | Mode | Power capability (approx)               |
| --------- | ---- | --------------------------------------- |
| 0         | OFF  | 0 kW                                    |
| 1         | LOW  | up to min(P_MOD_KW, P_GUN_LIMIT[g])     |
| 2         | HIGH | up to min(2 × P_MOD_KW, P_GUN_LIMIT[g]) |

We want to keep the **smallest n_modules** that meets demand.

## 3.3 “0…N guns” objective

At any time:

* 0 to 12 guns may be **active** (EV sessions).
* The controller must:

  * **Allocate power budget** per gun.
  * Convert that to 0/1/2 modules per gun.
  * Map modules to islands & MN contactors.
  * Respect **module health, cable limits, grid limit**, and safety.

---

# 4. Data Structures (for implementation)

Pseudo‑types (language‑agnostic):

```pseudo
struct Slot {
    id: int                # 1..12
    modules: list[string]  # ["M1_0", "M1_1"]
    gun_id: int            # 1..12
    gc_id: string          # "GC_1"
    mc_id: string          # "MC_1"
    cw_id: int             # clockwise neighbor
    ccw_id: int            # counter-clockwise neighbor
}

struct ModuleState {
    id: string             # "M1_0"
    slot_id: int
    healthy: bool
    enabled: bool          # currently ON (MN closed)
    mn_id: string          # "MN_1_0"
}

enum GunFSMState { IDLE, EV_DETECTED, READY, ISLAND_READY, CHARGING, RAMP_DOWN, FAULT }

struct GunState {
    id: int
    slot_id: int
    gc_id: string
    fsm_state: GunFSMState
    ev_session_active: bool
    ev_req_power_kw: float
    ev_req_voltage_v: float
    P_GUN_LIMIT_kw: float
    priority: int
    island_id: int or null
    i_meas_a: float
    i_set_a: float
    connector_temp_c: float
}

struct IslandState {
    id: int
    slot_ids: list[int]
    module_ids: list[string]
    gun_id: int or null
    V_set_v: float
    P_set_kw: float
}
```

Control tables:

```pseudo
desired_MC_state[mc_id] ∈ {OPEN, CLOSED}
desired_GC_state[gc_id] ∈ {OPEN, CLOSED}
desired_MN_state[mn_id] ∈ {OPEN, CLOSED}
```

---

# 5. Core Algorithms

## 5.1 Gun FSM (session states)

Each gun follows:

```text
IDLE -> EV_DETECTED -> READY -> ISLAND_READY -> CHARGING -> RAMP_DOWN -> IDLE
                                   \__________> FAULT <__________/
```

Simplified transitions:

* `IDLE → EV_DETECTED`: CP/PP detection (vehicle plugged).
* `EV_DETECTED → READY`: handshake OK (protocol, auth, OCPP StartTransaction).
* `READY → ISLAND_READY`: island allocated, modules selected/output precharged (before GC closes).
* `ISLAND_READY → CHARGING`: GC closed, current allowed.
* `CHARGING → RAMP_DOWN`: session stop requested or fault.
* `RAMP_DOWN → IDLE`: current ~ 0, GC opened.
* Any → FAULT on local error (overcurrent, IMD, etc.).

Only guns in `READY` / `ISLAND_READY` / `CHARGING` participate in power allocation.

## 5.2 Algorithm A – power budgets (continuous kW) for 0…N guns

Inputs:

* `active_guns` = guns with `ev_session_active == true` and FSM ∈ {READY, ISLAND_READY, CHARGING}.
* For each g in active_guns:

  * `P_req_raw[g]` – EV/BMS requested power.
  * `P_GUN_LIMIT[g]`.
  * `priority[g]`.
* System limits:

  * `M_healthy` – count of healthy modules.
  * `P_MOD_KW`.
  * `P_GRID_LIMIT`.

### A.1 Clamp per‑gun request

```pseudo
for g in active_guns:
    P_req_limited[g] = min(P_req_raw[g], P_GUN_LIMIT[g])
```

### A.2 Compute system capacity

```pseudo
P_module_total = M_healthy * P_MOD_KW
P_system_cap = min(P_module_total, P_GRID_LIMIT)
```

### A.3 Allocate P_budget per gun

If `Σ P_req_limited[g] ≤ P_system_cap`:

```pseudo
P_budget[g] = P_req_limited[g] for all g
```

Else use weighted proportional sharing with caps:

```pseudo
function compute_power_budgets(active_guns):
    # initialization
    P_budget[g] = 0 for all g in active_guns
    remaining_guns = set(active_guns)
    remaining_power = P_system_cap

    while remaining_guns and remaining_power > 0:
        total_weight = sum(1 + priority[h] for h in remaining_guns)
        any_capped = false

        for g in list(remaining_guns):
            share = remaining_power * (1 + priority[g]) / total_weight
            candidate = P_budget[g] + share

            if candidate >= P_req_limited[g]:
                extra = P_req_limited[g] - P_budget[g]
                P_budget[g] = P_req_limited[g]
                remaining_power -= max(extra, 0)
                remaining_guns.remove(g)
                any_capped = true
            else:
                P_budget[g] = candidate

        if not any_capped:
            remaining_power = 0

    return P_budget
```

Result: `P_budget[g]` is the **contiguous power target** for each gun.

## 5.3 Algorithm B – discrete modules per gun (0/1/2)

### B.1 Ideal modules from P_budget

```pseudo
function ideal_modules_for_gun(g, P_budget_g):
    if P_budget_g <= 0:
        return 0

    # Basic thresholds (tunable)
    if P_budget_g <= 0.8 * P_MOD_KW:
        n = 1
    elif P_budget_g <= 1.6 * P_MOD_KW:
        n = 2
    else:
        n = 2

    max_by_cable = floor(P_GUN_LIMIT[g] / P_MOD_KW)
    if max_by_cable < 2:
        n = min(n, max_by_cable)

    if n < 0: n = 0
    if n > 2: n = 2
    return n
```

Compute:

```pseudo
for g in active_guns:
    n_ideal[g] = ideal_modules_for_gun(g, P_budget[g])
M_ideal_total = sum(n_ideal[g] for g in active_guns)
```

### B.2 Downgrade to fit module pool

If `M_ideal_total ≤ M_healthy`, set `n_modules[g] = n_ideal[g]`.

If `M_ideal_total > M_healthy`, downgrade:

```pseudo
function discrete_module_allocation(active_guns, P_budget, n_ideal, M_healthy):
    n_modules[g] = n_ideal[g] for g in active_guns
    total = sum(n_modules[g] for g in active_guns)

    if total <= M_healthy:
        return n_modules

    # Stage 1: downgrade 2 -> 1
    while total > M_healthy:
        candidates = [g for g in active_guns if n_modules[g] == 2]
        if not candidates:
            break
        g_star = argmin_over(candidates, key = (priority[g], P_budget[g]))
        n_modules[g_star] -= 1
        total -= 1

    # Stage 2: downgrade 1 -> 0
    while total > M_healthy:
        candidates = [g for g in active_guns if n_modules[g] == 1]
        if not candidates:
            break
        g_star = argmin_over(candidates, key = (priority[g], P_budget[g]))
        n_modules[g_star] -= 1
        total -= 1

    return n_modules
```

Now `n_modules[g] ∈ {0,1,2}` and `Σ n_modules[g] ≤ M_healthy`.

This is your **discrete power sharing**.

---

# 6. Algorithm C – mapping modules and MN contactors

### 6.1 Local per‑gun MN mapping (simple variant)

For now, we assume each gun uses **modules in its own slot** only:

* Slot s for gun g has modules `[M_s_0, M_s_1]` and MN contactors `[MN_s_0, MN_s_1]`.

We control them as:

```pseudo
function apply_module_contactors_for_gun(gid, n_modules_g):
    slot_id = guns[gid].slot_id
    mod_ids = slots[slot_id].modules           # ["M1_0", "M1_1"]
    m0 = mod_ids[0] ; m1 = mod_ids[1]
    MN0 = module_state[m0].mn_id
    MN1 = module_state[m1].mn_id

    if n_modules_g == 0:
        open_contactor(MN0)
        open_contactor(MN1)
        module_state[m0].enabled = false
        module_state[m1].enabled = false

    elif n_modules_g == 1:
        # Prefer m0, fallback to m1
        if module_state[m0].healthy:
            close_contactor(MN0) ; module_state[m0].enabled = true
            open_contactor(MN1)  ; module_state[m1].enabled = false
        elif module_state[m1].healthy:
            open_contactor(MN0)  ; module_state[m0].enabled = false
            close_contactor(MN1) ; module_state[m1].enabled = true
        else:
            open_contactor(MN0)
            open_contactor(MN1)
            module_state[m0].enabled = false
            module_state[m1].enabled = false
            n_modules_g = 0   # effective

    elif n_modules_g == 2:
        # Enable up to 2 healthy modules
        if module_state[m0].healthy:
            close_contactor(MN0) ; module_state[m0].enabled = true
        else:
            open_contactor(MN0)  ; module_state[m0].enabled = false

        if module_state[m1].healthy:
            close_contactor(MN1) ; module_state[m1].enabled = true
        else:
            open_contactor(MN1)  ; module_state[m1].enabled = false
```

This uses per‑module contactors to keep extra modules OFF when not required.

Later, if you decide to allow cross‑slot sharing, you just change this mapping to pick modules from neighboring slots in the same island.

---

# 7. Algorithm D – islands & MC (ring segmentation)

Even if initially you only use “local” modules per gun, you still need **per‑gun isolated DC buses** (each EV has its own voltage/current), so MCs are used to isolate each gun’s slot into its own island.

### 7.1 One slot per island (simple default)

For min 1 / max 2 modules per gun, simplest mode is:

* Each active gun g uses **its own slot only** as island.
* We isolate slot s (gun g) by opening MCs around it so no other slot is electrically connected.

Example: for gun g at slot s:

* Open **MC_{s-1}** (between slot s-1 and s).
* Open **MC_s** (between slot s and s+1).
* Close all other MCs (if no other islands).

If another gun h is active at slot t, we open MC_{t-1} and MC_t as well, potentially splitting the ring into many islands and idle segments.

### 7.2 MC planning from active guns

Simplified rule (one-slot islands):

```pseudo
function recompute_mc_states_for_one_slot_islands():
    # Default: all closed
    for slot in slots:
        desired_MC_state[slot.mc_id] = CLOSED

    active_guns = list_active_guns()
    for g in active_guns:
        s = guns[g].slot_id
        s_prev = slots[s].ccw_id
        # Open boundary MCs around this slot
        desired_MC_state[slots[s_prev].mc_id] = OPEN
        desired_MC_state[slots[s].mc_id]     = OPEN
```

You can refine this later to allocate multi‑slot islands (for more modules), but for min/max 1–2 modules per gun in same slot, this simple one‑slot island model works.

### 7.3 Applying MC states safely

Always:

* Bring affected islands to **zero current** before changing MCs.
* Verify MC auxiliary contacts after command.

```pseudo
function apply_mc_states_safely():
    # 1) Identify which MCs will change
    mc_to_change = []
    for slot in slots:
        mc = slot.mc_id
        if desired_MC_state[mc] != read_mc_aux(mc):
            mc_to_change.append(mc)

    # 2) For any island affected by MC change, ramp power to zero first
    affected_islands = islands_touched_by_mc_list(mc_to_change)
    for island in affected_islands:
        island.P_set_kw = 0
        wait_until_island_current_zero(island.id)

    # 3) Now switch MCs
    for mc in mc_to_change:
        if desired_MC_state[mc] == OPEN:
            open_contactor(mc)
        else:
            close_contactor(mc)

    # 4) Verify feedback
    for mc in mc_to_change:
        if read_mc_aux(mc) != desired_MC_state[mc]:
            mark_mc_failed(mc)
            handle_mc_failure(mc)  # e.g., disable near guns, log fault
```

---

# 8. Algorithm E – per‑island voltage & current control

Once MC, MN, and GC are configured:

* Each island has:

  * Some enabled modules (1 or 2 per gun).
  * One gun connected via GC.

We set:

* `V_set_v` for the island based on EV’s requested voltage.
* `P_set_kw` = min(`P_budget[g]`, module cap, P_GUN_LIMIT[g]).

### 8.1 Island setpoints

```pseudo
function update_island_setpoints():
    for island in islands:
        if island.gun_id is null:
            continue

        g = guns[island.gun_id]

        V_target = clamp(g.ev_req_voltage_v, V_EV_MIN[g.id], V_EV_MAX[g.id])
        P_cap_modules = count_enabled_modules(island) * P_MOD_KW

        island.V_set_v  = V_target
        island.P_set_kw = min(P_budget[g.id], P_cap_modules, P_GUN_LIMIT[g.id])
```

### 8.2 Convert to per‑gun current limit

For gun g in island i:

```pseudo
V_meas = measure_island_voltage(i)
V_safe = max(V_meas, V_MIN_FOR_DIV)     # avoid divide-by-zero
I_target = island.P_set_kw * 1000 / V_safe

# Enforce cable limit
if I_target > I_GUN_MAX[g.id]:
    I_target = I_GUN_MAX[g.id]

# Optional ramping:
step = I_RAMP_MAX_STEP
I_prev = guns[g.id].i_set_a
if I_target > I_prev + step:
    I_target = I_prev + step
if I_target < I_prev - step:
    I_target = I_prev - step
if I_target < 0:
    I_target = 0

guns[g.id].i_set_a = I_target
send_current_limit_to_hardware(g.id, I_target)
```

Modules in that island use internal voltage regulation loops; the above is **EV‑side current limit**.

---

# 9. Safety & Fault Handling

## 9.1 Global safety checks (every cycle)

```pseudo
function safety_ok():
    if emergency_stop_active(): return False
    if any_door_interlock_open(): return False
    if global_isolation_fault(): return False

    # Over/under voltage per island
    for island in islands:
        V = measure_island_voltage(island.id)
        if V > V_OV_TRIP or V < V_UV_TRIP:
            return False

    # Overcurrent per island
    for island in islands:
        I = measure_island_current(island.id)
        if I > I_ISLAND_TRIP:
            return False

    # Module temperature
    for m in modules:
        if m.temperature > T_MODULE_TRIP:
            return False

    # Gun connector temperature & current
    for g in guns:
        if guns[g].connector_temp_c > T_CONNECTOR_TRIP:
            return False
        if guns[g].i_meas_a > I_GUN_MAX[g] * I_OV_FACTOR:
            return False

    return True
```

If false:

```pseudo
function enter_global_fault():
    # 1) Ramp down power on all islands
    for island in islands:
        island.P_set_kw = 0
    wait_until_all_islands_currents_zero()

    # 2) Open all GC & MC & MN
    for g in guns: open_contactor(g.gc_id)
    for slot in slots: open_contactor(slot.mc_id)
    for m in modules: open_contactor(module_state[m].mn_id)

    # 3) Disable modules
    disable_all_modules()

    # 4) Set states and report
    for g in guns: g.fsm_state = FAULT
    global_state = FAULT
    send_fault_status_to_ocpp()
```

## 9.2 Island‑level safety

* **Single GC per island**:

```pseudo
function enforce_single_gc_per_island():
    for island in islands:
        closed_guns = []
        for slot_id in island.slot_ids:
            gun_id = slots[slot_id].gun_id
            if gun_id and is_gc_closed(guns[gun_id].gc_id):
                closed_guns.append(gun_id)
        if len(closed_guns) > 1:
            # Open all GCs and mark fault
            for gid in closed_guns:
                open_contactor(guns[gid].gc_id)
                guns[gid].fsm_state = FAULT
            raise_island_fault(island.id, "Multiple GCs closed")
```

* IMD per island: if isolation fault, shut down that island only.

## 9.3 MN & MC weld detection

For each contactor:

* If command OPEN but aux = CLOSED → welded.
* If welded MC: don’t plan boundaries at that MC; avoid segmentation there.
* If welded MN: module is forced ON; mark it unhealthy and never energize that island.

---

# 10. Main Control Loop (high‑level pseudocode)

This is what runs periodically (e.g. every 50–100 ms) in your local controller.

```pseudo
function main_loop():
    # 1) Read measurements (voltages, currents, temps, aux contacts, IMD)
    read_all_measurements()

    # 2) Safety
    if not safety_ok():
        enter_global_fault()
        return

    # 3) Update gun FSM from EV & OCPP events
    update_gun_fsm_from_ev_and_ocpp()

    # 4) Build list of active guns
    active_guns = [g for g in guns if g.ev_session_active
                                  and g.fsm_state in {READY, ISLAND_READY, CHARGING}]

    if active_guns is empty:
        # No active charging
        handle_zero_guns_state()
        return

    # 5) Compute power budgets
    P_budget = compute_power_budgets(active_guns)

    # 6) Compute discrete modules per gun (0/1/2)
    n_ideal = {}
    for g in active_guns:
        n_ideal[g] = ideal_modules_for_gun(g, P_budget[g])

    M_healthy = count_healthy_modules()
    n_modules = discrete_module_allocation(active_guns, P_budget, n_ideal, M_healthy)

    # 7) For each gun, update MN contactors according to n_modules[g]
    for g in active_guns:
        apply_module_contactors_for_gun(g, n_modules[g])

    # 8) Replan MC states for one-slot islands (per-gun isolation)
    recompute_mc_states_for_one_slot_islands()
    apply_mc_states_safely()

    # 9) Update islands structure (slots per gun)
    rebuild_islands_from_mc_and_guns()

    # 10) Set island setpoints & gun current limits
    update_island_setpoints()
    for island in islands:
        if island.gun_id is not null:
            apply_current_limit_for_gun(island.gun_id, island)

    # 11) Enforce island-level protections
    enforce_single_gc_per_island()
    enforce_island_safety_limits()
```

Integration with OCPP:

* `StartTransaction` / `RemoteStartTransaction` → mark `ev_session_active=true`, set initial `ev_req_power_kw`, `ev_req_voltage_v`, `P_GUN_LIMIT`.
* `StopTransaction` / `RemoteStopTransaction` / EV unplug → mark `ev_session_active=false`, FSM → RAMP_DOWN then IDLE.
* Smart Charging profiles → adjust `P_req_raw[g]` or `P_GUN_LIMIT[g]`.

---

# 11. Edge Cases & Handling

## 11.1 Zero guns active

* `active_guns` empty.
* All GC open.
* MN contactors open (modules OFF).
* MC can be:

  * All closed (one ring) for precharge or diagnostics, or
  * All open (fully de‑energized) for energy saving.

## 11.2 One gun at low power

* `P_budget[g]` small → `n_modules[g] = 1`.
* Only one MN closed (e.g. MN_i_0).
* MCs open around that slot to isolate one‑slot island.
* EV sees 1 module, up to ~P_MOD_KW.

## 11.3 One gun at high power

* `P_budget[g]` high → `n_modules[g] = 2`.
* Both MN_i_0, MN_i_1 closed (if healthy).
* Same island, same slot, but 2 modules share power.
* Max power limited by `2 * P_MOD_KW` and `P_GUN_LIMIT[g]`.

## 11.4 Many guns, total demand < capacity

* All `n_ideal[g]` possible.
* No downgrades (M_ideal_total ≤ M_healthy).
* Each gun gets 1 or 2 modules as needed.

## 11.5 Many guns, total demand > capacity

* Continuous power budgets `P_budget` share the total power fairly.
* Then discrete downgrades:

  * Some `n_modules[g]` move from 2→1.
  * If needed, some from 1→0.
* Lower priority and lower power guns are downgraded first.

## 11.6 Module failure

* Module’s `healthy` flips to false.
* `M_healthy` reduces.
* Next cycle:

  * `compute_power_budgets` reduces total available power.
  * `discrete_module_allocation` may lower `n_modules[g]`.
  * `apply_module_contactors_for_gun` avoids failed module.
* Gun may drop from 2 modules → 1 module; if both in its slot failed, gun can be reduced to 0 power or session ended.

## 11.7 Gun cable over‑current

* If hardware detects `i_meas > I_GUN_MAX * I_OV_FACTOR`:

  * Fast local action:

    * Reduce current command to zero.
    * Open GC.
    * Mark gun FAULT.
  * Modules for that gun are turned OFF in next cycles; freed modules go back into module pool.

## 11.8 IMD / isolation fault in one island

* IMD reading associated with specific island signals low insulation.
* Action:

  * Ramp island power to zero.
  * Open GC for that island.
  * Open MNs (all modules in that island).
  * If necessary, open MCs for that slot to keep the rest of ring safe.
  * Mark gun + island as FAULT, but other islands may continue charging.

## 11.9 MC weld

* During MC operation:

  * If we command OPEN but `aux == CLOSED`:

    * Mark MC as failed.
    * Prevent any plan that relies on that MC creating an island boundary.
    * Depending on severity, you may:

      * Stop nearby guns, or
      * Restrict operation to a smaller subset of islands.

## 11.10 EV unplug mid‑charge

* CP/PP loss or EV‑side stop:

  * FSM: CHARGING → RAMP_DOWN.
  * Ramp island power to zero, open GC.
  * Gun state → IDLE; `ev_session_active=false`.
  * Next allocation cycle reuses that gun’s modules.

---

# 12. Limitations & Extensions

### 12.1 Current limitations (by design choice)

* At this stage, each gun uses **only its own slot’s modules** (1 or 2), not cross‑slot.
* Each island is effectively a **single‑slot island** around the gun.
* Dynamic mid‑session re‑partitioning across slots is not implemented (keeps EV current stable and logic simple).

### 12.2 Future extensions (same architecture)

* Allow a gun to get **more than 2 modules** by extending its island to neighbor slots (using MCs and MNs).
* Implement **dynamic re‑partitioning** of islands mid‑session (with controlled shut‑down, re‑segmentation, and restart).
* Add **thermal‑aware allocation** (move modules between guns based on module temperatures).
* Integrate with site EMS for **price‑based priority** or grid support.

* A. Configuration model (JSON) for hardware + limits
* B. Concrete data structures and enums (C‑style)
* C. Detailed pseudocode for core flows (Start, Stop, module failure, grid limit change)
* D. Safety logic wiring (where each check happens)
* E. Short testing / commissioning plan

I’ll assume the **latest design** we converged on:

* 12 guns, 12 slots on a ring.
* 2 modules per slot, each with its own MN contactor.
* Min 0, max 2 modules per gun.
* For now, each gun only uses **its own modules** (no cross‑slot sharing), but the ring (MC) remains available for isolation / future expansion.

---

## A. Configuration Model (JSON / config file)

You want one source of truth for both the OCPP backend and the local controller.

### A.1 Hardware topology config

```json
{
  "chargerId": "SITE_001_CAB_01",
  "modulePowerKW": 30.0,
  "slots": [
    {
      "id": 1,
      "modules": [
        { "id": "M1_0", "mn": "MN_1_0" },
        { "id": "M1_1", "mn": "MN_1_1" }
      ],
      "gunId": 1,
      "gc": "GC_1",
      "mc": "MC_1",
      "cw": 2,
      "ccw": 12
    },
    {
      "id": 2,
      "modules": [
        { "id": "M2_0", "mn": "MN_2_0" },
        { "id": "M2_1", "mn": "MN_2_1" }
      ],
      "gunId": 2,
      "gc": "GC_2",
      "mc": "MC_2",
      "cw": 3,
      "ccw": 1
    }
    // ...
  ],
  "guns": [
    {
      "id": 1,
      "slotId": 1,
      "gc": "GC_1",
      "maxVoltageV": 1000.0,
      "minVoltageV": 200.0,
      "maxCurrentA": 200.0,
      "priorityDefault": 0
    },
    {
      "id": 2,
      "slotId": 2,
      "gc": "GC_2",
      "maxVoltageV": 1000.0,
      "minVoltageV": 200.0,
      "maxCurrentA": 200.0,
      "priorityDefault": 0
    }
    // ...
  ],
  "siteLimits": {
    "gridPowerLimitKW": 360.0,
    "defaultVoltageV": 800.0
  }
}
```

### A.2 Runtime state is NOT in config

The config is static. The following are runtime only:

* Module health, temperature, enabled state
* Gun FSM states, EV requested power / voltage
* Island definitions
* Safety status flags

---

## B. Concrete Data Structures (C‑style)

This is close to what you’d implement in C/C++ or embedded C with some adaptations.

```c
typedef enum {
    GUN_STATE_IDLE = 0,
    GUN_STATE_EV_DETECTED,
    GUN_STATE_READY,
    GUN_STATE_ISLAND_READY,
    GUN_STATE_CHARGING,
    GUN_STATE_RAMP_DOWN,
    GUN_STATE_FAULT
} GunStateEnum;

typedef struct {
    int   id;              // 1..12
    char  mnId[16];        // e.g. "MN_1_0"
    int   slotId;
    bool  healthy;
    bool  enabled;
    float temperatureC;
} ModuleState;

typedef struct {
    int    id;             // slot number 1..12
    char   gcId[16];       // e.g. "GC_1"
    char   mcId[16];       // e.g. "MC_1"
    int    cwId;           // clockwise slot id
    int    ccwId;          // counter-clockwise slot id
    int    gunId;          // 1..12
    char   moduleIds[2][16]; // e.g. {"M1_0", "M1_1"}
} SlotConfig;

typedef struct {
    int            id;                // 1..12
    int            slotId;
    char           gcId[16];
    GunStateEnum   state;
    bool           evSessionActive;
    float          evReqPowerKW;
    float          evReqVoltageV;
    float          maxVoltageV;
    float          minVoltageV;
    float          maxCurrentA;
    float          maxPowerKW;        // derived: maxVoltage * maxCurrent
    int            priority;
    int            islandId;          // -1 if none
    float          iMeasA;
    float          iSetA;
    float          connectorTempC;
} GunState;

typedef struct {
    int   id;              // island id
    int   slotId;          // single-slot island now
    char  moduleIds[2][16];
    int   nModules;        // 0,1,2 enabled
    int   gunId;           // -1 if idle
    float VsetV;
    float PsetKW;
} IslandState;
```

Global arrays:

```c
#define NUM_SLOTS 12
#define NUM_GUNS  12
#define MODULES_PER_SLOT 2
#define NUM_MODULES (NUM_SLOTS * MODULES_PER_SLOT)

SlotConfig   g_slots[NUM_SLOTS];
ModuleState  g_modules[NUM_MODULES];
GunState     g_guns[NUM_GUNS];
IslandState  g_islands[NUM_GUNS];     // 1 island per potential gun (simple mode)
int          g_numIslands;            // <= NUM_GUNS

// Limits
float g_modulePowerKW = 30.0f;
float g_gridPowerLimitKW;

// Contactor commands / states
bool  g_desiredMCState[NUM_SLOTS];    // indexed by slot -> MC_i
bool  g_desiredGCState[NUM_GUNS];
bool  g_desiredMNState[NUM_MODULES];
```

---

## C. Detailed Flows

### C.1 EV plug‑in and StartTransaction (Start Charging)

We’ll break it into phases:

1. EV detects & handshake
2. Power allocation
3. MN/MC configuration and module start
4. Precharge and GC close
5. Normal current control

#### 1) EV detected & handshake

High‑level:

```pseudo
on_ev_plug_in(gunId):
    GunState* g = &g_guns[gunId]
    g->state = GUN_STATE_EV_DETECTED

on_handshake_ok_and_ocpp_start(gunId, P_req_initial_kw, V_req_v):
    GunState* g = &g_guns[gunId]
    g->evSessionActive = true
    g->evReqPowerKW    = P_req_initial_kw
    g->evReqVoltageV   = V_req_v
    g->state           = GUN_STATE_READY
```

#### 2) Allocation step (continuous + discrete)

Executed in main loop whenever there is a new READY gun or a significant change:

```pseudo
function allocate_modules_and_power():
    activeGuns = list of gunIds where
                  g_guns[i].evSessionActive == true AND
                  g_guns[i].state in {READY, ISLAND_READY, CHARGING}

    if activeGuns empty:
        return

    // Build P_req_limited and compute budgets
    for gId in activeGuns:
        P_req_raw = g_guns[gId].evReqPowerKW
        P_limit   = g_guns[gId].maxPowerKW
        P_req_limited[gId] = min(P_req_raw, P_limit)

    M_healthy = countHealthyModules()
    P_module_total = M_healthy * g_modulePowerKW
    P_system_cap   = min(P_module_total, g_gridPowerLimitKW)

    P_budget = compute_power_budgets(activeGuns, P_req_limited, P_system_cap)

    // Discrete modules per gun
    for gId in activeGuns:
        n_ideal[gId] = ideal_modules_for_gun(gId, P_budget[gId])

    n_modules = discrete_module_allocation(activeGuns, P_budget, n_ideal, M_healthy)

    // Apply MN contactors locally per gun
    for gId in activeGuns:
        apply_module_contactors_for_gun(gId, n_modules[gId])

    // Derive islands and MC states (one-slot islands)
    recompute_mc_states_for_one_slot_islands()
    apply_mc_states_safely()

    // Build island objects
    rebuild_islands_from_mc_and_guns()
```

`rebuild_islands_from_mc_and_guns()` in one‑slot mode is trivial:

```pseudo
function rebuild_islands_from_mc_and_guns():
    g_numIslands = 0
    for each gun gId:
        if g_guns[gId].evSessionActive and (g_guns[gId].state in {READY, CHARGING}):
            island = &g_islands[g_numIslands++]
            island->id      = gId   // one island per gun
            island->slotId  = g_guns[gId].slotId
            island->gunId   = gId

            // copy modules for that slot
            slot = getSlotById(island->slotId)
            for i in [0..1]:
                strcpy(island->moduleIds[i], slot->moduleIds[i])
            island->nModules = count_enabled_modules(island)
```

#### 3) Precharge and GC close

Once an island is defined and its modules enabled, you bring the island bus up and close GC.

```pseudo
function ensure_island_ready_for_gun(gId):
    GunState* g = &g_guns[gId]
    if g->state != GUN_STATE_READY:
        return

    IslandState* isl = getIslandByGunId(gId)
    if isl == NULL:
        return

    // set setpoints
    update_island_setpoints()

    // internal module enable, with 0 current and ramped voltage
    enable_modules_for_island(isl->id)

    precharge_island_bus_to_ev_voltage(isl->id, g->evReqVoltageV)

    if precharge_ok(isl->id, gId):
        // safe to close GC
        close_contactor(g->gcId)
        g->state = GUN_STATE_CHARGING
```

After this, the **normal current control** (quoted before) runs continuously to track `P_budget` and `I_target`.

### C.2 StopTransaction / EV unplug / Stop Charging

Trigger conditions:

* OCPP `RemoteStopTransaction` / `StopTransaction`
* EV BMS stop command
* EV unplug (CP/PP lost)
* Fault (overcurrent, overtemp, etc.)

#### 1) FSM transition

```pseudo
function request_stop_charging(gunId):
    GunState* g = &g_guns[gunId]
    if g->state == GUN_STATE_CHARGING:
        g->state = GUN_STATE_RAMP_DOWN
```

#### 2) Ramping down & GC open

In main loop:

```pseudo
function handle_ramp_downs():
    for each g in g_guns:
        if g.state == GUN_STATE_RAMP_DOWN:
            isl = getIslandByGunId(g.id)
            if isl != NULL:
                isl->PsetKW = 0
            guns[g.id].iSetA = 0
            send_current_limit_to_hardware(g.id, 0)

            if island_current_below_threshold(isl->id):
                open_contactor(g.gcId)
                g.state            = GUN_STATE_IDLE
                g.evSessionActive  = false
                g.islandId         = -1

                // Disable modules for that gun
                disable_modules_for_slot(g.slotId)
```

Where `disable_modules_for_slot(slotId)` opens MN_i_0, MN_i_1 and marks modules disabled.

#### 3) De‑allocation

At the next allocation cycle, this gun is not in `activeGuns` anymore; its modules are returned to the pool.

---

### C.3 Grid power limit change event

If your site EMS / backend changes `P_GRID_LIMIT` at runtime:

```pseudo
on_grid_limit_change(newLimitKW):
    g_gridPowerLimitKW = newLimitKW
    // Next main_loop() cycle:
    // - compute_power_budgets() will use the new capacity
    // - n_modules allocation may downgrade some guns from 2->1 or 1->0
    // - MN per-gun will be adjusted accordingly
    // - island setpoints & gun currents will track reduced P_budget[]
```

No special sequence is needed, because the algorithm naturally re‑allocates power and modules under the new limit.

---

### C.4 Module failure / over‑temperature event

When a module fails or overtemperature triggers:

```pseudo
on_module_fault(moduleId):
    ModuleState* m = getModuleById(moduleId)
    m->healthy = false
    m->enabled = false
    // open MN contactor
    open_contactor(m->mnId)

    // Next allocation cycle:
    // - M_healthy decreases
    // - P_module_total = M_healthy * P_MOD_KW decreases
    // - compute_power_budgets() may reduce P_budget[]
    // - discrete_module_allocation() may reduce n_modules[g]
    // - modules per gun and setpoints are updated accordingly
```

If an entire slot loses both modules, that gun may end up with `n_modules[g] = 0` → effectively no power. You can either:

* Allow EV to sit in “ready but 0 kW” state, or
* Gracefully stop that session and report “Insufficient power / internal fault” via OCPP.

---

## D. Safety Wiring (who checks what, where)

### D.1 Fast safety – in local controller / hardware

* **Over‑current protections**:

  * Per‑module hardware protection / fast current limit.
  * Per‑gun current limit in inner control loop.
* **Over‑voltage protections**:

  * Bus OV detection inside module controller → immediate ramp down / trip.
* **Hardware interlocks**:

  * E‑stop line → direct trip to module and contactors.
  * Door switches hardwired to a safe circuit that prevents bus energization.

These **must not depend on OCPP** or higher‑level allocation logic.

### D.2 Medium‑speed safety – inside main loop

* `safety_ok()` run every cycle (50‑100 ms).

  * Checks IMD, bus voltages, island currents, connector temps, etc.
  * If failing → `enter_global_fault()`.

* `enforce_single_gc_per_island()` run every cycle.

  * Avoid multiple EVs on same DC bus.

* `verify_mc_feedback_before_energize()` whenever MC states are changed.

  * To detect MC weld / mismatch.

* `module_state[ ].healthy` and `enabled` updated per measurements and faults.

### D.3 OCPP‑level safety / policy

* Limiting how many guns are allowed to start at once (e.g. site policy).
* Priority decisions: the `priority[g]` you use in the allocator can be set from backend (fleet vs public).
* If backend detects persistent fault on a connector, it can set it to `Unavailable` so no sessions are started.

---

## E. Testing & Commissioning Plan (high‑level)

To de‑risk deployment, you should plan:

### E.1 Unit tests of core algorithms (offline)

* Pure software tests of:

  * `compute_power_budgets()` – fairness, respecting limits.
  * `discrete_module_allocation()` – correct downgrade behavior.
  * `apply_module_contactors_for_gun()` – correct MN states for 0/1/2 modules.
  * `recompute_mc_states_for_one_slot_islands()` – correct MC open/close sets.

* Scenarios:

  * 0…12 guns with random power demands.
  * Random module failures.
  * Step changes in grid power limit.

### E.2 HIL / simulator

* Hardware‑in‑the‑loop simulation where:

  * You emulate modules, contactors, EVs, IMD, etc.
  * Run your actual control firmware.
  * Force faults:

    * Over‑current, over‑voltage, IMD triggers.
    * Welded contactors (aux mismatch).
    * Temperature threshold breaches.

Verify:

* System always goes to safe state.
* No two GCs are closed on same island.
* No MC reconfiguration under high current.

### E.3 Field commissioning

* Start with 1 slot & 1 gun, one module enabled:

  * Validate handshake, current ramp, stop sequence.
* Gradually add modules and guns:

  * Check dynamic allocation behavior.
  * Check per‑gun current & cable temperature under maximum power.
