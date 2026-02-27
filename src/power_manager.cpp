// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>
#include <set>

namespace charger {

PowerManager::PowerManager(PlannerConfig cfg) : cfg_(std::move(cfg)) {}

void PowerManager::set_slots(std::vector<Slot> slots) {
    slots_ = std::move(slots);
    slot_lookup_.clear();
    for (const auto& s : slots_) {
        slot_lookup_[s.id] = s;
    }
}

void PowerManager::update_modules(const std::vector<ModuleState>& modules) {
    modules_.clear();
    for (const auto& m : modules) {
        modules_.emplace(m.id, m);
    }
}

void PowerManager::update_guns(const std::vector<GunState>& guns) {
    guns_.clear();
    for (const auto& g : guns) {
        guns_.emplace(g.id, g);
    }
}

std::vector<int> PowerManager::active_guns() const {
    std::vector<int> active;
    for (const auto& kv : guns_) {
        const auto& g = kv.second;
        if (!g.ev_session_active) continue;
        if (g.fsm_state == GunFsmState::Ready || g.fsm_state == GunFsmState::IslandReady ||
            g.fsm_state == GunFsmState::Charging) {
            active.push_back(g.id);
        }
    }
    return active;
}

Plan PowerManager::blank_plan() const {
    Plan plan;
    for (const auto& slot : slots_) {
        plan.mc_commands[slot.mc_id] = ContactorState::Closed;
        if (!slot.gc_id.empty()) {
            plan.gc_commands[slot.gc_id] = ContactorState::Open;
        }
    }
    for (const auto& m : modules_) {
        plan.mn_commands[m.second.mn_id] = ContactorState::Open;
    }
    return plan;
}

const Slot* PowerManager::find_slot(int slot_id) const {
    auto it = std::find_if(slots_.begin(), slots_.end(), [&](const Slot& s) { return s.id == slot_id; });
    return it == slots_.end() ? nullptr : &(*it);
}

int PowerManager::count_healthy_modules_in_slot(int slot_id) const {
    int healthy = 0;
    for (const auto& kv : modules_) {
        if (kv.second.slot_id == slot_id && kv.second.healthy) {
            healthy++;
        }
    }
    return healthy;
}

int PowerManager::ideal_modules_for_gun(const GunState& g, double p_budget) const {
    if (p_budget <= 0.0) {
        return 0;
    }
    if (cfg_.module_power_kw <= 0.0) {
        return cfg_.min_modules_per_active_gun;
    }
    int n = static_cast<int>(std::ceil(p_budget / cfg_.module_power_kw));
    int max_by_config = std::max(1, cfg_.max_modules_per_gun);
    int max_by_cable = max_by_config;
    if (g.gun_power_limit_kw > 0.0) {
        max_by_cable = static_cast<int>(std::ceil(g.gun_power_limit_kw / cfg_.module_power_kw));
        max_by_cable = std::min(max_by_cable, max_by_config);
    }
    const int min_allowed = p_budget > 0.0 ? std::max(0, cfg_.min_modules_per_active_gun) : 0;
    n = std::clamp(n, min_allowed, std::max(min_allowed, max_by_cable));
    return n;
}

std::vector<std::string> PowerManager::select_modules_for_slot(const Slot& slot, int n_needed,
                                                               Plan& plan,
                                                               const std::vector<std::string>& preferred) const {
    std::vector<std::string> selected;
    if (n_needed <= 0) {
        for (const auto& mod_id : slot.modules) {
            const auto it = modules_.find(mod_id);
            if (it != modules_.end()) {
                plan.mn_commands[it->second.mn_id] = ContactorState::Open;
            }
        }
        return selected;
    }

    std::vector<std::string> ordered;
    std::set<std::string> already;
    for (const auto& pref : preferred) {
        if (std::find(slot.modules.begin(), slot.modules.end(), pref) != slot.modules.end()) {
            ordered.push_back(pref);
            already.insert(pref);
        }
    }
    for (const auto& mod_id : slot.modules) {
        if (!already.count(mod_id)) {
            ordered.push_back(mod_id);
        }
    }

    auto maybe_enable = [&](const std::string& mod_id) {
        const auto it = modules_.find(mod_id);
        if (it == modules_.end()) return;
        const auto& m = it->second;
        if (!m.healthy) {
            plan.mn_commands[m.mn_id] = ContactorState::Open;
            return;
        }
        if (static_cast<int>(selected.size()) < n_needed) {
            plan.mn_commands[m.mn_id] = ContactorState::Closed;
            selected.push_back(mod_id);
        } else {
            plan.mn_commands[m.mn_id] = ContactorState::Open;
        }
    };

    for (const auto& mod_id : ordered) {
        if (static_cast<int>(selected.size()) >= n_needed) break;
        maybe_enable(mod_id);
    }
    return selected;
}

std::vector<int> PowerManager::build_island_slots_for_gun(const GunState& g, int n_needed,
                                                          const std::set<int>& active_home_slots,
                                                          const std::set<int>& reserved_slots,
                                                          std::set<int>& claimed_slots) const {
    std::vector<int> slots;
    if (!find_slot(g.slot_id)) return slots;
    slots.push_back(g.slot_id);
    claimed_slots.insert(g.slot_id);

    if (!cfg_.allow_cross_slot_islands || n_needed <= count_healthy_modules_in_slot(g.slot_id)) {
        // Local-only operation: keep ownership on the home slot.
        // This keeps the home gun's right-side tie open unless additional cabinets are
        // explicitly borrowed for extra power.
        return slots;
    }

    int cw_edge = g.slot_id;
    int ccw_edge = g.slot_id;
    int available = count_healthy_modules_in_slot(g.slot_id);
    int remaining = std::max(0, n_needed - available);
    int cw_steps = 0;
    int ccw_steps = 0;

    while (remaining > 0 && (cw_steps < cfg_.max_island_radius || ccw_steps < cfg_.max_island_radius)) {
        bool expanded = false;
        // Borrow anti-clockwise first, then clockwise if more capacity is still needed.
        if (ccw_steps < cfg_.max_island_radius) {
            if (slot_lookup_.count(ccw_edge)) {
                const int candidate = slot_lookup_.at(ccw_edge).ccw_id;
                if (slot_lookup_.count(candidate) && !claimed_slots.count(candidate) &&
                    !active_home_slots.count(candidate) &&
                    (!reserved_slots.count(candidate) || candidate == g.slot_id)) {
                    const int healthy = count_healthy_modules_in_slot(candidate);
                    slots.insert(slots.begin(), candidate);
                    ccw_edge = candidate;
                    claimed_slots.insert(candidate);
                    available += healthy;
                    remaining = std::max(0, n_needed - available);
                    expanded = true;
                }
                ccw_steps++;
            } else {
                ccw_steps = cfg_.max_island_radius;
            }
        }

        if (remaining <= 0) break;

        if (cw_steps < cfg_.max_island_radius) {
            if (slot_lookup_.count(cw_edge)) {
                const int candidate = slot_lookup_.at(cw_edge).cw_id;
                if (slot_lookup_.count(candidate) && !claimed_slots.count(candidate) &&
                    !active_home_slots.count(candidate) &&
                    (!reserved_slots.count(candidate) || candidate == g.slot_id)) {
                    const int healthy = count_healthy_modules_in_slot(candidate);
                    slots.push_back(candidate);
                    cw_edge = candidate;
                    claimed_slots.insert(candidate);
                    available += healthy;
                    remaining = std::max(0, n_needed - available);
                    expanded = true;
                }
                cw_steps++;
            } else {
                cw_steps = cfg_.max_island_radius;
            }
        }

        if (!expanded) {
            break;
        }
    }
    return slots;
}

std::vector<std::string> PowerManager::assign_modules_for_island(const std::vector<int>& slot_ids, int n_needed,
                                                                 Plan& plan,
                                                                 const std::vector<std::string>& preferred) const {
    std::vector<std::string> selected;
    int remaining = n_needed;
    for (int slot_id : slot_ids) {
        const auto* slot = find_slot(slot_id);
        if (!slot || remaining <= 0) {
            continue;
        }
        std::vector<std::string> slot_pref;
        for (const auto& id : preferred) {
            const auto it = modules_.find(id);
            if (it != modules_.end() && it->second.slot_id == slot_id) {
                slot_pref.push_back(id);
            }
        }
        auto slot_sel = select_modules_for_slot(*slot, remaining, plan, slot_pref);
        remaining -= static_cast<int>(slot_sel.size());
        selected.insert(selected.end(), slot_sel.begin(), slot_sel.end());
    }
    return selected;
}

std::map<int, double> PowerManager::compute_power_budgets(const std::vector<int>& active,
                                                          const std::map<int, double>& req_limited,
                                                          int healthy_modules) const {
    std::map<int, double> budgets;
    if (active.empty() || healthy_modules <= 0) {
        return budgets;
    }

    const double p_module_total = healthy_modules * cfg_.module_power_kw;
    const double system_cap = std::min(p_module_total, cfg_.grid_limit_kw);

    double total_req = 0.0;
    for (auto gid : active) {
        auto it = req_limited.find(gid);
        if (it != req_limited.end()) {
            total_req += it->second;
        }
    }

    if (total_req <= system_cap) {
        for (auto gid : active) {
            budgets[gid] = req_limited.at(gid);
        }
        return budgets;
    }

    std::set<int> remaining(active.begin(), active.end());
    double remaining_power = system_cap;
    while (!remaining.empty() && remaining_power > 0.0) {
        double total_weight = 0.0;
        for (auto gid : remaining) {
            const auto& g = guns_.at(gid);
            const double priority = static_cast<double>(std::max(0, g.priority));
            const double w = std::max(0.1, 1.0 / (1.0 + priority));
            total_weight += w;
        }

        bool any_capped = false;
        std::vector<int> remove_list;

        for (auto gid : remaining) {
            const double priority = static_cast<double>(std::max(0, guns_.at(gid).priority));
            const double weight = std::max(0.1, 1.0 / (1.0 + priority));
            const double share = remaining_power * (weight / total_weight);
            const double candidate = budgets[gid] + share;
            const double cap = req_limited.at(gid);
            if (candidate >= cap) {
                const double extra = cap - budgets[gid];
                budgets[gid] = cap;
                remaining_power = std::max(0.0, remaining_power - std::max(extra, 0.0));
                remove_list.push_back(gid);
                any_capped = true;
            } else {
                budgets[gid] = candidate;
            }
        }

        for (auto gid : remove_list) {
            remaining.erase(gid);
        }
        if (!any_capped) {
            remaining_power = 0.0;
        }
    }

    return budgets;
}

std::map<int, int> PowerManager::compute_module_allocation(const std::vector<int>& active,
                                                           const std::map<int, double>& budgets,
                                                           const std::map<int, int>& ideal,
                                                           int healthy_modules) const {
    std::map<int, int> n_modules = ideal;
    int total = 0;
    for (auto gid : active) {
        total += n_modules[gid];
    }
    if (total <= healthy_modules) {
        return n_modules;
    }

    auto reduce_once = [&](bool respect_minimum) -> bool {
        int chosen = -1;
        int from_count = -1;
        int worst_priority = std::numeric_limits<int>::min();
        double lowest_budget = std::numeric_limits<double>::max();
        for (auto gid : active) {
            const double budget = budgets.at(gid);
            const auto& g = guns_.at(gid);
            const int min_allowed = (budget > 0.0 && respect_minimum)
                                        ? std::max(0, cfg_.min_modules_per_active_gun)
                                        : 0;
            if (n_modules[gid] <= min_allowed) continue;
            if (n_modules[gid] > from_count ||
                (n_modules[gid] == from_count && (g.priority > worst_priority ||
                                                 (g.priority == worst_priority && budget < lowest_budget)))) {
                chosen = gid;
                from_count = n_modules[gid];
                worst_priority = g.priority;
                lowest_budget = budget;
            }
        }
        if (chosen == -1) return false;
        n_modules[chosen] -= 1;
        total -= 1;
        return true;
    };

    // First reduce down to per-gun minimums, then below if still over-subscribed.
    while (total > healthy_modules && reduce_once(true)) {
    }
    while (total > healthy_modules && reduce_once(false)) {
    }
    return n_modules;
}

double PowerManager::compute_voltage_margin_v(const GunState& g, double v_ceiling_v) const {
    double margin_v = std::max(0.0, cfg_.voltage_margin_v);
    if (v_ceiling_v <= 0.0) {
        return margin_v;
    }
    // Keep precharge/non-HLC phases on the fixed margin policy.
    if (g.voltage_guard_active) {
        return margin_v;
    }
    const double pct_low = std::clamp(cfg_.final_voltage_margin_low_pct, 0.0, 0.20);
    const double pct_high = std::clamp(cfg_.final_voltage_margin_high_pct, 0.0, 0.20);
    const double high = std::max(pct_low, pct_high);
    const double low = std::min(pct_low, pct_high);
    if (high <= 0.0) {
        return margin_v;
    }

    double proximity = 0.0;
    if (g.v_meas_v > 0.0) {
        proximity = std::clamp(g.v_meas_v / std::max(1.0, v_ceiling_v), 0.0, 1.0);
    }
    const double pct = high - ((high - low) * proximity);
    const double pct_margin_v = v_ceiling_v * pct;
    margin_v = std::max(margin_v, pct_margin_v);
    return std::max(0.0, margin_v);
}

double PowerManager::apply_voltage_guard(const GunState& g, double i_target_a, double v_ceiling_v) const {
    if (i_target_a <= 0.0) {
        return 0.0;
    }
    // In active HLC power phase (CurrentDemand), current-limiting via voltage guard
    // can double-derate against already headroomed EV targets from PLC telemetry.
    // Keep guard for precharge/non-HLC phases, bypass for steady power regulation.
    if (!g.voltage_guard_active) {
        return std::max(0.0, i_target_a);
    }
    const double guard_v = std::max(0.0, cfg_.voltage_guard_band_v);
    if (guard_v <= 0.0 || g.v_meas_v <= 0.0 || v_ceiling_v <= 0.0) {
        return std::max(0.0, i_target_a);
    }
    const double dv = v_ceiling_v - g.v_meas_v;
    if (dv <= 0.0) {
        return 0.0;
    }
    if (dv >= guard_v) {
        return std::max(0.0, i_target_a);
    }
    return std::max(0.0, i_target_a * (dv / guard_v));
}

double PowerManager::apply_current_ramp(int gun_id, const GunState& g, double i_target_a, bool emergency_drop,
                                        std::chrono::steady_clock::time_point now) {
    auto& st = ramp_state_by_gun_[gun_id];
    if (!st.initialized) {
        st.current_a = std::max(0.0, g.i_set_a);
        st.rate_a_per_s = 0.0;
        st.initialized = true;
        st.last_update = now;
    }

    double dt_s = std::chrono::duration_cast<std::chrono::duration<double>>(now - st.last_update).count();
    st.last_update = now;
    if (dt_s <= 0.0) {
        dt_s = 0.05;
    }
    dt_s = std::clamp(dt_s, 0.01, 0.25);

    i_target_a = std::max(0.0, i_target_a);
    const double up_min = std::max(1e-3, cfg_.ramp_up_min_a_per_s);
    const double up_max = std::max(up_min, cfg_.ramp_up_max_a_per_s);
    const double down_min = std::max(1e-3, cfg_.ramp_down_min_a_per_s);
    const double down_max_normal = std::max(down_min, cfg_.ramp_down_max_a_per_s);
    const double down_max = emergency_drop
                                ? std::max(down_max_normal, cfg_.ramp_down_emergency_a_per_s)
                                : down_max_normal;
    const double down_min_active = emergency_drop ? std::max(down_min, cfg_.ramp_down_emergency_a_per_s) : down_min;
    const double response_s = std::max(0.05, cfg_.ramp_response_s);
    const double jerk_limit = std::max(1.0, cfg_.ramp_jerk_a_per_s2);
    const double capture_i = std::max(0.01, cfg_.ramp_capture_current_a);
    const double capture_rate = std::max(0.01, cfg_.ramp_capture_rate_a_per_s);

    const double err = i_target_a - st.current_a;
    double desired_rate = std::clamp(err / response_s, -down_max, up_max);
    if (std::fabs(err) > capture_i) {
        if (desired_rate > 0.0) {
            desired_rate = std::max(desired_rate, up_min);
        } else if (desired_rate < 0.0) {
            desired_rate = std::min(desired_rate, -down_min_active);
        }
    }

    const double max_rate_delta = jerk_limit * dt_s;
    st.rate_a_per_s += std::clamp(desired_rate - st.rate_a_per_s, -max_rate_delta, max_rate_delta);
    const double prev_current = st.current_a;
    st.current_a += st.rate_a_per_s * dt_s;

    // Prevent crossing/oscillation around target due to integration + quantization.
    const double prev_err = i_target_a - prev_current;
    const double new_err = i_target_a - st.current_a;
    if (prev_err == 0.0 || (prev_err > 0.0 && new_err < 0.0) || (prev_err < 0.0 && new_err > 0.0)) {
        st.current_a = i_target_a;
        st.rate_a_per_s = 0.0;
    }

    if (std::fabs(i_target_a - st.current_a) <= capture_i && std::fabs(st.rate_a_per_s) <= capture_rate) {
        st.current_a = i_target_a;
        st.rate_a_per_s = 0.0;
    }

    if (i_target_a <= 0.0 && st.current_a < capture_i) {
        st.current_a = 0.0;
        if (st.rate_a_per_s < 0.0) {
            st.rate_a_per_s = 0.0;
        }
    }

    return std::max(0.0, st.current_a);
}

void PowerManager::prune_inactive_ramp_state(const std::vector<int>& active) {
    std::set<int> active_set(active.begin(), active.end());
    for (auto it = ramp_state_by_gun_.begin(); it != ramp_state_by_gun_.end();) {
        if (!active_set.count(it->first)) {
            it = ramp_state_by_gun_.erase(it);
        } else {
            ++it;
        }
    }
}

Plan PowerManager::build_plan(const std::vector<int>& active, const std::map<int, double>& budgets,
                              const std::map<int, int>& modules_per_gun,
                              const std::set<int>& reserved_slots,
                              const std::set<int>& full_island_guns,
                              std::chrono::steady_clock::time_point now) {
    Plan plan = blank_plan();
    next_island_id_ = 1;

    std::set<int> active_home_slots;
    for (auto gid : active) {
        active_home_slots.insert(guns_.at(gid).slot_id);
    }

    std::set<int> claimed_slots = active_home_slots;
    std::map<int, std::vector<int>> island_slots;
    for (auto gid : active) {
        const auto& g = guns_.at(gid);
        const int n_needed = modules_per_gun.at(gid);
        if (full_island_guns.count(gid)) {
            std::vector<int> full;
            const int start_id = g.slot_id > 0 ? g.slot_id : (slots_.empty() ? 0 : slots_.front().id);
            if (start_id > 0 && slot_lookup_.count(start_id)) {
                std::set<int> visited;
                int current = start_id;
                while (current != 0 && !visited.count(current)) {
                    visited.insert(current);
                    full.push_back(current);
                    const auto it = slot_lookup_.find(current);
                    if (it == slot_lookup_.end()) {
                        break;
                    }
                    const int next = it->second.cw_id;
                    if (next == 0 || next == start_id) {
                        break;
                    }
                    current = next;
                }
            }
            if (full.size() != slots_.size()) {
                full.clear();
                for (const auto& s : slots_) {
                    full.push_back(s.id);
                }
            }
            island_slots[gid] = std::move(full);
            continue;
        }
        if (n_needed <= 0) {
            island_slots[gid] = {g.slot_id};
            continue;
        }
        island_slots[gid] = build_island_slots_for_gun(g, n_needed, active_home_slots, reserved_slots, claimed_slots);
        if (island_slots[gid].empty()) {
            island_slots[gid].push_back(g.slot_id);
        }
    }

    for (auto gid : active) {
        const auto& g = guns_.at(gid);
        const auto* home_slot = find_slot(g.slot_id);
        if (!home_slot) {
            continue;
        }

        int n_needed = modules_per_gun.at(gid);
        if (!g.safety_ok || g.gc_welded || g.mc_welded) {
            n_needed = 0;
        }

        const auto slots_for_g = island_slots[gid];

        const auto preferred = last_module_ids_[gid];
        auto selected = assign_modules_for_island(slots_for_g, n_needed, plan, preferred);
        const int assigned = static_cast<int>(selected.size());
        if (assigned == 0) {
            plan.gc_commands[home_slot->gc_id] = ContactorState::Open;
        } else {
            plan.gc_commands[home_slot->gc_id] = ContactorState::Closed;
        }

        IslandState island;
        island.id = next_island_id_++;
        island.slot_ids = slots_for_g;
        island.gun_id = gid;
        island.module_ids = selected;

        const double p_cap_modules = assigned * cfg_.module_power_kw;
        const double p_budget = budgets.at(gid);
        const double p_set = std::min({p_budget, p_cap_modules,
                                       g.gun_power_limit_kw > 0.0 ? g.gun_power_limit_kw : p_cap_modules});
        const double min_v = g.min_voltage_v > 0.0 ? g.min_voltage_v : cfg_.min_voltage_v_for_div;
        const double max_v = g.max_voltage_v > 0.0 ? g.max_voltage_v : std::numeric_limits<double>::max();
        double v_ceiling = g.ev_req_voltage_v > 0.0 ? g.ev_req_voltage_v : cfg_.default_voltage_v;
        if (v_ceiling < min_v) v_ceiling = min_v;
        if (v_ceiling > max_v) v_ceiling = max_v;
        const double v_margin = compute_voltage_margin_v(g, v_ceiling);
        double v_target = std::max(min_v, v_ceiling - v_margin);
        if (v_target > max_v) v_target = max_v;
        island.p_set_kw = p_set;
        island.v_set_v = std::max(v_target, cfg_.min_voltage_v_for_div);

        GunDispatch dispatch;
        dispatch.gun_id = gid;
        dispatch.p_budget_kw = p_budget;
        dispatch.p_set_kw = p_set;
        dispatch.modules_assigned = assigned;
        dispatch.voltage_set_v = island.v_set_v;
        const double v_safe = std::max(cfg_.min_voltage_v_for_div, island.v_set_v);
        double i_target = (p_set * 1000.0) / v_safe;
        if (g.gun_current_limit_a > 0.0) {
            i_target = std::min(i_target, g.gun_current_limit_a);
        }
        i_target = std::max(0.0, i_target - std::max(0.0, cfg_.current_margin_a));
        i_target = apply_voltage_guard(g, i_target, v_ceiling);
        const bool emergency_drop = !g.safety_ok || g.gc_welded || g.mc_welded;
        i_target = apply_current_ramp(gid, g, i_target, emergency_drop, now);
        dispatch.current_limit_a = i_target;
        dispatch.p_set_kw = std::min(dispatch.p_set_kw, (dispatch.current_limit_a * v_safe) / 1000.0);
        island.p_set_kw = dispatch.p_set_kw;

        plan.islands.push_back(std::move(island));
        plan.guns.push_back(dispatch);
    }

    // Canonical tie truth rule for a ring:
    // close tie(i->i+1) iff both adjacent cabinets share the same owner.
    //
    // Owners:
    // - gun_id for active island members
    // - 0 for idle/unassigned pool cabinets
    std::map<int, int> slot_owner;
    for (const auto& s : slots_) {
        slot_owner[s.id] = 0;
    }
    for (const auto& island : plan.islands) {
        if (!island.gun_id.has_value()) {
            continue;
        }
        const int owner = island.gun_id.value();
        for (int sid : island.slot_ids) {
            slot_owner[sid] = owner;
        }
    }
    for (const auto& s : slots_) {
        ContactorState tie_state = ContactorState::Open;
        const auto cw_it = slot_lookup_.find(s.cw_id);
        const auto own_it = slot_owner.find(s.id);
        if (cw_it != slot_lookup_.end() && own_it != slot_owner.end()) {
            const auto cw_owner_it = slot_owner.find(cw_it->first);
            if (cw_owner_it != slot_owner.end() && own_it->second == cw_owner_it->second) {
                tie_state = ContactorState::Closed;
            }
        }
        plan.mc_commands[s.mc_id] = tie_state;
    }

    // Keep one deterministic ring break open when a single owner spans all cabinets.
    // This avoids a fully closed ring while preserving island integrity.
    int common_owner = -1;
    bool single_owner = !slots_.empty();
    for (const auto& s : slots_) {
        const int owner = slot_owner[s.id];
        if (owner <= 0) {
            single_owner = false;
            break;
        }
        if (common_owner < 0) {
            common_owner = owner;
        } else if (owner != common_owner) {
            single_owner = false;
            break;
        }
    }
    if (single_owner) {
        std::set<int> avoid_slots;
        const Slot* owner_home = nullptr;
        const auto owner_it = guns_.find(common_owner);
        if (owner_it != guns_.end()) {
            owner_home = find_slot(owner_it->second.slot_id);
            if (owner_home) {
                avoid_slots.insert(owner_home->id);
                // In split mappings, the home slot's CW neighbor is commonly the same cabinet's
                // secondary/pass-through slot used for relay-mask derivation.
                if (owner_home->cw_id > 0) {
                    avoid_slots.insert(owner_home->cw_id);
                }
            }
        }

        auto choose_ring_break = [&](auto&& predicate) -> const Slot* {
            const Slot* chosen = nullptr;
            for (const auto& s : slots_) {
                if (!predicate(s)) {
                    continue;
                }
                if (!chosen || s.id > chosen->id) {
                    chosen = &s;
                }
            }
            return chosen;
        };

        const Slot* ring_break = choose_ring_break([&](const Slot& s) {
            return !avoid_slots.count(s.id) && s.gun_id == 0;
        });
        if (!ring_break) {
            ring_break = choose_ring_break([&](const Slot& s) {
                return !avoid_slots.count(s.id);
            });
        }
        if (!ring_break && owner_home) {
            ring_break = choose_ring_break([&](const Slot& s) {
                return s.id != owner_home->id;
            });
        }
        if (!ring_break && !slots_.empty()) {
            ring_break = &slots_.back();
        }

        if (ring_break) {
            plan.mc_commands[ring_break->mc_id] = ContactorState::Open;
        }
    }

    return plan;
}

Plan PowerManager::compute_plan() {
    const auto now = std::chrono::steady_clock::now();
    const auto active = active_guns();
    next_island_id_ = 1;
    if (active.empty()) {
        prune_inactive_ramp_state(active);
        return blank_plan();
    }
    prune_inactive_ramp_state(active);

    std::set<int> reserved_slots;
    for (const auto& kv : guns_) {
        const auto& g = kv.second;
        if (g.plugged_in || g.reserved) {
            reserved_slots.insert(g.slot_id);
        }
        if (g.fsm_state == GunFsmState::Charging) {
            reserved_slots.insert(g.slot_id);
        }
    }

    int healthy_modules = 0;
    for (const auto& m : modules_) {
        if (m.second.healthy) {
            healthy_modules++;
        }
    }

    std::map<int, double> req_limited;
    for (auto gid : active) {
        const auto& g = guns_.at(gid);
        double req = std::max(0.0, g.ev_req_power_kw);
        const double gun_cap = g.gun_power_limit_kw > 0.0 ? g.gun_power_limit_kw : req;
        req = std::min(req, gun_cap);
        req_limited[gid] = req;
    }

    std::map<int, double> budgets;
    if (healthy_modules <= 0) {
        // No healthy modules available: still return a deterministic plan for the active guns with
        // 0 modules/power assigned so upstream logic can reflect the constraint.
        for (auto gid : active) {
            budgets[gid] = 0.0;
        }
    } else {
        budgets = compute_power_budgets(active, req_limited, healthy_modules);
    }
    if (budgets.empty()) {
        return blank_plan();
    }

    std::map<int, int> ideal_modules;
    for (auto gid : active) {
        const double p = budgets.at(gid);
        ideal_modules[gid] = ideal_modules_for_gun(guns_.at(gid), p);
    }

    auto modules_per_gun = compute_module_allocation(active, budgets, ideal_modules, healthy_modules);
    std::set<int> full_island_guns;
    if (healthy_modules > 0 && active.size() == 1 && cfg_.allow_cross_slot_islands) {
        const int gid = active.front();
        const auto g_it = guns_.find(gid);
        const int home_slot = g_it != guns_.end() ? g_it->second.slot_id : 0;
        bool reserved_block = false;
        for (int slot_id : reserved_slots) {
            if (slot_id != home_slot) {
                reserved_block = true;
                break;
            }
        }
        if (!reserved_block && modules_per_gun[gid] >= healthy_modules) {
            full_island_guns.insert(gid);
        }
    }
    auto plan = build_plan(active, budgets, modules_per_gun, reserved_slots, full_island_guns, now);
    apply_hysteresis(plan, now);
    if (!validate_plan(plan)) {
        return blank_plan();
    }
    return plan;
}

void PowerManager::apply_hysteresis(Plan& plan, std::chrono::steady_clock::time_point now) {
    const auto hold = std::chrono::milliseconds(cfg_.min_module_hold_ms);
    std::map<std::string, int> module_owner;
    for (const auto& island : plan.islands) {
        if (!island.gun_id) continue;
        const int gid = island.gun_id.value();
        for (const auto& mid : island.module_ids) {
            module_owner[mid] = gid;
        }
    }
    std::set<int> reserved_slots;
    for (const auto& kv : guns_) {
        const auto& g = kv.second;
        if (g.plugged_in || g.reserved || g.fsm_state == GunFsmState::Charging) {
            reserved_slots.insert(g.slot_id);
        }
    }
    for (auto& dispatch : plan.guns) {
        const int gid = dispatch.gun_id;
        const int prev = last_modules_assigned_[gid];
        const auto it_time = last_alloc_change_.find(gid);
        const bool have_time = it_time != last_alloc_change_.end() &&
            it_time->second.time_since_epoch().count() != 0;
        auto island_it = std::find_if(plan.islands.begin(), plan.islands.end(),
                                      [&](const IslandState& isl) {
                                          return isl.gun_id && isl.gun_id.value() == gid;
                                      });

        const bool within_hold = have_time && (now - it_time->second) < hold;
        const bool request_differs = dispatch.modules_assigned != prev;
        const auto git = guns_.find(gid);
        const GunState* g_state = git != guns_.end() ? &git->second : nullptr;

        const bool can_hold = request_differs && within_hold && dispatch.modules_assigned > 0 && prev > 0 &&
            dispatch.modules_assigned < prev && dispatch.p_budget_kw > 0.0 && g_state &&
            g_state->safety_ok && !g_state->gc_welded && !g_state->mc_welded;

        if (can_hold && island_it != plan.islands.end() && !last_module_ids_[gid].empty()) {
            const auto& desired_modules = last_module_ids_[gid];
            std::set<int> island_slots(island_it->slot_ids.begin(), island_it->slot_ids.end());
            bool previous_available = true;
            for (const auto& mod_id : desired_modules) {
                const auto mit = modules_.find(mod_id);
                if (mit == modules_.end() || !mit->second.healthy) {
                    previous_available = false;
                    break;
                }
                if (!island_slots.count(mit->second.slot_id)) {
                    previous_available = false;
                    break;
                }
                const auto owner_it = module_owner.find(mod_id);
                if (owner_it != module_owner.end() && owner_it->second != gid) {
                    previous_available = false;
                    break;
                }
                if (reserved_slots.count(mit->second.slot_id) && g_state->slot_id != mit->second.slot_id) {
                    previous_available = false;
                    break;
                }
            }

            if (previous_available) {
                dispatch.modules_assigned = prev;
                island_it->module_ids = desired_modules;

                std::set<std::string> desired_set(desired_modules.begin(), desired_modules.end());
                for (int slot_id : island_it->slot_ids) {
                    const auto* slot = find_slot(slot_id);
                    if (!slot) continue;
                    for (const auto& mod_id : slot->modules) {
                        const auto mit = modules_.find(mod_id);
                        if (mit == modules_.end()) continue;
                        plan.mn_commands[mit->second.mn_id] =
                            desired_set.count(mod_id) ? ContactorState::Closed : ContactorState::Open;
                    }
                }

                const double v_target = island_it->v_set_v;
                const double p_cap = prev * cfg_.module_power_kw;
                const double p_set = std::min({dispatch.p_budget_kw, p_cap,
                                               g_state->gun_power_limit_kw > 0.0
                                                   ? g_state->gun_power_limit_kw
                                                   : p_cap});
                const double v_safe = std::max(cfg_.min_voltage_v_for_div, v_target);
                double i_target = v_safe > 0.0 ? (p_set * 1000.0) / v_safe : 0.0;
                if (g_state->gun_current_limit_a > 0.0) {
                    i_target = std::min(i_target, g_state->gun_current_limit_a);
                }
                i_target = std::max(0.0, i_target - std::max(0.0, cfg_.current_margin_a));
                const double v_ceiling_for_guard =
                    g_state->ev_req_voltage_v > 0.0
                        ? g_state->ev_req_voltage_v
                        : (v_target + compute_voltage_margin_v(*g_state, v_target));
                i_target = apply_voltage_guard(*g_state, i_target, v_ceiling_for_guard);
                const bool emergency_drop = !g_state->safety_ok || g_state->gc_welded || g_state->mc_welded;
                i_target = apply_current_ramp(gid, *g_state, i_target, emergency_drop, now);
                dispatch.p_set_kw = p_set;
                dispatch.current_limit_a = i_target;
                dispatch.voltage_set_v = v_target;
                dispatch.p_set_kw = std::min(dispatch.p_set_kw, (dispatch.current_limit_a * v_safe) / 1000.0);
                island_it->p_set_kw = dispatch.p_set_kw;

                if (const auto* slot = find_slot(g_state->slot_id)) {
                    plan.gc_commands[slot->gc_id] = prev > 0 ? ContactorState::Closed : ContactorState::Open;
                }
            }
        }

        if (island_it != plan.islands.end()) {
            last_module_ids_[gid] = island_it->module_ids;
        } else {
            last_module_ids_[gid].clear();
        }
        if (dispatch.modules_assigned != prev || !have_time) {
            last_alloc_change_[gid] = now;
            last_modules_assigned_[gid] = dispatch.modules_assigned;
        }
    }
}

bool PowerManager::validate_plan(const Plan& plan) const {
    std::set<std::string> used_modules;
    std::set<int> used_slots;
    std::map<int, int> gun_modules;
    for (const auto& d : plan.guns) {
        gun_modules[d.gun_id] = d.modules_assigned;
    }

    for (const auto& island : plan.islands) {
        if (island.gun_id) {
            const int gid = island.gun_id.value();
            if (gun_modules.count(gid) && gun_modules[gid] != static_cast<int>(island.module_ids.size())) {
                return false;
            }
        }
        for (int sid : island.slot_ids) {
            if (!used_slots.insert(sid).second) {
                return false;
            }
        }
        for (const auto& mid : island.module_ids) {
            if (!used_modules.insert(mid).second) {
                return false;
            }
            auto mit = modules_.find(mid);
            if (mit == modules_.end() || !mit->second.healthy) {
                return false;
            }
        }
    }
    return true;
}

} // namespace charger
