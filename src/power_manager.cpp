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

    auto maybe_add_passthrough = [&](int candidate_id, bool prepend) {
        if (candidate_id == 0 || claimed_slots.count(candidate_id) || active_home_slots.count(candidate_id)) {
            return false;
        }
        const auto* cand = find_slot(candidate_id);
        if (!cand) return false;
        if (!cand->modules.empty() || cand->gun_id > 0) return false;
        if (reserved_slots.count(candidate_id) && candidate_id != g.slot_id) return false;
        if (prepend) {
            slots.insert(slots.begin(), candidate_id);
        } else {
            slots.push_back(candidate_id);
        }
        claimed_slots.insert(candidate_id);
        return true;
    };

    if (!cfg_.allow_cross_slot_islands || n_needed <= count_healthy_modules_in_slot(g.slot_id)) {
        // Keep single-module islands from opening the module's own MC contactor by
        // including an adjacent pass-through slot (no modules, no gun) when present.
        const auto* home = find_slot(g.slot_id);
        if (home) {
            if (!maybe_add_passthrough(home->cw_id, false)) {
                (void)maybe_add_passthrough(home->ccw_id, true);
            }
        }
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

        if (remaining <= 0) break;

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

Plan PowerManager::build_plan(const std::vector<int>& active, const std::map<int, double>& budgets,
                              const std::map<int, int>& modules_per_gun,
                              const std::set<int>& reserved_slots,
                              const std::set<int>& full_island_guns) {
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
        const bool full_island = slots_for_g.size() == slots_.size() && !slots_.empty();
        if (!g.mc_welded && !slots_for_g.empty() && n_needed > 0 && !full_island) {
            const auto* ccw_boundary = find_slot(slots_for_g.front());
            if (ccw_boundary) {
                const auto* prev_slot = find_slot(ccw_boundary->ccw_id);
                if (prev_slot) {
                    plan.mc_commands[prev_slot->mc_id] = ContactorState::Open;
                }
            }
            const auto* cw_boundary = find_slot(slots_for_g.back());
            if (cw_boundary) {
                plan.mc_commands[cw_boundary->mc_id] = ContactorState::Open;
            }
        }

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
        double target_v = g.ev_req_voltage_v > 0.0 ? g.ev_req_voltage_v : cfg_.default_voltage_v;
        if (target_v < min_v) target_v = min_v;
        if (target_v > max_v) target_v = max_v;
        island.p_set_kw = p_set;
        island.v_set_v = std::max(target_v, cfg_.min_voltage_v_for_div);

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
        const double prev = g.i_set_a;
        if (cfg_.ramp_step_a > 0.0) {
            if (i_target > prev + cfg_.ramp_step_a) i_target = prev + cfg_.ramp_step_a;
            if (i_target < prev - cfg_.ramp_step_a) i_target = prev - cfg_.ramp_step_a;
        }
        if (i_target < 0.0) i_target = 0.0;
        dispatch.current_limit_a = i_target;

        plan.islands.push_back(std::move(island));
        plan.guns.push_back(dispatch);
    }

    return plan;
}

Plan PowerManager::compute_plan() {
    const auto now = std::chrono::steady_clock::now();
    const auto active = active_guns();
    next_island_id_ = 1;
    if (active.empty()) {
        return blank_plan();
    }

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
    if (healthy_modules <= 0) {
        return blank_plan();
    }

    std::map<int, double> req_limited;
    for (auto gid : active) {
        const auto& g = guns_.at(gid);
        double req = std::max(0.0, g.ev_req_power_kw);
        const double gun_cap = g.gun_power_limit_kw > 0.0 ? g.gun_power_limit_kw : req;
        req = std::min(req, gun_cap);
        req_limited[gid] = req;
    }

    const auto budgets = compute_power_budgets(active, req_limited, healthy_modules);
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
    if (active.size() == 1 && cfg_.allow_cross_slot_islands) {
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
    auto plan = build_plan(active, budgets, modules_per_gun, reserved_slots, full_island_guns);
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
                if (i_target < 0.0) i_target = 0.0;
                dispatch.p_set_kw = p_set;
                dispatch.current_limit_a = i_target;
                dispatch.voltage_set_v = v_target;
                island_it->p_set_kw = p_set;

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
