// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
        plan.gc_commands[slot.gc_id] = ContactorState::Open;
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
    int n = 0;
    if (p_budget <= cfg_.ideal_low_factor * cfg_.module_power_kw) {
        n = 1;
    } else if (p_budget <= cfg_.ideal_high_factor * cfg_.module_power_kw) {
        n = 2;
    } else {
        n = 2;
    }
    int max_by_cable = 2;
    if (g.gun_power_limit_kw > 0.0) {
        max_by_cable = static_cast<int>(std::floor(g.gun_power_limit_kw / cfg_.module_power_kw));
    }
    if (max_by_cable < 0) max_by_cable = 0;
    if (max_by_cable > 2) max_by_cable = 2;
    n = std::min(n, max_by_cable);
    if (n < 0) n = 0;
    if (n > 2) n = 2;
    return n;
}

std::vector<std::string> PowerManager::select_modules_for_slot(const Slot& slot, int n_needed,
                                                               Plan& plan) const {
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

    // Prefer first module, then second.
    if (!slot.modules.empty()) maybe_enable(slot.modules[0]);
    if (slot.modules.size() > 1) maybe_enable(slot.modules[1]);

    // If still need modules (e.g., first unhealthy), attempt remaining modules if present.
    if (static_cast<int>(selected.size()) < n_needed) {
        for (std::size_t idx = 2; idx < slot.modules.size(); ++idx) {
            maybe_enable(slot.modules[idx]);
        }
    }
    return selected;
}

std::vector<int> PowerManager::build_island_slots_for_gun(const GunState& g, int n_needed,
                                                          const std::set<int>& active_home_slots,
                                                          std::set<int>& claimed_slots) const {
    std::vector<int> slots;
    if (!find_slot(g.slot_id)) return slots;
    slots.push_back(g.slot_id);
    claimed_slots.insert(g.slot_id);
    int cw_edge = g.slot_id;
    int ccw_edge = g.slot_id;
    int available = count_healthy_modules_in_slot(g.slot_id);
    int remaining = std::max(0, n_needed - available);
    bool try_cw = true;
    int attempts = 0;
    const int max_attempts = static_cast<int>(slots_.size()) * 2;

    while (remaining > 0 && attempts < max_attempts) {
        attempts++;
        const int candidate = try_cw ? slot_lookup_.at(cw_edge).cw_id : slot_lookup_.at(ccw_edge).ccw_id;
        try_cw = !try_cw;
        if (claimed_slots.count(candidate) || active_home_slots.count(candidate)) {
            continue;
        }
        if (!slot_lookup_.count(candidate)) {
            continue;
        }
        const int healthy = count_healthy_modules_in_slot(candidate);
        if (healthy <= 0) {
            continue;
        }
        if (slot_lookup_.at(cw_edge).cw_id == candidate) {
            slots.push_back(candidate);
            cw_edge = candidate;
        } else {
            slots.insert(slots.begin(), candidate);
            ccw_edge = candidate;
        }
        claimed_slots.insert(candidate);
        available += healthy;
        remaining = std::max(0, n_needed - available);
    }
    return slots;
}

std::vector<std::string> PowerManager::assign_modules_for_island(const std::vector<int>& slot_ids, int n_needed,
                                                                 Plan& plan) const {
    std::vector<std::string> selected;
    int remaining = n_needed;
    for (int slot_id : slot_ids) {
        const auto* slot = find_slot(slot_id);
        if (!slot || remaining <= 0) {
            continue;
        }
        auto slot_sel = select_modules_for_slot(*slot, remaining, plan);
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
            total_weight += 1.0 + static_cast<double>(g.priority);
        }

        bool any_capped = false;
        std::vector<int> remove_list;

        for (auto gid : remaining) {
            const double weight = 1.0 + static_cast<double>(guns_.at(gid).priority);
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

    auto downgrade = [&](int from, int to) {
        bool changed = false;
        while (total > healthy_modules) {
            int chosen = -1;
            double best_budget = std::numeric_limits<double>::max();
            int best_priority = std::numeric_limits<int>::max();
            for (auto gid : active) {
                if (n_modules[gid] != from) continue;
                const auto& g = guns_.at(gid);
                const double budget = budgets.at(gid);
                if (g.priority < best_priority ||
                    (g.priority == best_priority && budget < best_budget)) {
                    chosen = gid;
                    best_priority = g.priority;
                    best_budget = budget;
                }
            }
            if (chosen == -1) break;
            n_modules[chosen] = to;
            total -= (from - to);
            changed = true;
        }
        return changed;
    };

    downgrade(2, 1);
    downgrade(1, 0);
    return n_modules;
}

Plan PowerManager::build_plan(const std::vector<int>& active, const std::map<int, double>& budgets,
                              const std::map<int, int>& modules_per_gun) {
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
        if (n_needed <= 0) {
            island_slots[gid] = {g.slot_id};
            continue;
        }
        island_slots[gid] = build_island_slots_for_gun(g, n_needed, active_home_slots, claimed_slots);
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
        if (!g.mc_welded && !slots_for_g.empty()) {
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

        auto selected = assign_modules_for_island(slots_for_g, n_needed, plan);
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
        const double p_set = std::min({p_budget, p_cap_modules, g.gun_power_limit_kw});
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
    const auto active = active_guns();
    next_island_id_ = 1;
    if (active.empty()) {
        return blank_plan();
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

    const auto modules_per_gun = compute_module_allocation(active, budgets, ideal_modules, healthy_modules);
    return build_plan(active, budgets, modules_per_gun);
}

} // namespace charger
