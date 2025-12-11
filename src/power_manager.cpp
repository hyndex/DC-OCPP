// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <cmath>

namespace charger {

PowerManager::PowerManager(PlannerConfig cfg) : cfg_(std::move(cfg)) {}

void PowerManager::set_slots(std::vector<Slot> slots) {
    slots_ = std::move(slots);
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
    Plan plan;

    for (const auto& slot : slots_) {
        plan.mc_commands[slot.mc_id] = ContactorState::Closed;
        plan.gc_commands[slot.gc_id] = ContactorState::Open;
    }
    for (const auto& m : modules_) {
        plan.mn_commands[m.second.mn_id] = ContactorState::Open;
    }

    for (auto gid : active) {
        const auto& g = guns_.at(gid);
        const int slot_id = g.slot_id;
        const auto it_slot = std::find_if(slots_.begin(), slots_.end(), [&](const Slot& s) { return s.id == slot_id; });
        if (it_slot == slots_.end()) continue;
        const auto& slot = *it_slot;
        const int n_needed = modules_per_gun.at(gid);

        auto prev_slot = std::find_if(slots_.begin(), slots_.end(),
                                      [&](const Slot& s) { return s.id == slot.ccw_id; });
        if (prev_slot != slots_.end()) {
            plan.mc_commands[prev_slot->mc_id] = ContactorState::Open;
        }
        plan.mc_commands[slot.mc_id] = ContactorState::Open;

        int enabled_count = 0;
        for (const auto& mod_id : slot.modules) {
            auto m_it = modules_.find(mod_id);
            if (m_it == modules_.end()) continue;
            auto& m = m_it->second;
            if (enabled_count < n_needed && m.healthy) {
                plan.mn_commands[m.mn_id] = ContactorState::Closed;
                enabled_count++;
            } else {
                plan.mn_commands[m.mn_id] = ContactorState::Open;
            }
        }

        if (n_needed > 0) {
            plan.gc_commands[slot.gc_id] = ContactorState::Closed;
        }

        IslandState island;
        island.id = next_island_id_++;
        island.slot_ids = {slot.id};
        island.gun_id = gid;
        for (const auto& mod_id : slot.modules) {
            auto it_m = modules_.find(mod_id);
            if (it_m != modules_.end() && plan.mn_commands[it_m->second.mn_id] == ContactorState::Closed) {
                island.module_ids.push_back(mod_id);
            }
        }

        const double p_cap_modules = island.module_ids.size() * cfg_.module_power_kw;
        const double p_budget = budgets.at(gid);
        const double p_set = std::min({p_budget, p_cap_modules, g.gun_power_limit_kw});
        island.p_set_kw = p_set;
        island.v_set_v = g.ev_req_voltage_v;

        GunDispatch dispatch;
        dispatch.gun_id = gid;
        dispatch.p_budget_kw = p_budget;
        dispatch.modules_assigned = static_cast<int>(island.module_ids.size());
        dispatch.voltage_set_v = island.v_set_v;
        const double v_safe = std::max(1.0, island.v_set_v);
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

    int healthy_modules = 0;
    for (const auto& m : modules_) {
        if (m.second.healthy) {
            healthy_modules++;
        }
    }

    std::map<int, double> req_limited;
    for (auto gid : active) {
        const auto& g = guns_.at(gid);
        const double req = std::max(0.0, std::min(g.ev_req_power_kw, g.gun_power_limit_kw));
        req_limited[gid] = req;
    }

    const auto budgets = compute_power_budgets(active, req_limited, healthy_modules);
    if (budgets.empty()) {
        Plan empty;
        for (const auto& slot : slots_) {
            empty.mc_commands[slot.mc_id] = ContactorState::Closed;
            empty.gc_commands[slot.gc_id] = ContactorState::Open;
        }
        for (const auto& m : modules_) {
            empty.mn_commands[m.second.mn_id] = ContactorState::Open;
        }
        return empty;
    }

    std::map<int, int> ideal_modules;
    for (auto gid : active) {
        const double p = budgets.at(gid);
        int n = 0;
        if (p > 0.0) {
            if (p <= 0.8 * cfg_.module_power_kw) {
                n = 1;
            } else if (p <= 1.6 * cfg_.module_power_kw) {
                n = 2;
            } else {
                n = 2;
            }
            const int cable_cap = g.gun_power_limit_kw > 0.0
                                      ? static_cast<int>(std::floor(g.gun_power_limit_kw / cfg_.module_power_kw))
                                      : 2;
            n = std::max(0, std::min(n, 2));
            n = std::min(n, std::max(0, cable_cap));
        }
        ideal_modules[gid] = n;
    }

    const auto modules_per_gun = compute_module_allocation(active, budgets, ideal_modules, healthy_modules);
    return build_plan(active, budgets, modules_per_gun);
}

} // namespace charger
