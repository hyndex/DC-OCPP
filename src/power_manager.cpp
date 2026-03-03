// SPDX-License-Identifier: Apache-2.0
#include "power_manager.hpp"

#include <algorithm>
#include <cmath>
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
        if (!g.ev_session_active) {
            continue;
        }
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
        // Keep ties open in simple-router mode.
        plan.mc_commands[slot.mc_id] = ContactorState::Open;
        if (!slot.gc_id.empty()) {
            plan.gc_commands[slot.gc_id] = ContactorState::Open;
        }
    }
    for (const auto& kv : modules_) {
        plan.mn_commands[kv.second.mn_id] = ContactorState::Open;
    }
    return plan;
}

const Slot* PowerManager::find_slot(int slot_id) const {
    auto it = std::find_if(slots_.begin(), slots_.end(), [&](const Slot& s) { return s.id == slot_id; });
    return it == slots_.end() ? nullptr : &(*it);
}

Plan PowerManager::compute_plan() {
    const auto active = active_guns();
    Plan plan = blank_plan();
    next_island_id_ = 1;

    if (active.empty()) {
        return plan;
    }

    std::set<std::string> used_modules;
    for (int gun_id : active) {
        const auto git = guns_.find(gun_id);
        if (git == guns_.end()) {
            continue;
        }
        const auto& g = git->second;
        const auto* home_slot = find_slot(g.slot_id);
        if (!home_slot) {
            continue;
        }

        const bool safety_ok = g.safety_ok && !g.gc_welded && !g.mc_welded;
        const bool route_enabled = safety_ok && !g.delivery_lost;

        double target_v = g.ev_req_voltage_v > 0.0 ? g.ev_req_voltage_v : cfg_.default_voltage_v;
        if (!std::isfinite(target_v) || target_v <= 0.0) {
            target_v = cfg_.default_voltage_v;
        }
        if (g.min_voltage_v > 0.0) {
            target_v = std::max(target_v, g.min_voltage_v);
        }
        if (g.max_voltage_v > 0.0) {
            target_v = std::min(target_v, g.max_voltage_v);
        }
        target_v = std::max(target_v, cfg_.min_voltage_v_for_div);

        double target_p_kw = route_enabled ? std::max(0.0, g.ev_req_power_kw) : 0.0;
        if (!std::isfinite(target_p_kw) || target_p_kw < 0.0) {
            target_p_kw = 0.0;
        }
        if (g.gun_power_limit_kw > 0.0) {
            target_p_kw = std::min(target_p_kw, g.gun_power_limit_kw);
        }

        double target_i_a = target_p_kw > 0.0 ? (target_p_kw * 1000.0) / target_v : 0.0;
        if (!std::isfinite(target_i_a) || target_i_a < 0.0) {
            target_i_a = 0.0;
        }
        if (g.gun_current_limit_a > 0.0) {
            target_i_a = std::min(target_i_a, g.gun_current_limit_a);
        }

        int desired_modules = 0;
        if (target_i_a > 0.0 && target_v > 0.0) {
            const double module_kw = cfg_.module_power_kw > 0.0 ? cfg_.module_power_kw : target_p_kw;
            if (module_kw > 0.0) {
                desired_modules = static_cast<int>(std::ceil(target_p_kw / module_kw));
            }
            desired_modules = std::max(desired_modules, std::max(1, cfg_.min_modules_per_active_gun));
            desired_modules = std::min(desired_modules, std::max(1, cfg_.max_modules_per_gun));
        }

        std::vector<std::string> selected;
        double selected_cap_kw = 0.0;
        double selected_cap_current_a = 0.0;
        if (desired_modules > 0) {
            for (const auto& module_id : home_slot->modules) {
                if (static_cast<int>(selected.size()) >= desired_modules) {
                    break;
                }
                const auto mit = modules_.find(module_id);
                if (mit == modules_.end()) {
                    continue;
                }

                const auto& mod = mit->second;
                const bool capability_ready =
                    !mod.capability_fresh || mod.available_power_kw > 0.0 || mod.available_current_a > 0.0;
                const bool usable = mod.healthy && !mod.severe_fault && capability_ready;
                if (!usable || used_modules.count(module_id)) {
                    continue;
                }

                selected.push_back(module_id);
                used_modules.insert(module_id);
                selected_cap_kw += (mod.available_power_kw > 0.0) ? mod.available_power_kw : cfg_.module_power_kw;
                selected_cap_current_a += std::max(0.0, mod.available_current_a);
                plan.mn_commands[mod.mn_id] = ContactorState::Closed;
            }
        }

        const int assigned = static_cast<int>(selected.size());
        if (!home_slot->gc_id.empty()) {
            plan.gc_commands[home_slot->gc_id] =
                (route_enabled && assigned > 0) ? ContactorState::Closed : ContactorState::Open;
        }

        double p_set_kw = 0.0;
        double i_set_a = 0.0;
        if (route_enabled && assigned > 0) {
            p_set_kw = std::min(target_p_kw, std::max(0.0, selected_cap_kw));
            i_set_a = target_v > 0.0 ? (p_set_kw * 1000.0) / target_v : 0.0;
            if (g.gun_current_limit_a > 0.0) {
                i_set_a = std::min(i_set_a, g.gun_current_limit_a);
            }
            if (selected_cap_current_a > 0.0) {
                i_set_a = std::min(i_set_a, selected_cap_current_a);
            }
            p_set_kw = (i_set_a * target_v) / 1000.0;
        }

        GunDispatch dispatch;
        dispatch.gun_id = gun_id;
        dispatch.p_budget_kw = target_p_kw;
        dispatch.p_set_kw = p_set_kw;
        dispatch.modules_assigned = assigned;
        dispatch.current_limit_a = i_set_a;
        dispatch.voltage_set_v = target_v;
        plan.guns.push_back(dispatch);

        IslandState island;
        island.id = next_island_id_++;
        island.slot_ids = {home_slot->id};
        island.module_ids = selected;
        island.gun_id = gun_id;
        island.v_set_v = target_v;
        island.p_set_kw = p_set_kw;
        plan.islands.push_back(std::move(island));
    }

    return plan;
}

} // namespace charger
