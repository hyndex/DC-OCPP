// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_config_helpers.hpp"
#include "test_hardware.hpp"

#include <chrono>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "cablecheck-no-hv-test";
    cfg.connectors = {ConnectorConfig{.id = 1, .require_lock = false}};
    cfg.max_modules_per_gun = 1;
    populate_minimal_slots(cfg);
    for (auto& slot : cfg.slots) {
        if (slot.modules.size() > 1) {
            slot.modules.resize(1);
        }
    }
    return cfg;
}

static void seed_session(OcppAdapter& adapter, int connector, bool authorized) {
    OcppAdapter::TestHook::ActiveSession session{};
    session.session_id = "sess";
    session.id_token = "TAG";
    session.authorized = authorized;
    session.ev_connected = true;
    session.transaction_started = authorized;
    session.connected_at = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(OcppAdapter::TestHook::session_mutex(adapter));
        OcppAdapter::TestHook::sessions(adapter)[connector] = session;
    }
}

static GunStatus make_status(uint8_t hlc_stage) {
    GunStatus st{};
    st.safety_ok = true;
    st.plugged_in = true;
    st.cp_state = 'B';
    st.hlc_stage = hlc_stage;
    st.hlc_precharge_active = false;
    st.hlc_power_ready = false;
    st.hlc_charge_complete = false;
    st.lock_engaged = true;
    st.authorization_granted = false;
    st.present_voltage_v = 0.0;
    st.present_current_a = 0.0;
    st.last_telemetry = std::chrono::steady_clock::now();
    return st;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);

    // Stage 7 is CableCheck in the PLC enum; regardless of OCPP auth, we must not energize modules / raise HV.
    constexpr uint8_t HLC_STAGE_CABLE_CHECK = 7;

    seed_session(adapter, 1, false);
    hw->set_status_override(1, make_status(HLC_STAGE_CABLE_CHECK));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto mreq = OcppAdapter::TestHook::last_module_command_for_slot(adapter, 1);
    if (!mreq.has_value()) {
        std::cerr << "CableCheck no-HV test failed: missing module command\n";
        return 1;
    }
    if (mreq->enable || mreq->mask != 0u || mreq->voltage_v != 0.0 || mreq->current_a != 0.0 || mreq->power_kw != 0.0) {
        std::cerr << "CableCheck no-HV test failed: expected modules OFF at CableCheck"
                  << " enable=" << (mreq->enable ? "1" : "0")
                  << " mask=0x" << std::hex << static_cast<int>(mreq->mask) << std::dec
                  << " V=" << mreq->voltage_v
                  << " I=" << mreq->current_a
                  << " P=" << mreq->power_kw << "\n";
        return 1;
    }

    seed_session(adapter, 1, true);
    hw->set_status_override(1, make_status(HLC_STAGE_CABLE_CHECK));
    OcppAdapter::TestHook::apply_power_plan(adapter);
    const auto mreq2 = OcppAdapter::TestHook::last_module_command_for_slot(adapter, 1);
    if (!mreq2.has_value()) {
        std::cerr << "CableCheck no-HV test failed (auth=1): missing module command\n";
        return 1;
    }
    if (mreq2->enable || mreq2->mask != 0u || mreq2->voltage_v != 0.0 || mreq2->current_a != 0.0 || mreq2->power_kw != 0.0) {
        std::cerr << "CableCheck no-HV test failed (auth=1): expected modules OFF at CableCheck\n";
        return 1;
    }

    std::cout << "cable_check_no_hv_tests passed\n";
    return 0;
}

