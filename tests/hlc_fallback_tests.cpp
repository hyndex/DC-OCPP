// SPDX-License-Identifier: Apache-2.0
#include "ocpp_adapter.hpp"
#include "test_hardware.hpp"
#include "test_config_helpers.hpp"

#include <cassert>
#include <iostream>

using namespace charger;

static ChargerConfig make_cfg() {
    ChargerConfig cfg{};
    cfg.charge_point_id = "hlc-fallback-test";
    cfg.connectors = {ConnectorConfig{.id = 1}};
    cfg.autocharge_id_source = "emaid";
    cfg.hlc_auth_timeout_s = 1;
    cfg.pnc_block_ttl_s = 1;
    populate_minimal_slots(cfg);
    return cfg;
}

int main() {
    auto cfg = make_cfg();
    auto hw = std::make_shared<TestHardware>(cfg);
    OcppAdapter adapter(cfg, hw);
    const auto now = std::chrono::steady_clock::now();

    GunStatus st{};
    st.plugged_in = true;
    st.hlc_stage = 5; // HLC_WAIT_AUTHORIZATION

    // Pre-HLC: autocharge disabled => digital comm stays off, PnC blocked.
    OcppAdapter::TestHook::set_autocharge_enabled(adapter, false);
    OcppAdapter::TestHook::ActiveSession sess{};
    auto out = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                        std::nullopt, false, now);
    assert(out.desired_digital == false);
    assert(out.desired_pnc_blocked == true);

    // Autocharge auth timeout triggers PnC block + EIM fallback.
    OcppAdapter::TestHook::set_autocharge_enabled(adapter, true);
    out = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                   std::nullopt, false, now);
    auto out2 = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                         std::nullopt, false,
                                                         now + std::chrono::seconds(cfg.hlc_auth_timeout_s + 1));
    assert(out2.auth_timeout_triggered == true);
    assert(out2.force_auth_denied == true);
    assert(out2.desired_pnc_blocked == true);
    assert(out2.desired_digital == false);

    // PnC block TTL expiry restores Contract offer.
    auto out3 = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                         std::optional<std::string>("AUTO1"), false, now);
    assert(out3.desired_pnc_blocked == true);
    auto out4 = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                         std::nullopt, false,
                                                         now + std::chrono::seconds(cfg.pnc_block_ttl_s + 1));
    assert(out4.desired_pnc_blocked == false);

    std::cout << "HLC fallback tests passed\n";
    return 0;
}
