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

    // Autocharge disabled: keep HLC digital comm enabled while EV is present, but block PnC offers.
    OcppAdapter::TestHook::set_autocharge_enabled(adapter, false);
    OcppAdapter::TestHook::ActiveSession sess{};
    auto out = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                        std::nullopt, false, now);
    assert(out.desired_digital == true);
    assert(out.desired_pnc_blocked == true);

    // Once digital comm is enabled, a short telemetry/CP unknown blip must not flap it OFF.
    OcppAdapter::TestHook::set_autocharge_enabled(adapter, true);
    GunStatus present{};
    present.plugged_in = true;
    present.cp_state = 'B';
    const auto t_present = now + std::chrono::seconds(2);
    auto digital_on = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, present, false, sess, false,
                                                               std::nullopt, false, t_present);
    assert(digital_on.desired_digital == true);

    GunStatus blip{};
    blip.plugged_in = false;
    blip.cp_state = 'U';
    auto digital_blip = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, blip, false, sess, false,
                                                                 std::nullopt, false,
                                                                 t_present + std::chrono::milliseconds(200));
    assert(digital_blip.desired_digital == true);

    // Disable only after explicit unplug (CP=A + unplugged) is sustained past debounce.
    GunStatus explicit_absent{};
    explicit_absent.plugged_in = false;
    explicit_absent.cp_state = 'A';
    auto absent_short = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, explicit_absent, false, sess, false,
                                                                 std::nullopt, false,
                                                                 t_present + std::chrono::milliseconds(500));
    assert(absent_short.desired_digital == true);
    auto absent_long = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, explicit_absent, false, sess, false,
                                                                std::nullopt, false,
                                                                t_present + std::chrono::seconds(4));
    assert(absent_long.desired_digital == false);

    // Autocharge auth timeout while CSMS is offline should not force fallback.
    OcppAdapter::TestHook::set_autocharge_enabled(adapter, true);
    OcppAdapter::TestHook::set_csms_connected(adapter, false);
    out = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                   std::nullopt, false, now);
    auto out2 = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                         std::nullopt, false,
                                                         now + std::chrono::seconds(cfg.hlc_auth_timeout_s + 1));
    assert(out2.auth_timeout_triggered == true);
    assert(out2.force_auth_denied == false);
    assert(out2.desired_pnc_blocked == false);
    assert(out2.desired_digital == true);

    // Autocharge auth timeout while CSMS is online should trigger PnC block + EIM fallback.
    GunStatus unplugged{};
    unplugged.plugged_in = false;
    unplugged.hlc_stage = 0;
    const auto t0 = now + std::chrono::seconds(10);
    OcppAdapter::TestHook::apply_hlc_control(adapter, 1, unplugged, false, sess, false,
                                             std::nullopt, false, t0);
    OcppAdapter::TestHook::set_csms_connected(adapter, true);
    OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                             std::nullopt, false, t0 + std::chrono::seconds(1));
    auto out3 = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                         std::nullopt, false,
                                                         t0 + std::chrono::seconds(cfg.hlc_auth_timeout_s + 2));
    assert(out3.auth_timeout_triggered == true);
    assert(out3.force_auth_denied == true);
    assert(out3.desired_pnc_blocked == true);
    assert(out3.desired_digital == true);

    // PnC block TTL expiry restores Contract offer.
    auto out4 = OcppAdapter::TestHook::apply_hlc_control(adapter, 1, st, false, sess, false,
                                                         std::nullopt, false,
                                                         t0 + std::chrono::seconds(cfg.hlc_auth_timeout_s + cfg.pnc_block_ttl_s + 3));
    assert(out4.desired_pnc_blocked == false);

    std::cout << "HLC fallback tests passed\n";
    return 0;
}
