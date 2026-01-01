// SPDX-License-Identifier: Apache-2.0
#pragma once
namespace can_contract {
inline constexpr uint8_t kProtoVersion = 1;
inline constexpr uint8_t kConfigParamAuthState = 20;
inline constexpr uint8_t kConfigParamAuthPending = 21;
inline constexpr uint8_t kConfigParamLockCmd = 30;
inline constexpr uint8_t kConfigParamEvseLimitAck = 90;
inline constexpr uint8_t kConfigParamProtoVersion = 91;
// HLC stage constants (must stay in sync with PLC firmware tcp.cpp)
inline constexpr uint8_t kHlcStageWaitPowerDelivery = 9;
} // namespace can_contract
