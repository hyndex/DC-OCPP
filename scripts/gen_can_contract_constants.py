#!/usr/bin/env python3
"""
Generate controller/PLC CAN contract constant headers from a single JSON source.

Inputs: docs/can_contract_constants.json
Outputs:
  - include/can_contract.hpp (C++ constexpr)
  - Ref/Basic/include/can_contract.h (C macros)
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_JSON = ROOT / "docs" / "can_contract_constants.json"
CXX_OUT = ROOT / "include" / "can_contract.hpp"
C_OUT = ROOT / "Ref" / "Basic" / "include" / "can_contract.h"


def load_constants():
    with SRC_JSON.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_cxx(constants):
    cfg = constants["config_params"]
    lines = [
        "// SPDX-License-Identifier: Apache-2.0",
        "#pragma once",
        "namespace can_contract {",
        f"inline constexpr uint8_t kProtoVersion = {constants['protocol_version']};",
        f"inline constexpr uint8_t kConfigParamAuthState = {cfg['auth_state']};",
        f"inline constexpr uint8_t kConfigParamAuthPending = {cfg['auth_pending']};",
        f"inline constexpr uint8_t kConfigParamLockCmd = {cfg['lock_cmd']};",
        f"inline constexpr uint8_t kConfigParamEvseLimitAck = {cfg['evse_limit_ack']};",
        f"inline constexpr uint8_t kConfigParamProtoVersion = {cfg['proto_version']};",
        "} // namespace can_contract",
        "",
    ]
    CXX_OUT.write_text("\n".join(lines), encoding="utf-8")


def write_c(constants):
    cfg = constants["config_params"]
    lines = [
        "// SPDX-License-Identifier: Apache-2.0",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define CAN_CONTRACT_PROTO_VERSION {constants['protocol_version']}",
        f"#define CAN_CONTRACT_PARAM_AUTH_STATE {cfg['auth_state']}",
        f"#define CAN_CONTRACT_PARAM_AUTH_PENDING {cfg['auth_pending']}",
        f"#define CAN_CONTRACT_PARAM_LOCK_CMD {cfg['lock_cmd']}",
        f"#define CAN_CONTRACT_PARAM_EVSE_LIMIT_ACK {cfg['evse_limit_ack']}",
        f"#define CAN_CONTRACT_PARAM_PROTO_VERSION {cfg['proto_version']}",
        "",
    ]
    C_OUT.write_text("\n".join(lines), encoding="utf-8")


def main():
    constants = load_constants()
    write_cxx(constants)
    write_c(constants)
    print("Generated:", CXX_OUT, "and", C_OUT)


if __name__ == "__main__":
    main()
