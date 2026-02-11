#!/usr/bin/env python3
"""
Generate PLC CAN contract constants from docs/can_contract_constants.json.

Output:
  - Ref/Basic/include/can_contract.h

This generator intentionally does not write controller headers.
Controller-side CAN contract code lives in include/can_contract.hpp.
"""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_JSON = ROOT / "docs" / "can_contract_constants.json"
C_OUT = ROOT / "Ref" / "Basic" / "include" / "can_contract.h"


def load_constants():
    with SRC_JSON.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_c(constants):
    cfg = constants["config_params"]
    lines = [
        "// SPDX-License-Identifier: Apache-2.0",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define CAN_CONTRACT_PARAM_AUTH_STATE {cfg['auth_state']}",
        f"#define CAN_CONTRACT_PARAM_AUTH_PENDING {cfg['auth_pending']}",
        f"#define CAN_CONTRACT_PARAM_HLC_ENABLE {cfg['hlc_enable']}",
        f"#define CAN_CONTRACT_PARAM_PNC_BLOCKED {cfg['pnc_blocked']}",
        f"#define CAN_CONTRACT_PARAM_LOCK_CMD {cfg['lock_cmd']}",
        "",
    ]
    C_OUT.write_text("\n".join(lines), encoding="utf-8")


def main():
    constants = load_constants()
    write_c(constants)
    print("Generated:", C_OUT)


if __name__ == "__main__":
    main()
