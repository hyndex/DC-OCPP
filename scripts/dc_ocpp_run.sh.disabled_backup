#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

BIN_DEFAULT="${REPO_DIR}/build/dc_ocpp"
CONFIG_DEFAULT="${REPO_DIR}/configs/charger.json"

BIN="${DC_OCPP_BIN:-${BIN_DEFAULT}}"
CONFIG="${DC_OCPP_CONFIG:-${CONFIG_DEFAULT}}"

if [[ ! -x "${BIN}" ]]; then
  echo "dc_ocpp binary not found or not executable: ${BIN}" >&2
  exit 1
fi

# Dev convenience: autogenerate a local (self-managed) client cert/key when SecurityProfile=3.
# This is opt-in because it is not production PKI.
if [[ "${DC_OCPP_AUTO_CERTS:-0}" == "1" ]]; then
  sp="$(
    python3 - <<'PY' "${CONFIG}" 2>/dev/null || true
import json
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1]).resolve(strict=False)
base_dir = cfg_path.parent
cfg = json.loads(cfg_path.read_text(encoding="utf-8"))

def resolve_from(base: Path, p: str) -> Path:
    pp = Path(p)
    if pp.is_absolute():
        return pp
    return (base / pp).resolve(strict=False)

ocpp = cfg.get("ocpp")
if isinstance(ocpp, dict):
    sec = ocpp.get("Security") if isinstance(ocpp.get("Security"), dict) else {}
    sp = sec.get("SecurityProfile", "")
    print(sp)
    raise SystemExit(0)

ocpp_cfg = str(cfg.get("ocppConfig", "")).strip()
if not ocpp_cfg:
    print("")
    raise SystemExit(0)
ocpp_path = resolve_from(base_dir, ocpp_cfg)
if not ocpp_path.exists():
    print("")
    raise SystemExit(0)
base = json.loads(ocpp_path.read_text(encoding="utf-8"))
sec = base.get("Security") if isinstance(base.get("Security"), dict) else {}
print(sec.get("SecurityProfile", ""))
PY
  )"
  if [[ "${sp}" == "3" ]]; then
    "${REPO_DIR}/scripts/provision_ocpp_client_cert.py" --config "${CONFIG}" --dev-ca || true
  fi
fi

exec "${BIN}" -c "${CONFIG}"
