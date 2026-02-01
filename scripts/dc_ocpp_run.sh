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

exec "${BIN}" -c "${CONFIG}"
