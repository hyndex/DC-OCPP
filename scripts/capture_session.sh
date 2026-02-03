#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
LOG_DIR="${REPO_DIR}/logs"

CAN_IFACE="${CAN_IFACE:-can0}"
TS="$(date -u +%Y%m%dT%H%M%SZ)"

mkdir -p "${LOG_DIR}"

CANDUMP_LOG="${LOG_DIR}/candump_${CAN_IFACE}_${TS}.log"
TAIL_LOG="${LOG_DIR}/dc_ocpp_tail_${TS}.log"
INDEX_LOG="${LOG_DIR}/session_capture_index.log"

STDOUT_LOG="${DC_OCPP_STDOUT_LOG:-}"
if [[ -z "${STDOUT_LOG}" ]]; then
  if ls "${LOG_DIR}"/dc_ocpp_stdout_*.log >/dev/null 2>&1; then
    STDOUT_LOG="$(ls -1t "${LOG_DIR}"/dc_ocpp_stdout_*.log | head -n 1)"
  fi
fi

if ! command -v candump >/dev/null 2>&1; then
  echo "ERROR: candump not found (install can-utils)." >&2
  exit 1
fi

if ! ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
  echo "ERROR: CAN interface not found: ${CAN_IFACE}" >&2
  exit 1
fi

echo "Session capture:"
echo "  can_iface : ${CAN_IFACE}"
echo "  candump   : ${CANDUMP_LOG}"
if [[ -n "${STDOUT_LOG}" ]]; then
  echo "  dc_ocpp   : tail -F ${STDOUT_LOG} -> ${TAIL_LOG}"
else
  echo "  dc_ocpp   : (no dc_ocpp stdout log found; set DC_OCPP_STDOUT_LOG to enable tail capture)"
fi

candump -tz -x "${CAN_IFACE}" >"${CANDUMP_LOG}" 2>&1 &
CANDUMP_PID=$!

TAIL_PID=""
if [[ -n "${STDOUT_LOG}" ]]; then
  tail -n 0 -F "${STDOUT_LOG}" >"${TAIL_LOG}" 2>&1 &
  TAIL_PID=$!
fi

cleanup() {
  set +e
  if [[ -n "${TAIL_PID}" ]] && kill -0 "${TAIL_PID}" 2>/dev/null; then
    kill "${TAIL_PID}" 2>/dev/null || true
    wait "${TAIL_PID}" 2>/dev/null || true
  fi
  if kill -0 "${CANDUMP_PID}" 2>/dev/null; then
    kill "${CANDUMP_PID}" 2>/dev/null || true
    wait "${CANDUMP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "${TS} candump_pid=${CANDUMP_PID} tail_pid=${TAIL_PID:-0} can_iface=${CAN_IFACE} candump_log=${CANDUMP_LOG} tail_log=${TAIL_LOG} stdout_log=${STDOUT_LOG}" \
  | tee -a "${INDEX_LOG}"

echo
DURATION_S="${CAPTURE_DURATION_S:-${1:-}}"
if [[ -n "${DURATION_S}" ]]; then
  if ! [[ "${DURATION_S}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: CAPTURE_DURATION_S/arg must be an integer number of seconds (got: ${DURATION_S})" >&2
    exit 1
  fi
  echo "Now plug EV + tap RFID. Capturing for ${DURATION_S}s..."
  sleep "${DURATION_S}"
else
  echo "Now plug EV + tap RFID. Press ENTER to stop capture."
  read -r _
fi
