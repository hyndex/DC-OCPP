#!/usr/bin/env bash
set -euo pipefail

iface="${1:-can0}"
if ! command -v candump >/dev/null 2>&1; then
  echo "candump not found (install can-utils)." >&2
  exit 1
fi
if ! command -v cansend >/dev/null 2>&1; then
  echo "cansend not found (install can-utils)." >&2
  exit 1
fi

tmp_file="$(mktemp)"
cleanup() {
  rm -f "${tmp_file}"
}
trap cleanup EXIT

candump -L "${iface}" > "${tmp_file}" &
dump_pid=$!

sleep 0.5
cansend "${iface}" 18FF50E5#1122334455667788
sleep 0.5

kill "${dump_pid}" >/dev/null 2>&1 || true
wait "${dump_pid}" >/dev/null 2>&1 || true

if command -v rg >/dev/null 2>&1; then
  if rg -qi "18FF50E5" "${tmp_file}"; then
    echo "Loop test passed on ${iface}"
    exit 0
  fi
else
  if grep -qi "18FF50E5" "${tmp_file}"; then
    echo "Loop test passed on ${iface}"
    exit 0
  fi
fi

echo "Loop test failed: no frame seen on ${iface}" >&2
exit 1
