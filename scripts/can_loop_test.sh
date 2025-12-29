#!/usr/bin/env bash
set -euo pipefail

iface="can0"
require_waveshare="${REQUIRE_WAVESHARE_USB_CAN:-0}"

usage() {
  echo "Usage: $0 [--require-waveshare] [iface]" >&2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --require-waveshare)
      require_waveshare=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
    *)
      iface="$1"
      shift
      ;;
  esac
done

has_pattern() {
  local pattern="$1"
  local file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -qi "${pattern}" "${file}"
  else
    grep -qi "${pattern}" "${file}"
  fi
}

detect_waveshare() {
  if ! command -v lsusb >/dev/null 2>&1; then
    echo "lsusb not found (install usbutils) to verify Waveshare USB-CAN-B." >&2
    return 1
  fi
  if lsusb | grep -qiE '04d8:0053|USB-CAN|USBCAN|CANalyst'; then
    return 0
  fi
  return 1
}

iface_is_vcan() {
  if ! command -v ip >/dev/null 2>&1; then
    echo "ip not found (install iproute2)." >&2
    return 2
  fi
  ip -details link show "${iface}" 2>/dev/null | grep -qi "vcan"
}

bridge_running_for_iface() {
  local iface_name="$1"
  local bridge_lines
  if command -v pgrep >/dev/null 2>&1; then
    bridge_lines="$(pgrep -a -f usbcan_bridge || true)"
  else
    bridge_lines="$(ps -eo pid,args | grep -E '[u]sbcan_bridge' || true)"
  fi
  if [[ -z "${bridge_lines}" ]]; then
    return 1
  fi
  if echo "${bridge_lines}" | grep -qE -- "--iface(=|[[:space:]])${iface_name}"; then
    return 0
  fi
  if [[ "${iface_name}" == "can0" ]] && echo "${bridge_lines}" | grep -q "usbcan_bridge"; then
    return 0
  fi
  return 1
}
if ! command -v candump >/dev/null 2>&1; then
  echo "candump not found (install can-utils)." >&2
  exit 1
fi
if ! command -v cansend >/dev/null 2>&1; then
  echo "cansend not found (install can-utils)." >&2
  exit 1
fi
if [[ "${require_waveshare}" != "0" ]]; then
  if ! detect_waveshare; then
    echo "Waveshare USB-CAN-B not detected (lsusb missing 04d8:0053/USB-CAN)." >&2
    exit 1
  fi
  if iface_is_vcan; then
    if ! bridge_running_for_iface "${iface}"; then
      echo "Waveshare device detected but usbcan_bridge is not running for ${iface}." >&2
      echo "Start it: ./tools/waveshare_usbcan/usbcan_bridge --iface ${iface} --device 0 --channel 0 --bitrate 125000 --verbose" >&2
      exit 1
    fi
  fi
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

if has_pattern "18FF50E5" "${tmp_file}"; then
  echo "Loop test passed on ${iface}"
  exit 0
fi

echo "Loop test failed: no frame seen on ${iface}" >&2
exit 1
