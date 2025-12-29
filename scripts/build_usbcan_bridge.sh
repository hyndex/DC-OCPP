#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${ROOT_DIR}/tools/waveshare_usbcan/usbcan_bridge.cpp"
OUT="${ROOT_DIR}/tools/waveshare_usbcan/usbcan_bridge"
INC_DIR="${ROOT_DIR}/third_party/waveshare-usb-can/include"
LIB_DIR="${ROOT_DIR}/third_party/waveshare-usb-can/lib"

if [[ ! -f "${INC_DIR}/controlcan.h" || ! -f "${LIB_DIR}/libcontrolcan.so" ]]; then
  echo "Missing ControlCAN headers/libs. Run scripts/waveshare_usbcan_setup.sh first." >&2
  exit 1
fi

CXX="${CXX:-g++}"

"${CXX}" -std=c++17 -O2 -pthread \
  "${SRC}" \
  -I"${INC_DIR}" \
  -L"${LIB_DIR}" -lcontrolcan \
  -Wl,-rpath,'$ORIGIN/../../third_party/waveshare-usb-can/lib' \
  -o "${OUT}"

echo "Built ${OUT}"
