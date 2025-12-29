#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_DIR="${ROOT_DIR}/third_party/waveshare-usb-can"
INC_DIR="${DEST_DIR}/include"
LIB_DIR="${DEST_DIR}/lib"

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required." >&2
  exit 1
fi
if ! command -v unzip >/dev/null 2>&1; then
  echo "unzip is required." >&2
  exit 1
fi

arch="$(uname -m)"
zip_url=""
header_path=""
lib_path=""

case "${arch}" in
  aarch64|arm64)
    zip_url="https://files.waveshare.com/upload/1/1c/USB-CAN-B_Code.zip"
    header_path="USB-CAN-B_Code/Jetson/c/controlcan.h"
    lib_path="USB-CAN-B_Code/Jetson/c/libcontrolcan.so"
    ;;
  armv7l|armv6l)
    zip_url="https://files.waveshare.com/upload/1/1c/USB-CAN-B_Code.zip"
    header_path="USB-CAN-B_Code/Raspberry/c/controlcan.h"
    lib_path="USB-CAN-B_Code/Raspberry/c/libcontrolcan.so"
    ;;
  x86_64)
    zip_url="https://files.waveshare.com/wiki/USB-CAN-B/Demo/lib/second-library.zip"
    header_path="second-library/controlcan.h"
    lib_path="second-library/x86/64-linux/libcontrolcan.so"
    ;;
  i386|i686)
    zip_url="https://files.waveshare.com/wiki/USB-CAN-B/Demo/lib/second-library.zip"
    header_path="second-library/controlcan.h"
    lib_path="second-library/x86/32-linux/libcontrolcan.so"
    ;;
  *)
    echo "Unsupported architecture: ${arch}" >&2
    exit 1
    ;;
esac

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

echo "Downloading ${zip_url}"
curl -L -o "${tmp_dir}/source.zip" "${zip_url}"
unzip -q "${tmp_dir}/source.zip" -d "${tmp_dir}/unzip"

if [[ ! -f "${tmp_dir}/unzip/${header_path}" ]]; then
  echo "Header not found at ${header_path}" >&2
  exit 1
fi
if [[ ! -f "${tmp_dir}/unzip/${lib_path}" ]]; then
  echo "Library not found at ${lib_path}" >&2
  exit 1
fi

mkdir -p "${INC_DIR}" "${LIB_DIR}"
install -m 644 "${tmp_dir}/unzip/${header_path}" "${INC_DIR}/controlcan.h"
install -m 755 "${tmp_dir}/unzip/${lib_path}" "${LIB_DIR}/libcontrolcan.so"

echo "Installed controlcan.h and libcontrolcan.so to ${DEST_DIR}"
