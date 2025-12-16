#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./docker/build-rpi.sh
#   PLATFORM=linux/arm/v7 ./docker/build-rpi.sh  # 32-bit Raspberry Pi OS

PLATFORM=${PLATFORM:-linux/arm64}
BUILD_TYPE=${BUILD_TYPE:-Release}
BUILD_CONFIG_WEB_UI=${BUILD_CONFIG_WEB_UI:-OFF}
EXTRA_CMAKE_FLAGS=${EXTRA_CMAKE_FLAGS:-}
OUT_DIR=${OUT_DIR:-build-rpi}

echo "Building dc_ocpp for ${PLATFORM} (type=${BUILD_TYPE}, webui=${BUILD_CONFIG_WEB_UI})"

mkdir -p "${OUT_DIR}"
docker buildx build \
  --platform "${PLATFORM}" \
  -f docker/Dockerfile.rpi \
  --target export \
  --output "type=local,dest=${OUT_DIR}" \
  --build-arg BUILD_TYPE="${BUILD_TYPE}" \
  --build-arg BUILD_CONFIG_WEB_UI="${BUILD_CONFIG_WEB_UI}" \
  --build-arg EXTRA_CMAKE_FLAGS="${EXTRA_CMAKE_FLAGS}" \
  .
