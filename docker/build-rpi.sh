#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./docker/build-rpi.sh
#   PLATFORM=linux/arm/v7 ./docker/build-rpi.sh  # 32-bit Raspberry Pi OS

PLATFORM=${PLATFORM:-linux/arm64}
BUILD_TYPE=${BUILD_TYPE:-Release}
EXTRA_CMAKE_FLAGS=${EXTRA_CMAKE_FLAGS:-}
OUT_DIR=${OUT_DIR:-build-rpi}

echo "Building dc_ocpp for ${PLATFORM} (type=${BUILD_TYPE})"

mkdir -p "${OUT_DIR}"
docker buildx build \
  --platform "${PLATFORM}" \
  -f docker/Dockerfile.rpi \
  --target export \
  --output "type=local,dest=${OUT_DIR}" \
  --build-arg BUILD_TYPE="${BUILD_TYPE}" \
  --build-arg EXTRA_CMAKE_FLAGS="${EXTRA_CMAKE_FLAGS}" \
  .
