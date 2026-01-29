#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${root_dir}/build}"
parallel="${CMAKE_BUILD_PARALLEL_LEVEL:-}"
if [[ -z "${parallel}" ]]; then
  if command -v nproc >/dev/null 2>&1; then
    parallel="$(nproc)"
  else
    parallel="1"
  fi
fi

if [[ ! -f "${build_dir}/CMakeCache.txt" ]]; then
  echo "[build_dc_ocpp] configuring ${build_dir}"
  cmake -S "${root_dir}" -B "${build_dir}" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_APP_TESTS=OFF
fi

echo "[build_dc_ocpp] building dc_ocpp (parallel=${parallel})"
cmake --build "${build_dir}" --target dc_ocpp --parallel "${parallel}"
echo "[build_dc_ocpp] done: ${build_dir}/dc_ocpp"
