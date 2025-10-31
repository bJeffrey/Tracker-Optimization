#!/usr/bin/env bash
set -euo pipefail
BUILD_DIR=${1:-build}

cpu_vendor="other"
if [ -f /proc/cpuinfo ] && grep -qi 'GenuineIntel' /proc/cpuinfo; then
  cpu_vendor="intel"
fi

use_mkl=OFF
# Use MKL only if Intel CPU AND MKLROOT is set (typical when oneAPI env is active)
if [ "$cpu_vendor" = "intel" ] && [ -n "${MKLROOT:-}" ]; then
  use_mkl=ON
fi

echo "[detect] cpu_vendor=$cpu_vendor  MKLROOT=${MKLROOT:-<unset>}  USE_MKL=$use_mkl"
mkdir -p "$BUILD_DIR"
cmake -S . -B "$BUILD_DIR" -G Ninja -DUSE_MKL="$use_mkl"
cmake --build "$BUILD_DIR" -j
echo "[done] run: $BUILD_DIR/demo"
