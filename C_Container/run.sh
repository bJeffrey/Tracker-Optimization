#!/usr/bin/env bash
# run.sh - convenience wrapper for demo execution

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
DEMO_BIN="${BUILD_DIR}/demo"
export TRACKER_LOG_LEVEL="${TRACKER_LOG_LEVEL:-debug}"
RUN_S="${RUN_S:-10.0}"
WARM_COMMIT_EVERY_N_SCANS="${WARM_COMMIT_EVERY_N_SCANS:-2}"

if [[ ! -x "$DEMO_BIN" ]]; then
  echo "ERROR: demo binary not found at ${DEMO_BIN}"
  echo "Build first with: ${ROOT_DIR}/build.sh"
  exit 1
fi

exec "$DEMO_BIN" \
  --config "${ROOT_DIR}/config/system.xml" \
  --xsd-dir "${ROOT_DIR}/schemas" \
  --run-s "${RUN_S}" \
  --warm-commit-every "${WARM_COMMIT_EVERY_N_SCANS}" \
  --verbose
