#!/usr/bin/env bash
# run.sh - convenience wrapper for demo execution

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
DEMO_BIN="${BUILD_DIR}/demo"
export TRACKER_LOG_LEVEL="${TRACKER_LOG_LEVEL:-INFO}"
RUN_S="${RUN_S:-30.0}"
WARM_COMMIT_EVERY_N_SCANS="${WARM_COMMIT_EVERY_N_SCANS:-0}"
export TRACKER_LOG_SENSOR_COURSE_GATES="${TRACKER_LOG_SENSOR_COURSE_GATES:-0}"
export TRACKER_LOG_ASSOC_MODE="${TRACKER_LOG_ASSOC_MODE:-1}"
export TRACKER_LOG_TRACK_TRUTH_MEAS="${TRACKER_LOG_TRACK_TRUTH_MEAS:-0}"
export TRACKER_LOG_TRACK_GATE_BOUNDS="${TRACKER_LOG_TRACK_GATE_BOUNDS:-0}"

if [[ ! -x "$DEMO_BIN" ]]; then
  echo "ERROR: demo binary not found at ${DEMO_BIN}"
  echo "Build first with: ${ROOT_DIR}/build.sh"
  exit 1
fi

exec "$DEMO_BIN" \
  --config "${ROOT_DIR}/config/system.xml" \
  --xsd-dir "${ROOT_DIR}/schemas" \
  --run-s "${RUN_S}" \
