#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SENSORS_XML="${SENSORS_XML:-$ROOT_DIR/config/sensors.xml}"
GATE_SIZING_XML="${GATE_SIZING_XML:-$ROOT_DIR/config/gate_sizing.xml}"
XSD_DIR="${XSD_DIR:-$ROOT_DIR/schemas}"
SENSOR_ID="${SENSOR_ID:-}"

ARGS=("--sensors" "$SENSORS_XML" "--gate-sizing" "$GATE_SIZING_XML" "--xsd-dir" "$XSD_DIR")
if [[ -n "$SENSOR_ID" ]]; then
  ARGS+=("--sensor-id" "$SENSOR_ID")
fi

"$ROOT_DIR/build/gate_sizing" "${ARGS[@]}"
