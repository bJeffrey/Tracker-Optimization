#pragma once

#include "config_types.h"
#include "rtree_sqlite.h"

namespace idx {

// Compute a conservative ECEF AABB for the sensor's scan frustum.
// Intended for coarse "candidate track" queries.
//
// Current approximation (ECEF-only):
// - If ScanVolume.Frame == "OWNERSHIP_BODY", interpret az/el/range as a local
//   Cartesian frame aligned with ECEF axes (no yaw/pitch/roll; no ENU tangent conversion).
// - Otherwise, fall back to a deterministic, conservative ECEF-aligned box.
EcefAabb ComputeScanAabbEcefApprox(const cfg::SensorCfg& sensor, const cfg::Ownship& ownship);

} // namespace idx
