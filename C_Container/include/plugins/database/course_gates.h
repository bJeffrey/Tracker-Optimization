#pragma once
/**
 * @file course_gates.h
 * @brief Build and query coarse course gates for spatial indexing.
 */

#include "config/config_types.h"
#include "plugins/database/spatial_index_3d.h"
#include "tracker_types.h"

#include <cstddef>
#include <vector>

namespace db {

class ITrackDatabase;

struct CourseGateSet {
  bool enabled = false;
  idx::EcefAabb prefetch_aabb{};
  std::vector<idx::EcefAabb> gates;
};

CourseGateSet BuildCourseGateSet(const cfg::Ownship& ownship,
                                 const cfg::CourseGatesCfg& cfg);

trk::IdList QueryCourseGates(const ITrackDatabase& track_db,
                             const std::vector<idx::EcefAabb>& gates,
                             std::size_t n_tracks);

} // namespace db
