#pragma once
/**
 * @file model_buckets.hpp
 * @brief Persistent buckets for motion propagation (execute-by-bucket).
 *
 * Buckets store track indices by current motion model.
 * move_track() moves a track index between buckets and updates tb.model_id.
 */

#include <cstddef>
#include <vector>
#include <algorithm>

#include "plugins/track_mgmt/track_kinematics_batch.hpp"

namespace trk {

struct ModelBuckets
{
  std::vector<std::size_t> bucket_ca;
  std::vector<std::size_t> bucket_ct;

  void clear()
  {
    bucket_ca.clear();
    bucket_ct.clear();
  }

  void init_from_track_batch(TrackKinematicsBatch& tb)
  {
    clear();
    bucket_ca.reserve(tb.n);
    bucket_ct.reserve(tb.n);

    for (std::size_t i = 0; i < tb.n; ++i) {
      const MotionModelId m = tb.model_id[i];
      if (m == MotionModelId::CT_XY_OMEGAZ) {
        bucket_ct.push_back(i);
      } else {
        // Default/fallback to CA
        tb.model_id[i] = MotionModelId::CA9;
        bucket_ca.push_back(i);
      }
    }
  }

  static inline void erase_index(std::vector<std::size_t>& v, std::size_t track_i)
  {
    // O(n) erase; OK for “rarely changes” assumption.
    auto it = std::find(v.begin(), v.end(), track_i);
    if (it != v.end()) {
      *it = v.back();
      v.pop_back();
    }
  }

  void move_track(TrackKinematicsBatch& tb, std::size_t track_i, MotionModelId new_model)
  {
    const MotionModelId old_model = tb.model_id[track_i];
    if (old_model == new_model) return;

    // Remove from old bucket
    if (old_model == MotionModelId::CT_XY_OMEGAZ) {
      erase_index(bucket_ct, track_i);
    } else {
      erase_index(bucket_ca, track_i);
    }

    // Add to new bucket
    tb.model_id[track_i] = new_model;
    if (new_model == MotionModelId::CT_XY_OMEGAZ) {
      bucket_ct.push_back(track_i);
    } else {
      bucket_ca.push_back(track_i);
    }
  }
};

} // namespace trk
