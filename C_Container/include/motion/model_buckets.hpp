// include/motion/model_buckets.hpp
// Persistent motion-model buckets (Option B).
//
// Tracks rarely change motion model. Maintain persistent index lists per model to enable
// per-bucket batch execution and avoid per-track branching.
//
// O(1) change-model update achieved via:
// - Each bucket holds track indices.
// - Each track stores its `bucket_slot` position inside its current bucket.
// - Move operation uses swap-remove from old bucket + push_back to new bucket.

#pragma once

#include <cstddef>
#include <vector>
#include <cassert>

#include "track_batch_soa.hpp"

namespace motion {

struct ModelBuckets {
    std::vector<std::size_t> bucket_ca;
    std::vector<std::size_t> bucket_ct;

    inline void clear() {
        bucket_ca.clear();
        bucket_ct.clear();
    }

    inline std::vector<std::size_t>& bucket_for(MotionModelId id) {
        switch (id) {
            case MotionModelId::CA9:          return bucket_ca;
            case MotionModelId::CT_XY_OMEGAZ: return bucket_ct;
            default:                          return bucket_ca;
        }
    }

    inline const std::vector<std::size_t>& bucket_for(MotionModelId id) const {
        switch (id) {
            case MotionModelId::CA9:          return bucket_ca;
            case MotionModelId::CT_XY_OMEGAZ: return bucket_ct;
            default:                          return bucket_ca;
        }
    }

    inline void init_from_track_batch(TrackBatchSoA& tb) {
        tb.assert_sizes();
        clear();
        bucket_ca.reserve(tb.n);
        bucket_ct.reserve(tb.n);

        for (std::size_t i = 0; i < tb.n; ++i) {
            auto& b = bucket_for(tb.model_id[i]);
            tb.bucket_slot[i] = b.size();
            b.push_back(i);
        }
    }

    inline void move_track(TrackBatchSoA& tb, std::size_t i, MotionModelId new_model) {
        tb.assert_sizes();
        const MotionModelId old_model = tb.model_id[i];
        if (old_model == new_model) return;

        auto& old_b = bucket_for(old_model);
        auto& new_b = bucket_for(new_model);

        const std::size_t slot = tb.bucket_slot[i];
        assert(slot < old_b.size());

        // swap-remove from old bucket
        const std::size_t j = old_b.back();
        old_b[slot] = j;
        tb.bucket_slot[j] = slot;
        old_b.pop_back();

        // push into new bucket
        tb.bucket_slot[i] = new_b.size();
        new_b.push_back(i);

        tb.model_id[i] = new_model;
    }
};

} // namespace motion
