// include/motion/track_batch_soa.hpp
// SoA TrackBatch for high-throughput tracker batch operations.
//
// Canonical state dimension (fixed): 9
//   [x y z vx vy vz ax ay az]  (CA9 layout conceptually)
// Physical storage: Structure-of-Arrays (SoA) for cache-friendly loops and SIMD-friendly kernels.
//
// Notes:
// - `pos_x/pos_y/pos_z` are optional "query columns" (can be aliases/views in future).
// - Covariance storage is intentionally omitted here; keep your existing batched P storage until you
//   decide on an SoA/blocked representation.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cassert>

namespace motion {

enum class MotionModelId : std::uint8_t {
    CA9 = 0,
    CT_XY_OMEGAZ = 1
};

struct TrackBatchSoA {
    std::size_t n = 0;

    // Hot columns (state)
    std::vector<double> x, y, z;
    std::vector<double> vx, vy, vz;
    std::vector<double> ax, ay, az;

    // Optional query columns (used by spatial indexing, etc.)
    std::vector<double> pos_x, pos_y, pos_z;

    // Motion metadata
    std::vector<MotionModelId> model_id;
    std::vector<std::size_t> bucket_slot;     // slot index inside its current bucket (for O(1) swap-remove)
    std::vector<double> ct_omega_z_radps;     // per-track turn rate (rad/s) for CT model

    void resize(std::size_t n_tracks) {
        n = n_tracks;

        x.resize(n); y.resize(n); z.resize(n);
        vx.resize(n); vy.resize(n); vz.resize(n);
        ax.resize(n); ay.resize(n); az.resize(n);

        pos_x.resize(n); pos_y.resize(n); pos_z.resize(n);

        model_id.assign(n, MotionModelId::CA9);
        bucket_slot.assign(n, 0);
        ct_omega_z_radps.assign(n, 0.0);
    }

    inline void assert_sizes() const {
        assert(x.size() == n && y.size() == n && z.size() == n);
        assert(vx.size() == n && vy.size() == n && vz.size() == n);
        assert(ax.size() == n && ay.size() == n && az.size() == n);
        assert(pos_x.size() == n && pos_y.size() == n && pos_z.size() == n);
        assert(model_id.size() == n);
        assert(bucket_slot.size() == n);
        assert(ct_omega_z_radps.size() == n);
    }

    // Keep query columns synced to state position.
    // If you prefer, remove pos_* and read x/y/z directly in your index builder.
    inline void sync_pos_from_state() {
        pos_x = x; pos_y = y; pos_z = z;
    }
};

} // namespace motion
