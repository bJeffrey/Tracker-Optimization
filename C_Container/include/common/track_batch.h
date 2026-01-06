#pragma once
/**
 * @file track_batch.h
 * @brief Structure-of-Arrays (SoA) hot columns + interleaved canonical state buffer.
 *
 * Hybrid approach:
 *  - Canonical state remains interleaved in `x` for minimal disruption.
 *  - Maintain SoA "hot columns" (pos_x/pos_y/pos_z) for fast spatial indexing and coarse gating.
 */

#include "common/track_state.h"
#include "common/track_status.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace trk {

struct TrackBatch
{
  static constexpr int kDim  = kStateDim;
  static constexpr int kCovN = kStateCovN;

  std::size_t n_tracks = 0;

  // --- Metadata ---
  std::vector<std::uint64_t> track_id;
  std::vector<double>        last_update_s;     // last measurement/track update time (future)
  std::vector<double>        last_cov_prop_s;   // used as t_pred_s (state+cov aged-to time)
  std::vector<TrackStatus>   status;
  std::vector<float>         quality;

  // --- Canonical interleaved state ---
  // x: [n_tracks x kDim] row-major by track: [x y z vx vy vz ax ay az]
  std::vector<double> x;

  // --- Hot SoA columns (kept in sync with `x`) ---
  std::vector<double> pos_x;
  std::vector<double> pos_y;
  std::vector<double> pos_z;

  // --- Covariance ---
  std::vector<double> P;

  void resize(std::size_t n)
  {
    n_tracks = n;
    track_id.resize(n_tracks);
    last_update_s.resize(n_tracks);
    last_cov_prop_s.resize(n_tracks);
    status.resize(n_tracks);
    quality.resize(n_tracks);

    x.assign(n_tracks * static_cast<std::size_t>(kDim), 0.0);

    pos_x.assign(n_tracks, 0.0);
    pos_y.assign(n_tracks, 0.0);
    pos_z.assign(n_tracks, 0.0);

    P.assign(n_tracks * static_cast<std::size_t>(kCovN), 0.0);
  }

  // Track-major helpers
  double* x_ptr(std::size_t i) { return x.data() + i * static_cast<std::size_t>(kDim); }
  const double* x_ptr(std::size_t i) const { return x.data() + i * static_cast<std::size_t>(kDim); }

  double* P_ptr(std::size_t i) { return P.data() + i * static_cast<std::size_t>(kCovN); }
  const double* P_ptr(std::size_t i) const { return P.data() + i * static_cast<std::size_t>(kCovN); }

  void sync_pos_from_x()
  {
    const std::size_t stride = static_cast<std::size_t>(kDim);
    const double* px = x.data();
    for (std::size_t i = 0; i < n_tracks; ++i, px += stride) {
      pos_x[i] = px[0];
      pos_y[i] = px[1];
      pos_z[i] = px[2];
    }
  }

  void sync_x_pos_from_pos()
  {
    const std::size_t stride = static_cast<std::size_t>(kDim);
    for (std::size_t i = 0; i < n_tracks; ++i) {
      const std::size_t base = i * stride;
      x[base + 0] = pos_x[i];
      x[base + 1] = pos_y[i];
      x[base + 2] = pos_z[i];
    }
  }

  void set_cov_identity_scaled(double scalar)
  {
    const std::size_t nn = static_cast<std::size_t>(kCovN);
    for (std::size_t i = 0; i < n_tracks; ++i) {
      double* Pi = P.data() + i * nn;
      for (int r = 0; r < kDim; ++r) {
        for (int c = 0; c < kDim; ++c) {
          Pi[r * kDim + c] = (r == c) ? scalar : 0.0;
        }
      }
    }
  }
};

} // namespace trk
