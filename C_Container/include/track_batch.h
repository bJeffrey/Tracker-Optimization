#pragma once
/**
 * @file track_batch.h
 * @brief Structure-of-Arrays (SoA) track storage for high-throughput batch processing.
 *
 * Step 1 objective:
 *  - Introduce a fast, contiguous TrackBatch container that holds:
 *    - metadata (id, last_update_time, status, optional quality)
 *    - state vector x (ECEF CA9: [x y z vx vy vz ax ay az])
 *    - covariance P (full 9x9 for now; 81 doubles per track)
 *
 * Notes:
 *  - Storage is batch-major and contiguous, designed for gather/compute/scatter workflows.
 *  - No database/index integration yet (that’s Step 3+).
 */

#include "track_status.h"

#include <cstddef>
#include <cstdint>
#include <vector>

struct TrackBatch
{
  // Current state dimension (temporary: fixed 9D CA model)
  static constexpr int kDim = 9;
  static constexpr int kCovN = kDim * kDim; // 81

  std::size_t n_tracks = 0;

  // --- Metadata ---
  std::vector<std::uint64_t> track_id;       // length n_tracks
  std::vector<double>        last_update_s;  // length n_tracks
  std::vector<TrackStatus>   status;         // length n_tracks
  std::vector<float>         quality;        // length n_tracks (optional usage)

  // --- State & Covariance (contiguous buffers) ---
  // x: [n_tracks x kDim] row-major by track (track-major)
  std::vector<double> x;

  // P: [n_tracks x kCovN] row-major by track (track-major), each 9x9 row-major
  std::vector<double> P;

  void resize(std::size_t n)
  {
    n_tracks = n;
    track_id.resize(n_tracks);
    last_update_s.resize(n_tracks);
    status.resize(n_tracks);
    quality.resize(n_tracks);

    x.assign(n_tracks * static_cast<std::size_t>(kDim), 0.0);
    P.assign(n_tracks * static_cast<std::size_t>(kCovN), 0.0);
  }

  // Track-major helpers
  double* x_ptr(std::size_t i) { return x.data() + i * static_cast<std::size_t>(kDim); }
  const double* x_ptr(std::size_t i) const { return x.data() + i * static_cast<std::size_t>(kDim); }

  double* P_ptr(std::size_t i) { return P.data() + i * static_cast<std::size_t>(kCovN); }
  const double* P_ptr(std::size_t i) const { return P.data() + i * static_cast<std::size_t>(kCovN); }

  // Convenience: set P_i = scalar * I (full 9x9)
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
