#pragma once

#include "spatial_index_3d.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace idx {

// IndexUpdateManager controls WHEN we update the spatial index for each track.
// - Track predict/covariance propagation is separate and handled elsewhere.
// - This manager only decides when to update index entries to keep coarse candidate queries correct.
class IndexUpdateManager {
public:
  void Reset(std::size_t n_tracks);

  // Configure thresholds (recommended to come from config; can vary by platform/sensor).
  void Configure(double d_th_m, double t_max_s);

  // Optional: enable uncertainty-growth trigger (caller supplies sigma_pos array if used).
  void EnableSigmaTrigger(bool enabled, double sigma_th_m);

  // Apply updates using positions at time t_now.
  // xs/ys/zs must be arrays of length n_tracks.
  void Apply(double t_now_s,
             const double* xs, const double* ys, const double* zs,
             ISpatialIndex3D& index);

  // Apply updates for a subset of ids (caller already decided which ids changed).
  void ApplySubset(double t_now_s,
                   const double* xs, const double* ys, const double* zs,
                   const std::uint64_t* ids, std::size_t n_ids,
                   ISpatialIndex3D& index);
  // Apply updates for subset but honor movement/time thresholds.
  void ApplySubsetThreshold(double t_now_s,
                            const double* xs, const double* ys, const double* zs,
                            const std::uint64_t* ids, std::size_t n_ids,
                            ISpatialIndex3D& index);

  std::size_t NumUpdatedLastApply() const { return n_updated_last_; }

private:
  double d_th_m_ = 500.0;
  double d_th2_m2_ = 500.0 * 500.0;
  double t_max_s_ = 1.0;

  bool sigma_enabled_ = false;
  double sigma_th_m_ = 0.0;

  std::vector<double> last_x_;
  std::vector<double> last_y_;
  std::vector<double> last_z_;
  std::vector<double> last_t_;
  std::vector<double> last_sigma_;

  std::size_t n_updated_last_ = 0;
};

} // namespace idx
