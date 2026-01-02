#include "index_update_manager.h"

#include <cmath>
#include <stdexcept>

namespace idx {

void IndexUpdateManager::Reset(std::size_t n_tracks) {
  last_x_.assign(n_tracks, 0.0);
  last_y_.assign(n_tracks, 0.0);
  last_z_.assign(n_tracks, 0.0);
  last_t_.assign(n_tracks, -1e30);
  last_sigma_.assign(n_tracks, 0.0);
  n_updated_last_ = 0;
}

void IndexUpdateManager::Configure(double d_th_m, double t_max_s) {
  if (!(d_th_m > 0.0)) throw std::runtime_error("IndexUpdateManager: d_th_m must be > 0");
  if (!(t_max_s > 0.0)) throw std::runtime_error("IndexUpdateManager: t_max_s must be > 0");
  d_th_m_ = d_th_m;
  d_th2_m2_ = d_th_m_ * d_th_m_;
  t_max_s_ = t_max_s;
}

void IndexUpdateManager::EnableSigmaTrigger(bool enabled, double sigma_th_m) {
  sigma_enabled_ = enabled;
  sigma_th_m_ = sigma_th_m;
}

void IndexUpdateManager::Apply(double t_now_s,
                               const double* xs, const double* ys, const double* zs,
                               ISpatialIndex3D& index) {
  if (!xs || !ys || !zs) throw std::runtime_error("IndexUpdateManager::Apply: null pointers");
  const std::size_t n = last_x_.size();
  n_updated_last_ = 0;

  // If backend doesn't support incremental updates, degrade to rebuild once per scan.
  if (!index.SupportsIncrementalUpdates()) {
    index.BuildFromXYZ(xs, ys, zs, n);
    n_updated_last_ = n;
    // Refresh "last indexed" values so future comparisons are sane.
    for (std::size_t i = 0; i < n; ++i) {
      last_x_[i] = xs[i];
      last_y_[i] = ys[i];
      last_z_[i] = zs[i];
      last_t_[i] = t_now_s;
    }
    return;
  }

  // Incremental updates
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = xs[i] - last_x_[i];
    const double dy = ys[i] - last_y_[i];
    const double dz = zs[i] - last_z_[i];

    const double d2 = dx*dx + dy*dy + dz*dz;
    const double age = t_now_s - last_t_[i];

    bool do_update = (d2 > d_th2_m2_) || (age >= t_max_s_);

    // sigma trigger is optional; leaving hook here for later integration.
    if (sigma_enabled_) {
      // Caller must update last_sigma_ externally or we extend API to pass sigma array.
      // For now, treat as disabled unless you extend Apply signature.
      (void)do_update;
    }

    if (do_update) {
      index.UpdatePoint(static_cast<std::uint64_t>(i), xs[i], ys[i], zs[i]);
      last_x_[i] = xs[i];
      last_y_[i] = ys[i];
      last_z_[i] = zs[i];
      last_t_[i] = t_now_s;
      ++n_updated_last_;
    }
  }
}

void IndexUpdateManager::ApplySubset(double t_now_s,
                                     const double* xs, const double* ys, const double* zs,
                                     const std::uint64_t* ids, std::size_t n_ids,
                                     ISpatialIndex3D& index) {
  if (!xs || !ys || !zs || !ids) throw std::runtime_error("IndexUpdateManager::ApplySubset: null pointers");
  n_updated_last_ = 0;

  for (std::size_t k = 0; k < n_ids; ++k) {
    const std::size_t i = static_cast<std::size_t>(ids[k]);
    index.UpdatePoint(static_cast<std::uint64_t>(i), xs[i], ys[i], zs[i]);
    last_x_[i] = xs[i];
    last_y_[i] = ys[i];
    last_z_[i] = zs[i];
    last_t_[i] = t_now_s;
    ++n_updated_last_;
  }
}

} // namespace idx
