// include/motion/motion_models.hpp
// Motion model interfaces + two starter models: CA9 and CT-in-XY with per-track omega_z.
//
// Key performance intent:
// - Execute by bucket (a contiguous index list) to avoid per-track branching.
// - "Plugin-like" interface is acceptable because it's called once per bucket, not per track.
// - Later, replace the scalar hot loops with backend-dispatched batch kernels (MKL/Eigen/STD).

#pragma once

#include <cstddef>
#include <vector>
#include <cmath>

#include "track_batch_soa.hpp"

namespace motion {

// Light-weight context that can grow over time.
struct MotionContext {
  double t_s = 0.0;
};

struct IMotionModel {
  virtual ~IMotionModel() = default;
  virtual MotionModelId id() const = 0;
  virtual const char* name() const = 0;

  // idx: indices of tracks to propagate
  virtual void propagate(TrackBatchSoA& tb,
                         const std::vector<std::size_t>& idx,
                         double dt_s,
                         const MotionContext& ctx) const = 0;
};

//===============================
// CA9 (constant acceleration) propagation
//===============================
struct MotionModel_CA9 final : IMotionModel {
  MotionModelId id() const override { return MotionModelId::CA9; }
  const char* name() const override { return "CA9"; }

  void propagate(TrackBatchSoA& tb,
                 const std::vector<std::size_t>& idx,
                 double dt_s,
                 const MotionContext&) const override
  {
    const double dt2 = dt_s * dt_s;
    const double half_dt2 = 0.5 * dt2;

    for (std::size_t k = 0; k < idx.size(); ++k) {
      const std::size_t i = idx[k];

      tb.x(i)  += tb.vx(i) * dt_s + tb.ax(i) * half_dt2;
      tb.y(i)  += tb.vy(i) * dt_s + tb.ay(i) * half_dt2;
      tb.z(i)  += tb.vz(i) * dt_s + tb.az(i) * half_dt2;

      tb.vx(i) += tb.ax(i) * dt_s;
      tb.vy(i) += tb.ay(i) * dt_s;
      tb.vz(i) += tb.az(i) * dt_s;

      // ax/ay/az unchanged
    }
  }
};

//===============================
// CT in XY using per-track omega_z (rad/s); fixed 9D state.
// - updates x,y using exact integration for constant turn-rate (omega) with initial velocity
// - rotates vx,vy by theta = omega*dt
// - z axis uses CA9-like update with existing az
//===============================
struct MotionModel_CT_XY_OmegaZ final : IMotionModel {
  MotionModelId id() const override { return MotionModelId::CT_XY_OMEGAZ; }
  const char* name() const override { return "CT_XY_OmegaZ"; }

  void propagate(TrackBatchSoA& tb,
                 const std::vector<std::size_t>& idx,
                 double dt_s,
                 const MotionContext&) const override
  {
    constexpr double eps = 1e-9;

    for (std::size_t k = 0; k < idx.size(); ++k) {
      const std::size_t i = idx[k];

      const double w   = tb.ct_omega_z_radps[i];
      const double vx0 = tb.vx(i);
      const double vy0 = tb.vy(i);

      // Z axis CA-style
      tb.z(i)  += tb.vz(i) * dt_s + 0.5 * tb.az(i) * dt_s * dt_s;
      tb.vz(i) += tb.az(i) * dt_s;

      if (std::abs(w) < eps) {
        tb.x(i) += vx0 * dt_s;
        tb.y(i) += vy0 * dt_s;
        continue;
      }

      const double theta = w * dt_s;
      const double c = std::cos(theta);
      const double s = std::sin(theta);

      // exact integration for constant turn with constant speed in XY
      tb.x(i) += (vx0 * s + vy0 * (1.0 - c)) / w;
      tb.y(i) += (vy0 * s - vx0 * (1.0 - c)) / w;

      // rotate velocity
      tb.vx(i) =  c * vx0 - s * vy0;
      tb.vy(i) =  s * vx0 + c * vy0;

      // ax/ay left unchanged for now
    }
  }
};

//===============================
// Model selection (single-best-model) stub.
// Replace with IMM, heuristic, or scenario-defined selection later.
//===============================
struct ModelSelector {
  inline void update_models(TrackBatchSoA& tb, class ModelBuckets& buckets) const;
};

} // namespace motion

#include "model_buckets.hpp"

namespace motion {

inline void ModelSelector::update_models(TrackBatchSoA& tb, ModelBuckets& buckets) const {
  for (std::size_t i = 0; i < tb.n; ++i) {
    const MotionModelId desired = (std::abs(tb.ct_omega_z_radps[i]) > 0.0)
                               ? MotionModelId::CT_XY_OMEGAZ
                               : MotionModelId::CA9;
    if (tb.model_id[i] != desired) {
      buckets.move_track(tb, i, desired);
    }
  }
}

} // namespace motion
