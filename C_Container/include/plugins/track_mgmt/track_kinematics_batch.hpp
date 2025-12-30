#pragma once
/**
 * @file track_kinematics_batch.hpp
 * @brief Minimal kinematics SoA used by motion propagation + scan/index queries.
 *
 * This is intentionally separate from trk::TrackBatch:
 * - TrackBatch owns "tracker state" (covariance, IDs, status, etc.)
 * - TrackKinematicsBatch owns hot kinematics slabs for propagation/indexing (x/y/z, v, a, omega, model_id).
 */

#include <cstddef>
#include <vector>
#include <cstdint>
#include <stdexcept>

namespace trk {

enum class MotionModelId : std::uint8_t
{
  CA9 = 0,
  CT_XY_OMEGAZ = 1
};

struct TrackKinematicsBatch
{
  std::size_t n = 0;

  // Kinematics slabs
  std::vector<double> x_s, y_s, z_s;
  std::vector<double> vx_s, vy_s, vz_s;
  std::vector<double> ax_s, ay_s, az_s;

  // CT parameter (per-track), rad/s
  std::vector<double> ct_omega_z_radps;

  // Model id per track
  std::vector<MotionModelId> model_id;

  void resize(std::size_t n_tracks)
  {
    n = n_tracks;

    x_s.resize(n);  y_s.resize(n);  z_s.resize(n);
    vx_s.resize(n); vy_s.resize(n); vz_s.resize(n);
    ax_s.resize(n); ay_s.resize(n); az_s.resize(n);

    ct_omega_z_radps.resize(n, 0.0);
    model_id.resize(n, MotionModelId::CA9);
  }

  // Slab pointers (for index update manager)
  inline const double* x_ptr() const { return x_s.data(); }
  inline const double* y_ptr() const { return y_s.data(); }
  inline const double* z_ptr() const { return z_s.data(); }

  inline double* x_ptr() { return x_s.data(); }
  inline double* y_ptr() { return y_s.data(); }
  inline double* z_ptr() { return z_s.data(); }

  // Accessors
  inline double& x(std::size_t i) { return x_s[i]; }
  inline double& y(std::size_t i) { return y_s[i]; }
  inline double& z(std::size_t i) { return z_s[i]; }

  inline double& vx(std::size_t i) { return vx_s[i]; }
  inline double& vy(std::size_t i) { return vy_s[i]; }
  inline double& vz(std::size_t i) { return vz_s[i]; }

  inline double& ax(std::size_t i) { return ax_s[i]; }
  inline double& ay(std::size_t i) { return ay_s[i]; }
  inline double& az(std::size_t i) { return az_s[i]; }

  inline const double& x(std::size_t i) const { return x_s[i]; }
  inline const double& y(std::size_t i) const { return y_s[i]; }
  inline const double& z(std::size_t i) const { return z_s[i]; }

  inline const double& vx(std::size_t i) const { return vx_s[i]; }
  inline const double& vy(std::size_t i) const { return vy_s[i]; }
  inline const double& vz(std::size_t i) const { return vz_s[i]; }

  inline const double& ax(std::size_t i) const { return ax_s[i]; }
  inline const double& ay(std::size_t i) const { return ay_s[i]; }
  inline const double& az(std::size_t i) const { return az_s[i]; }
};

} // namespace trk
