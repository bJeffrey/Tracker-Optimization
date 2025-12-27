// include/motion/track_batch_soa.hpp
// SoA TrackBatch for high-throughput tracker batch operations.
//
// Canonical state dimension (fixed): 9
//   [x y z vx vy vz ax ay az]  (CA9 layout conceptually)
// Physical storage: SoA slab: state[comp*n + i]
//
// Notes:
// - No duplicated pos_x/pos_y/pos_z. Indexing can read x_ptr()/y_ptr()/z_ptr() directly.
// - Covariance storage remains outside (TrackBatch::P) for now.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <array>
#include <cassert>

namespace motion {

enum class MotionModelId : std::uint8_t {
  CA9 = 0,
  CT_XY_OMEGAZ = 1
};

enum class StateComp : std::uint8_t {
  X  = 0,
  Y  = 1,
  Z  = 2,
  VX = 3,
  VY = 4,
  VZ = 5,
  AX = 6,
  AY = 7,
  AZ = 8,
  COUNT = 9
};

struct TrackBatchSoA {
  std::size_t n = 0;

  // SoA slab: comp-major blocks, each block has length n.
  // Layout: state[comp*n + i]
  std::vector<double> state;

  // Motion metadata
  std::vector<MotionModelId> model_id;
  std::vector<std::size_t> bucket_slot;      // slot index inside its current bucket (swap-remove)
  std::vector<double> ct_omega_z_radps;      // per-track turn rate (rad/s)

  void resize(std::size_t n_tracks) {
    n = n_tracks;
    state.resize(static_cast<std::size_t>(StateComp::COUNT) * n);

    model_id.assign(n, MotionModelId::CA9);
    bucket_slot.assign(n, 0);
    ct_omega_z_radps.assign(n, 0.0);
  }

  inline void assert_sizes() const {
    assert(state.size() == static_cast<std::size_t>(StateComp::COUNT) * n);
    assert(model_id.size() == n);
    assert(bucket_slot.size() == n);
    assert(ct_omega_z_radps.size() == n);
  }

  // Raw pointers (for index builders / kernels)
  inline double* comp_ptr(StateComp c) {
    return state.data() + static_cast<std::size_t>(c) * n;
  }
  inline const double* comp_ptr(StateComp c) const {
    return state.data() + static_cast<std::size_t>(c) * n;
  }

  // Named pointer views
  inline double* x_ptr()  { return comp_ptr(StateComp::X);  }
  inline double* y_ptr()  { return comp_ptr(StateComp::Y);  }
  inline double* z_ptr()  { return comp_ptr(StateComp::Z);  }
  inline double* vx_ptr() { return comp_ptr(StateComp::VX); }
  inline double* vy_ptr() { return comp_ptr(StateComp::VY); }
  inline double* vz_ptr() { return comp_ptr(StateComp::VZ); }
  inline double* ax_ptr() { return comp_ptr(StateComp::AX); }
  inline double* ay_ptr() { return comp_ptr(StateComp::AY); }
  inline double* az_ptr() { return comp_ptr(StateComp::AZ); }

  inline const double* x_ptr()  const { return comp_ptr(StateComp::X);  }
  inline const double* y_ptr()  const { return comp_ptr(StateComp::Y);  }
  inline const double* z_ptr()  const { return comp_ptr(StateComp::Z);  }
  inline const double* vx_ptr() const { return comp_ptr(StateComp::VX); }
  inline const double* vy_ptr() const { return comp_ptr(StateComp::VY); }
  inline const double* vz_ptr() const { return comp_ptr(StateComp::VZ); }
  inline const double* ax_ptr() const { return comp_ptr(StateComp::AX); }
  inline const double* ay_ptr() const { return comp_ptr(StateComp::AY); }
  inline const double* az_ptr() const { return comp_ptr(StateComp::AZ); }

  // Named element accessors (minimal churn in model code)
  inline double& x(std::size_t i)  { return x_ptr()[i];  }
  inline double& y(std::size_t i)  { return y_ptr()[i];  }
  inline double& z(std::size_t i)  { return z_ptr()[i];  }
  inline double& vx(std::size_t i) { return vx_ptr()[i]; }
  inline double& vy(std::size_t i) { return vy_ptr()[i]; }
  inline double& vz(std::size_t i) { return vz_ptr()[i]; }
  inline double& ax(std::size_t i) { return ax_ptr()[i]; }
  inline double& ay(std::size_t i) { return ay_ptr()[i]; }
  inline double& az(std::size_t i) { return az_ptr()[i]; }

  inline const double& x(std::size_t i)  const { return x_ptr()[i];  }
  inline const double& y(std::size_t i)  const { return y_ptr()[i];  }
  inline const double& z(std::size_t i)  const { return z_ptr()[i];  }
  inline const double& vx(std::size_t i) const { return vx_ptr()[i]; }
  inline const double& vy(std::size_t i) const { return vy_ptr()[i]; }
  inline const double& vz(std::size_t i) const { return vz_ptr()[i]; }
  inline const double& ax(std::size_t i) const { return ax_ptr()[i]; }
  inline const double& ay(std::size_t i) const { return ay_ptr()[i]; }
  inline const double& az(std::size_t i) const { return az_ptr()[i]; }
};

} // namespace motion
