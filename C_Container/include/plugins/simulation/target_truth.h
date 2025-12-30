#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <cassert>

namespace sim {

// Authoritative truth for simulated targets (ECEF, CA9 kinematics).
// Storage can be SoA internally, but the name stays meaning-based.
struct TargetTruth
{
  std::size_t n = 0;

  // Optional: stable sim target ids
  std::vector<std::uint64_t> target_id;

  // Time bookkeeping (truth time)
  double t_truth_s = 0.0;

  // SoA kinematics (ECEF): each vector length n
  std::vector<double> x,  y,  z;
  std::vector<double> vx, vy, vz;
  std::vector<double> ax, ay, az;

  void resize(std::size_t n_targets)
  {
    n = n_targets;
    target_id.resize(n);

    x.resize(n);  y.resize(n);  z.resize(n);
    vx.resize(n); vy.resize(n); vz.resize(n);
    ax.resize(n); ay.resize(n); az.resize(n);
  }

  void assert_sizes() const
  {
    assert(target_id.size() == n);
    assert(x.size() == n && y.size() == n && z.size() == n);
    assert(vx.size() == n && vy.size() == n && vz.size() == n);
    assert(ax.size() == n && ay.size() == n && az.size() == n);
  }
};

} // namespace sim
