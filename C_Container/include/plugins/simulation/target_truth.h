#pragma once
#include "common/coordinate_utilities/coordinate_batches.h"

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

  // SoA kinematics (ECEF)
  coord::EcefBatch pos;
  coord::EcefBatch vel;
  coord::EcefBatch acc;

  void resize(std::size_t n_targets)
  {
    n = n_targets;
    target_id.resize(n);

    pos.resize(n);
    vel.resize(n);
    acc.resize(n);
  }

  void assert_sizes() const
  {
    assert(target_id.size() == n);
    pos.assert_sizes();
    vel.assert_sizes();
    acc.assert_sizes();
  }
};

} // namespace sim
