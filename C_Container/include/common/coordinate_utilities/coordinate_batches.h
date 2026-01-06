#pragma once

#include <cassert>
#include <cstddef>
#include <vector>

namespace coord {

struct EcefBatch {
  std::size_t n = 0;
  std::vector<double> x, y, z;

  void resize(std::size_t n_vals) {
    n = n_vals;
    x.resize(n);
    y.resize(n);
    z.resize(n);
  }

  void assert_sizes() const {
    assert(x.size() == n && y.size() == n && z.size() == n);
  }
};

struct AerBatch {
  std::size_t n = 0;
  std::vector<double> az_deg, el_deg, r_m;

  void resize(std::size_t n_vals) {
    n = n_vals;
    az_deg.resize(n);
    el_deg.resize(n);
    r_m.resize(n);
  }

  void assert_sizes() const {
    assert(az_deg.size() == n && el_deg.size() == n && r_m.size() == n);
  }
};

struct AerRateBatch {
  std::size_t n = 0;
  std::vector<double> r_dot_mps, az_rate_dps, el_rate_dps;

  void resize(std::size_t n_vals) {
    n = n_vals;
    r_dot_mps.resize(n);
    az_rate_dps.resize(n);
    el_rate_dps.resize(n);
  }

  void assert_sizes() const {
    assert(r_dot_mps.size() == n && az_rate_dps.size() == n && el_rate_dps.size() == n);
  }
};

} // namespace coord
