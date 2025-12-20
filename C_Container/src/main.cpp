/**
 * @file main.cpp
 * @brief Config-driven demo driver for batched covariance propagation (CA9 full 9x9).
 *
 * This integrates the config loader and uses the selected backend (Eigen/MKL/STD)
 * to apply the fast random-walk covariance update:
 *
 *   P_i <- P_i + (Q_per_sec * dt)
 *
 * Notes:
 *  - This is intentionally the "fast path" (F = I), which is what our Matrix Optimization work
 *    has focused on first. A full CA/CV FPF^T + Q path can be layered in later.
 *  - For now we intentionally allocate/store full 9x9 (81 doubles) per track.
 */

#include "la_batch_rw.h"

#include "config/config_loader.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string arg_value(int argc, char** argv, const std::string& key, const std::string& def) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == key && i + 1 < argc) {
      return std::string(argv[i + 1]);
    }
  }
  return def;
}

std::size_t arg_size_t(int argc, char** argv, const std::string& key, std::size_t def) {
  const std::string s = arg_value(argc, argv, key, "");
  if (s.empty()) return def;
  return static_cast<std::size_t>(std::stoull(s));
}

double checksum(const std::vector<double>& v) {
  // A stable-ish checksum to sanity-check backend parity.
  long double acc = 0.0L;
  for (double x : v) acc += static_cast<long double>(x);
  return static_cast<double>(acc);
}

} // namespace

int main(int argc, char** argv) try {
  const std::string system_xml = arg_value(argc, argv, "--config",  "./config/system.xml");
  const std::string xsd_dir    = arg_value(argc, argv, "--xsd-dir", "./schemas");

  // Load config (with schema validation)
  const cfg::ConfigBundle cfg = cfg::ConfigLoader::Load(system_xml, xsd_dir);

  const int n = cfg.tracker_model.state.dim;
  if (n != 9) {
    std::cerr << "ERROR: This integration step expects dim=9 for now (got dim=" << n << ").\n";
    return 2;
  }

  // Use runtime update rate as dt by default.
  if (cfg.runtime.update_rate_hz <= 0.0) {
    std::cerr << "ERROR: runtime.update_rate_hz must be > 0.\n";
    return 2;
  }
  const double dt_s = 1.0 / cfg.runtime.update_rate_hz;

  // Batch size: default to max active tracks, allow CLI override.
  const std::size_t batch = arg_size_t(argc, argv, "--tracks",
                                       static_cast<std::size_t>(std::max(0, cfg.runtime.max_tracks_active)));

  if (batch == 0) {
    std::cerr << "ERROR: batch size is 0.\n";
    return 2;
  }

  // Allocate full 9x9 P per track (row-major)
  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
  std::vector<double> P(batch * nn, 0.0);

  // Simple initial covariance: 100*I
  for (std::size_t i = 0; i < batch; ++i) {
    double* Pi = P.data() + i * nn;
    for (int d = 0; d < n; ++d) {
      Pi[d * n + d] = 100.0;
    }
  }

  // Define a simple Q_per_sec (row-major) consistent with a CA9 state ordering.
  // For now: diagonal with different scales for pos/vel/acc blocks.
  std::vector<double> Q(nn, 0.0);
  const double q_pos = 1.0;   // (m^2)/s
  const double q_vel = 0.1;   // (m/s)^2 / s
  const double q_acc = (cfg.tracker_model.process.noise.sigma_accel_mps2 > 0.0)
                         ? (cfg.tracker_model.process.noise.sigma_accel_mps2 *
                            cfg.tracker_model.process.noise.sigma_accel_mps2)
                         : 0.01; // (m/s^2)^2 / s

  for (int k = 0; k < 3; ++k) Q[(k+0) * n + (k+0)] = q_pos; // x,y,z
  for (int k = 0; k < 3; ++k) Q[(k+3) * n + (k+3)] = q_vel; // vx,vy,vz
  for (int k = 0; k < 3; ++k) Q[(k+6) * n + (k+6)] = q_acc; // ax,ay,az

  // Time the propagation
  const auto t0 = std::chrono::high_resolution_clock::now();
  la::rw_add_qdt_batch(P.data(), Q.data(), dt_s, n, batch);
  const auto t1 = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> dt = t1 - t0;

  std::cout << "Config: " << system_xml << "\n";
  std::cout << "XSD:    " << xsd_dir << "\n";
  std::cout << "dim=" << n << " tracks=" << batch << " dt_s=" << dt_s << "\n";
  std::cout << "RW batch propagation time: " << dt.count()
            << " s, checksum " << checksum(P) << "\n";

  return 0;
} catch (const std::exception& e) {
  std::cerr << "FATAL: " << e.what() << "\n";
  return 1;
}
