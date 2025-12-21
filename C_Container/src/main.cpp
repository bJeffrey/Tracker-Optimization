/**
 * @file main.cpp
 * @brief Config-driven demo driver for batched covariance propagation (CA9 full 9x9).
 *
 * This integrates the config loader and uses the selected backend (Eigen/MKL/STD)
 * to apply the fast random-walk covariance update:
 *
 *   P_i <- P_i + (Q_per_sec * dt)
 *
 * Step 2 integration:
 *  - Generate deterministic synthetic ECEF tracks from targets_gen_*.xml into TrackBatch.
 *  - Allow separating generation timing from propagation timing.
 */

#include "la_batch_rw.h"

#include "config/config_loader.h"
#include "targets/targets_generator.h"
#include "index/rtree_sqlite.h"
#include "index/scan_volume.h"
#include "track_batch.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
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

bool has_flag(int argc, char** argv, const std::string& key) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == key) return true;
  }
  return false;
}

std::size_t arg_size_t(int argc, char** argv, const std::string& key, std::size_t def) {
  const std::string s = arg_value(argc, argv, key, "");
  if (s.empty()) return def;
  return static_cast<std::size_t>(std::stoull(s));
}

double checksum(const std::vector<double>& v) {
  long double acc = 0.0L;
  for (double x : v) acc += static_cast<long double>(x);
  return static_cast<double>(acc);
}

} // namespace

int main(int argc, char** argv) try {
  const std::string system_xml = arg_value(argc, argv, "--config",  "./config/system.xml");
  const std::string xsd_dir    = arg_value(argc, argv, "--xsd-dir", "./schemas");

  const bool gen_only = has_flag(argc, argv, "--gen-only");
  const bool no_gen   = has_flag(argc, argv, "--no-gen");

  // Load config (with optional schema validation depending on xsd_dir)
  const cfg::ConfigBundle cfg = cfg::ConfigLoader::Load(system_xml, xsd_dir);

  const int n = cfg.tracker_model.state.dim;
  if (n != TrackBatch::kDim) {
    std::cerr << "ERROR: This demo expects dim=" << TrackBatch::kDim
              << " for now (got dim=" << n << ").\n";
    return 2;
  }

  // Use runtime update rate as dt by default.
  if (cfg.runtime.update_rate_hz <= 0.0) {
    std::cerr << "ERROR: runtime.update_rate_hz must be > 0.\n";
    return 2;
  }
  const double dt_s = 1.0 / cfg.runtime.update_rate_hz;

  // Batch size: default to max active tracks, allow CLI override.
  const std::size_t batch = arg_size_t(
      argc,
      argv,
      "--tracks",
      static_cast<std::size_t>(std::max(0, cfg.runtime.max_tracks_active)));

  if (batch == 0) {
    std::cerr << "ERROR: batch size is 0.\n";
    return 2;
  }

  TrackBatch tb;
  tb.resize(batch);

  // ------------------------------
  // Step 2: deterministic target generation (ECEF CA9)
  // ------------------------------
  double gen_s = 0.0;
  if (!no_gen) {
    const auto g0 = std::chrono::high_resolution_clock::now();
    targets::GenerateFromXml(cfg.paths.targets_xml, cfg.paths.xsd_dir, cfg.ownship, tb);
    const auto g1 = std::chrono::high_resolution_clock::now();
    gen_s = std::chrono::duration<double>(g1 - g0).count();
  }

  // Define a simple Q_per_sec (row-major) consistent with a CA9 state ordering.
  // For now: diagonal with different scales for pos/vel/acc blocks.
  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
  std::vector<double> Q(nn, 0.0);

  const double q_pos = 1.0;   // (m^2)/s
  const double q_vel = 0.1;   // (m/s)^2 / s
  const double q_acc =
      (cfg.tracker_model.process.noise.sigma_accel_mps2 > 0.0)
          ? (cfg.tracker_model.process.noise.sigma_accel_mps2 *
             cfg.tracker_model.process.noise.sigma_accel_mps2)
          : 0.01; // (m/s^2)^2 / s

  for (int k = 0; k < 3; ++k) Q[(k + 0) * n + (k + 0)] = q_pos; // x,y,z
  for (int k = 0; k < 3; ++k) Q[(k + 3) * n + (k + 3)] = q_vel; // vx,vy,vz
  for (int k = 0; k < 3; ++k) Q[(k + 6) * n + (k + 6)] = q_acc; // ax,ay,az

  // Propagate unless --gen-only
  double prop_s = 0.0;
  if (!gen_only) {
    const auto t0 = std::chrono::high_resolution_clock::now();
    la::rw_add_qdt_batch(tb.P.data(), Q.data(), dt_s, n, batch);
    const auto t1 = std::chrono::high_resolution_clock::now();
    prop_s = std::chrono::duration<double>(t1 - t0).count();
  }

  std::cout << "Config: " << system_xml << "\n";
  std::cout << "XSD:    " << xsd_dir << "\n";
  std::cout << "dim=" << n << " tracks=" << batch << " dt_s=" << dt_s << "\n";

  if (!no_gen) {
    std::cout << "Generation time: " << gen_s << " s (targets_xml=" << cfg.paths.targets_xml << ")\n";
  } else {
    std::cout << "Generation skipped (--no-gen)\n";
  }

  // ------------------------------------------------------------
  // Step 3 (scan-driven coarse query): build a fast 3D R-Tree index
  // of track positions (ECEF) and query by the sensor scan volume.
  //
  // Notes:
  // - Tracks are stored in ECEF.
  // - sensors.xml expresses the scan volume in a frame relative to ownship.
  //   For now we interpret OWNERSHIP_BODY as a local tangent ENU frame at the
  //   ownship location (no yaw/pitch/roll). We'll add full body attitude later.
  // - This is a *coarse* candidate query (AABB). Fine gating/association still
  //   happens downstream.
  // ------------------------------------------------------------
  if (cfg.has_sensors && !cfg.sensors.sensors.empty()) {
    const cfg::SensorCfg& sensor = cfg.sensors.sensors.front();

    idx::EcefAabb scan_aabb = idx::ComputeScanAabbEcefApprox(sensor, cfg.ownship);

    idx::SqliteRTreeIndex rtree;

    // TrackBatch stores interleaved state: [x y z vx vy vz ax ay az] per track
    rtree.BuildFromInterleavedXYZ(
        tb.x.data(),
        tb.n_tracks,
        static_cast<std::size_t>(TrackBatch::kDim),
        /*x_off=*/0,
        /*y_off=*/1,
        /*z_off=*/2);

    const auto ids = rtree.QueryAabb(scan_aabb);

    std::cout << "Scan-driven coarse query:\n";
    std::cout << "  sensor.id=" << sensor.id << " frame=" << sensor.scan.frame
              << " inflate_m=" << sensor.scan.query_aabb_inflate_m << "\n";
    std::cout << "  aabb.min=(" << scan_aabb.min_x << "," << scan_aabb.min_y << "," << scan_aabb.min_z << ")\n";
    std::cout << "  aabb.max=(" << scan_aabb.max_x << "," << scan_aabb.max_y << "," << scan_aabb.max_z << ")\n";
    std::cout << "  candidates=" << ids.size() << " / " << tb.n_tracks << "\n\n";
  } else {
    std::cout << "Scan-driven coarse query: skipped (no sensors loaded)\n\n";
  }

  if (!gen_only) {
    std::cout << "RW batch propagation time: " << prop_s
              << " s, checksum " << checksum(tb.P) << "\n";
  } else {
    std::cout << "Propagation skipped (--gen-only)\n";
  }

  return 0;

} catch (const std::exception& e) {
  std::cerr << "FATAL: " << e.what() << "\n";
  return 1;
}
