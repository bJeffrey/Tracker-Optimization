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
 *
 * Step 3 integration (scan-driven coarse query + motion loop scaffold):
 *  - Maintain SoA TrackBatchSoA + persistent motion buckets.
 *  - Run a dt-tick propagation loop and perform scan-tick coarse queries (AABB) via a pluggable
 *    spatial index backend (SQLite RTree or UniformGridIndex).
 */

#include "la_batch_rw.h"

#include "config/config_loader.h"
#include "targets/targets_generator.h"

// Motion folder
#include "motion/scenario_loop.hpp"

// Index + scan volume
#include "index/scan_volume.h"

// NEW: backend-agnostic spatial index + update policy
#include "index/index_update_manager.h"
#include "index/spatial_index_factory.h"
#include "index/spatial_index_3d.h"
#include "index/uniform_grid_index.h"

#include "track_batch.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <memory>
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

double arg_double(int argc, char** argv, const std::string& key, double def) {
  const std::string s = arg_value(argc, argv, key, "");
  if (s.empty()) return def;
  return std::stod(s);
}

double checksum(const std::vector<double>& v) {
  long double acc = 0.0L;
  for (double x : v) acc += static_cast<long double>(x);
  return static_cast<double>(acc);
}

// Deterministic "cheap RNG" inlined (no <random>) for initializing demo velocities/omegas.
inline double hash01(std::uint32_t x) {
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return (static_cast<double>(x) / static_cast<double>(0xFFFFFFFFu));
}

// Build SoA from your existing TrackBatch position columns.
inline void build_motion_soa_from_track_batch(const TrackBatch& tb_in, motion::TrackBatchSoA& tb_out) {
  tb_out.resize(tb_in.n_tracks);

  for (std::size_t i = 0; i < tb_in.n_tracks; ++i) {
    tb_out.x[i] = tb_in.pos_x[i];
    tb_out.y[i] = tb_in.pos_y[i];
    tb_out.z[i] = tb_in.pos_z[i];

    // deterministic pseudo-velocities (m/s), just to make things move
    const double u0 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 1u));
    const double u1 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 2u));
    const double u2 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 3u));
    tb_out.vx[i] = (u0 - 0.5) * 400.0;
    tb_out.vy[i] = (u1 - 0.5) * 400.0;
    tb_out.vz[i] = (u2 - 0.5) * 20.0;

    tb_out.ax[i] = 0.0;
    tb_out.ay[i] = 0.0;
    tb_out.az[i] = 0.0;

    // per-track omega_z (rad/s): make ~30% of tracks CT, rest CA
    const double u3 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 4u));
    tb_out.ct_omega_z_radps[i] = (u3 < 0.3) ? ((u3 - 0.15) * 0.08) : 0.0;

    tb_out.model_id[i] = motion::MotionModelId::CA9;
  }

  tb_out.sync_pos_from_state();
}

} // namespace

int main(int argc, char** argv) try {
  const std::string system_xml = arg_value(argc, argv, "--config",  "./config/system.xml");
  const std::string xsd_dir    = arg_value(argc, argv, "--xsd-dir", "./schemas");

  const bool gen_only = has_flag(argc, argv, "--gen-only");
  const bool no_gen   = has_flag(argc, argv, "--no-gen");

  // Motion/scan loop controls (CLI overrides)
  const double run_s      = arg_double(argc, argv, "--run-s",  3.0);
  const double scan_hz    = arg_double(argc, argv, "--scan-hz", 2.0);

  // Spatial index controls (CLI overrides; XML drives defaults)
  // NOTE: empty string means "no override"
  const std::string index_backend_cli = arg_value(argc, argv, "--index-backend", "");
  const double cell_m_cli             = arg_double(argc, argv, "--cell-m", 0.0);     // 0 => no override
  const double d_th_m_cli             = arg_double(argc, argv, "--dth-m", 0.0);      // 0 => no override
  const double t_max_s_cli            = arg_double(argc, argv, "--tmax-s", 0.0);     // 0 => no override

  // Load config (with optional schema validation depending on xsd_dir)
  const cfg::ConfigBundle cfg = cfg::ConfigLoader::Load(system_xml, xsd_dir);

  const int n = cfg.tracker_model.state.dim;
  if (n != TrackBatch::kDim) {
    std::cerr << "ERROR: This demo expects dim=" << TrackBatch::kDim
              << " for now (got dim=" << n << ").\n";
    return 2;
  }

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
  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
  std::vector<double> Q(nn, 0.0);

  const double q_pos = 1.0;
  const double q_vel = 0.1;
  const double q_acc =
      (cfg.tracker_model.process.noise.sigma_accel_mps2 > 0.0)
          ? (cfg.tracker_model.process.noise.sigma_accel_mps2 *
             cfg.tracker_model.process.noise.sigma_accel_mps2)
          : 0.01;

  for (int k = 0; k < 3; ++k) Q[(k + 0) * n + (k + 0)] = q_pos;
  for (int k = 0; k < 3; ++k) Q[(k + 3) * n + (k + 3)] = q_vel;
  for (int k = 0; k < 3; ++k) Q[(k + 6) * n + (k + 6)] = q_acc;

  // Propagate covariance unless --gen-only
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
  // Step 3A–3F: SoA motion loop + scan-tick coarse query via pluggable index backend.
  // ------------------------------------------------------------
  if (!gen_only && cfg.has_sensors && !cfg.sensors.sensors.empty()) {
    const cfg::SensorCfg& sensor = cfg.sensors.sensors.front();

    idx::EcefAabb scan_aabb = idx::ComputeScanAabbEcefApprox(sensor, cfg.ownship);

    // Build SoA from TrackBatch positions.
    motion::TrackBatchSoA tbm;
    build_motion_soa_from_track_batch(tb, tbm);

    // Persistent buckets + models.
    motion::ModelBuckets buckets;
    buckets.init_from_track_batch(tbm);

    motion::MotionModel_CA9 model_ca;
    motion::MotionModel_CT_XY_OmegaZ model_ct;
    motion::ModelSelector selector;

    motion::ScenarioConfig mcfg;
    mcfg.T_run_s = run_s;
    mcfg.dt_s = dt_s;
    mcfg.scan_rate_hz = (scan_hz > 0.0) ? scan_hz : 1.0;

    // ---- Create spatial index backend (XML default, CLI override)
    idx::SpatialIndexConfig icfg;

    const std::string backend_xml = sensor.scan.index.backend;
    icfg.backend = !index_backend_cli.empty() ? index_backend_cli : backend_xml;
    if (icfg.backend.empty()) {
      icfg.backend = "sqlite_rtree"; // final fallback
    }

    // Base value for derivations
    const double scan_inflate_m =
      (sensor.scan.query_aabb_inflate_m > 0.0) ? sensor.scan.query_aabb_inflate_m : 2000.0;

    // Cell size: CLI > XML > derived
    icfg.grid_cell_m =
      (cell_m_cli > 0.0) ? cell_m_cli :
      (sensor.scan.index.cell_m > 0.0 ? sensor.scan.index.cell_m : scan_inflate_m);

    // Move threshold: CLI > XML > derived
    const double d_th_m =
      (d_th_m_cli > 0.0) ? d_th_m_cli :
      (sensor.scan.index.d_th_m > 0.0 ? sensor.scan.index.d_th_m : (0.25 * icfg.grid_cell_m));

    // Max age: CLI > XML > derived (~2 scans)
    const double t_max_s =
      (t_max_s_cli > 0.0) ? t_max_s_cli :
      (sensor.scan.index.t_max_s > 0.0 ? sensor.scan.index.t_max_s : (2.0 / mcfg.scan_rate_hz));

    std::unique_ptr<idx::ISpatialIndex3D> index = idx::CreateSpatialIndex(icfg);
    index->Reserve(tbm.n);

    // Apply probe limit if backend supports it (UniformGridIndex)
    if (icfg.backend == "uniform_grid") {
      // Requires: #include "index/uniform_grid_index.h" at top of file
      if (auto* grid = dynamic_cast<idx::UniformGridIndex*>(index.get())) {
        grid->SetDenseCellProbeLimit(sensor.scan.index.dense_cell_probe_limit);
      }
    }


    idx::IndexUpdateManager upd;
    upd.Reset(tbm.n);
    upd.Configure(d_th_m, t_max_s);

    std::cout << "Scan-driven coarse query (loop):\n";
    std::cout << "  sensor.id=" << sensor.id << " frame=" << sensor.scan.frame
              << " inflate_m=" << sensor.scan.query_aabb_inflate_m << "\n";
    std::cout << "  aabb.min=(" << scan_aabb.min_x << "," << scan_aabb.min_y << "," << scan_aabb.min_z << ")\n";
    std::cout << "  aabb.max=(" << scan_aabb.max_x << "," << scan_aabb.max_y << "," << scan_aabb.max_z << ")\n";
    std::cout << "  run_s=" << mcfg.T_run_s << " dt_s=" << mcfg.dt_s << " scan_hz=" << mcfg.scan_rate_hz << "\n";
    std::cout << "  index.backend=" << icfg.backend
              << " supports_incremental=" << (index->SupportsIncrementalUpdates() ? "yes" : "no") << "\n";
    if (icfg.backend == "uniform_grid") {
      std::cout << "  grid.cell_m=" << icfg.grid_cell_m
                << " d_th_m=" << d_th_m
                << " t_max_s=" << t_max_s
                << " dense_cell_probe_limit=" << sensor.scan.index.dense_cell_probe_limit
                << "\n";
    } else {
      std::cout << "  d_th_m=" << d_th_m
                << " t_max_s=" << t_max_s
                << " (used by update manager; backend may rebuild)\n";
    }

    std::size_t scan_count = 0;
    const auto loop_t0 = std::chrono::high_resolution_clock::now();

    double last_cov_scan_s = 0.0;
    bool cov_inited = false;


    motion::run_scenario_loop(
        tbm, buckets, model_ca, model_ct, selector, mcfg,
        [&](const motion::TrackBatchSoA& tb_ref, double t_s) {
          ++scan_count;

          static double update_acc = 0.0, query_acc = 0.0;
          static double nupd_acc = 0.0;
          static std::size_t k_acc = 0;

          const auto t0 = std::chrono::high_resolution_clock::now();
          upd.Apply(t_s, tb_ref.pos_x.data(), tb_ref.pos_y.data(), tb_ref.pos_z.data(), *index);
          const auto t1 = std::chrono::high_resolution_clock::now();
          const auto ids = index->QueryAabb(scan_aabb);
          const auto t2 = std::chrono::high_resolution_clock::now();

          update_acc += std::chrono::duration<double>(t1 - t0).count();
          query_acc  += std::chrono::duration<double>(t2 - t1).count();
          nupd_acc   += static_cast<double>(upd.NumUpdatedLastApply());
          k_acc++;

          // --- Age covariance for candidate tracks to scan time (constant ΔT per scan) ---
          if (!cov_inited) {
            last_cov_scan_s = t_s;
            cov_inited = true;
          } else {
            const double dT = t_s - last_cov_scan_s;
            if (dT > 0.0 && !ids.empty()) {
              la::rw_add_qdt_subset(tb.P.data(), Q.data(), dT, n, ids.data(), ids.size());
            }
            last_cov_scan_s = t_s;
          }

          if (k_acc % 5 == 0) {
            std::cout << "  avg_update_s=" << (update_acc / k_acc)
                      << " avg_query_s="  << (query_acc / k_acc)
                      << " avg_n_updated=" << (nupd_acc / k_acc)
                      << "\n";
          }

          if (scan_count <= 3 || (scan_count % 10 == 0)) {
            std::cout << "  scan=" << scan_count
                      << " t_s=" << t_s
                      << " n_updated=" << upd.NumUpdatedLastApply()
                      << " candidates=" << ids.size() << " / " << tb_ref.n << "\n";
          }
        });

    const auto loop_t1 = std::chrono::high_resolution_clock::now();
    const double loop_s = std::chrono::duration<double>(loop_t1 - loop_t0).count();

    std::cout << "  loop_time_s=" << loop_s << " scans=" << scan_count << "\n\n";
  } else {
    std::cout << "Scan-driven coarse query (loop): skipped (gen-only or no sensors loaded)\n\n";
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
