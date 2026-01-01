/**
 * @file main.cpp
 * @brief Config-driven demo driver for batched covariance propagation (CA9 full 9x9).
 *
 * This integrates the config loader and uses the selected backend (Eigen/MKL/STD)
 * to apply the fast random-walk covariance update:
 *
 *   P_i <- P_i + (Q_per_sec * dt)
 *
 * Step 2 integration (truth/track separation):
 *  - Generate deterministic synthetic ECEF truth from targets_gen_*.xml into sim::TargetTruth.
 *  - Seed tracker trk::TrackBatch from that truth (cov/age/meta owned by tracker).
 *  - Allow separating generation timing from propagation timing.
 *
 * Step 3 integration (scan-driven coarse query + motion loop scaffold):
 *  - Maintain kinematics batch + persistent motion buckets.
 *  - Run a dt-tick propagation loop and perform scan-tick coarse queries (AABB) via a pluggable
 *    spatial index backend (SQLite RTree or UniformGridIndex).
 */

// Unified include surface
#include "tracker.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

static bool file_exists(const std::string& p) {
  std::error_code ec;
  return !p.empty() && fs::exists(fs::path(p), ec);
}

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

bool ensure_output_dir(const std::string& dir) {
  if (dir.empty()) return false;
  std::error_code ec;
  fs::create_directories(fs::path(dir), ec);
  return !ec;
}

std::tm local_tm_now() {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  return tm;
}

std::string format_tm(const std::tm& tm, const char* fmt) {
  std::ostringstream oss;
  oss << std::put_time(&tm, fmt);
  return oss.str();
}

std::string read_xml_version_attr(const std::string& path) {
  std::ifstream in(path);
  if (!in) return "";
  std::string line;
  while (std::getline(in, line)) {
    const std::string key = "version=";
    const std::size_t pos = line.find(key);
    if (pos == std::string::npos) continue;
    const std::size_t q1 = line.find('"', pos + key.size());
    if (q1 == std::string::npos) continue;
    const std::size_t q2 = line.find('"', q1 + 1);
    if (q2 == std::string::npos) continue;
    return line.substr(q1 + 1, q2 - q1 - 1);
  }
  return "";
}

// Deterministic "cheap RNG" inlined (no <random>) for initializing demo velocities/omegas.
inline double hash01(std::uint32_t x) {
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return (static_cast<double>(x) / static_cast<double>(0xFFFFFFFFu));
}

// Build kinematics batch from TrackBatch.
// (Positions come from TrackBatch hot columns; v/a are deterministically assigned for demo motion loop.)
inline void build_kinematics_from_track_batch(const trk::TrackBatch& tb_in,
                                              trk::TrackKinematicsBatch& tb_out)
{
  tb_out.resize(tb_in.n_tracks);

  for (std::size_t i = 0; i < tb_in.n_tracks; ++i) {
    // positions from TrackBatch
    tb_out.x(i) = tb_in.pos_x[i];
    tb_out.y(i) = tb_in.pos_y[i];
    tb_out.z(i) = tb_in.pos_z[i];

    // deterministic pseudo-velocities (m/s)
    const double u0 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 1u));
    const double u1 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 2u));
    const double u2 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 3u));
    tb_out.vx(i) = (u0 - 0.5) * 400.0;
    tb_out.vy(i) = (u1 - 0.5) * 400.0;
    tb_out.vz(i) = (u2 - 0.5) * 20.0;

    // accelerations default to 0
    tb_out.ax(i) = 0.0;
    tb_out.ay(i) = 0.0;
    tb_out.az(i) = 0.0;

    // ~30% CT, rest CA (omega_z in rad/s)
    const double u3 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 4u));
    tb_out.ct_omega_z_radps[i] = (u3 < 0.3) ? ((u3 - 0.15) * 0.08) : 0.0;

    // Start all as CA; selector will move-to-CT based on omega.
    tb_out.model_id[i] = trk::MotionModelId::CA9;
  }
}

} // namespace

int main(int argc, char** argv) try {
  const std::string system_xml = arg_value(argc, argv, "--config",  "./config/system.xml");
  const std::string xsd_dir    = arg_value(argc, argv, "--xsd-dir", "./schemas");

  // IMPORTANT: do NOT default targets_xml from a hard-coded path.
  // Let system.xml -> scenario -> targets drive it, unless CLI overrides.
  const std::string targets_xml_cli = arg_value(argc, argv, "--targets-xml", "");

  const bool gen_only = has_flag(argc, argv, "--gen-only");
  const bool no_gen   = has_flag(argc, argv, "--no-gen");

  // Motion/scan loop controls (CLI overrides)
  const double run_s   = arg_double(argc, argv, "--run-s",   3.0);
  const double scan_hz = arg_double(argc, argv, "--scan-hz", 2.0);

  // Spatial index controls (CLI overrides; XML drives defaults)
  const std::string index_backend_cli = arg_value(argc, argv, "--index-backend", "");
  const double cell_m_cli             = arg_double(argc, argv, "--cell-m", 0.0); // 0 => no override
  const double d_th_m_cli             = arg_double(argc, argv, "--dth-m",  0.0); // 0 => no override
  const double t_max_s_cli            = arg_double(argc, argv, "--tmax-s", 0.0); // 0 => no override

  // Output controls
  const std::string output_dir = arg_value(argc, argv, "--output-dir", "./output");

  // printing
  const bool verbose = has_flag(argc, argv, "--verbose");

  // Load config (with optional schema validation depending on xsd_dir)
  const cfg::ConfigBundle cfg = cfg::ConfigLoader::Load(system_xml, xsd_dir);

  // Resolve targets generator XML:
  //  1) CLI override if provided
  //  2) scenario targets from system.xml -> scenario catalog if present
  //  3) fallback hard-coded path ONLY if missing/unloadable
  std::string targets_xml =
    !targets_xml_cli.empty() ? targets_xml_cli : cfg.paths.targets_xml;

  if (!file_exists(targets_xml)) {
    const std::string fallback = "./config/targets/targets_gen_1m_ca9.xml";
    if (file_exists(fallback)) {
      if (verbose) {
        std::cerr << "WARNING: targets_xml not found ('" << targets_xml
                  << "'), falling back to '" << fallback << "'.\n";
      }
      targets_xml = fallback;
    } else {
      throw std::runtime_error(
        "Targets XML not found. From config/CLI: '" + targets_xml +
        "'. Fallback also missing: '" + fallback + "'.");
    }
  }

  const int n = cfg.tracker_model.state.dim;
  if (n != trk::TrackBatch::kDim) {
    std::cerr << "ERROR: This demo expects dim=" << trk::TrackBatch::kDim
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

  // Tracker storage container (estimate-side)
  trk::TrackBatch tb;
  tb.resize(batch);

  // ------------------------------
  // Step 2: deterministic generation (truth -> tracker seeding)
  // ------------------------------
  if (!no_gen) {
    sim::TargetTruth truth;
    truth.resize(batch);

    // sim: truth-only
    sim::GenerateTruthFromXml(targets_xml, xsd_dir, cfg.ownship, truth);

    // trk: explicit bridge (cov init + stale age + meta)
    trk::SeedTracksFromTruthXml(targets_xml, xsd_dir, cfg.ownship, truth, tb);
  } else {
    // Fallback: simple initialization if generation is disabled.
    tb.set_cov_identity_scaled(1.0);
    std::fill(tb.last_update_s.begin(), tb.last_update_s.end(), 0.0);
    std::fill(tb.last_cov_prop_s.begin(), tb.last_cov_prop_s.end(), 0.0);

    for (std::size_t i = 0; i < tb.n_tracks; ++i) {
      tb.track_id[i] = static_cast<std::uint64_t>(i + 1);
      tb.status[i]   = TrackStatus::ACTIVE;
      tb.quality[i]  = 1.0f;
      tb.pos_x[i]    = 0.0;
      tb.pos_y[i]    = 0.0;
      tb.pos_z[i]    = 0.0;
      double* xi = tb.x_ptr(i);
      std::fill(xi, xi + trk::TrackBatch::kDim, 0.0);
    }
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

  std::cout << "Config:   " << system_xml << "\n";
  std::cout << "XSD:      " << xsd_dir << "\n";
  std::cout << "Targets:  " << targets_xml << (no_gen ? " (skipped)" : "") << "\n";
  std::cout << "dim=" << n << " tracks=" << batch << " dt_s=" << dt_s << "\n";

  // Covariance aging stats (scan-driven), printed once at end.
  double cov_age_acc_s = 0.0;
  std::size_t cov_age_calls = 0;
  std::size_t cov_age_tracks_acc = 0;

  // Scan loop stats (optional)
  std::size_t scan_count = 0;
  double loop_s = 0.0;
  double update_acc = 0.0;
  double query_acc = 0.0;
  double nupd_acc = 0.0;
  std::size_t k_acc = 0;

  // ------------------------------------------------------------
  // Step 3A-3F: Motion loop + scan-tick coarse query via pluggable index backend.
  // ------------------------------------------------------------
  if (!gen_only && cfg.has_sensors && !cfg.sensors.sensors.empty()) {
    const cfg::SensorCfg& sensor = cfg.sensors.sensors.front();

    idx::EcefAabb scan_aabb = idx::ComputeScanAabbEcefApprox(sensor, cfg.ownship);

    // Build kinematics batch from TrackBatch positions.
    trk::TrackKinematicsBatch tbm;
    build_kinematics_from_track_batch(tb, tbm);

    // Persistent buckets + models.
    trk::ModelBuckets buckets;
    buckets.init_from_track_batch(tbm);

    trk::MotionModel_CA9 model_ca;
    trk::MotionModel_CT_XY_OmegaZ model_ct;
    trk::ModelSelector selector;

    trk::ScenarioConfig mcfg;
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

    const auto loop_t0 = std::chrono::high_resolution_clock::now();

    trk::run_scenario_loop(
      tbm, buckets, model_ca, model_ct, selector, mcfg,
      [&](trk::TrackKinematicsBatch& tb_ref, double t_s) {
        ++scan_count;

        const auto t0 = std::chrono::high_resolution_clock::now();
        upd.Apply(t_s, tb_ref.x_ptr(), tb_ref.y_ptr(), tb_ref.z_ptr(), *index);
        const auto t1 = std::chrono::high_resolution_clock::now();
        auto ids = index->QueryAabb(scan_aabb);
        const auto t2 = std::chrono::high_resolution_clock::now();

        update_acc += std::chrono::duration<double>(t1 - t0).count();
        query_acc  += std::chrono::duration<double>(t2 - t1).count();
        nupd_acc   += static_cast<double>(upd.NumUpdatedLastApply());
        ++k_acc;

        // --- Scan-driven covariance aging (lazy): age only candidates to scan time t_s ---
        // Use TrackBatch::last_cov_prop_s as t_pred_s (P-aged-to time) per-track.
        if (!ids.empty()) {
          static std::vector<double> dt_ids;
          if (dt_ids.capacity() < ids.size()) dt_ids.reserve(ids.size());
          dt_ids.resize(ids.size());

          for (std::size_t k = 0; k < ids.size(); ++k) {
            const std::size_t i = static_cast<std::size_t>(ids[k]);

            const double dt_i = t_s - tb.last_cov_prop_s[i];
            dt_ids[k] = (dt_i > 0.0) ? dt_i : 0.0;

            // Mark covariance as aged-to scan time (scan times should be monotonic)
            tb.last_cov_prop_s[i] = t_s;
          }

          const auto tc0 = std::chrono::high_resolution_clock::now();
          la::rw_add_qdt_subset_var_dt(tb.P.data(), Q.data(), n,
                                       ids.data(), dt_ids.data(), ids.size());
          const auto tc1 = std::chrono::high_resolution_clock::now();

          cov_age_acc_s += std::chrono::duration<double>(tc1 - tc0).count();
          ++cov_age_calls;
          cov_age_tracks_acc += ids.size();
        }

        if (verbose && (k_acc % 5 == 0)) {
          std::cout << "  avg_update_s=" << (update_acc / k_acc)
                    << " avg_query_s="  << (query_acc / k_acc)
                    << " avg_n_updated=" << (nupd_acc / k_acc)
                    << "\n";
        }

        if (verbose && (scan_count <= 3 || (scan_count % 10 == 0))) {
          std::cout << "  scan=" << scan_count
                    << " t_s=" << t_s
                    << " n_updated=" << upd.NumUpdatedLastApply()
                    << " candidates=" << ids.size() << " / " << tb_ref.n << "\n";
        }
      });

    const auto loop_t1 = std::chrono::high_resolution_clock::now();
    loop_s = std::chrono::duration<double>(loop_t1 - loop_t0).count();

    std::cout << "  loop_time_s=" << loop_s << " scans=" << scan_count << "\n\n";
  } else {
    std::cout << "Scan-driven coarse query (loop): skipped (gen-only or no sensors loaded)\n\n";
  }

  if (!gen_only) {
    std::cout << "Covariance aging (scan-driven): "
              << "calls=" << cov_age_calls
              << " avg_tracks_per_call=" << (cov_age_calls ? (static_cast<double>(cov_age_tracks_acc) / cov_age_calls) : 0.0)
              << " total_s=" << cov_age_acc_s
              << " avg_s_per_call=" << (cov_age_calls ? (cov_age_acc_s / cov_age_calls) : 0.0)
              << " checksum " << checksum(tb.P) << "\n";
  } else {
    std::cout << "Propagation skipped (--gen-only)\n";
  }

  // Write performance metrics to CSV (single-row, time-stamped per run)
  if (ensure_output_dir(output_dir)) {
    const std::tm run_tm = local_tm_now();
    const std::string date_str = format_tm(run_tm, "%Y-%m-%d");
    const std::string time_str = format_tm(run_tm, "%H:%M:%S");
    const std::string version_str = read_xml_version_attr(system_xml);
    const std::string version = version_str.empty() ? "unknown" : version_str;

    const std::string csv_path =
      (fs::path(output_dir) / ("performance_" + format_tm(run_tm, "%Y%m%d_%H%M%S") + ".csv")).string();
    std::ofstream out(csv_path, std::ios::trunc);
    if (out) {
      const double avg_update_s = (k_acc ? (update_acc / k_acc) : 0.0);
      const double avg_query_s = (k_acc ? (query_acc / k_acc) : 0.0);
      const double avg_n_updated = (k_acc ? (nupd_acc / k_acc) : 0.0);
      const double avg_tracks_per_call =
          (cov_age_calls ? (static_cast<double>(cov_age_tracks_acc) / cov_age_calls) : 0.0);
      const double avg_s_per_call = (cov_age_calls ? (cov_age_acc_s / cov_age_calls) : 0.0);

      auto write_header = [](std::ostream& os) {
        os
          << "date,time,version,system_xml,xsd_dir,targets_xml,dim,tracks,dt_s,run_s,scan_hz,"
             "scan_count,loop_time_s,avg_update_s,avg_query_s,avg_n_updated,"
             "cov_age_calls,avg_tracks_per_call,cov_age_total_s,cov_age_avg_s,checksum\n";
      };
      auto write_row = [&](std::ostream& os) {
        os
          << date_str << ','
          << time_str << ','
          << version << ','
          << system_xml << ','
          << xsd_dir << ','
          << targets_xml << ','
          << n << ','
          << batch << ','
          << dt_s << ','
          << run_s << ','
          << scan_hz << ','
          << scan_count << ','
          << loop_s << ','
          << avg_update_s << ','
          << avg_query_s << ','
          << avg_n_updated << ','
          << cov_age_calls << ','
          << avg_tracks_per_call << ','
          << cov_age_acc_s << ','
          << avg_s_per_call << ','
          << checksum(tb.P)
          << "\n";
      };

      write_header(out);
      write_row(out);

      const fs::path append_path = fs::path(output_dir) / "performance.csv";
      std::error_code ec;
      bool has_header = false;
      if (fs::exists(append_path, ec) && !ec) {
        const auto sz = fs::file_size(append_path, ec);
        has_header = (!ec && sz > 0);
      }
      std::ofstream append_out(append_path, std::ios::app);
      if (append_out) {
        if (!has_header) {
          write_header(append_out);
        }
        write_row(append_out);
      } else if (verbose) {
        std::cerr << "WARNING: failed to open performance CSV at '" << append_path.string() << "'.\n";
      }
    } else if (verbose) {
      std::cerr << "WARNING: failed to open performance CSV at '" << csv_path << "'.\n";
    }
  } else if (verbose) {
    std::cerr << "WARNING: failed to create output directory '" << output_dir << "'.\n";
  }

  return 0;

} catch (const std::exception& e) {
  std::cerr << "FATAL: " << e.what() << "\n";
  return 1;
}
