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
#include "pipeline/demo_pipeline.h"
#include "tracker.h"
#include "plugins/database/course_gates.h"
#include "common/coordinate_utilities/coordinate_transform.h"
#include "plugins/measurement/IMeasurementModel.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

/**
 * @brief Check if a filesystem path exists (no exceptions).
 */
static bool file_exists(const std::string& p) {
  std::error_code ec;
  return !p.empty() && fs::exists(fs::path(p), ec);
}

namespace {

// ------------------------------
// CLI parsing
// ------------------------------
/**
 * @brief Parsed command-line arguments and overrides.
 */
struct CliArgs {
  std::string system_xml;
  std::string xsd_dir;
  std::string targets_xml_cli;
  bool gen_only = false;
  bool no_gen = false;
  double run_s = 3.0;
  double scan_hz = 2.0;
  std::string index_backend_cli;
  double cell_m_cli = 0.0;
  double d_th_m_cli = 0.0;
  double t_max_s_cli = 0.0;
  std::string output_dir;
  std::size_t tracks_cli = 0;
  std::size_t warm_commit_every_scans = 0;
};

// ------------------------------
// Runtime state + stats
// ------------------------------
/**
 * @brief Runtime accumulators for scan loop and covariance aging stats.
 */
struct RunStats {
  double cov_age_acc_s = 0.0;
  std::size_t cov_age_calls = 0;
  std::size_t cov_age_tracks_acc = 0;

  std::size_t scan_count = 0;
  double loop_s = 0.0;
  double update_acc = 0.0;
  double query_acc = 0.0;
  double nupd_acc = 0.0;
  std::size_t k_acc = 0;
  double finalize_acc_s = 0.0;
  std::size_t finalize_calls = 0;

  double prefetch_acc_s = 0.0;
  std::size_t prefetch_calls = 0;
  double finalize_begin_acc_s = 0.0;
  double finalize_apply_acc_s = 0.0;
  double finalize_commit_acc_s = 0.0;
};

/**
 * @brief Scan-loop components and configuration resolved from config/CLI.
 */
struct ScanContext {
  const cfg::SensorCfg* sensor = nullptr;
  idx::EcefAabb scan_aabb{};
  db::CourseGateSet course_gates{};
  struct EnuAabb {
    double min_e = 0.0;
    double max_e = 0.0;
    double min_n = 0.0;
    double max_n = 0.0;
    double min_u = 0.0;
    double max_u = 0.0;
  };
  std::vector<EnuAabb> course_gates_enu{};
  struct GateGrid {
    bool enabled = false;
    double origin_e = 0.0;
    double origin_n = 0.0;
    double step_e = 0.0;
    double step_n = 0.0;
    double radius_m = 0.0;
    std::size_t count_e = 0;
    std::size_t count_n = 0;
    std::vector<int> map; // index -> gate_index, -1 if no gate
  };
  GateGrid gate_grid{};
  idx::SpatialIndexConfig icfg{};
  std::unique_ptr<db::ITrackDatabase> track_db;
  trk::IdList hot_ids;
  double hot_prefetch_min_s = 0.0;
  double hot_prefetch_window_s = 0.0;
  double last_prefetch_t_s = 0.0;
  bool hot_prefetch_cadence_enabled = false;
  trk::TrackKinematicsBatch tbm;
  trk::ModelBuckets buckets;
  trk::MotionModel_CA9 model_ca;
  trk::MotionModel_CT_XY_OmegaZ model_ct;
  trk::ModelSelector selector;
  trk::ScenarioConfig mcfg{};
  double scan_inflate_m = 0.0;
  double d_th_m = 0.0;
  double t_max_s = 0.0;
};

/**
 * @brief Aggregated runtime state shared across pipeline stages.
 */
struct PipelineState {
  const cfg::ConfigBundle* cfg = nullptr;
  CliArgs cli{};
  std::string targets_xml;
  int n = 0;
  double dt_s = 0.0;
  std::size_t batch = 0;
  sim::TargetTruth truth;
  bool has_truth = false;
  trk::TrackBatch tb;
  std::vector<double> Q;
  RunStats stats;
  ScanContext scan;
  bool debug_timing = false;
  trk::IdList changed_ids;
  trk::IdList all_ids;
  meas::MeasurementBatch meas_batch;
};

enum class LogLevel : int {
  TRACE = 0,
  DEBUG = 1,
  INFO  = 2,
  WARN  = 3,
  ERROR = 4
};

std::string to_upper(std::string s) {
  for (char& c : s) {
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  }
  return s;
}

LogLevel parse_log_level() {
  const char* level = std::getenv("TRACKER_LOG_LEVEL");
  if (!level || !*level) return LogLevel::INFO;
  const std::string v = to_upper(level);
  if (v == "TRACE") return LogLevel::TRACE;
  if (v == "DEBUG") return LogLevel::DEBUG;
  if (v == "INFO")  return LogLevel::INFO;
  if (v == "WARN")  return LogLevel::WARN;
  if (v == "ERROR") return LogLevel::ERROR;
  return LogLevel::INFO;
}

LogLevel current_log_level() {
  static LogLevel level = parse_log_level();
  return level;
}

bool should_log(LogLevel level) {
  return static_cast<int>(level) >= static_cast<int>(current_log_level());
}

/**
 * @brief Return the value that follows a CLI flag.
 */
std::string arg_value(int argc, char** argv, const std::string& key, const std::string& def) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == key && i + 1 < argc) {
      return std::string(argv[i + 1]);
    }
  }
  return def;
}

/**
 * @brief Check if a CLI flag is present.
 */
bool has_flag(int argc, char** argv, const std::string& key) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == key) return true;
  }
  return false;
}

/**
 * @brief Parse a size_t CLI argument with default.
 */
std::size_t arg_size_t(int argc, char** argv, const std::string& key, std::size_t def) {
  const std::string s = arg_value(argc, argv, key, "");
  if (s.empty()) return def;
  return static_cast<std::size_t>(std::stoull(s));
}

/**
 * @brief Parse a double CLI argument with default.
 */
double arg_double(int argc, char** argv, const std::string& key, double def) {
  const std::string s = arg_value(argc, argv, key, "");
  if (s.empty()) return def;
  return std::stod(s);
}

/**
 * @brief Parse CLI arguments into a structured bundle.
 */
CliArgs parse_cli(int argc, char** argv) {
  CliArgs cli;
  cli.system_xml        = arg_value(argc, argv, "--config",  "./config/system.xml");
  cli.xsd_dir           = arg_value(argc, argv, "--xsd-dir", "./schemas");
  cli.targets_xml_cli   = arg_value(argc, argv, "--targets-xml", "");
  cli.gen_only          = has_flag(argc, argv, "--gen-only");
  cli.no_gen            = has_flag(argc, argv, "--no-gen");
  cli.run_s             = arg_double(argc, argv, "--run-s", 3.0);
  cli.scan_hz           = arg_double(argc, argv, "--scan-hz", 2.0);
  cli.index_backend_cli = arg_value(argc, argv, "--index-backend", "");
  cli.cell_m_cli        = arg_double(argc, argv, "--cell-m", 0.0);
  cli.d_th_m_cli        = arg_double(argc, argv, "--dth-m", 0.0);
  cli.t_max_s_cli       = arg_double(argc, argv, "--tmax-s", 0.0);
  cli.output_dir        = arg_value(argc, argv, "--output-dir", "./logs");
  cli.tracks_cli        = arg_size_t(argc, argv, "--tracks", 0);
  cli.warm_commit_every_scans = arg_size_t(argc, argv, "--warm-commit-every", 0);
  return cli;
}

/**
 * @brief Simple checksum of a vector (order-dependent).
 */
double checksum(const std::vector<double>& v) {
  long double acc = 0.0L;
  for (double x : v) acc += static_cast<long double>(x);
  return static_cast<double>(acc);
}

/**
 * @brief Ensure an output directory exists (creates if needed).
 */
bool ensure_output_dir(const std::string& dir) {
  if (dir.empty()) return false;
  std::error_code ec;
  fs::create_directories(fs::path(dir), ec);
  return !ec;
}

/**
 * @brief Get local time as std::tm.
 */
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

/**
 * @brief Format a std::tm with a strftime-compatible format string.
 */
std::string format_tm(const std::tm& tm, const char* fmt) {
  std::ostringstream oss;
  oss << std::put_time(&tm, fmt);
  return oss.str();
}

std::uint64_t fnv1a64(const std::string& s) {
  const std::uint64_t kOffset = 1469598103934665603ull;
  const std::uint64_t kPrime = 1099511628211ull;
  std::uint64_t h = kOffset;
  for (unsigned char c : s) {
    h ^= static_cast<std::uint64_t>(c);
    h *= kPrime;
  }
  return h;
}

std::string hex_u64(std::uint64_t v) {
  std::ostringstream oss;
  oss << std::hex << std::nouppercase << v;
  return oss.str();
}

std::string build_warm_signature(const PipelineState& state) {
  std::ostringstream oss;
  oss << "version=" << TRACKER_VERSION_STRING << "\n";
  oss << "system_xml=" << state.cli.system_xml << "\n";
  oss << "targets_xml=" << state.targets_xml << "\n";
  oss << "scenario_id=" << state.cfg->scenario.id << "\n";
  oss << "targets_seed=" << state.cfg->targets_gen.seed << "\n";
  oss << "targets_count=" << state.cfg->targets_gen.count << "\n";
  oss << "state_dim=" << state.cfg->tracker_model.state.dim << "\n";
  oss << "index_backend=" << state.cfg->store_profile.index.backend << "\n";
  oss << "index_cell_m=" << state.cfg->store_profile.index.cell_m << "\n";
  oss << "index_d_th_m=" << state.cfg->store_profile.index.d_th_m << "\n";
  oss << "index_t_max_s=" << state.cfg->store_profile.index.t_max_s << "\n";
  return hex_u64(fnv1a64(oss.str()));
}

// ------------------------------
// Config + init helpers
// ------------------------------
/**
 * @brief Resolve targets XML using CLI override, config path, and fallback.
 */
std::string resolve_targets_xml(const std::string& cli_value,
                                const cfg::ResolvedPaths& paths,
                                LogLevel log_level) {
  std::string targets_xml = !cli_value.empty() ? cli_value : paths.targets_xml;
  if (!file_exists(targets_xml)) {
    const std::string fallback = "./config/targets/targets_gen_1m_ca9.xml";
    if (file_exists(fallback)) {
      if (log_level <= LogLevel::WARN && should_log(LogLevel::WARN)) {
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
  return targets_xml;
}

/**
 * @brief Compute effective batch size from CLI/config.
 */
std::size_t compute_batch_size(const CliArgs& cli, const cfg::ConfigBundle& cfg) {
  if (cli.tracks_cli > 0) return cli.tracks_cli;
  return static_cast<std::size_t>(std::max(0, cfg.runtime.max_tracks_active));
}

/**
 * @brief Build a simple diagonal process noise matrix (row-major).
 */
std::vector<double> build_process_noise_q(const cfg::ConfigBundle& cfg, int n) {
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

  return Q;
}

/**
 * @brief Seed tracks from truth (or a fallback init if generation is disabled).
 */
void seed_tracks(PipelineState& state) {
  if (!state.cli.no_gen) {
    state.truth.resize(state.batch);

    // sim: truth-only
    sim::GenerateTruthFromXml(state.targets_xml,
                              state.cli.xsd_dir,
                              state.cfg->ownship,
                              state.truth);

    // trk: explicit bridge (cov init + stale age + meta)
    trk::SeedTracksFromTruthXml(state.targets_xml,
                                state.cli.xsd_dir,
                                state.cfg->ownship,
                                state.truth,
                                state.tb);
    state.has_truth = true;
  } else {
    // Fallback: simple initialization if generation is disabled.
    state.tb.set_cov_identity_scaled(1.0);
    std::fill(state.tb.last_update_s.begin(), state.tb.last_update_s.end(), 0.0);
    std::fill(state.tb.last_cov_prop_s.begin(), state.tb.last_cov_prop_s.end(), 0.0);

    for (std::size_t i = 0; i < state.tb.n_tracks; ++i) {
      state.tb.track_id[i] = static_cast<std::uint64_t>(i + 1);
      state.tb.status[i]   = TrackStatus::ACTIVE;
      state.tb.quality[i]  = 1.0f;
      state.tb.pos_x[i]    = 0.0;
      state.tb.pos_y[i]    = 0.0;
      state.tb.pos_z[i]    = 0.0;
      double* xi = state.tb.x_ptr(i);
      std::fill(xi, xi + trk::TrackBatch::kDim, 0.0);
    }
    state.has_truth = false;
  }
}

/**
 * @brief Deterministic "cheap RNG" for initializing demo velocities/omegas.
 */
inline double hash01(std::uint32_t x) {
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return (static_cast<double>(x) / static_cast<double>(0xFFFFFFFFu));
}

/**
 * @brief Build kinematics batch from TrackBatch positions (with demo velocities/omegas).
 */
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
    tb_out.vx(i)    = (u0 - 0.5) * 400.0;
    tb_out.vy(i)    = (u1 - 0.5) * 400.0;
    tb_out.vz(i)    = (u2 - 0.5) * 20.0;

    // deterministic pseudo-accelerations (m/s^2)
    const double u4 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 5u));
    const double u5 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 6u));
    const double u6 = hash01(static_cast<std::uint32_t>(i * 2654435761u + 7u));
    tb_out.ax(i) = (u4 - 0.5) * 2.0;
    tb_out.ay(i) = (u5 - 0.5) * 2.0;
    tb_out.az(i) = (u6 - 0.5) * 0.2;

    // ~30% CT, rest CA (omega_z in rad/s)
    const double u3             = hash01(static_cast<std::uint32_t>(i * 2654435761u + 4u));
    tb_out.ct_omega_z_radps[i]  = (u3 < 0.3) ? ((u3 - 0.15) * 0.08) : 0.0;

    // Start all as CA; selector will move-to-CT based on omega.
    tb_out.model_id[i] = trk::MotionModelId::CA9;
  }
}

// ------------------------------
// Pipeline stage stubs
// ------------------------------
/**
 * @brief Placeholder for measurement generation stage.
 */
void stage_measurement_generation(PipelineState& state, double t_s) {
  if (!state.has_truth || !state.cfg->has_sensors || state.cfg->sensors.sensors.empty()) {
    return;
  }
  const cfg::SensorCfg& sensor = state.cfg->sensors.sensors.front();
  if (!sensor.meas_model.has_radar ||
      sensor.meas_model.radar.type == cfg::RadarMeasurementType::UNKNOWN) {
    return;
  }

  coord::AerRadBatch aer;
  coord::EcefToAerBatch(state.truth.pos, state.cfg->ownship, aer);

  meas::MeasurementBatch batch;
  batch.meta.sensor_id = sensor.id;
  batch.meta.frame = sensor.scan.frame;
  batch.meta.timestamp_s = t_s;
  batch.meta.ownship_x_ecef = state.cfg->ownship.pos_x;
  batch.meta.ownship_y_ecef = state.cfg->ownship.pos_y;
  batch.meta.ownship_z_ecef = state.cfg->ownship.pos_z;
  batch.type = sensor.meas_model.radar.type;
  batch.measurements.clear();
  batch.measurements.reserve(state.truth.n + sensor.meas_model.radar.false_alarm_count);

  const double snr_lin = std::pow(10.0, 0.1 * sensor.meas_model.radar.ref_snr_db);
  const double bw_az_rad = (sensor.beamwidth_3db_az_deg > 0.0)
                             ? (sensor.beamwidth_3db_az_deg * 3.14159265358979323846 / 180.0)
                             : 0.0;
  const double bw_el_rad = (sensor.beamwidth_3db_el_deg > 0.0)
                             ? (sensor.beamwidth_3db_el_deg * 3.14159265358979323846 / 180.0)
                             : 0.0;
  const double sigma_r = (snr_lin > 0.0)
                           ? (sensor.meas_model.radar.ref_range_m / snr_lin)
                           : 0.0;
  const double sigma_az = (snr_lin > 0.0) ? (bw_az_rad / snr_lin) : 0.0;
  const double sigma_el = (snr_lin > 0.0) ? (bw_el_rad / snr_lin) : 0.0;

  static std::mt19937 rng(1337u);
  std::normal_distribution<double> nr(0.0, sigma_r);
  std::normal_distribution<double> naz(0.0, sigma_az);
  std::normal_distribution<double> nel(0.0, sigma_el);
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  const double det_p = std::min(1.0, std::max(0.0, sensor.meas_model.radar.detection_prob));
  const double az_min = sensor.scan.frustum.az_min_deg * 3.14159265358979323846 / 180.0;
  const double az_max = sensor.scan.frustum.az_max_deg * 3.14159265358979323846 / 180.0;
  const double el_min = sensor.scan.frustum.el_min_deg * 3.14159265358979323846 / 180.0;
  const double el_max = sensor.scan.frustum.el_max_deg * 3.14159265358979323846 / 180.0;
  const double r_min = sensor.scan.frustum.r_min_m;
  const double r_max = sensor.scan.frustum.r_max_m;
  std::uniform_real_distribution<double> uni_az(az_min, az_max);
  std::uniform_real_distribution<double> uni_el(el_min, el_max);
  std::uniform_real_distribution<double> uni_r(r_min, r_max);

  std::uint64_t meas_seq = 0;
  for (std::size_t i = 0; i < state.truth.n; ++i) {
    if (uni01(rng) > det_p) continue;
    meas::RadarMeasurement m{};
    const std::uint64_t t_us = static_cast<std::uint64_t>(std::llround(t_s * 1e6));
    m.id = (t_us << 20) | (meas_seq++ & 0xFFFFFu);
    const double r = aer.r_m[i];
    const double az = aer.az_rad[i];
    const double el = aer.el_rad[i];

    m.range_m = r + nr(rng);
    m.az_rad = (batch.type == cfg::RadarMeasurementType::RANGE_AZ_EL) ? (az + naz(rng)) : 0.0;
    m.el_rad = (batch.type == cfg::RadarMeasurementType::RANGE_AZ_EL) ? (el + nel(rng)) : 0.0;
    m.snr_db = sensor.meas_model.radar.ref_snr_db;
    m.rcs_m2 = sensor.meas_model.radar.ref_rcs_m2;

    m.R = {0.0, 0.0, 0.0,
           0.0, 0.0, 0.0,
           0.0, 0.0, 0.0};
    m.R[0] = sigma_r * sigma_r;
    if (batch.type == cfg::RadarMeasurementType::RANGE_AZ_EL) {
      m.R[4] = sigma_az * sigma_az;
      m.R[8] = sigma_el * sigma_el;
    }

    batch.measurements.push_back(m);
  }

  for (std::uint32_t k = 0; k < sensor.meas_model.radar.false_alarm_count; ++k) {
    if (r_max <= r_min) break;
    meas::RadarMeasurement m{};
    const std::uint64_t t_us = static_cast<std::uint64_t>(std::llround(t_s * 1e6));
    m.id = (t_us << 20) | (meas_seq++ & 0xFFFFFu);
    const double r = uni_r(rng);
    const double az = uni_az(rng);
    const double el = uni_el(rng);

    m.range_m = r + nr(rng);
    m.az_rad = (batch.type == cfg::RadarMeasurementType::RANGE_AZ_EL) ? (az + naz(rng)) : 0.0;
    m.el_rad = (batch.type == cfg::RadarMeasurementType::RANGE_AZ_EL) ? (el + nel(rng)) : 0.0;
    m.snr_db = sensor.meas_model.radar.ref_snr_db;
    m.rcs_m2 = sensor.meas_model.radar.ref_rcs_m2;

    m.R = {0.0, 0.0, 0.0,
           0.0, 0.0, 0.0,
           0.0, 0.0, 0.0};
    m.R[0] = sigma_r * sigma_r;
    if (batch.type == cfg::RadarMeasurementType::RANGE_AZ_EL) {
      m.R[4] = sigma_az * sigma_az;
      m.R[8] = sigma_el * sigma_el;
    }

    batch.measurements.push_back(m);
  }

  state.meas_batch = std::move(batch);

  if (state.scan.course_gates.enabled && state.scan.gate_grid.enabled) {
    const auto& grid = state.scan.gate_grid;
    for (auto& m : state.meas_batch.measurements) {
      double e = 0.0, n = 0.0, u = 0.0;
      coord::AerToEnuPoint(m.az_rad, m.el_rad, m.range_m, e, n, u);
      m.gate_index = -1;

      const double fe = (e - grid.origin_e) / grid.step_e;
      const double fn = (n - grid.origin_n) / grid.step_n;
      const std::int64_t ie = static_cast<std::int64_t>(std::floor(fe + 1e-9));
      const std::int64_t in = static_cast<std::int64_t>(std::floor(fn + 1e-9));
      if (ie < 0 || in < 0) continue;
      if (static_cast<std::size_t>(ie) >= grid.count_e ||
          static_cast<std::size_t>(in) >= grid.count_n) continue;
      const std::size_t map_idx = static_cast<std::size_t>(in) * grid.count_e +
                                  static_cast<std::size_t>(ie);
      if (map_idx >= grid.map.size()) continue;
      const int gate_idx = grid.map[map_idx];
      if (gate_idx >= 0) {
        m.gate_index = gate_idx;
      }
    }
  }

  if (!ensure_output_dir(state.cli.output_dir)) {
    return;
  }

  static bool log_init = false;
  static bool wrote_header_current = false;
  static bool wrote_header_archive = false;
  static fs::path log_path;
  static fs::path archive_path;
  if (!log_init) {
    const std::tm run_tm = local_tm_now();
    const std::string date_folder = format_tm(run_tm, "%Y%m%d");
    const fs::path archive_dir = fs::path(state.cli.output_dir) / "archive" / date_folder;
    ensure_output_dir(archive_dir.string());
    archive_path =
      archive_dir / ("sensor_course_gate_" + format_tm(run_tm, "%Y%m%d_%H%M%S") + ".csv");
    log_path = fs::path(state.cli.output_dir) / "sensor_course_gate_log.csv";
    log_init = true;
  }

  const char* header =
    "date,time,scan_index,t_scan_s,meas_time_us,sensor_id,meas_uid,gate_index,range_m,az_rad,el_rad,snr_db,rcs_m2\n";

  if (!wrote_header_current) {
    std::ofstream trunc_out(log_path, std::ios::trunc);
    if (!trunc_out) return;
    trunc_out << header;
    wrote_header_current = true;
  }

  if (!wrote_header_archive) {
    std::ofstream trunc_out(archive_path, std::ios::trunc);
    if (!trunc_out) return;
    trunc_out << header;
    wrote_header_archive = true;
  }

  std::ofstream out(log_path, std::ios::app);
  if (!out) return;
  std::ofstream out_archive(archive_path, std::ios::app);
  if (!out_archive) return;

  const std::tm run_tm = local_tm_now();
  const std::string date_str = format_tm(run_tm, "%Y-%m-%d");
  const std::string time_str = format_tm(run_tm, "%H:%M:%S");

  const std::uint64_t t_us = static_cast<std::uint64_t>(std::llround(t_s * 1e6));
  for (const auto& m : state.meas_batch.measurements) {
    if (m.gate_index < 0) continue;
    out << date_str << ','
        << time_str << ','
        << state.stats.scan_count << ','
        << t_s << ','
        << t_us << ','
        << state.meas_batch.meta.sensor_id << ','
        << m.id << ','
        << m.gate_index << ','
        << m.range_m << ','
        << m.az_rad << ','
        << m.el_rad << ','
        << m.snr_db << ','
        << m.rcs_m2 << '\n';
    out_archive << date_str << ','
                << time_str << ','
                << state.stats.scan_count << ','
                << t_s << ','
                << t_us << ','
                << state.meas_batch.meta.sensor_id << ','
                << m.id << ','
                << m.gate_index << ','
                << m.range_m << ','
                << m.az_rad << ','
                << m.el_rad << ','
                << m.snr_db << ','
                << m.rcs_m2 << '\n';
  }
}
/**
 * @brief Placeholder for coarse gating stage.
 */
void stage_coarse_gating(PipelineState&, const trk::IdList&, double) {}
/**
 * @brief Placeholder for fine gating stage.
 */
void stage_fine_gating(PipelineState&, const trk::IdList&, double) {}
/**
 * @brief Placeholder for association stage.
 */
void stage_association(PipelineState&, const trk::IdList&, double) {}
/**
 * @brief Placeholder for filter update stage.
 */
void stage_filter_update(PipelineState&, const trk::IdList&, double) {}
/**
 * @brief Placeholder for track maintenance stage.
 */
void stage_track_maintenance(PipelineState&, double) {}

// ------------------------------
// Scan loop setup + tick
// ------------------------------
/**
 * @brief Initialize scan context and spatial index backend for the run.
 */
bool setup_scan_context(PipelineState& state) {
  if (state.cli.gen_only || !state.cfg->has_sensors || state.cfg->sensors.sensors.empty()) {
    return false;
  }

  const cfg::SensorCfg& sensor  = state.cfg->sensors.sensors.front();
  const cfg::StoreProfile& store = state.cfg->store_profile;
  state.scan.sensor             = &sensor;
  const double inflate_m =
    (store.index.query_aabb_inflate_m > 0.0) ? store.index.query_aabb_inflate_m : 2000.0;
  state.scan.scan_aabb          = idx::ComputeScanAabbEcefApprox(sensor, state.cfg->ownship, inflate_m);

  if (state.cfg->has_gating_assoc) {
    state.scan.course_gates =
      db::BuildCourseGateSet(state.cfg->ownship, state.cfg->gating_assoc.course_gates);
    if (state.scan.course_gates.enabled) {
      state.scan.scan_aabb = state.scan.course_gates.prefetch_aabb;
      state.scan.course_gates_enu.clear();
      state.scan.course_gates_enu.reserve(state.scan.course_gates.gates.size());
      for (const auto& gate : state.scan.course_gates.gates) {
        const double xs[2] = {gate.min_x, gate.max_x};
        const double ys[2] = {gate.min_y, gate.max_y};
        const double zs[2] = {gate.min_z, gate.max_z};
        ScanContext::EnuAabb enu_gate{};
        bool first = true;
        for (double x : xs) {
          for (double y : ys) {
            for (double z : zs) {
              double e = 0.0, n = 0.0, u = 0.0;
              coord::EcefToEnuPoint(x, y, z, state.cfg->ownship, e, n, u);
              if (first) {
                enu_gate.min_e = enu_gate.max_e = e;
                enu_gate.min_n = enu_gate.max_n = n;
                enu_gate.min_u = enu_gate.max_u = u;
                first = false;
              } else {
                enu_gate.min_e = std::min(enu_gate.min_e, e);
                enu_gate.max_e = std::max(enu_gate.max_e, e);
                enu_gate.min_n = std::min(enu_gate.min_n, n);
                enu_gate.max_n = std::max(enu_gate.max_n, n);
                enu_gate.min_u = std::min(enu_gate.min_u, u);
                enu_gate.max_u = std::max(enu_gate.max_u, u);
              }
            }
          }
        }
        state.scan.course_gates_enu.push_back(enu_gate);
      }

      // Build an O(1) ENU gate lookup grid matching BuildCourseGateSet iteration order.
      state.scan.gate_grid = ScanContext::GateGrid{};
      const cfg::CourseGatesCfg& cg = state.cfg->gating_assoc.course_gates;
      if (cg.radius_m > 0.0 && cg.side_x_m > 0.0 && cg.side_y_m > 0.0) {
        const double r = cg.radius_m;
        const double step_e = cg.side_x_m;
        const double step_n = cg.side_y_m;
        std::size_t count_e = 0;
        for (double e = -r; e <= r + 1e-6; e += step_e) ++count_e;
        std::size_t count_n = 0;
        for (double n = -r; n <= r + 1e-6; n += step_n) ++count_n;

        state.scan.gate_grid.enabled = (count_e > 0 && count_n > 0);
        state.scan.gate_grid.origin_e = -r;
        state.scan.gate_grid.origin_n = -r;
        state.scan.gate_grid.step_e = step_e;
        state.scan.gate_grid.step_n = step_n;
        state.scan.gate_grid.radius_m = r;
        state.scan.gate_grid.count_e = count_e;
        state.scan.gate_grid.count_n = count_n;
        state.scan.gate_grid.map.assign(count_e * count_n, -1);

        std::size_t gate_idx = 0;
        std::size_t j = 0;
        for (double n = -r; n <= r + 1e-6; n += step_n, ++j) {
          std::size_t i = 0;
          for (double e = -r; e <= r + 1e-6; e += step_e, ++i) {
            if ((e * e + n * n) > (r * r)) continue;
            const std::size_t map_idx = j * count_e + i;
            if (map_idx < state.scan.gate_grid.map.size()) {
              state.scan.gate_grid.map[map_idx] = static_cast<int>(gate_idx++);
            }
          }
        }
      }
      if (state.scan.course_gates.gates.size() > 5000 && should_log(LogLevel::WARN)) {
        std::cerr << "WARNING: course gate count is large ("
                  << state.scan.course_gates.gates.size()
                  << "); consider increasing gate size or reducing radius.\n";
      }
    }
  }

  // Build kinematics batch from TrackBatch positions.
  build_kinematics_from_track_batch(state.tb, state.scan.tbm);

  // Persistent buckets + models.
  state.scan.buckets.init_from_track_batch(state.scan.tbm);

  state.scan.mcfg.T_run_s       = state.cli.run_s;
  state.scan.mcfg.dt_s          = state.dt_s;
  state.scan.mcfg.scan_rate_hz  = (state.cli.scan_hz > 0.0) ? state.cli.scan_hz : 1.0;

  // ---- Create spatial index backend (XML default, CLI override)
  const std::string backend_xml = store.index.backend;
  state.scan.icfg.backend =
    !state.cli.index_backend_cli.empty() ? state.cli.index_backend_cli : backend_xml;
  if (state.scan.icfg.backend.empty()) {
    state.scan.icfg.backend = "sqlite_rtree"; // final fallback
  }

  // Base value for derivations
  state.scan.scan_inflate_m = inflate_m;

  // Cell size: CLI > XML > derived
  state.scan.icfg.grid_cell_m =
    (state.cli.cell_m_cli > 0.0) ? state.cli.cell_m_cli :
    (store.index.cell_m > 0.0 ? store.index.cell_m : state.scan.scan_inflate_m);

  // Move threshold: CLI > XML > derived
  state.scan.d_th_m =
    (state.cli.d_th_m_cli > 0.0) ? state.cli.d_th_m_cli :
    (store.index.d_th_m > 0.0 ? store.index.d_th_m : (0.25 * state.scan.icfg.grid_cell_m));

  // Max age: CLI > XML > derived (~2 scans)
  state.scan.t_max_s =
    (state.cli.t_max_s_cli > 0.0) ? state.cli.t_max_s_cli :
    (store.index.t_max_s > 0.0 ? store.index.t_max_s : (2.0 / state.scan.mcfg.scan_rate_hz));

  // Hot prefetch cadence (optional)
  state.scan.hot_prefetch_min_s = std::max(0.0, store.index.hot_prefetch_min_s);
  state.scan.hot_prefetch_window_s = std::max(0.0, store.index.hot_prefetch_window_s);
  if (state.scan.hot_prefetch_window_s > 0.0 &&
      state.scan.hot_prefetch_window_s < state.scan.hot_prefetch_min_s) {
    state.scan.hot_prefetch_window_s = state.scan.hot_prefetch_min_s;
  }
  state.scan.hot_prefetch_cadence_enabled =
    (state.scan.hot_prefetch_min_s > 0.0 || state.scan.hot_prefetch_window_s > 0.0);

  db::TrackDatabaseConfig db_cfg;
  db_cfg.mode = store.mode;
  db_cfg.backend = state.scan.icfg.backend;
  db_cfg.sqlite_db_uri = store.sqlite.db_uri;
  db_cfg.grid_cell_m = state.scan.icfg.grid_cell_m;
  db_cfg.dense_cell_probe_limit = store.index.dense_cell_probe_limit;
  db_cfg.d_th_m = state.scan.d_th_m;
  db_cfg.t_max_s = state.scan.t_max_s;
  const std::size_t commit_scans =
    (state.cli.warm_commit_every_scans > 0)
      ? state.cli.warm_commit_every_scans
      : state.cfg->store_profile.warm_commit_every_scans;
  db_cfg.warm_commit_every_scans = std::max<std::size_t>(1, commit_scans);
  db_cfg.warm_commit_after_tracks =
    std::max<std::size_t>(1, state.cfg->store_profile.warm_commit_after_tracks);

  state.scan.track_db = db::CreateTrackDatabase(db_cfg);
  state.scan.track_db->Reserve(state.scan.tbm.n);
  state.scan.track_db->Configure(state.scan.d_th_m, state.scan.t_max_s);
  if (store.mode == "HOT_PLUS_WARM") {
    const std::string& db_uri = store.sqlite.db_uri;
    bool reuse_db = false;
    const std::string signature = build_warm_signature(state);
    if (!db_uri.empty() && db_uri != ":memory:" &&
        state.cfg->scenario.refs.targets_use_prepopulated_db) {
      std::error_code ec;
      const std::uintmax_t sz = fs::file_size(fs::path(db_uri), ec);
      reuse_db = (!ec && sz > 0);
    }

    if (reuse_db) {
      reuse_db = state.scan.track_db->WarmSignatureMatches(signature);
    }

    if (!reuse_db) {
      // Initialize warm store before the scan loop so first-scan latency is not impacted.
      const auto init_t0 = std::chrono::high_resolution_clock::now();
      state.scan.track_db->FinalizeScan(
        0.0,
        state.tb,
        nullptr);
      const auto init_t1 = std::chrono::high_resolution_clock::now();
      const double init_s = std::chrono::duration<double>(init_t1 - init_t0).count();
      if (should_log(LogLevel::INFO)) {
        std::cout << "  warm_init_s=" << init_s << "\n";
      }
      state.scan.track_db->StoreWarmSignature(signature);
      state.scan.track_db->ResetTimingStats();
    } else if (should_log(LogLevel::INFO)) {
      std::cout << "  warm_init_s=skipped (signature match)\n";
    }

    // Stage 1: warm prefetch to define hot working set for the scan volume.
    state.scan.hot_ids = state.scan.track_db->PrefetchHot(state.scan.scan_aabb);
    state.scan.last_prefetch_t_s = 0.0;
    if (should_log(LogLevel::INFO)) {
      std::cout << "  hot_prefetch_n=" << state.scan.hot_ids.size() << "\n";
    }
  }

  if (should_log(LogLevel::INFO)) {
    std::cout << "Scan-driven coarse query (loop):\n";
    std::cout << "  sensor.id=" << sensor.id << " frame=" << sensor.scan.frame
              << " inflate_m=" << state.scan.scan_inflate_m << "\n";
    std::cout << "  aabb.min=(" << state.scan.scan_aabb.min_x << "," << state.scan.scan_aabb.min_y
              << "," << state.scan.scan_aabb.min_z << ")\n";
    std::cout << "  aabb.max=(" << state.scan.scan_aabb.max_x << "," << state.scan.scan_aabb.max_y
              << "," << state.scan.scan_aabb.max_z << ")\n";
    if (state.scan.course_gates.enabled && state.cfg->has_gating_assoc) {
      const cfg::CourseGatesCfg& cg = state.cfg->gating_assoc.course_gates;
      std::cout << "  course_gates.enabled=yes"
                << " radius_m=" << cg.radius_m
                << " side_m=(" << cg.side_x_m << "," << cg.side_y_m << "," << cg.side_z_m << ")"
                << " count=" << state.scan.course_gates.gates.size() << "\n";
    }
    std::cout << "  run_s=" << state.scan.mcfg.T_run_s << " dt_s=" << state.scan.mcfg.dt_s
              << " scan_hz=" << state.scan.mcfg.scan_rate_hz << "\n";
    std::cout << "  index.backend=" << state.scan.icfg.backend
              << " supports_incremental="
              << (state.scan.track_db->SupportsIncrementalUpdates() ? "yes" : "no") << "\n";
    if (state.scan.hot_prefetch_cadence_enabled &&
        state.cfg->store_profile.mode == "HOT_PLUS_WARM") {
      std::cout << "  hot_prefetch_min_s=" << state.scan.hot_prefetch_min_s
                << " hot_prefetch_window_s=" << state.scan.hot_prefetch_window_s << "\n";
    }
    if (state.scan.icfg.backend == "uniform_grid") {
      std::cout << "  grid.cell_m=" << state.scan.icfg.grid_cell_m
                << " d_th_m=" << state.scan.d_th_m
                << " t_max_s=" << state.scan.t_max_s
                << " dense_cell_probe_limit=" << store.index.dense_cell_probe_limit
                << "\n";
    } else {
      std::cout << "  d_th_m=" << state.scan.d_th_m
                << " t_max_s=" << state.scan.t_max_s
                << " (used by update manager; backend may rebuild)\n";
    }
  }

  return true;
}

/**
 * @brief Execute a single scan tick: update index, pipeline stages, and covariance aging.
 */
void process_scan_tick(PipelineState& state,
                       trk::TrackKinematicsBatch& tb_ref,
                       double t_s) {
  ++state.stats.scan_count;

  if (state.cfg->store_profile.mode == "HOT_PLUS_WARM") {
    bool do_prefetch = true;
    if (state.scan.hot_prefetch_cadence_enabled) {
      const double window_s =
        (state.scan.hot_prefetch_window_s > 0.0)
          ? state.scan.hot_prefetch_window_s
          : state.scan.hot_prefetch_min_s;
      if (window_s > 0.0) {
        const double elapsed = t_s - state.scan.last_prefetch_t_s;
        const double remaining = window_s - elapsed;
        do_prefetch = (remaining <= state.scan.hot_prefetch_min_s);
      }
    }
    if (do_prefetch) {
      state.scan.hot_ids = state.scan.track_db->PrefetchHot(state.scan.scan_aabb);
      state.scan.last_prefetch_t_s = t_s;
    }
  }

  const auto t0 = std::chrono::high_resolution_clock::now();
  state.scan.track_db->UpdateTracks(t_s, tb_ref.x_ptr(), tb_ref.y_ptr(), tb_ref.z_ptr(), tb_ref.n);
  const auto t1 = std::chrono::high_resolution_clock::now();

  stage_measurement_generation(state, t_s);

  const auto t1q = std::chrono::high_resolution_clock::now();
  // Candidate track indices for this scan.
  trk::IdList ids;
  if (state.scan.course_gates.enabled && !state.scan.course_gates.gates.empty() &&
      !state.meas_batch.measurements.empty()) {
    const std::size_t gate_count = state.scan.course_gates.gates.size();
    std::vector<std::uint8_t> gate_used(gate_count, 0);
    for (const auto& m : state.meas_batch.measurements) {
      if (m.gate_index < 0) continue;
      const std::size_t g = static_cast<std::size_t>(m.gate_index);
      if (g < gate_used.size()) gate_used[g] = 1;
    }

    std::vector<trk::IdList> gate_candidates(gate_count);
    for (std::size_t g = 0; g < gate_count; ++g) {
      if (!gate_used[g]) continue;
      gate_candidates[g] = state.scan.track_db->QueryAabb(state.scan.course_gates.gates[g]);
    }

    static std::vector<std::uint8_t> seen;
    if (seen.size() < tb_ref.n) {
      seen.assign(tb_ref.n, 0);
    } else {
      std::fill(seen.begin(), seen.end(), 0);
    }

    for (const auto& m : state.meas_batch.measurements) {
      if (m.gate_index < 0) continue;
      const std::size_t g = static_cast<std::size_t>(m.gate_index);
      if (g >= gate_candidates.size()) continue;
      const auto& gate_ids = gate_candidates[g];
      for (std::uint64_t id : gate_ids) {
        const std::size_t idx = static_cast<std::size_t>(id);
        if (idx >= seen.size()) continue;
        if (!seen[idx]) {
          seen[idx] = 1;
          ids.push_back(id);
        }
      }
    }
  } else if (state.scan.course_gates.enabled && !state.scan.course_gates.gates.empty()) {
    ids = db::QueryCourseGates(*state.scan.track_db,
                               state.scan.course_gates.gates,
                               tb_ref.n);
  } else {
    ids = state.scan.track_db->QueryAabb(state.scan.scan_aabb);
  }
  const auto t2 = std::chrono::high_resolution_clock::now();

  state.stats.update_acc += std::chrono::duration<double>(t1 - t0).count();
  state.stats.query_acc  += std::chrono::duration<double>(t2 - t1q).count();
  state.stats.nupd_acc   += static_cast<double>(state.scan.track_db->NumUpdatedLastUpdate());
  ++state.stats.k_acc;

  // Placeholder pipeline stages (gating -> association -> filter -> maintenance)
  stage_coarse_gating(state, ids, t_s);
  stage_fine_gating(state, ids, t_s);
  stage_association(state, ids, t_s);
  stage_filter_update(state, ids, t_s);
  stage_track_maintenance(state, t_s);
  const auto t3 = std::chrono::high_resolution_clock::now();

  // Persist post-scan updates to warm store (if configured).
  // Heuristic: treat all scan candidates as changed until filter/maintenance marks specific ids.
  state.changed_ids = ids;
  bool did_finalize = false;
    if (!state.changed_ids.empty()) {
    state.scan.track_db->FinalizeScan(
      t_s, state.tb, &state.changed_ids);
    did_finalize = true;
  }
  const auto t4 = std::chrono::high_resolution_clock::now();

  // --- Scan-driven covariance aging: age all hot tracks to scan time t_s ---
  // Use TrackBatch::last_cov_prop_s as t_pred_s (P-aged-to time) per-track.
  const trk::IdList& cov_ids =
      (!state.scan.hot_ids.empty() ? state.scan.hot_ids : state.all_ids);
  if (!cov_ids.empty()) {
    static std::vector<double> dt_ids;
    if (dt_ids.capacity() < cov_ids.size()) dt_ids.reserve(cov_ids.size());
    dt_ids.resize(cov_ids.size());

    for (std::size_t k = 0; k < cov_ids.size(); ++k) {
      const std::size_t i = static_cast<std::size_t>(cov_ids[k]);

      const double dt_i = t_s - state.tb.last_cov_prop_s[i];
      dt_ids[k] = (dt_i > 0.0) ? dt_i : 0.0;

      // Mark covariance as aged-to scan time (scan times should be monotonic)
      state.tb.last_cov_prop_s[i] = t_s;
    }

    const auto tc0 = std::chrono::high_resolution_clock::now();
    la::rw_add_qdt_subset_var_dt(state.tb.P.data(), state.Q.data(), state.n,
                                 cov_ids.data(), dt_ids.data(), cov_ids.size());
    const auto tc1 = std::chrono::high_resolution_clock::now();

    state.stats.cov_age_acc_s += std::chrono::duration<double>(tc1 - tc0).count();
    ++state.stats.cov_age_calls;
    state.stats.cov_age_tracks_acc += cov_ids.size();
  }
  const auto t5 = std::chrono::high_resolution_clock::now();

  if (did_finalize) {
    state.stats.finalize_acc_s += std::chrono::duration<double>(t4 - t3).count();
    ++state.stats.finalize_calls;
  }

  if (state.debug_timing) {
    const db::TrackDbTimingStats db_stats = state.scan.track_db->GetTimingStats();
    if (db_stats.prefetch_calls > state.stats.prefetch_calls) {
      state.stats.prefetch_acc_s = db_stats.prefetch_acc_s;
      state.stats.prefetch_calls = db_stats.prefetch_calls;
    }
    if (db_stats.finalize_calls > state.stats.finalize_calls) {
      state.stats.finalize_begin_acc_s = db_stats.finalize_begin_acc_s;
      state.stats.finalize_apply_acc_s = db_stats.finalize_apply_acc_s;
      state.stats.finalize_commit_acc_s = db_stats.finalize_commit_acc_s;
    }

    const double update_s = std::chrono::duration<double>(t1 - t0).count();
    const double query_s = std::chrono::duration<double>(t2 - t1).count();
    const double stages_s = std::chrono::duration<double>(t3 - t2).count();
    const double finalize_s = did_finalize ? std::chrono::duration<double>(t4 - t3).count() : 0.0;
    const double cov_s = std::chrono::duration<double>(t5 - t4).count();
    const double prefetch_s = db_stats.last_prefetch_s;
    const double finalize_begin_s = db_stats.last_finalize_begin_s;
    const double finalize_apply_s = db_stats.last_finalize_apply_s;
    const double finalize_commit_s = db_stats.last_finalize_commit_s;
    std::cout << "  timing_s"
              << " prefetch=" << prefetch_s
              << " update=" << update_s
              << " query=" << query_s
              << " stages=" << stages_s
              << " finalize=" << finalize_s
              << " finalize_begin=" << finalize_begin_s
              << " finalize_apply=" << finalize_apply_s
              << " finalize_commit=" << finalize_commit_s
              << " cov_age=" << cov_s
              << "\n";
  }

  if (should_log(LogLevel::INFO) && (state.stats.k_acc % 5 == 0)) {
    std::cout << "  avg_update_s=" << (state.stats.update_acc / state.stats.k_acc)
              << " avg_query_s="  << (state.stats.query_acc / state.stats.k_acc)
              << " avg_n_updated=" << (state.stats.nupd_acc / state.stats.k_acc)
              << "\n";
  }

  if (should_log(LogLevel::INFO) &&
      (state.stats.scan_count <= 3 || (state.stats.scan_count % 10 == 0))) {
    std::cout << "  scan=" << state.stats.scan_count
              << " t_s=" << t_s
              << " n_updated=" << state.scan.track_db->NumUpdatedLastUpdate()
              << " candidates=" << ids.size() << " / " << tb_ref.n << "\n";
  }
}

/**
 * @brief Write a per-run CSV and append to the rolling performance.csv.
 */
void write_csvs(const PipelineState& state) {
  if (!ensure_output_dir(state.cli.output_dir)) {
    if (should_log(LogLevel::WARN)) {
      std::cerr << "WARNING: failed to create output directory '" << state.cli.output_dir << "'.\n";
    }
    return;
  }

  const std::tm run_tm          = local_tm_now();
  const std::string date_str    = format_tm(run_tm, "%Y-%m-%d");
  const std::string time_str    = format_tm(run_tm, "%H:%M:%S");
  const std::string version = TRACKER_VERSION_STRING;

  const std::string date_folder = format_tm(run_tm, "%Y%m%d");
  const fs::path archive_dir = fs::path(state.cli.output_dir) / "archive" / date_folder;
  if (!ensure_output_dir(archive_dir.string())) {
    if (should_log(LogLevel::WARN)) {
      std::cerr << "WARNING: failed to create archive directory '" << archive_dir.string() << "'.\n";
    }
    return;
  }

  const std::string csv_path =
    (archive_dir / ("performance_" + format_tm(run_tm, "%Y%m%d_%H%M%S") + ".csv")).string();
  std::ofstream out(csv_path, std::ios::trunc);
  if (!out) {
    if (should_log(LogLevel::WARN)) {
      std::cerr << "WARNING: failed to open performance CSV at '" << csv_path << "'.\n";
    }
    return;
  }

  const double avg_update_s         = (state.stats.k_acc ? (state.stats.update_acc / state.stats.k_acc) : 0.0);
  const double avg_query_s          = (state.stats.k_acc ? (state.stats.query_acc / state.stats.k_acc) : 0.0);
  const double avg_n_updated        = (state.stats.k_acc ? (state.stats.nupd_acc / state.stats.k_acc) : 0.0);
  const double avg_tracks_per_call  =
      (state.stats.cov_age_calls
           ? (static_cast<double>(state.stats.cov_age_tracks_acc) / state.stats.cov_age_calls)
           : 0.0);
  const double avg_s_per_call =
      (state.stats.cov_age_calls ? (state.stats.cov_age_acc_s / state.stats.cov_age_calls) : 0.0);
  const double avg_finalize_s =
      (state.stats.finalize_calls ? (state.stats.finalize_acc_s / state.stats.finalize_calls) : 0.0);
  const db::TrackDbTimingStats db_stats =
      state.scan.track_db ? state.scan.track_db->GetTimingStats() : db::TrackDbTimingStats{};
  const double avg_prefetch_s =
      (db_stats.prefetch_calls ? (db_stats.prefetch_acc_s / db_stats.prefetch_calls) : 0.0);
  const double avg_finalize_begin_s =
      (db_stats.finalize_begin_calls ? (db_stats.finalize_begin_acc_s / db_stats.finalize_begin_calls) : 0.0);
  const double avg_finalize_apply_s =
      (db_stats.finalize_calls ? (db_stats.finalize_apply_acc_s / db_stats.finalize_calls) : 0.0);
  const double avg_finalize_commit_s =
      (db_stats.finalize_commit_calls ? (db_stats.finalize_commit_acc_s / db_stats.finalize_commit_calls) : 0.0);

  auto write_header = [](std::ostream& os) {
    os
      << "date,time,version,system_xml,xsd_dir,targets_xml,dim,tracks,dt_s,run_s,scan_hz,"
         "scan_count,loop_time_s,loop_time_per_scan_s,avg_update_s,avg_query_s,avg_n_updated,finalize_total_s,finalize_avg_s,"
         "warm_commit_every_scans,hot_prefetch_min_s,hot_prefetch_window_s,"
         "finalize_begin_total_s,finalize_begin_avg_s,finalize_apply_total_s,finalize_apply_avg_s,"
         "finalize_commit_total_s,finalize_commit_avg_s,prefetch_total_s,prefetch_avg_s,prefetch_calls,"
         "cov_age_calls,avg_tracks_per_call,cov_age_total_s,cov_age_avg_s,checksum\n";
  };
  auto write_row = [&](std::ostream& os) {
    const double loop_per_scan =
      (state.stats.scan_count > 0 ? (state.stats.loop_s / static_cast<double>(state.stats.scan_count)) : 0.0);
    const std::size_t warm_commit_scans =
      (state.cli.warm_commit_every_scans > 0)
        ? state.cli.warm_commit_every_scans
        : state.cfg->store_profile.warm_commit_every_scans;
    os
      << date_str << ','
      << time_str << ','
      << version << ','
      << state.cli.system_xml << ','
      << state.cli.xsd_dir << ','
      << state.targets_xml << ','
      << state.n << ','
      << state.batch << ','
      << state.dt_s << ','
      << state.cli.run_s << ','
      << state.cli.scan_hz << ','
      << state.stats.scan_count << ','
      << state.stats.loop_s << ','
      << loop_per_scan << ','
      << avg_update_s << ','
      << avg_query_s << ','
      << avg_n_updated << ','
      << state.stats.finalize_acc_s << ','
      << avg_finalize_s << ','
      << warm_commit_scans << ','
      << state.scan.hot_prefetch_min_s << ','
      << state.scan.hot_prefetch_window_s << ','
      << db_stats.finalize_begin_acc_s << ','
      << avg_finalize_begin_s << ','
      << db_stats.finalize_apply_acc_s << ','
      << avg_finalize_apply_s << ','
      << db_stats.finalize_commit_acc_s << ','
      << avg_finalize_commit_s << ','
      << db_stats.prefetch_acc_s << ','
      << avg_prefetch_s << ','
      << db_stats.prefetch_calls << ','
      << state.stats.cov_age_calls << ','
      << avg_tracks_per_call << ','
      << state.stats.cov_age_acc_s << ','
      << avg_s_per_call << ','
      << checksum(state.tb.P)
      << "\n";
  };

  write_header(out);
  write_row(out);

  const fs::path append_path = fs::path(state.cli.output_dir) / "performance.csv";
  std::error_code ec;
  bool has_header = false;
  bool header_matches = false;
  if (fs::exists(append_path, ec) && !ec) {
    const auto sz = fs::file_size(append_path, ec);
    has_header = (!ec && sz > 0);
    if (has_header) {
      std::ifstream in(append_path);
      std::string line;
      if (in && std::getline(in, line)) {
        std::ostringstream expected;
        write_header(expected);
        std::string expected_line = expected.str();
        if (!expected_line.empty() && expected_line.back() == '\n') {
          expected_line.pop_back();
        }
        header_matches = (line == expected_line);
      }
    }
  }
  std::ofstream append_out(append_path, std::ios::app);
  if (append_out) {
    // Write header only when the rolling file is created.
    if (!has_header || !header_matches) {
      write_header(append_out);
    }
    write_row(append_out);
  } else if (should_log(LogLevel::WARN)) {
    std::cerr << "WARNING: failed to open performance CSV at '" << append_path.string() << "'.\n";
  }
}

} // namespace

/**
 * @brief Demo driver entrypoint.
 */
int demo::RunDemo(int argc, char** argv) try {
  const CliArgs cli = parse_cli(argc, argv);

  // Load config (with optional schema validation depending on xsd_dir)
  const cfg::ConfigBundle cfg = cfg::ConfigLoader::Load(cli.system_xml, cli.xsd_dir);

  PipelineState state;
  state.cfg = &cfg;
  state.cli = cli;
  state.debug_timing = should_log(LogLevel::DEBUG);

  // Resolve targets generator XML:
  //  1) CLI override if provided
  //  2) scenario targets from system.xml -> scenario catalog if present
  //  3) fallback hard-coded path ONLY if missing/unloadable
  state.targets_xml = resolve_targets_xml(state.cli.targets_xml_cli, cfg.paths, current_log_level());

  state.n = cfg.tracker_model.state.dim;
  if (state.n != trk::TrackBatch::kDim) {
    std::cerr << "ERROR: This demo expects dim=" << trk::TrackBatch::kDim
              << " for now (got dim=" << state.n << ").\n";
    return 2;
  }

  if (cfg.runtime.update_rate_hz <= 0.0) {
    std::cerr << "ERROR: runtime.update_rate_hz must be > 0.\n";
    return 2;
  }
  state.dt_s = 1.0 / cfg.runtime.update_rate_hz;

  // Batch size: default to max active tracks, allow CLI override.
  state.batch = compute_batch_size(state.cli, cfg);
  if (state.batch == 0) {
    std::cerr << "ERROR: batch size is 0.\n";
    return 2;
  }

  // Tracker storage container (estimate-side)
  state.tb.resize(state.batch);
  state.all_ids.resize(state.batch);
  for (std::size_t i = 0; i < state.batch; ++i) {
    state.all_ids[i] = static_cast<std::uint64_t>(i);
  }

  // ------------------------------
  // Step 2: deterministic generation (truth -> tracker seeding)
  // ------------------------------
  seed_tracks(state);

  // Define a simple Q_per_sec (row-major) consistent with a CA9 state ordering.
  state.Q = build_process_noise_q(cfg, state.n);

  std::cout << "Config:   " << state.cli.system_xml << "\n";
  std::cout << "XSD:      " << state.cli.xsd_dir << "\n";
  std::cout << "Targets:  " << state.targets_xml << (state.cli.no_gen ? " (skipped)" : "") << "\n";
  std::cout << "dim=" << state.n << " tracks=" << state.batch << " dt_s=" << state.dt_s << "\n";

  // ------------------------------------------------------------
  // Step 3A-3F: Motion loop + scan-tick coarse query via pluggable index backend.
  // ------------------------------------------------------------
  if (setup_scan_context(state)) {
    const auto loop_t0 = std::chrono::high_resolution_clock::now();

    trk::run_scenario_loop(
      state.scan.tbm,
      state.scan.buckets,
      state.scan.model_ca,
      state.scan.model_ct,
      state.scan.selector,
      state.scan.mcfg,
      [&](trk::TrackKinematicsBatch& tb_ref, double t_s) {
        process_scan_tick(state, tb_ref, t_s);
      });

    const auto loop_t1 = std::chrono::high_resolution_clock::now();
    state.stats.loop_s = std::chrono::duration<double>(loop_t1 - loop_t0).count();

    std::cout << "  loop_time_s=" << state.stats.loop_s
              << " scans=" << state.stats.scan_count << "\n\n";
  } else {
    std::cout << "Scan-driven coarse query (loop): skipped (gen-only or no sensors loaded)\n\n";
  }

  if (!state.cli.gen_only) {
    std::cout << "Covariance aging (scan-driven): "
              << "calls=" << state.stats.cov_age_calls
              << " avg_tracks_per_call="
              << (state.stats.cov_age_calls
                      ? (static_cast<double>(state.stats.cov_age_tracks_acc) /
                         state.stats.cov_age_calls)
                      : 0.0)
              << " total_s=" << state.stats.cov_age_acc_s
              << " avg_s_per_call="
              << (state.stats.cov_age_calls
                      ? (state.stats.cov_age_acc_s / state.stats.cov_age_calls)
                      : 0.0)
              << " checksum " << checksum(state.tb.P) << "\n";
  } else {
    std::cout << "Propagation skipped (--gen-only)\n";
  }

    // Flush any async warm commits before writing CSV so commit timings are included.
    if (state.scan.track_db) {
      state.scan.track_db->FlushWarmUpdates();
    }
    // Write performance metrics to CSV (single-row, time-stamped per run)
    write_csvs(state);

  return 0;

} catch (const std::exception& e) {
  std::cerr << "FATAL: " << e.what() << "\n";
  return 1;
}
