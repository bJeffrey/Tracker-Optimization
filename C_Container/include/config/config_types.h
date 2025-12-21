#pragma once
/**
 * @file config_types.h
 * @brief Plain-old-data structures holding parsed tracker configuration.
 *
 * Design goals:
 *  - Keep types simple and stable.
 *  - Parse/validation is handled by ConfigLoader.
 *  - Backend (eigen/std/mkl) is compile-time (container) and is NOT a config option.
 */

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cfg {

// ------------------------------
// System wiring (system.xml)
// ------------------------------
struct ActiveSelection {
  std::string runtime_profile_id;
  std::string store_profile_id;
  std::string scenario_id;
};

struct SystemRefs {
  std::string base_dir;  // optional
  std::string runtime_profiles_href;
  std::string tracker_model_href;
  std::string performance_href;   // optional
  std::string sensors_href;       // optional
  std::string gating_assoc_href;  // optional
  std::string store_href;
  std::string persistence_href;   // optional
  std::string scenario_file_href;
};

struct SystemConfig {
  ActiveSelection active;
  SystemRefs refs;
};

// ------------------------------
// Runtime profiles (runtime_profiles.xml)
// ------------------------------
struct Batching {
  int fetch_batch_size = 0;   // optional
  int update_batch_size = 0;  // optional
  int commit_batch_size = 0;  // optional
};

struct RuntimeProfile {
  std::string id;
  double update_rate_hz = 0.0;
  int max_tracks_active = 0;
  Batching batching{};
  double prefetch_horizon_s = 0.0;  // optional
  int max_tracks_in_query = 0;      // optional
  double time_budget_ms = 0.0;      // optional
};

struct RuntimeProfiles {
  std::map<std::string, RuntimeProfile> by_id;
};

// ------------------------------
// Tracker model (tracker_model.xml)
// ------------------------------
struct TrackerState {
  std::string frame;  // e.g., "ECEF"
  int dim = 0;
  std::string order;  // "x y z vx vy vz ax ay az"
};

struct ProcessNoise {
  double sigma_accel_mps2 = 0.0;  // optional
  double sigma_jerk_mps3 = 0.0;   // optional
};

struct ProcessModel {
  std::string type;            // "CA", "CV", ...
  double default_dt_s = 0.0;   // optional
  ProcessNoise noise{};
};

struct CovarianceCfg {
  std::string storage;              // "upper_tri_45" etc.
  bool symmetrize_on_write = false; // optional
  double min_eig_clamp = 0.0;       // optional
  bool store_cholesky = false;      // optional
};

struct TrackerModel {
  TrackerState state;
  ProcessModel process;  // optional; if absent, type may be empty
  CovarianceCfg cov;
};

// ------------------------------
// Performance (performance.xml)
// ------------------------------
struct EnvVar {
  std::string name;
  std::string value;
};

struct ThreadsCfg {
  int default_omp_threads = 0;            // optional
  bool allow_env_override = true;         // optional
  bool set_env_defaults_if_unset = true;  // optional
};

struct DiagnosticsCfg {
  bool print_startup_summary = true;  // optional
  bool print_store_stats = false;     // optional
  bool print_timing_stats = false;    // optional
};

struct PerformanceCfg {
  ThreadsCfg threads{};
  std::vector<EnvVar> env_defaults;
  DiagnosticsCfg diagnostics{};
}; // <-- FIX: close PerformanceCfg

// ------------------------------
// Sensors (sensors.xml)
// ------------------------------
enum class SensorType {
  RADAR,
  ESM,
  EOIR,
  UNKNOWN
};

inline SensorType SensorTypeFromText(const std::string& s) {
  if (s == "RADAR") return SensorType::RADAR;
  if (s == "ESM")   return SensorType::ESM;
  if (s == "EOIR")  return SensorType::EOIR;
  return SensorType::UNKNOWN;
}

struct ScanFrustumCfg {
  // Degrees and meters; interpreted in the declared ScanVolume frame.
  double az_min_deg = 0.0;
  double az_max_deg = 0.0;
  double el_min_deg = 0.0;
  double el_max_deg = 0.0;
  double r_min_m = 0.0;
  double r_max_m = 0.0;
};

struct ScanVolumeCfg {
  std::string frame;               // e.g., "OWNERSHIP_BODY"
  ScanFrustumCfg frustum{};
  double query_aabb_inflate_m = 0.0; // inflate ECEF AABB for coarse DB query
};

struct SensorCfg {
  std::string id;
  SensorType type = SensorType::UNKNOWN;
  ScanVolumeCfg scan{};
  double scan_rate_hz = 0.0;  // schedule hint
};

struct SensorsCfg {
  std::vector<SensorCfg> sensors;

  const SensorCfg* FindById(const std::string& id) const {
    for (const auto& s : sensors) {
      if (s.id == id) return &s;
    }
    return nullptr;
  }
}; // <-- FIX: do NOT close namespace here

// ------------------------------
// Store (store.xml)
// ------------------------------
struct SqliteCfg {
  std::string db_uri;          // optional
  std::string journal_mode;    // optional
  std::string synchronous;     // optional
  int page_size = 0;           // optional
  int cache_size_pages = 0;    // optional
  std::string temp_store;      // optional
};

struct RTreeCfg {
  double padding_m = 0.0;          // optional
  double rebuild_interval_s = 0.0; // optional
};

struct StoreProfile {
  std::string id;
  std::string mode;     // HOT_ONLY / HOT_PLUS_WARM
  std::string backend;  // SQLITE_RTREE / RAM_NATIVE
  SqliteCfg sqlite{};   // optional depending on backend
  RTreeCfg rtree{};     // optional depending on backend
};

struct StoreCfg {
  std::map<std::string, StoreProfile> by_id;
};

// ------------------------------
// Persistence (persistence.xml)
// ------------------------------
struct SnapshotsCfg {
  bool enabled = false;              // optional
  double interval_s = 0.0;           // optional
  std::string directory;             // optional
  int retain_count = 0;              // optional
  std::string compression;           // optional ("none","zstd"...)
  bool snapshot_on_shutdown = false; // optional
  bool snapshot_on_signal = false;   // optional
};

struct RecoveryCfg {
  bool load_latest_on_startup = true; // optional
  bool delta_log_enabled = false;     // optional
};

struct PersistenceCfg {
  SnapshotsCfg snapshots{};
  RecoveryCfg recovery{};
};

// ------------------------------
// Scenario (scenario_default.xml + ownship + targets)
// ------------------------------
struct ScenarioRefs {
  std::string base_dir;  // optional
  std::string ownship_href;
  std::string targets_source_type; // "generator", etc.
  std::string targets_href;
};

struct LoadPolicy {
  std::string if_db_populated_action; // optional
  std::vector<std::pair<std::string, std::string>> on_cli_flags; // (name, action)
};

struct Scenario {
  std::string id;
  std::string name;
  ScenarioRefs refs;
  LoadPolicy load_policy;
};

struct ScenarioCatalog {
  std::map<std::string, Scenario> by_id;
};

struct Ownship {
  std::string id;
  std::string frame;
  double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
  bool has_vel = false;
  double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;
};

struct TargetGenCfg {
  std::string id;
  int count = 0;
  uint32_t seed = 0;
  // Minimal for now; we can extend as needed.
};

// ------------------------------
// Resolved bundle (what the app uses)
// ------------------------------
struct ResolvedPaths {
  std::string system_xml;
  std::string xsd_dir; // optional
  std::string runtime_profiles_xml;
  std::string tracker_model_xml;
  std::string performance_xml;   // optional
  std::string sensors_xml;       // optional
  std::string gating_assoc_xml;  // optional
  std::string store_xml;
  std::string persistence_xml;   // optional
  std::string scenario_xml;
  std::string ownship_xml;
  std::string targets_xml;
};

struct ConfigBundle {
  ResolvedPaths paths;

  SystemConfig system;
  RuntimeProfile runtime;
  TrackerModel tracker_model;

  PerformanceCfg performance;   // default-initialized if not present
  bool has_performance = false;

  SensorsCfg sensors;           // default-initialized if not present
  bool has_sensors = false;

  StoreProfile store_profile;

  PersistenceCfg persistence;
  bool has_persistence = false;

  Scenario scenario;
  Ownship ownship;
  TargetGenCfg targets_gen;
};

} // namespace cfg
