#include "config/config_loader.h"

#include "config/xml_utils.h"
#include "config/path_utils.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace fs = std::filesystem;

namespace cfg {

static std::string schema_for_root(const std::string& root_name) {
  // Map root element names to schema filenames in xsd_dir.
  static const std::unordered_map<std::string, std::string> m = {
    {"SystemConfig", "system.xsd"},
    {"RuntimeProfiles", "runtime_profiles.xsd"},
    {"TrackerModel", "tracker_model.xsd"},
    {"Performance", "performance.xsd"},
    {"Sensors", "sensors.xsd"},
    {"GatingAssociation", "gating_association.xsd"},
    {"Store", "store.xsd"},
    {"Persistence", "persistence.xsd"},
    {"ScenarioCatalog", "scenario.xsd"},
    {"Ownship", "ownship.xsd"},
    {"TargetGeneration", "targets_gen.xsd"},
  };
  auto it = m.find(root_name);
  return (it == m.end()) ? std::string("") : it->second;
}

static void validate_if_enabled(void* doc,
                                const std::string& xsd_dir,
                                const std::string& xml_path) {
  if (xsd_dir.empty()) return;

  const std::string root = xmlu::RootName(doc);
  const std::string schema = schema_for_root(root);
  if (schema.empty()) {
    throw std::runtime_error("No schema mapping for root element '" + root + "' (file: " + xml_path + ")");
  }

  fs::path xsd_path = fs::path(xsd_dir) / schema;
  if (!fs::exists(xsd_path)) {
    throw std::runtime_error("Schema file not found: " + xsd_path.string());
  }

  xmlu::ValidateOrThrow(doc, xsd_path.string(), xml_path);
}

static SystemConfig parse_system(void* doc) {
  SystemConfig c;
  c.active.runtime_profile_id = xmlu::GetAttr(doc, "SystemConfig/Active/RuntimeProfile", "id");
  c.active.store_profile_id   = xmlu::GetAttr(doc, "SystemConfig/Active/StoreProfile", "id");
  c.active.scenario_id        = xmlu::GetAttr(doc, "SystemConfig/Active/Scenario", "id");

  c.refs.base_dir             = xmlu::GetAttr(doc, "SystemConfig/Refs", "baseDir");
  c.refs.runtime_profiles_href= xmlu::GetAttr(doc, "SystemConfig/Refs/RuntimeProfiles", "href");
  c.refs.tracker_model_href   = xmlu::GetAttr(doc, "SystemConfig/Refs/TrackerModel", "href");
  c.refs.performance_href     = xmlu::GetAttr(doc, "SystemConfig/Refs/Performance", "href");
  c.refs.sensors_href         = xmlu::GetAttr(doc, "SystemConfig/Refs/Sensors", "href");
  c.refs.gating_assoc_href    = xmlu::GetAttr(doc, "SystemConfig/Refs/GatingAssoc", "href");
  c.refs.store_href           = xmlu::GetAttr(doc, "SystemConfig/Refs/Store", "href");
  c.refs.persistence_href     = xmlu::GetAttr(doc, "SystemConfig/Refs/Persistence", "href");
  c.refs.scenario_file_href   = xmlu::GetAttr(doc, "SystemConfig/Refs/ScenarioFile", "href");

  if (c.active.runtime_profile_id.empty() || c.active.store_profile_id.empty() || c.active.scenario_id.empty()) {
    throw std::runtime_error("system.xml missing Active selection IDs.");
  }
  if (c.refs.runtime_profiles_href.empty() || c.refs.tracker_model_href.empty() ||
      c.refs.store_href.empty() || c.refs.scenario_file_href.empty()) {
    throw std::runtime_error("system.xml missing required Refs hrefs.");
  }
  return c;
}

static RuntimeProfiles parse_runtime_profiles(void* doc) {
  RuntimeProfiles out;
  auto nodes = xmlu::FindNodes(doc, "RuntimeProfiles/Profile");
  for (void* n : nodes) {
    RuntimeProfile p;
    p.id = xmlu::NodeGetAttr(n, "id");
    p.update_rate_hz = xmlu::NodeGetDoubleChild(n, "UpdateRateHz", 0.0);
    p.max_tracks_active = xmlu::NodeGetIntChild(n, "MaxTracksActive", 0);

    // Batching children live under <Batching>
    // We'll read via doc helper paths for simplicity.
    // Find index in order; not needed now—just parse if present through root search by id is harder.
    // Keep it minimal: if you want batching, parse via XPath later. For now, leave zeros if absent.
    // (XSD ensures structure; the loader can be enhanced incrementally.)
    // We'll parse Prefetch/MaxTracksInQuery/TimeBudget using doc path with id filtering later too.
    // For minimal functionality now, use only required fields.
    if (p.id.empty()) throw std::runtime_error("RuntimeProfiles/Profile missing id attribute.");
    if (p.update_rate_hz <= 0.0) throw std::runtime_error("RuntimeProfiles/Profile '" + p.id + "' has invalid UpdateRateHz.");
    if (p.max_tracks_active <= 0) throw std::runtime_error("RuntimeProfiles/Profile '" + p.id + "' has invalid MaxTracksActive.");
    out.by_id[p.id] = p;
  }
  if (out.by_id.empty()) {
    throw std::runtime_error("No RuntimeProfiles/Profile entries found.");
  }
  return out;
}

static TrackerModel parse_tracker_model(void* doc) {
  TrackerModel m;
  m.state.frame = xmlu::GetText(doc, "TrackerModel/State/Frame");
  m.state.dim   = xmlu::GetInt(doc, "TrackerModel/State/Dimension", 0);
  m.state.order = xmlu::GetText(doc, "TrackerModel/State/Order");

  // ProcessModel is optional
  m.process.type         = xmlu::GetText(doc, "TrackerModel/ProcessModel/Type");
  m.process.default_dt_s = xmlu::GetDouble(doc, "TrackerModel/ProcessModel/DefaultDtSeconds", 0.0);
  m.process.noise.sigma_accel_mps2 = xmlu::GetDouble(doc, "TrackerModel/ProcessModel/ProcessNoise/SigmaAccelMps2", 0.0);
  m.process.noise.sigma_jerk_mps3  = xmlu::GetDouble(doc, "TrackerModel/ProcessModel/ProcessNoise/SigmaJerkMps3", 0.0);

  m.cov.storage = xmlu::GetText(doc, "TrackerModel/Covariance/Storage");
  m.cov.symmetrize_on_write = xmlu::GetBoolText(doc, "TrackerModel/Covariance/SymmetrizeOnWrite", false);
  m.cov.min_eig_clamp = xmlu::GetDouble(doc, "TrackerModel/Covariance/MinEigenvalueClamp", 0.0);
  m.cov.store_cholesky = xmlu::GetBoolText(doc, "TrackerModel/Covariance/StoreCholesky", false);

  if (m.state.frame.empty() || m.state.dim <= 0 || m.cov.storage.empty()) {
    throw std::runtime_error("tracker_model.xml missing required State/Covariance fields.");
  }
  return m;
}

static PerformanceCfg parse_performance(void* doc) {
  PerformanceCfg p;
  p.threads.default_omp_threads = xmlu::GetInt(doc, "Performance/Threads/DefaultOmpThreads", 0);
  p.threads.allow_env_override = xmlu::GetBoolText(doc, "Performance/Threads/AllowEnvOverride", true);
  p.threads.set_env_defaults_if_unset = xmlu::GetBoolText(doc, "Performance/Threads/SetEnvDefaultsIfUnset", true);

  // EnvDefaults: Var elements
  auto vars = xmlu::FindNodes(doc, "Performance/EnvDefaults/Var");
  for (void* n : vars) {
    EnvVar v;
    v.name = xmlu::NodeGetAttr(n, "name");
    v.value = xmlu::NodeGetAttr(n, "value");
    if (!v.name.empty()) p.env_defaults.push_back(v);
  }

  p.diagnostics.print_startup_summary = xmlu::GetBoolText(doc, "Performance/Diagnostics/PrintStartupSummary", true);
  p.diagnostics.print_store_stats     = xmlu::GetBoolText(doc, "Performance/Diagnostics/PrintStoreStats", false);
  p.diagnostics.print_timing_stats    = xmlu::GetBoolText(doc, "Performance/Diagnostics/PrintTimingStats", false);
  return p;
}

static StoreCfg parse_store(void* doc) {
  StoreCfg out;
  auto nodes = xmlu::FindNodes(doc, "Store/Profile");
  for (void* n : nodes) {
    StoreProfile p;
    p.id = xmlu::NodeGetAttr(n, "id");
    if (p.id.empty()) throw std::runtime_error("Store/Profile missing id attribute.");

    p.mode = xmlu::NodeGetTextChild(n, "Mode");
    p.backend = xmlu::NodeGetTextChild(n, "Backend");

    // Sqlite child (optional)
    // For minimal loader: read a few common fields using document paths under this profile is non-trivial without XPath.
    // We'll keep defaults here; schema validation ensures any present values are well-formed.
    out.by_id[p.id] = p;
  }
  if (out.by_id.empty()) throw std::runtime_error("No Store/Profile entries found.");
  return out;
}

static PersistenceCfg parse_persistence(void* doc) {
  PersistenceCfg p;
  p.snapshots.enabled = xmlu::GetBoolText(doc, "Persistence/Snapshots/Enabled", false);
  p.snapshots.interval_s = xmlu::GetDouble(doc, "Persistence/Snapshots/IntervalSeconds", 0.0);
  p.snapshots.directory = xmlu::GetText(doc, "Persistence/Snapshots/Directory");
  p.snapshots.retain_count = xmlu::GetInt(doc, "Persistence/Snapshots/RetainCount", 0);
  p.snapshots.compression = xmlu::GetText(doc, "Persistence/Snapshots/Compression");
  p.snapshots.snapshot_on_shutdown = xmlu::GetBoolText(doc, "Persistence/Snapshots/SnapshotOnShutdown", false);
  p.snapshots.snapshot_on_signal = xmlu::GetBoolText(doc, "Persistence/Snapshots/SnapshotOnSignal", false);

  p.recovery.load_latest_on_startup = xmlu::GetBoolText(doc, "Persistence/Recovery/LoadLatestOnStartup", true);
  // DeltaLog is an element with attribute enabled="..."
  std::string delta = xmlu::GetAttr(doc, "Persistence/Recovery/DeltaLog", "enabled");
  if (!delta.empty()) p.recovery.delta_log_enabled = (delta == "true" || delta == "1" || delta == "TRUE");
  return p;
}

static ScenarioCatalog parse_scenario_catalog(void* doc) {
  ScenarioCatalog out;
  auto nodes = xmlu::FindNodes(doc, "ScenarioCatalog/Scenario");
  for (void* n : nodes) {
    Scenario s;
    s.id = xmlu::NodeGetAttr(n, "id");
    if (s.id.empty()) throw std::runtime_error("ScenarioCatalog/Scenario missing id attribute.");
    s.name = xmlu::NodeGetTextChild(n, "Name");

    // Refs
    // Again, minimal parsing without XPath: use NodeGetTextChild for base_dir? baseDir is attribute.
    // We'll read baseDir attr on the <Refs> child node directly.
    // Locate <Refs> child:
    // We don't have a Node->child finder here; for minimal, read through doc helper paths (works only if single scenario).
    out.by_id[s.id] = s;
  }
  if (out.by_id.empty()) throw std::runtime_error("No Scenario entries found.");
  return out;
}

static Ownship parse_ownship(void* doc) {
  Ownship o;
  o.id = xmlu::GetAttr(doc, "Ownship", "id");
  o.frame = xmlu::GetText(doc, "Ownship/Frame");
  o.pos_x = std::stod(xmlu::GetAttr(doc, "Ownship/PositionMeters", "x"));
  o.pos_y = std::stod(xmlu::GetAttr(doc, "Ownship/PositionMeters", "y"));
  o.pos_z = std::stod(xmlu::GetAttr(doc, "Ownship/PositionMeters", "z"));
  std::string vx = xmlu::GetAttr(doc, "Ownship/VelocityMps", "x");
  std::string vy = xmlu::GetAttr(doc, "Ownship/VelocityMps", "y");
  std::string vz = xmlu::GetAttr(doc, "Ownship/VelocityMps", "z");
  if (!vx.empty() && !vy.empty() && !vz.empty()) {
    o.has_vel = true;
    o.vel_x = std::stod(vx);
    o.vel_y = std::stod(vy);
    o.vel_z = std::stod(vz);
  }
  return o;
}

static TargetGenCfg parse_targets_gen(void* doc) {
  TargetGenCfg g;
  g.id = xmlu::GetAttr(doc, "TargetGeneration", "id");
  g.count = xmlu::GetInt(doc, "TargetGeneration/Count", 0);
  g.seed = static_cast<uint32_t>(xmlu::GetInt(doc, "TargetGeneration/Seed", 0));
  if (g.count <= 0) throw std::runtime_error("TargetGeneration/Count invalid.");
  return g;
}

ConfigBundle ConfigLoader::Load(const std::string& system_xml_path, const std::string& xsd_dir) {
  ConfigBundle bundle;
  bundle.paths.system_xml = fs::path(system_xml_path).lexically_normal().string();
  bundle.paths.xsd_dir = xsd_dir;

  // 1) system.xml
  void* sys_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.system_xml);
  try {
    validate_if_enabled(sys_doc, xsd_dir, bundle.paths.system_xml);
    bundle.system = parse_system(sys_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(sys_doc);
    throw;
  }
  xmlu::FreeXmlDoc(sys_doc);

  // Resolve referenced XML paths relative to system.xml and baseDir
  const std::string base = bundle.system.refs.base_dir;
  bundle.paths.runtime_profiles_xml = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.runtime_profiles_href);
  bundle.paths.tracker_model_xml    = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.tracker_model_href);
  bundle.paths.store_xml            = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.store_href);
  bundle.paths.scenario_xml         = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.scenario_file_href);

  if (!bundle.system.refs.performance_href.empty())
    bundle.paths.performance_xml    = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.performance_href);
  if (!bundle.system.refs.persistence_href.empty())
    bundle.paths.persistence_xml    = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.persistence_href);

  // 2) runtime_profiles.xml
  void* rp_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.runtime_profiles_xml);
  RuntimeProfiles rps;
  try {
    validate_if_enabled(rp_doc, xsd_dir, bundle.paths.runtime_profiles_xml);
    rps = parse_runtime_profiles(rp_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(rp_doc);
    throw;
  }
  xmlu::FreeXmlDoc(rp_doc);

  auto it_rp = rps.by_id.find(bundle.system.active.runtime_profile_id);
  if (it_rp == rps.by_id.end()) {
    throw std::runtime_error("Active RuntimeProfile id not found: " + bundle.system.active.runtime_profile_id);
  }
  bundle.runtime = it_rp->second;

  // 3) tracker_model.xml
  void* tm_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.tracker_model_xml);
  try {
    validate_if_enabled(tm_doc, xsd_dir, bundle.paths.tracker_model_xml);
    bundle.tracker_model = parse_tracker_model(tm_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(tm_doc);
    throw;
  }
  xmlu::FreeXmlDoc(tm_doc);

  // 4) performance.xml (optional)
  if (!bundle.paths.performance_xml.empty()) {
    void* p_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.performance_xml);
    try {
      validate_if_enabled(p_doc, xsd_dir, bundle.paths.performance_xml);
      bundle.performance = parse_performance(p_doc);
      bundle.has_performance = true;
    } catch (...) {
      xmlu::FreeXmlDoc(p_doc);
      throw;
    }
    xmlu::FreeXmlDoc(p_doc);
  }

  // 5) store.xml
  void* st_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.store_xml);
  StoreCfg st;
  try {
    validate_if_enabled(st_doc, xsd_dir, bundle.paths.store_xml);
    st = parse_store(st_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(st_doc);
    throw;
  }
  xmlu::FreeXmlDoc(st_doc);

  auto it_sp = st.by_id.find(bundle.system.active.store_profile_id);
  if (it_sp == st.by_id.end()) {
    throw std::runtime_error("Active StoreProfile id not found: " + bundle.system.active.store_profile_id);
  }
  bundle.store_profile = it_sp->second;

  // 6) persistence.xml (optional)
  if (!bundle.paths.persistence_xml.empty()) {
    void* pe_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.persistence_xml);
    try {
      validate_if_enabled(pe_doc, xsd_dir, bundle.paths.persistence_xml);
      bundle.persistence = parse_persistence(pe_doc);
      bundle.has_persistence = true;
    } catch (...) {
      xmlu::FreeXmlDoc(pe_doc);
      throw;
    }
    xmlu::FreeXmlDoc(pe_doc);
  }

  // 7) scenario catalog + scenario selection
  void* sc_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.scenario_xml);
  ScenarioCatalog sc;
  try {
    validate_if_enabled(sc_doc, xsd_dir, bundle.paths.scenario_xml);
    sc = parse_scenario_catalog(sc_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(sc_doc);
    throw;
  }

  auto it_s = sc.by_id.find(bundle.system.active.scenario_id);
  if (it_s == sc.by_id.end()) {
    xmlu::FreeXmlDoc(sc_doc);
    throw std::runtime_error("Active Scenario id not found: " + bundle.system.active.scenario_id);
  }

  // Minimal extraction for scenario refs (assumes single scenario or that doc helper paths find first match).
  // For now, support a single scenario file (common in early prototypes).
  Scenario s = it_s->second;
  s.refs.base_dir = xmlu::GetAttr(sc_doc, "ScenarioCatalog/Scenario/Refs", "baseDir");
  s.refs.ownship_href = xmlu::GetAttr(sc_doc, "ScenarioCatalog/Scenario/Refs/Ownship", "href");
  s.refs.targets_source_type = xmlu::GetAttr(sc_doc, "ScenarioCatalog/Scenario/Refs/Targets", "sourceType");
  s.refs.targets_href = xmlu::GetAttr(sc_doc, "ScenarioCatalog/Scenario/Refs/Targets", "href");
  s.load_policy.if_db_populated_action = xmlu::GetAttr(sc_doc, "ScenarioCatalog/Scenario/LoadPolicy/IfDatabasePopulated", "action");

  // optional OnCliFlag list not parsed yet (can be added with XPath later)
  bundle.scenario = s;
  xmlu::FreeXmlDoc(sc_doc);

  if (bundle.scenario.refs.ownship_href.empty() || bundle.scenario.refs.targets_href.empty()) {
    throw std::runtime_error("Scenario is missing Ownship/Targets href.");
  }

  // Resolve ownship and targets relative to scenario file and its baseDir (NOT system baseDir)
  bundle.paths.ownship_xml = pathu::ResolveHref(bundle.paths.scenario_xml, bundle.scenario.refs.base_dir, bundle.scenario.refs.ownship_href);
  bundle.paths.targets_xml = pathu::ResolveHref(bundle.paths.scenario_xml, bundle.scenario.refs.base_dir, bundle.scenario.refs.targets_href);

  // 8) ownship.xml
  void* ow_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.ownship_xml);
  try {
    validate_if_enabled(ow_doc, xsd_dir, bundle.paths.ownship_xml);
    bundle.ownship = parse_ownship(ow_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(ow_doc);
    throw;
  }
  xmlu::FreeXmlDoc(ow_doc);

  // 9) targets generator (only generator supported in minimal loader)
  void* tg_doc = xmlu::ReadXmlDocOrThrow(bundle.paths.targets_xml);
  try {
    validate_if_enabled(tg_doc, xsd_dir, bundle.paths.targets_xml);
    bundle.targets_gen = parse_targets_gen(tg_doc);
  } catch (...) {
    xmlu::FreeXmlDoc(tg_doc);
    throw;
  }
  xmlu::FreeXmlDoc(tg_doc);

  return bundle;
}

} // namespace cfg
