#include "config_loader.h"

#include "xml_utils.h"

#include <libxml/tree.h>
#include "path_utils.h"

#include <filesystem>
#include <cctype>
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

static SensorsCfg parse_sensors(const std::string& sensors_xml,
                                const std::string& xsd_dir) {
  SensorsCfg out{};

  if (sensors_xml.empty()) {
    return out;
  }

  // ---- Load + validate ----
  void* doc_void = cfg::xmlu::ReadXmlDocOrThrow(sensors_xml);
  struct DocGuard {
    void* d = nullptr;
    ~DocGuard() { if (d) cfg::xmlu::FreeXmlDoc(d); }
  } guard{doc_void};

  validate_if_enabled(doc_void, xsd_dir, sensors_xml);

  auto* doc = reinterpret_cast<xmlDocPtr>(doc_void);
  xmlNodePtr root = xmlDocGetRootElement(doc);
  if (!root) {
    throw std::runtime_error("Sensors XML has no root element: " + sensors_xml);
  }

  auto node_name_eq = [](xmlNodePtr n, const char* name) -> bool {
    return n && n->type == XML_ELEMENT_NODE && n->name &&
           (std::string(reinterpret_cast<const char*>(n->name)) == name);
  };

  auto get_attr = [](xmlNodePtr n, const char* attr) -> std::string {
    if (!n) return {};
    xmlChar* v = xmlGetProp(n, reinterpret_cast<const xmlChar*>(attr));
    if (!v) return {};
    std::string out_s(reinterpret_cast<const char*>(v));
    xmlFree(v);
    return out_s;
  };

  auto trim_in_place = [](std::string& s) {
    auto is_ws = [](unsigned char ch) { return std::isspace(ch) != 0; };
    while (!s.empty() && is_ws(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && is_ws(static_cast<unsigned char>(s.back()))) s.pop_back();
  };

  auto child_text = [&](xmlNodePtr parent, const char* child_name) -> std::string {
    if (!parent) return {};
    for (xmlNodePtr c = parent->children; c; c = c->next) {
      if (node_name_eq(c, child_name)) {
        xmlChar* txt = xmlNodeGetContent(c);
        std::string s = txt ? reinterpret_cast<const char*>(txt) : "";
        if (txt) xmlFree(txt);
        trim_in_place(s);
        return s;
      }
    }
    return {};
  };

  auto child_double = [&](xmlNodePtr parent, const char* child_name, double def) -> double {
    std::string s = child_text(parent, child_name);
    if (s.empty()) return def;
    try { return std::stod(s); } catch (...) { return def; }
  };

  auto child_u64 = [&](xmlNodePtr parent, const char* child_name, std::uint64_t def) -> std::uint64_t {
    std::string s = child_text(parent, child_name);
    if (s.empty()) return def;
    try { return static_cast<std::uint64_t>(std::stoull(s)); } catch (...) { return def; }
  };

  auto find_child = [&](xmlNodePtr parent, const char* child_name) -> xmlNodePtr {
    if (!parent) return nullptr;
    for (xmlNodePtr c = parent->children; c; c = c->next) {
      if (node_name_eq(c, child_name)) return c;
    }
    return nullptr;
  };

  auto child_attr_double = [&](xmlNodePtr parent,
                               const char* child_name,
                               const char* attr,
                               double def) -> double {
    xmlNodePtr c = find_child(parent, child_name);
    if (!c) return def;
    std::string s = get_attr(c, attr);
    if (s.empty()) return def;
    try { return std::stod(s); } catch (...) { return def; }
  };

  // ---- Iterate <Sensor id="..."> ----
  for (xmlNodePtr n = root->children; n; n = n->next) {
    if (!node_name_eq(n, "Sensor")) continue;

    SensorCfg s{};
    s.id = get_attr(n, "id");
    s.type = SensorTypeFromText(child_text(n, "Type"));

    // ScanVolume (optional)
    if (xmlNodePtr scan = find_child(n, "ScanVolume")) {
      s.scan.frame = child_text(scan, "Frame");


      bool have_frustum = false;

      // Frustum (optional)
      if (xmlNodePtr fr = find_child(scan, "Frustum")) {
        // Each of these elements is optional per XSD, so default safely to 0.
        s.scan.frustum.az_min_deg = child_attr_double(fr, "AzimuthDeg",   "min", 0.0);
        s.scan.frustum.az_max_deg = child_attr_double(fr, "AzimuthDeg",   "max", 0.0);
        s.scan.frustum.el_min_deg = child_attr_double(fr, "ElevationDeg", "min", 0.0);
        s.scan.frustum.el_max_deg = child_attr_double(fr, "ElevationDeg", "max", 0.0);
        s.scan.frustum.r_min_m    = child_attr_double(fr, "RangeMeters",  "min", 0.0);
        s.scan.frustum.r_max_m    = child_attr_double(fr, "RangeMeters",  "max", 0.0);

        // Consider frustum “present” if any bounds were explicitly provided or r_max > 0.
        have_frustum = (s.scan.frustum.r_max_m > 0.0) ||
                       (s.scan.frustum.az_min_deg != 0.0) || (s.scan.frustum.az_max_deg != 0.0) ||
                       (s.scan.frustum.el_min_deg != 0.0) || (s.scan.frustum.el_max_deg != 0.0) ||
                       (s.scan.frustum.r_min_m    != 0.0);
      }

      // Sphere (optional)
      if (!have_frustum) {
        if (xmlNodePtr sphere = find_child(scan, "Sphere")) {
          const double r = child_double(sphere, "RangeMeters", 0.0);
          if (r > 0.0) {
            s.scan.frustum.az_min_deg = -180.0;
            s.scan.frustum.az_max_deg =  180.0;
            s.scan.frustum.el_min_deg =  -90.0;
            s.scan.frustum.el_max_deg =   90.0;
            s.scan.frustum.r_min_m    =    0.0;
            s.scan.frustum.r_max_m    =      r;
          }
        }
      }
    }

    // Schedule (optional)
    if (xmlNodePtr sched = find_child(n, "Schedule")) {
      s.scan_rate_hz = child_double(sched, "ScanRateHz", 0.0);
      // (Scan events ignored for now)
    }

    out.sensors.push_back(std::move(s));
  }

  return out; // DocGuard frees
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

    // Index settings (optional; default values from config_types.h)
    p.index.query_aabb_inflate_m =
      xmlu::NodeGetDoubleChild(n, "QueryAabbInflateMeters", p.index.query_aabb_inflate_m);

    const std::string idx_backend = xmlu::NodeGetTextChild(n, "IndexBackend");
    if (!idx_backend.empty()) {
      p.index.backend = idx_backend;
    }

    p.index.cell_m = xmlu::NodeGetDoubleChild(n, "IndexCellMeters", p.index.cell_m);
    p.index.d_th_m = xmlu::NodeGetDoubleChild(n, "IndexMoveThresholdMeters", p.index.d_th_m);
    p.index.t_max_s = xmlu::NodeGetDoubleChild(n, "IndexMaxAgeSeconds", p.index.t_max_s);

    const int dense = xmlu::NodeGetIntChild(
      n,
      "IndexDenseCellProbeLimit",
      static_cast<int>(p.index.dense_cell_probe_limit));
    if (dense > 0) {
      p.index.dense_cell_probe_limit = static_cast<std::uint64_t>(dense);
    }

    const int commit_scans = xmlu::NodeGetIntChild(
      n,
      "WarmCommitEveryScans",
      static_cast<int>(p.warm_commit_every_scans));
    if (commit_scans > 0) {
      p.warm_commit_every_scans = static_cast<std::size_t>(commit_scans);
    }

    const int commit_tracks = xmlu::NodeGetIntChild(
      n,
      "WarmCommitAfterTracks",
      static_cast<int>(p.warm_commit_after_tracks));
    if (commit_tracks > 0) {
      p.warm_commit_after_tracks = static_cast<std::size_t>(commit_tracks);
    }

    // Sqlite child (optional)
    auto node_name_eq = [](xmlNodePtr node, const char* name) -> bool {
      return node && node->type == XML_ELEMENT_NODE && node->name &&
             (std::string(reinterpret_cast<const char*>(node->name)) == name);
    };
    auto find_child = [&](xmlNodePtr parent, const char* child_name) -> xmlNodePtr {
      if (!parent) return nullptr;
      for (xmlNodePtr c = parent->children; c; c = c->next) {
        if (node_name_eq(c, child_name)) return c;
      }
      return nullptr;
    };
    auto child_text = [&](xmlNodePtr parent, const char* child_name) -> std::string {
      xmlNodePtr c = find_child(parent, child_name);
      if (!c) return {};
      xmlChar* txt = xmlNodeGetContent(c);
      std::string s = txt ? reinterpret_cast<const char*>(txt) : "";
      if (txt) xmlFree(txt);
      // trim
      auto is_ws = [](unsigned char ch) { return std::isspace(ch) != 0; };
      while (!s.empty() && is_ws(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
      while (!s.empty() && is_ws(static_cast<unsigned char>(s.back()))) s.pop_back();
      return s;
    };

    xmlNodePtr node = reinterpret_cast<xmlNodePtr>(n);
    if (xmlNodePtr sqlite = find_child(node, "Sqlite")) {
      const std::string db_uri = child_text(sqlite, "DbUri");
      if (!db_uri.empty()) p.sqlite.db_uri = db_uri;
      const std::string journal = child_text(sqlite, "JournalMode");
      if (!journal.empty()) p.sqlite.journal_mode = journal;
      const std::string sync = child_text(sqlite, "Synchronous");
      if (!sync.empty()) p.sqlite.synchronous = sync;
    }

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

static ScenarioCatalog parse_scenario_catalog(void* doc_void) {
  ScenarioCatalog out;

  auto nodes = xmlu::FindNodes(doc_void, "ScenarioCatalog/Scenario");
  if (nodes.empty()) {
    throw std::runtime_error("No ScenarioCatalog/Scenario entries found.");
  }

  auto node_name_eq = [](xmlNodePtr n, const char* name) -> bool {
    return n && n->type == XML_ELEMENT_NODE && n->name &&
           (std::string(reinterpret_cast<const char*>(n->name)) == name);
  };

  auto get_attr = [](xmlNodePtr n, const char* attr) -> std::string {
    if (!n) return {};
    xmlChar* v = xmlGetProp(n, reinterpret_cast<const xmlChar*>(attr));
    if (!v) return {};
    std::string s(reinterpret_cast<const char*>(v));
    xmlFree(v);
    return s;
  };

  auto find_child = [&](xmlNodePtr parent, const char* child_name) -> xmlNodePtr {
    if (!parent) return nullptr;
    for (xmlNodePtr c = parent->children; c; c = c->next) {
      if (node_name_eq(c, child_name)) return c;
    }
    return nullptr;
  };

  auto trim_in_place = [](std::string& s) {
    auto is_ws = [](unsigned char ch) { return std::isspace(ch) != 0; };
    while (!s.empty() && is_ws(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && is_ws(static_cast<unsigned char>(s.back()))) s.pop_back();
  };

  auto child_text = [&](xmlNodePtr parent, const char* child_name) -> std::string {
    xmlNodePtr c = find_child(parent, child_name);
    if (!c) return {};
    xmlChar* txt = xmlNodeGetContent(c);
    std::string s = txt ? reinterpret_cast<const char*>(txt) : "";
    if (txt) xmlFree(txt);
    trim_in_place(s);
    return s;
  };

  for (void* n_void : nodes) {
    xmlNodePtr scen = reinterpret_cast<xmlNodePtr>(n_void);

    Scenario s{};
    s.id = get_attr(scen, "id");
    if (s.id.empty()) {
      throw std::runtime_error("ScenarioCatalog/Scenario missing id attribute.");
    }
    s.name = child_text(scen, "Name");

    // ---- Refs ----
    if (xmlNodePtr refs = find_child(scen, "Refs")) {
      s.refs.base_dir = get_attr(refs, "baseDir");

      if (xmlNodePtr own = find_child(refs, "Ownship")) {
        s.refs.ownship_href = get_attr(own, "href");
      }
      if (xmlNodePtr tgt = find_child(refs, "Targets")) {
        s.refs.targets_source_type = get_attr(tgt, "sourceType");
        s.refs.targets_href        = get_attr(tgt, "href");
        const std::string use_db   = get_attr(tgt, "usePrepopulatedDb");
        if (!use_db.empty()) {
          s.refs.targets_use_prepopulated_db =
            (use_db == "true" || use_db == "1" || use_db == "TRUE");
        }
      }
    }

    // ---- LoadPolicy ----
    if (xmlNodePtr lp = find_child(scen, "LoadPolicy")) {
      if (xmlNodePtr ifdb = find_child(lp, "IfDatabasePopulated")) {
        s.load_policy.if_db_populated_action = get_attr(ifdb, "action");
      }

      // Optional: if your Scenario struct has somewhere to store these later,
      // you can parse OnCliFlag nodes here. For now you said it’s OK to defer.
      //
      // Example:
      // for (xmlNodePtr c = lp->children; c; c = c->next) {
      //   if (node_name_eq(c, "OnCliFlag")) { ... }
      // }
    }

    out.by_id[s.id] = std::move(s);
  }

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
  
  if (!bundle.system.refs.sensors_href.empty()) {
    bundle.paths.sensors_xml = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.sensors_href);
  }
  // Optional but consistent with system.xml having it:
  if (!bundle.system.refs.gating_assoc_href.empty()) {
    bundle.paths.gating_assoc_xml = pathu::ResolveHref(bundle.paths.system_xml, base, bundle.system.refs.gating_assoc_href);
  }
 
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

  
// 4b) sensors.xml (optional)
if (!bundle.paths.sensors_xml.empty()) {
  bundle.sensors = parse_sensors(bundle.paths.sensors_xml, xsd_dir);
  bundle.has_sensors = !bundle.sensors.sensors.empty();
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
  xmlu::FreeXmlDoc(sc_doc);

  auto it_s = sc.by_id.find(bundle.system.active.scenario_id);
  if (it_s == sc.by_id.end()) {
    throw std::runtime_error("Active Scenario id not found: " + bundle.system.active.scenario_id);
  }

  bundle.scenario = it_s->second;

  if (bundle.scenario.refs.ownship_href.empty() || bundle.scenario.refs.targets_href.empty()) {
    throw std::runtime_error("Scenario '" + bundle.scenario.id + "' is missing Ownship/Targets href.");
  }

  // Resolve ownship and targets relative to scenario file and its baseDir (NOT system baseDir)
  bundle.paths.ownship_xml = pathu::ResolveHref(
      bundle.paths.scenario_xml,
      bundle.scenario.refs.base_dir,
      bundle.scenario.refs.ownship_href);

  bundle.paths.targets_xml = pathu::ResolveHref(
      bundle.paths.scenario_xml,
      bundle.scenario.refs.base_dir,
      bundle.scenario.refs.targets_href);


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
