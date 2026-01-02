/**
 * @file config_check.cpp
 * @brief Minimal executable that loads + (optionally) validates + prints tracker config bundle.
 *
 * This is intentionally separate from src/main.cpp to avoid disrupting the existing LA demo.
 */
#include "config_loader.h"

#include <iostream>
#include <string>

static const char* sensor_type_to_text(cfg::SensorType t) {
  switch (t) {
    case cfg::SensorType::RADAR: return "RADAR";
    case cfg::SensorType::ESM:   return "ESM";
    case cfg::SensorType::EOIR:  return "EOIR";
    default:                     return "UNKNOWN";
  }
}

static void print_bundle(const cfg::ConfigBundle& b) {
  std::cout << "=== ConfigBundle Summary ===\n";
  std::cout << "system.xml: " << b.paths.system_xml << "\n";
  std::cout << "xsd_dir:    " << (b.paths.xsd_dir.empty() ? "(none)" : b.paths.xsd_dir) << "\n\n";

  std::cout << "[Active]\n";
  std::cout << "  RuntimeProfile: " << b.system.active.runtime_profile_id << "\n";
  std::cout << "  StoreProfile:   " << b.system.active.store_profile_id << "\n";
  std::cout << "  Scenario:       " << b.system.active.scenario_id << "\n\n";

  std::cout << "[Resolved Paths]\n";
  std::cout << "  runtime_profiles: " << b.paths.runtime_profiles_xml << "\n";
  std::cout << "  tracker_model:    " << b.paths.tracker_model_xml << "\n";
  if (!b.paths.performance_xml.empty()) std::cout << "  performance:      " << b.paths.performance_xml << "\n";
  if (!b.paths.sensors_xml.empty())     std::cout << "  sensors:          " << b.paths.sensors_xml << "\n";
  if (!b.paths.gating_assoc_xml.empty()) std::cout << "  gating_assoc:     " << b.paths.gating_assoc_xml << "\n";
  std::cout << "  store:            " << b.paths.store_xml << "\n";
  if (!b.paths.persistence_xml.empty()) std::cout << "  persistence:      " << b.paths.persistence_xml << "\n";
  std::cout << "  scenario:         " << b.paths.scenario_xml << "\n";
  std::cout << "  ownship:          " << b.paths.ownship_xml << "\n";
  std::cout << "  targets:          " << b.paths.targets_xml << "\n\n";

  std::cout << "[RuntimeProfile]\n";
  std::cout << "  id=" << b.runtime.id
            << " rate_hz=" << b.runtime.update_rate_hz
            << " max_tracks=" << b.runtime.max_tracks_active << "\n\n";

  std::cout << "[TrackerModel]\n";
  std::cout << "  frame=" << b.tracker_model.state.frame
            << " dim=" << b.tracker_model.state.dim
            << " order=\"" << b.tracker_model.state.order << "\"\n";
  std::cout << "  process.type=" << (b.tracker_model.process.type.empty() ? "(none)" : b.tracker_model.process.type)
            << " default_dt=" << b.tracker_model.process.default_dt_s << "\n";
  std::cout << "  cov.storage=" << b.tracker_model.cov.storage << "\n\n";

  std::cout << "[StoreProfile]\n";
  std::cout << "  id=" << b.store_profile.id
            << " mode=" << b.store_profile.mode
            << " backend=" << b.store_profile.backend << "\n";
  std::cout << "  index.backend=" << b.store_profile.index.backend
            << " inflate_m=" << b.store_profile.index.query_aabb_inflate_m
            << " cell_m=" << b.store_profile.index.cell_m
            << " d_th_m=" << b.store_profile.index.d_th_m
            << " t_max_s=" << b.store_profile.index.t_max_s
            << " dense_limit=" << b.store_profile.index.dense_cell_probe_limit
            << "\n\n";

  std::cout << "[Scenario]\n";
  std::cout << "  id=" << b.scenario.id << " name=\"" << b.scenario.name << "\"\n";
  std::cout << "  ownship_href=" << b.scenario.refs.ownship_href << "\n";
  std::cout << "  targets_sourceType=" << b.scenario.refs.targets_source_type
            << " targets_href=" << b.scenario.refs.targets_href << "\n\n";

  if (b.has_sensors) {
    std::cout << "[Sensors]\n";
    std::cout << "  count=" << b.sensors.sensors.size() << "\n";
    for (const auto& s : b.sensors.sensors) {
      std::cout << "  - id=" << s.id
                << " type=" << sensor_type_to_text(s.type)
                << " frame=" << s.scan.frame
                << " frustum(az=" << s.scan.frustum.az_min_deg << ".." << s.scan.frustum.az_max_deg
                << " el=" << s.scan.frustum.el_min_deg << ".." << s.scan.frustum.el_max_deg
                << " r=" << s.scan.frustum.r_min_m << ".." << s.scan.frustum.r_max_m << ")"
                << " rate_hz=" << s.scan_rate_hz
                << "\n";
    }
    std::cout << "\n";
  }

  std::cout << "[Ownship]\n";
  std::cout << "  frame=" << b.ownship.frame
            << " pos=(" << b.ownship.pos_x << "," << b.ownship.pos_y << "," << b.ownship.pos_z << ")\n";
  if (b.ownship.has_vel) {
    std::cout << "  vel=(" << b.ownship.vel_x << "," << b.ownship.vel_y << "," << b.ownship.vel_z << ")\n";
  }

  std::cout << "\n[TargetsGen]\n";
  std::cout << "  count=" << b.targets_gen.count << " seed=" << b.targets_gen.seed << "\n";
}

int main(int argc, char** argv) {
  std::string system_xml = "config/system.xml";
  std::string xsd_dir = ""; // e.g., "./schemas"

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      system_xml = argv[++i];
    } else if (a == "--xsd-dir" && i + 1 < argc) {
      xsd_dir = argv[++i];
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: config_check [--config <system.xml>] [--xsd-dir <dir>]\n";
      return 0;
    }
  }

  try {
    cfg::ConfigBundle bundle = cfg::ConfigLoader::Load(system_xml, xsd_dir);
    print_bundle(bundle);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Config load failed: " << e.what() << "\n";
    return 2;
  }
}
