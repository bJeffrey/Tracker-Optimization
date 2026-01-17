/**
 * @file gate_sizing_tool.cpp
 * @brief Standalone tool to compute coarse gate sizing from sensor + sizing configs.
 */
#include "config/xml_utils.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct SensorInfo {
  std::string id;
  double beamwidth_3db_az_deg = 0.0;
  double beamwidth_3db_el_deg = 0.0;
  double scan_rate_hz = 0.0;
  double scan_period_s = 0.0;
  std::string scan_frame;
  double az_min_deg = 0.0;
  double az_max_deg = 0.0;
  double el_min_deg = 0.0;
  double el_max_deg = 0.0;
  double r_min_m = 0.0;
  double r_max_m = 0.0;
};

struct GateSizingCfg {
  double max_speed_mps = 0.0;
  double velocity_buffer_fraction = 0.0;
  double max_altitude_m = 0.0;
  double altitude_bin_m = 0.0;
};

bool file_exists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

std::vector<SensorInfo> parse_sensors(const std::string& sensors_xml,
                                      const std::string& xsd_dir) {
  std::vector<SensorInfo> out;

  void* doc_void = cfg::xmlu::ReadXmlDocOrThrow(sensors_xml);
  struct DocGuard {
    void* d = nullptr;
    ~DocGuard() { if (d) cfg::xmlu::FreeXmlDoc(d); }
  } guard{doc_void};

  if (!xsd_dir.empty()) {
    const std::string xsd_path = xsd_dir + "/sensors.xsd";
    if (file_exists(xsd_path)) {
      cfg::xmlu::ValidateOrThrow(doc_void, xsd_path, sensors_xml);
    }
  }

  auto nodes = cfg::xmlu::FindNodes(doc_void, "Sensor");
  for (void* n : nodes) {
    SensorInfo s{};
    s.id = cfg::xmlu::NodeGetAttr(n, "id");
    s.beamwidth_3db_az_deg = 0.0;
    s.beamwidth_3db_el_deg = 0.0;
    {
      const std::string az = cfg::xmlu::NodeGetAttrPath(n, "Beamwidth3dBDeg", "azimuth");
      const std::string el = cfg::xmlu::NodeGetAttrPath(n, "Beamwidth3dBDeg", "elevation");
      if (!az.empty()) s.beamwidth_3db_az_deg = std::stod(az);
      if (!el.empty()) s.beamwidth_3db_el_deg = std::stod(el);
    }

    s.scan_frame = cfg::xmlu::NodeGetTextPath(n, "ScanVolume/Frame");
    {
      const std::string az_min = cfg::xmlu::NodeGetAttrPath(n, "ScanVolume/Frustum/AzimuthDeg", "min");
      const std::string az_max = cfg::xmlu::NodeGetAttrPath(n, "ScanVolume/Frustum/AzimuthDeg", "max");
      const std::string el_min = cfg::xmlu::NodeGetAttrPath(n, "ScanVolume/Frustum/ElevationDeg", "min");
      const std::string el_max = cfg::xmlu::NodeGetAttrPath(n, "ScanVolume/Frustum/ElevationDeg", "max");
      const std::string r_min = cfg::xmlu::NodeGetAttrPath(n, "ScanVolume/Frustum/RangeMeters", "min");
      const std::string r_max = cfg::xmlu::NodeGetAttrPath(n, "ScanVolume/Frustum/RangeMeters", "max");
      if (!az_min.empty()) s.az_min_deg = std::stod(az_min);
      if (!az_max.empty()) s.az_max_deg = std::stod(az_max);
      if (!el_min.empty()) s.el_min_deg = std::stod(el_min);
      if (!el_max.empty()) s.el_max_deg = std::stod(el_max);
      if (!r_min.empty()) s.r_min_m = std::stod(r_min);
      if (!r_max.empty()) s.r_max_m = std::stod(r_max);
    }

    s.scan_rate_hz = cfg::xmlu::NodeGetDoublePath(n, "Schedule/ScanRateHz", 0.0);
    s.scan_period_s = cfg::xmlu::NodeGetDoublePath(n, "Schedule/ScanPeriodSeconds", 0.0);
    if (s.scan_period_s <= 0.0 && s.scan_rate_hz > 0.0) {
      s.scan_period_s = 1.0 / s.scan_rate_hz;
    }

    out.push_back(std::move(s));
  }

  return out;
}

GateSizingCfg parse_gate_sizing(const std::string& gate_sizing_xml,
                                const std::string& xsd_dir) {
  GateSizingCfg cfg;

  void* doc_void = cfg::xmlu::ReadXmlDocOrThrow(gate_sizing_xml);
  struct DocGuard {
    void* d = nullptr;
    ~DocGuard() { if (d) cfg::xmlu::FreeXmlDoc(d); }
  } guard{doc_void};

  if (!xsd_dir.empty()) {
    const std::string xsd_path = xsd_dir + "/gate_sizing.xsd";
    if (file_exists(xsd_path)) {
      cfg::xmlu::ValidateOrThrow(doc_void, xsd_path, gate_sizing_xml);
    }
  }

  cfg.max_speed_mps = cfg::xmlu::GetDouble(doc_void, "Planar/MaxTargetSpeedMps", 0.0);
  cfg.velocity_buffer_fraction =
    cfg::xmlu::GetDouble(doc_void, "Planar/VelocityBufferFraction", 0.0);
  cfg.max_altitude_m =
    cfg::xmlu::GetDouble(doc_void, "AltitudeBinning/MaxTargetAltitudeMeters", 0.0);
  cfg.altitude_bin_m =
    cfg::xmlu::GetDouble(doc_void, "AltitudeBinning/AltitudeBinMeters", 0.0);

  return cfg;
}

std::vector<double> build_altitude_bins(double max_alt_m, double bin_m) {
  std::vector<double> out;
  const double max_alt = std::max(0.0, max_alt_m);
  if (bin_m <= 0.0) {
    out.push_back(max_alt);
    return out;
  }

  for (double alt = 0.0; alt <= max_alt + 1e-9; alt += bin_m) {
    out.push_back(alt);
  }
  if (out.empty() || out.back() + 1e-9 < max_alt) {
    out.push_back(max_alt);
  }

  return out;
}

double deg2rad(double d) {
  constexpr double kPi = 3.14159265358979323846;
  return d * kPi / 180.0;
}

bool update_course_gate_sizes(const std::string& gating_assoc_xml,
                              double x_side_m,
                              double y_side_m,
                              double z_side_m) {
  void* doc_void = nullptr;
  try {
    doc_void = cfg::xmlu::ReadXmlDocOrThrow(gating_assoc_xml);
    struct DocGuard {
      void* d = nullptr;
      ~DocGuard() { if (d) cfg::xmlu::FreeXmlDoc(d); }
    } guard{doc_void};

    cfg::xmlu::EnsurePathSetAttr(doc_void, "GatingAssociation/CourseGates/SurfaceGateSideMeters",
                                 "x", std::to_string(x_side_m));
    cfg::xmlu::EnsurePathSetAttr(doc_void, "GatingAssociation/CourseGates/SurfaceGateSideMeters",
                                 "y", std::to_string(y_side_m));
    cfg::xmlu::EnsurePathSetAttr(doc_void, "GatingAssociation/CourseGates/SurfaceGateSideMeters",
                                 "z", std::to_string(z_side_m));
    cfg::xmlu::SaveXmlDocOrThrow(doc_void, gating_assoc_xml);
    return true;
  } catch (...) {
    return false;
  }
}

} // namespace

int main(int argc, char** argv) {
  std::string sensors_xml = "config/sensors.xml";
  std::string gate_sizing_xml = "config/gate_sizing.xml";
  std::string gating_assoc_xml = "config/gating_association.xml";
  std::string xsd_dir;
  std::string sensor_id;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--sensors" && i + 1 < argc) {
      sensors_xml = argv[++i];
    } else if (a == "--gate-sizing" && i + 1 < argc) {
      gate_sizing_xml = argv[++i];
    } else if (a == "--xsd-dir" && i + 1 < argc) {
      xsd_dir = argv[++i];
    } else if (a == "--sensor-id" && i + 1 < argc) {
      sensor_id = argv[++i];
    } else if (a == "--gating-assoc" && i + 1 < argc) {
      gating_assoc_xml = argv[++i];
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: gate_sizing [--sensors <sensors.xml>]"
                   " [--gate-sizing <gate_sizing.xml>]"
                   " [--xsd-dir <dir>] [--sensor-id <id>]"
                   " [--gating-assoc <gating_association.xml>]\n";
      return 0;
    }
  }

  try {
    const auto sensors = parse_sensors(sensors_xml, xsd_dir);
    if (sensors.empty()) {
      std::cerr << "No sensors found in " << sensors_xml << "\n";
      return 2;
    }

    const SensorInfo* sensor = nullptr;
    if (!sensor_id.empty()) {
      for (const auto& s : sensors) {
        if (s.id == sensor_id) {
          sensor = &s;
          break;
        }
      }
      if (!sensor) {
        std::cerr << "Sensor id not found: " << sensor_id << "\n";
        return 2;
      }
    } else {
      sensor = &sensors.front();
      if (sensors.size() > 1) {
        std::cerr << "Multiple sensors found; using first: " << sensor->id
                  << " (override with --sensor-id)\n";
      }
    }

    const GateSizingCfg cfg = parse_gate_sizing(gate_sizing_xml, xsd_dir);

    const double scan_period_s = sensor->scan_period_s;
    const double vel_buffer = std::max(0.0, cfg.velocity_buffer_fraction);
    const double xy_half_m = cfg.max_speed_mps * scan_period_s * (1.0 + vel_buffer);

    const double range_ref_m = 0.5 * sensor->r_max_m;
    double z_half_m = 0.0;
    if (sensor->beamwidth_3db_el_deg > 0.0 && range_ref_m > 0.0) {
      const double el_half_rad = 0.5 * deg2rad(sensor->beamwidth_3db_el_deg);
      z_half_m = std::tan(el_half_rad) * range_ref_m;
    }

    if (scan_period_s <= 0.0) {
      std::cerr << "Warning: scan period is 0; check sensors.xml Schedule.\n";
    }
    if (cfg.max_speed_mps <= 0.0) {
      std::cerr << "Warning: MaxTargetSpeedMps is 0; gate XY size will be 0.\n";
    }
    if (sensor->beamwidth_3db_el_deg <= 0.0) {
      std::cerr << "Warning: elevation beamwidth is 0; gate Z size will be 0.\n";
    }
    if (sensor->r_max_m <= 0.0) {
      std::cerr << "Warning: max range is 0; gate Z size will be 0.\n";
    }

    constexpr double kEarthRadiusM = 6371000.0;
    const std::vector<double> alt_bins = build_altitude_bins(cfg.max_altitude_m, cfg.altitude_bin_m);

    std::cout << "Course gate sizing (simple coarse gates)\n";
    std::cout << "  sensor.id=" << sensor->id << "\n";
    std::cout << "  scan_frame=" << (sensor->scan_frame.empty() ? "(unset)" : sensor->scan_frame) << "\n";
    std::cout << "  scan_frustum_deg(az_min,az_max,el_min,el_max)="
              << sensor->az_min_deg << "," << sensor->az_max_deg << ","
              << sensor->el_min_deg << "," << sensor->el_max_deg << "\n";
    std::cout << "  scan_range_m(min,max)=" << sensor->r_min_m << "," << sensor->r_max_m << "\n";
    std::cout << "  beamwidth_3db_deg(az,el)="
              << sensor->beamwidth_3db_az_deg << "," << sensor->beamwidth_3db_el_deg << "\n";
    if (sensor->scan_rate_hz > 0.0) {
      std::cout << "  scan_rate_hz=" << sensor->scan_rate_hz << "\n";
    }
    std::cout << "  scan_period_s=" << scan_period_s << "\n";

    std::cout << "  xy_speed_mps=" << cfg.max_speed_mps
              << " velocity_buffer_fraction=" << vel_buffer << "\n";
    std::cout << "  xy_half_m=" << xy_half_m << "\n";

    std::cout << "  range_ref_m=" << range_ref_m << " (0.5 * max_range)\n";
    std::cout << "  z_half_m=" << z_half_m << "\n";

    std::cout << "Altitude bins (spherical earth scaling)\n";
    for (double alt_m : alt_bins) {
      const double scale = (kEarthRadiusM + alt_m) / kEarthRadiusM;
      const double x_side = 2.0 * xy_half_m * scale;
      const double y_side = 2.0 * xy_half_m * scale;
      const double z_side = 2.0 * z_half_m * scale;
      std::cout << "  alt_m=" << alt_m
                << " scale=" << scale
                << " gate_side_lengths_m(x,y,z)="
                << x_side << "," << y_side << "," << z_side << "\n";
    }

    const double surface_x = 2.0 * xy_half_m;
    const double surface_y = 2.0 * xy_half_m;
    const double surface_z = 2.0 * z_half_m;
    if (!gating_assoc_xml.empty()) {
      if (update_course_gate_sizes(gating_assoc_xml, surface_x, surface_y, surface_z)) {
        std::cout << "Updated gating association surface gate sizes in "
                  << gating_assoc_xml << "\n";
      } else {
        std::cerr << "Warning: failed to update gating association file: "
                  << gating_assoc_xml << "\n";
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Gate sizing failed: " << e.what() << "\n";
    return 2;
  }

  return 0;
}
