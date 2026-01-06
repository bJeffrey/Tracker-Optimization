#include "plugins/simulation/targets_generator.h"

#include "xml_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim {
namespace {

struct Band {
  double fraction = 0.0; // [0,1]
  double min = 0.0;
  double max = 0.0;
};

static double to_double_or(const std::string& s, double def) {
  if (s.empty()) return def;
  return std::stod(s);
}

static double clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static std::vector<Band> read_bands(void* doc, const std::string& path_to_bands) {
  std::vector<Band> out;
  const auto nodes = cfg::xmlu::FindNodes(doc, path_to_bands);
  out.reserve(nodes.size());
  for (void* node : nodes) {
    Band b{};
    b.fraction = to_double_or(cfg::xmlu::NodeGetAttr(node, "fraction"), 0.0);
    b.min      = to_double_or(cfg::xmlu::NodeGetAttr(node, "min"), 0.0);
    b.max      = to_double_or(cfg::xmlu::NodeGetAttr(node, "max"), 0.0);
    if (b.max < b.min) std::swap(b.min, b.max);
    out.push_back(b);
  }
  return out;
}

static double sample_band_mixture(std::mt19937_64& rng, const std::vector<Band>& bands, double def_val) {
  if (bands.empty()) return def_val;

  double sum = 0.0;
  for (const auto& b : bands) sum += std::max(0.0, b.fraction);
  if (sum <= 0.0) return def_val;

  std::uniform_real_distribution<double> unif01(0.0, 1.0);
  const double u = unif01(rng) * sum;

  double acc = 0.0;
  const Band* chosen = &bands.back();
  for (const auto& b : bands) {
    acc += std::max(0.0, b.fraction);
    if (u <= acc) { chosen = &b; break; }
  }

  std::uniform_real_distribution<double> unif(chosen->min, chosen->max);
  return unif(rng);
}

static void sample_unit_vector(std::mt19937_64& rng, double& ux, double& uy, double& uz) {
  std::uniform_real_distribution<double> unif01(0.0, 1.0);
  const double u1 = unif01(rng);
  const double u2 = unif01(rng);
  const double z = 2.0 * u1 - 1.0;
  const double t = 2.0 * M_PI * u2;
  const double r = std::sqrt(std::max(0.0, 1.0 - z*z));
  ux = r * std::cos(t);
  uy = r * std::sin(t);
  uz = z;
}

struct Vec3 { double x=0, y=0, z=0; };

static Vec3 read_vec3(void* doc, const std::string& path_to_vec3_node) {
  const auto nodes = cfg::xmlu::FindNodes(doc, path_to_vec3_node);
  if (nodes.empty()) return {};

  void* n = nodes.front();
  const std::string ax = cfg::xmlu::NodeGetAttr(n, "x");
  const std::string ay = cfg::xmlu::NodeGetAttr(n, "y");
  const std::string az = cfg::xmlu::NodeGetAttr(n, "z");

  Vec3 v{};
  if (!ax.empty() || !ay.empty() || !az.empty()) {
    v.x = to_double_or(ax, 0.0);
    v.y = to_double_or(ay, 0.0);
    v.z = to_double_or(az, 0.0);
    return v;
  }

  v.x = cfg::xmlu::NodeGetDoubleChild(n, "X", 0.0);
  v.y = cfg::xmlu::NodeGetDoubleChild(n, "Y", 0.0);
  v.z = cfg::xmlu::NodeGetDoubleChild(n, "Z", 0.0);
  return v;
}

static int read_int_req(void* doc, const std::string& path) {
  const int v = cfg::xmlu::GetInt(doc, path, -1);
  if (v < 0) throw std::runtime_error("Missing/invalid int at path: " + path);
  return v;
}

static std::uint32_t read_u32_req(void* doc, const std::string& path) {
  const int v = read_int_req(doc, path);
  if (v < 0) throw std::runtime_error("Invalid u32 at path: " + path);
  return static_cast<std::uint32_t>(v);
}

// --- Position sampling models ------------------------------------------------

static Vec3 sample_ecef_box(std::mt19937_64& rng,
                            const cfg::Ownship& ownship,
                            const std::string& center,
                            const Vec3& min_m,
                            const Vec3& max_m)
{
  std::uniform_real_distribution<double> ux(min_m.x, max_m.x);
  std::uniform_real_distribution<double> uy(min_m.y, max_m.y);
  std::uniform_real_distribution<double> uz(min_m.z, max_m.z);

  Vec3 p{};
  p.x = ux(rng);
  p.y = uy(rng);
  p.z = uz(rng);

  if (center == "ownship") {
    p.x += ownship.pos_x;
    p.y += ownship.pos_y;
    p.z += ownship.pos_z;
  }
  return p;
}

static Vec3 sample_global_geodetic(std::mt19937_64& rng,
                                  double alt_m)
{
  constexpr double R_EARTH_M = 6378137.0;

  std::uniform_real_distribution<double> unif01(0.0, 1.0);
  const double u = unif01(rng);
  const double v = unif01(rng);

  const double z = 2.0 * u - 1.0;
  const double lon = 2.0 * M_PI * v;
  const double r_xy = std::sqrt(std::max(0.0, 1.0 - z*z));

  const double radius = R_EARTH_M + alt_m;
  Vec3 p{};
  p.x = radius * r_xy * std::cos(lon);
  p.y = radius * r_xy * std::sin(lon);
  p.z = radius * z;
  return p;
}

} // namespace

void GenerateTruthFromXml(const std::string& targets_xml,
                          const std::string& xsd_dir,
                          const cfg::Ownship& ownship,
                          sim::TargetTruth& truth)
{
  if (truth.n == 0) throw std::runtime_error("TargetTruth is empty; call resize(n) first.");

  void* doc = cfg::xmlu::ReadXmlDocOrThrow(targets_xml);

  if (!xsd_dir.empty()) {
    const std::string xsd = xsd_dir + "/targets_gen.xsd";
    cfg::xmlu::ValidateOrThrow(doc, xsd, targets_xml);
  }

  const int cfg_count = read_int_req(doc, "Count");
  const std::uint32_t seed = read_u32_req(doc, "Seed");
  (void)cfg_count; // truth.n is authoritative

  std::mt19937_64 rng(static_cast<std::uint64_t>(seed));
  std::uniform_real_distribution<double> unif01(0.0, 1.0);

  // --- Mixture component selection (Background vs Local) ---
  const double frac_bg = to_double_or(cfg::xmlu::GetAttr(doc, "Mixture/Background", "fraction"), 1.0);
  const double frac_lc = to_double_or(cfg::xmlu::GetAttr(doc, "Mixture/Local", "fraction"), 0.0);
  double sum_frac = std::max(0.0, frac_bg) + std::max(0.0, frac_lc);
  if (sum_frac <= 0.0) sum_frac = 1.0;

  // Background model
  const std::string bg_type = cfg::xmlu::GetAttr(doc, "Mixture/Background/Model", "type");
  const std::string bg_center = cfg::xmlu::GetText(doc, "Mixture/Background/Model/Center");
  const auto bg_alt_bands = read_bands(doc, "Mixture/Background/Model/AltitudeMeters/Band");

  // Local model
  const std::string lc_type = cfg::xmlu::GetAttr(doc, "Mixture/Local/Model", "type");
  const std::string lc_center = cfg::xmlu::GetText(doc, "Mixture/Local/Model/Center");
  const Vec3 lc_min = read_vec3(doc, "Mixture/Local/Model/EcefBox/MinMeters");
  const Vec3 lc_max = read_vec3(doc, "Mixture/Local/Model/EcefBox/MaxMeters");

  if (!lc_type.empty() && lc_type != "ecef_box") {
    if (lc_type == "enu_bubble") {
      throw std::runtime_error("targets_gen Local model type enu_bubble not supported yet. Use ecef_box for now.");
    }
    throw std::runtime_error("Unsupported Local model type: " + lc_type);
  }

  if (!bg_type.empty() && bg_type != "global_geodetic" && bg_type != "ecef_box") {
    if (bg_type == "enu_bubble") {
      throw std::runtime_error("targets_gen Background model type enu_bubble not supported yet. Use ecef_box/global_geodetic.");
    }
    throw std::runtime_error("Unsupported Background model type: " + bg_type);
  }

  // --- Kinematics sampling ---
  const auto speed_bands = read_bands(doc, "Kinematics/Velocity/SpeedMps/Band");

  const std::string vs_model = cfg::xmlu::GetAttr(doc, "Kinematics/Velocity/VerticalSpeedMps", "model");
  const double vs_mean = cfg::xmlu::GetDouble(doc, "Kinematics/Velocity/VerticalSpeedMps/@mean", 0.0);
  const double vs_std  = cfg::xmlu::GetDouble(doc, "Kinematics/Velocity/VerticalSpeedMps/@std",  0.0);
  std::normal_distribution<double> norm_vs(vs_mean, (vs_std > 0.0) ? vs_std : 0.0);

  const auto acc_bands = read_bands(doc, "Kinematics/Acceleration/MagnitudeMps2/Band");

  // Truth time (generator treats NowSeconds as the "truth time")
  const double now_s = cfg::xmlu::GetDouble(doc, "LastUpdateTime/NowSeconds", 0.0);
  truth.t_truth_s = now_s;

  for (std::size_t i = 0; i < truth.n; ++i) {
    truth.target_id[i] = static_cast<std::uint64_t>(i + 1);

    // Choose component
    const double u = unif01(rng) * sum_frac;
    const bool use_bg = (u <= std::max(0.0, frac_bg));

    Vec3 p{};
    if (use_bg) {
      if (bg_type == "ecef_box") {
        const Vec3 bg_min = read_vec3(doc, "Mixture/Background/Model/EcefBox/MinMeters");
        const Vec3 bg_max = read_vec3(doc, "Mixture/Background/Model/EcefBox/MaxMeters");
        const std::string c = bg_center.empty() ? "ownship" : bg_center;
        p = sample_ecef_box(rng, ownship, c, bg_min, bg_max);
      } else {
        const double alt_m = sample_band_mixture(rng, bg_alt_bands, 0.0);
        p = sample_global_geodetic(rng, alt_m);
      }
    } else {
      const std::string c = lc_center.empty() ? "ownship" : lc_center;
      p = sample_ecef_box(rng, ownship, c, lc_min, lc_max);
    }

    // Velocity
    const double speed = sample_band_mixture(rng, speed_bands, 0.0);
    double ux, uy, uz;
    sample_unit_vector(rng, ux, uy, uz);

    Vec3 v{};
    v.x = speed * ux;
    v.y = speed * uy;
    v.z = speed * uz;

    if (!vs_model.empty() && vs_std > 0.0) {
      v.z += norm_vs(rng);
    }

    // Acceleration
    const double amag = sample_band_mixture(rng, acc_bands, 0.0);
    sample_unit_vector(rng, ux, uy, uz);
    Vec3 a{};
    a.x = amag * ux;
    a.y = amag * uy;
    a.z = amag * uz;

    // Write truth
    truth.pos.x[i] = p.x; truth.pos.y[i] = p.y; truth.pos.z[i] = p.z;
    truth.vel.x[i] = v.x; truth.vel.y[i] = v.y; truth.vel.z[i] = v.z;
    truth.acc.x[i] = a.x; truth.acc.y[i] = a.y; truth.acc.z[i] = a.z;
  }

  cfg::xmlu::FreeXmlDoc(doc);
}

} // namespace sim
