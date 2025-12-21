#include "targets/targets_generator.h"

#include "config/xml_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace targets {
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

  // If no bands exist, caller can treat as "missing".
  return out;
}

static double sample_band_mixture(std::mt19937_64& rng, const std::vector<Band>& bands, double def_val) {
  if (bands.empty()) return def_val;

  // Build normalized CDF (robust to fractions not summing to 1.0)
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
  // Uniform on unit sphere
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
  // Supports two encodings:
  //  1) attributes: <MinMeters x=".." y=".." z=".."/>
  //  2) child elements: <MinMeters><X>..</X><Y>..</Y><Z>..</Z></MinMeters>
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
  // Interpret min/max as meters in ECEF axes.
  // If Center=="ownship", treat as offsets from ownship ECEF position.
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
  // NOTE: for performance/sanity tests, use a simple spherical Earth conversion.
  // This is intentionally "good enough" for synthetic workloads; we can swap in
  // a WGS-84 ellipsoid later if needed.
  constexpr double R_EARTH_M = 6378137.0;

  std::uniform_real_distribution<double> unif01(0.0, 1.0);
  const double u = unif01(rng);
  const double v = unif01(rng);

  // Uniform on sphere: z = 2u-1, lon = 2πv
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

// --- Covariance init ----------------------------------------------------------

static void init_cov_diag(TrackBatch& tb,
                          std::mt19937_64& rng,
                          double sig_pos_m,
                          double sig_vel_mps,
                          double sig_acc_mps2)
{
  // Fill diagonal blocks (pos/vel/acc) for each track, full 9x9 row-major.
  const double var_p = sig_pos_m   * sig_pos_m;
  const double var_v = sig_vel_mps * sig_vel_mps;
  const double var_a = sig_acc_mps2 * sig_acc_mps2;

  const std::size_t nn = static_cast<std::size_t>(TrackBatch::kCovN);
  for (std::size_t i = 0; i < tb.n_tracks; ++i) {
    double* Pi = tb.P.data() + i * nn;
    // zero
    std::fill(Pi, Pi + nn, 0.0);
    for (int k = 0; k < 3; ++k) Pi[(k+0)*TrackBatch::kDim + (k+0)] = var_p;
    for (int k = 0; k < 3; ++k) Pi[(k+3)*TrackBatch::kDim + (k+3)] = var_v;
    for (int k = 0; k < 3; ++k) Pi[(k+6)*TrackBatch::kDim + (k+6)] = var_a;
  }

  (void)rng; // placeholder if we later randomize per-track within min/max
}

// --- Last update time ---------------------------------------------------------

static double sample_age_seconds(std::mt19937_64& rng,
                                 const std::string& model,
                                 double tau_s,
                                 double min_age_s,
                                 double max_age_s)
{
  min_age_s = std::max(0.0, min_age_s);
  max_age_s = std::max(min_age_s, max_age_s);

  std::uniform_real_distribution<double> unif(min_age_s, max_age_s);

  if (model == "uniform_age") {
    return unif(rng);
  } else if (model == "exponential_age") {
    // Truncated exponential with mean tau_s (approx). If tau not provided, fallback uniform.
    if (tau_s <= 0.0) return unif(rng);

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    // Inverse CDF for exponential: a = -tau * ln(1-u)
    // Then clamp into [min,max].
    const double u = std::max(1e-12, std::min(1.0 - 1e-12, u01(rng)));
    const double a = -tau_s * std::log(1.0 - u);
    return clamp(a, min_age_s, max_age_s);
  } else if (model == "mixture_age") {
    // Minimal placeholder: mixture_age not fully specified in v3 schema pack; use uniform for now.
    return unif(rng);
  }

  // Unknown model -> uniform fallback
  return unif(rng);
}

} // namespace

void GenerateFromXml(const std::string& targets_xml,
                     const std::string& xsd_dir,
                     const cfg::Ownship& ownship,
                     TrackBatch& tb)
{
  if (tb.n_tracks == 0) throw std::runtime_error("TrackBatch is empty; call resize(n) first.");

  void* doc = cfg::xmlu::ReadXmlDocOrThrow(targets_xml);

  // Optional schema validation (config loader already validated, but keep this for standalone use)
  if (!xsd_dir.empty()) {
    const std::string xsd = xsd_dir + "/targets_gen.xsd";
    cfg::xmlu::ValidateOrThrow(doc, xsd, targets_xml);
  }

  const int cfg_count = read_int_req(doc, "Count");
  const std::uint32_t seed = read_u32_req(doc, "Seed");

  if (cfg_count != static_cast<int>(tb.n_tracks)) {
    // Intentional: we allow runtime.max_tracks to differ from generator count;
    // TrackBatch size is authoritative for what we generate.
    // (This is useful for dev when targets_gen count is 1M but runtime is 50k.)
  }

  std::mt19937_64 rng(static_cast<std::uint64_t>(seed));

  // --- Mixture component selection (Background vs Local) ---
  const double frac_bg = to_double_or(cfg::xmlu::GetAttr(doc, "Mixture/Background", "fraction"), 1.0);
  const double frac_lc = to_double_or(cfg::xmlu::GetAttr(doc, "Mixture/Local", "fraction"), 0.0);
  double sum_frac = std::max(0.0, frac_bg) + std::max(0.0, frac_lc);
  if (sum_frac <= 0.0) sum_frac = 1.0;

  std::uniform_real_distribution<double> unif01(0.0, 1.0);

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
    // Unknown -> throw early to avoid silent misuse
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
  // Vertical speed element uses attributes; keep optional.
  const std::string vs_model = cfg::xmlu::GetAttr(doc, "Kinematics/Velocity/VerticalSpeedMps", "model");
  const double vs_mean = cfg::xmlu::GetDouble(doc, "Kinematics/Velocity/VerticalSpeedMps/@mean", 0.0); // may return default
  const double vs_std  = cfg::xmlu::GetDouble(doc, "Kinematics/Velocity/VerticalSpeedMps/@std",  0.0);

  const auto acc_bands = read_bands(doc, "Kinematics/Acceleration/MagnitudeMps2/Band");

  // --- LastUpdateTime ---
  const std::string age_model = cfg::xmlu::GetAttr(doc, "LastUpdateTime", "model");
  const double now_s = cfg::xmlu::GetDouble(doc, "LastUpdateTime/NowSeconds", 0.0);
  const double tau_s = cfg::xmlu::GetDouble(doc, "LastUpdateTime/TauSeconds", 0.0);
  const double max_age_s = cfg::xmlu::GetDouble(doc, "LastUpdateTime/MaxAgeSeconds", 0.0);
  const double min_age_s = cfg::xmlu::GetDouble(doc, "LastUpdateTime/MinAgeSeconds", 0.0);

  // --- Covariance init (simple) ---
  const double sig_pos_min = to_double_or(cfg::xmlu::GetAttr(doc, "Covariance/SigmaPosMeters", "min"), 50.0);
  const double sig_pos_max = to_double_or(cfg::xmlu::GetAttr(doc, "Covariance/SigmaPosMeters", "max"), sig_pos_min);

  const double sig_vel_min = to_double_or(cfg::xmlu::GetAttr(doc, "Covariance/SigmaVelMps", "min"), 10.0);
  const double sig_vel_max = to_double_or(cfg::xmlu::GetAttr(doc, "Covariance/SigmaVelMps", "max"), sig_vel_min);

  const double sig_acc_min = to_double_or(cfg::xmlu::GetAttr(doc, "Covariance/SigmaAccelMps2", "min"), 1.0);
  const double sig_acc_max = to_double_or(cfg::xmlu::GetAttr(doc, "Covariance/SigmaAccelMps2", "max"), sig_acc_min);

  std::uniform_real_distribution<double> unif_sig_pos(std::min(sig_pos_min, sig_pos_max), std::max(sig_pos_min, sig_pos_max));
  std::uniform_real_distribution<double> unif_sig_vel(std::min(sig_vel_min, sig_vel_max), std::max(sig_vel_min, sig_vel_max));
  std::uniform_real_distribution<double> unif_sig_acc(std::min(sig_acc_min, sig_acc_max), std::max(sig_acc_min, sig_acc_max));

  // If you want per-track randomized sigma, sample here; otherwise keep constant:
  // For now: one sigma sample for the whole batch (fast + deterministic).
  const double sig_pos = unif_sig_pos(rng);
  const double sig_vel = unif_sig_vel(rng);
  const double sig_acc = unif_sig_acc(rng);
  init_cov_diag(tb, rng, sig_pos, sig_vel, sig_acc);

  // Fill metadata and state
  std::normal_distribution<double> norm01(0.0, 1.0);
  std::normal_distribution<double> norm_vs(vs_mean, (vs_std > 0.0) ? vs_std : 0.0);

  for (std::size_t i = 0; i < tb.n_tracks; ++i) {
    tb.track_id[i] = static_cast<std::uint64_t>(i + 1);
    tb.status[i] = TrackStatus::ACTIVE;
    tb.quality[i] = 1.0f;

    // Choose component
    const double u = unif01(rng) * sum_frac;
    const bool use_bg = (u <= std::max(0.0, frac_bg));

    Vec3 p{};
    if (use_bg) {
      if (bg_type == "ecef_box") {
        // Background ecef_box is not used in your current example XML but supported.
        const Vec3 bg_min = read_vec3(doc, "Mixture/Background/Model/EcefBox/MinMeters");
        const Vec3 bg_max = read_vec3(doc, "Mixture/Background/Model/EcefBox/MaxMeters");
        const std::string c = bg_center.empty() ? "ownship" : bg_center;
        p = sample_ecef_box(rng, ownship, c, bg_min, bg_max);
      } else {
        // global_geodetic
        const double alt_m = sample_band_mixture(rng, bg_alt_bands, 0.0);
        p = sample_global_geodetic(rng, alt_m);
      }
    } else {
      // Local component
      const std::string c = lc_center.empty() ? "ownship" : lc_center;
      p = sample_ecef_box(rng, ownship, c, lc_min, lc_max);
    }

    // Velocity magnitude from bands (fallback: 0)
    const double speed = sample_band_mixture(rng, speed_bands, 0.0);
    double ux, uy, uz;
    sample_unit_vector(rng, ux, uy, uz);

    Vec3 v{};
    v.x = speed * ux;
    v.y = speed * uy;
    v.z = speed * uz;

    // Optional vertical speed (interpretation ambiguous in ECEF; apply to z for now if configured)
    if (!vs_model.empty() && vs_std > 0.0) {
      v.z += norm_vs(rng);
    }

    // Acceleration magnitude from bands (fallback: 0)
    const double amag = sample_band_mixture(rng, acc_bands, 0.0);
    sample_unit_vector(rng, ux, uy, uz);
    Vec3 a{};
    a.x = amag * ux;
    a.y = amag * uy;
    a.z = amag * uz;

    // Write CA9 state: [x y z vx vy vz ax ay az]
    double* xi = tb.x_ptr(i);
    xi[0] = p.x; xi[1] = p.y; xi[2] = p.z;
    xi[3] = v.x; xi[4] = v.y; xi[5] = v.z;
    xi[6] = a.x; xi[7] = a.y; xi[8] = a.z;

    // Last update time: now - age
    const double age_s = sample_age_seconds(rng, age_model, tau_s, min_age_s, max_age_s);
    tb.last_update_s[i] = now_s - age_s;
  }

  cfg::xmlu::FreeXmlDoc(doc);
}

} // namespace targets
