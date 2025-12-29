#include "trk/seed_tracks_from_xml.h"
#include "xml_utils.h"
#include "track_status.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace trk {
namespace {

struct Band {
  double fraction = 0.0;
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
    if (tau_s <= 0.0) return unif(rng);

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const double u = std::max(1e-12, std::min(1.0 - 1e-12, u01(rng)));
    const double a = -tau_s * std::log(1.0 - u);
    return clamp(a, min_age_s, max_age_s);
  } else if (model == "mixture_age") {
    return unif(rng);
  }

  return unif(rng);
}

static void init_cov_diag(trk::TrackBatch& tracks,
                          double sig_pos_m,
                          double sig_vel_mps,
                          double sig_acc_mps2)
{
  const double var_p = sig_pos_m   * sig_pos_m;
  const double var_v = sig_vel_mps * sig_vel_mps;
  const double var_a = sig_acc_mps2 * sig_acc_mps2;

  const std::size_t nn = static_cast<std::size_t>(trk::TrackBatch::kCovN);
  for (std::size_t i = 0; i < tracks.n_tracks; ++i) {
    double* Pi = tracks.P.data() + i * nn;
    std::fill(Pi, Pi + nn, 0.0);
    for (int k = 0; k < 3; ++k) Pi[(k+0)*trk::TrackBatch::kDim + (k+0)] = var_p;
    for (int k = 0; k < 3; ++k) Pi[(k+3)*trk::TrackBatch::kDim + (k+3)] = var_v;
    for (int k = 0; k < 3; ++k) Pi[(k+6)*trk::TrackBatch::kDim + (k+6)] = var_a;
  }
}

} // namespace

void SeedTracksFromTruthXml(const std::string& targets_xml,
                            const std::string& xsd_dir,
                            const cfg::Ownship& ownship,
                            const sim::TargetTruth& truth,
                            trk::TrackBatch& tracks)
{
  (void)ownship; // reserved for future tracker seeding variants

  if (truth.n == 0)   throw std::runtime_error("SeedTracksFromTruthXml: truth is empty.");
  if (tracks.n_tracks == 0) throw std::runtime_error("SeedTracksFromTruthXml: tracks is empty; call resize(n) first.");

  const std::size_t n = std::min(truth.n, tracks.n_tracks);

  void* doc = cfg::xmlu::ReadXmlDocOrThrow(targets_xml);

  if (!xsd_dir.empty()) {
    const std::string xsd = xsd_dir + "/targets_gen.xsd";
    cfg::xmlu::ValidateOrThrow(doc, xsd, targets_xml);
  }

  const std::uint32_t seed = static_cast<std::uint32_t>(cfg::xmlu::GetInt(doc, "Seed", 0));
  std::mt19937_64 rng(static_cast<std::uint64_t>(seed));

  // --- LastUpdateTime model for stale tracks ---
  const std::string age_model = cfg::xmlu::GetAttr(doc, "LastUpdateTime", "model");
  const double now_s = cfg::xmlu::GetDouble(doc, "LastUpdateTime/NowSeconds", truth.t_truth_s);
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

  const double sig_pos = unif_sig_pos(rng);
  const double sig_vel = unif_sig_vel(rng);
  const double sig_acc = unif_sig_acc(rng);
  init_cov_diag(tracks, sig_pos, sig_vel, sig_acc);

  // Seed tracks (CA9 only for now)
  for (std::size_t i = 0; i < n; ++i) {
    tracks.track_id[i] = truth.target_id[i];
    tracks.status[i]   = TrackStatus::ACTIVE;
    tracks.quality[i]  = 1.0f;

    const double age_s = sample_age_seconds(rng, age_model, tau_s, min_age_s, max_age_s);
    const double t0 = now_s - age_s;

    // Back-propagate truth(now) -> track(t0) under CA:
    // v0 = v - a*age
    // p0 = p - v*age + 0.5*a*age^2
    const double a2 = age_s * age_s;

    const double vx0 = truth.vx[i] - truth.ax[i] * age_s;
    const double vy0 = truth.vy[i] - truth.ay[i] * age_s;
    const double vz0 = truth.vz[i] - truth.az[i] * age_s;

    const double x0 = truth.x[i] - truth.vx[i] * age_s + 0.5 * truth.ax[i] * a2;
    const double y0 = truth.y[i] - truth.vy[i] * age_s + 0.5 * truth.ay[i] * a2;
    const double z0 = truth.z[i] - truth.vz[i] * age_s + 0.5 * truth.az[i] * a2;

    double* xi = tracks.x_ptr(i);
    xi[0]=x0;  xi[1]=y0;  xi[2]=z0;
    xi[3]=vx0; xi[4]=vy0; xi[5]=vz0;
    xi[6]=truth.ax[i]; xi[7]=truth.ay[i]; xi[8]=truth.az[i];

    tracks.pos_x[i] = xi[0];
    tracks.pos_y[i] = xi[1];
    tracks.pos_z[i] = xi[2];

    tracks.last_update_s[i]   = t0;
    tracks.last_cov_prop_s[i] = t0; // t_pred_s
  }

  // If tracks is larger than truth, mark the remainder inactive/empty
  for (std::size_t i = n; i < tracks.n_tracks; ++i) {
    tracks.track_id[i] = 0;
    tracks.status[i]   = TrackStatus::INACTIVE;
    tracks.quality[i]  = 0.0f;

    // "Aged-to" timestamps: set to now so we don't immediately re-age empty slots.
    tracks.last_update_s[i]   = now_s;
    tracks.last_cov_prop_s[i] = now_s;

    // Zero kinematics
    double* xi = tracks.x_ptr(i);
    std::fill(xi, xi + static_cast<std::size_t>(TrackBatch::kDim), 0.0);

    // Keep hot columns consistent
    tracks.pos_x[i] = 0.0;
    tracks.pos_y[i] = 0.0;
    tracks.pos_z[i] = 0.0;

    // Zero covariance
    double* Pi = tracks.P_ptr(i);
    std::fill(Pi, Pi + static_cast<std::size_t>(TrackBatch::kCovN), 0.0);
  }


  cfg::xmlu::FreeXmlDoc(doc);
}

} // namespace trk
