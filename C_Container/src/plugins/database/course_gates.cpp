#include "plugins/database/course_gates.h"

#include "plugins/database/track_database.h"

#include <cmath>
#include <vector>

namespace db {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  Vec3() = default;
  Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};

void enu_basis_from_ecef(const Vec3& p_ecef, Vec3& east, Vec3& north, Vec3& up) {
  const double lon = std::atan2(p_ecef.y, p_ecef.x);
  const double hyp = std::sqrt(p_ecef.x * p_ecef.x + p_ecef.y * p_ecef.y);
  const double lat = std::atan2(p_ecef.z, hyp);

  east  = Vec3(-std::sin(lon),  std::cos(lon), 0.0);
  north = Vec3(-std::sin(lat) * std::cos(lon),
               -std::sin(lat) * std::sin(lon),
               std::cos(lat));
  up    = Vec3( std::cos(lat) * std::cos(lon),
                std::cos(lat) * std::sin(lon),
                std::sin(lat));
}

Vec3 abs_vec(const Vec3& v) {
  return {std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)};
}

idx::EcefAabb make_gate_aabb(const Vec3& center_ecef,
                             const Vec3& east,
                             const Vec3& north,
                             const Vec3& up,
                             double half_e,
                             double half_n,
                             double half_u) {
  const Vec3 ae = abs_vec(east);
  const Vec3 an = abs_vec(north);
  const Vec3 au = abs_vec(up);
  const Vec3 ext{
    ae.x * half_e + an.x * half_n + au.x * half_u,
    ae.y * half_e + an.y * half_n + au.y * half_u,
    ae.z * half_e + an.z * half_n + au.z * half_u
  };

  idx::EcefAabb aabb{};
  aabb.min_x = center_ecef.x - ext.x;
  aabb.max_x = center_ecef.x + ext.x;
  aabb.min_y = center_ecef.y - ext.y;
  aabb.max_y = center_ecef.y + ext.y;
  aabb.min_z = center_ecef.z - ext.z;
  aabb.max_z = center_ecef.z + ext.z;
  return aabb;
}

} // namespace

CourseGateSet BuildCourseGateSet(const cfg::Ownship& ownship,
                                 const cfg::CourseGatesCfg& cfg) {
  CourseGateSet out{};
  if (!cfg.enabled || cfg.radius_m <= 0.0 ||
      cfg.side_x_m <= 0.0 || cfg.side_y_m <= 0.0 || cfg.side_z_m <= 0.0) {
    return out;
  }

  out.enabled = true;
  const Vec3 own(ownship.pos_x, ownship.pos_y, ownship.pos_z);
  out.prefetch_aabb.min_x = own.x - cfg.radius_m;
  out.prefetch_aabb.max_x = own.x + cfg.radius_m;
  out.prefetch_aabb.min_y = own.y - cfg.radius_m;
  out.prefetch_aabb.max_y = own.y + cfg.radius_m;
  out.prefetch_aabb.min_z = own.z - cfg.radius_m;
  out.prefetch_aabb.max_z = own.z + cfg.radius_m;

  Vec3 east, north, up;
  enu_basis_from_ecef(own, east, north, up);

  const double half_x = 0.5 * cfg.side_x_m;
  const double half_y = 0.5 * cfg.side_y_m;
  const double half_z = 0.5 * cfg.side_z_m;
  const double step_x = cfg.side_x_m;
  const double step_y = cfg.side_y_m;
  const double r2 = cfg.radius_m * cfg.radius_m;

  for (double e = -cfg.radius_m; e <= cfg.radius_m + 1e-6; e += step_x) {
    for (double n = -cfg.radius_m; n <= cfg.radius_m + 1e-6; n += step_y) {
      if ((e * e + n * n) > r2) continue;
      const Vec3 center_ecef = own + east * e + north * n;
      out.gates.push_back(make_gate_aabb(center_ecef, east, north, up,
                                         half_x, half_y, half_z));
    }
  }

  return out;
}

trk::IdList QueryCourseGates(const ITrackDatabase& track_db,
                             const std::vector<idx::EcefAabb>& gates,
                             std::size_t n_tracks) {
  trk::IdList ids;
  if (gates.empty() || n_tracks == 0) return ids;

  static std::vector<std::uint8_t> seen;
  if (seen.size() < n_tracks) {
    seen.assign(n_tracks, 0);
  } else {
    std::fill(seen.begin(), seen.end(), 0);
  }

  for (const auto& gate : gates) {
    const auto gate_ids = track_db.QueryAabb(gate);
    for (std::uint64_t id : gate_ids) {
      const std::size_t idx = static_cast<std::size_t>(id);
      if (idx >= seen.size()) continue;
      if (!seen[idx]) {
        seen[idx] = 1;
        ids.push_back(id);
      }
    }
  }

  return ids;
}

} // namespace db
