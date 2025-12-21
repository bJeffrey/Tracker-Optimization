#include "index/scan_volume.h"

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace idx {

static inline double deg2rad(double d) { constexpr double kPi = 3.14159265358979323846;
  return d * kPi / 180.0; }

struct Vec3 {
  double x=0, y=0, z=0;
  Vec3() = default;
  Vec3(double a,double b,double c):x(a),y(b),z(c){}
  Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
  Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
};

// Build ENU basis at a point using a spherical Earth approximation.
// (Good enough for a coarse AABB query; we'll swap to WGS-84 when we add full navigation.)
static void enu_basis_from_ecef(const Vec3& p_ecef, Vec3& east, Vec3& north, Vec3& up) {
  const double lon = std::atan2(p_ecef.y, p_ecef.x);
  const double hyp = std::sqrt(p_ecef.x*p_ecef.x + p_ecef.y*p_ecef.y);
  const double lat = std::atan2(p_ecef.z, hyp);

  east  = Vec3(-std::sin(lon),  std::cos(lon), 0.0);
  north = Vec3(-std::sin(lat)*std::cos(lon), -std::sin(lat)*std::sin(lon), std::cos(lat));
  up    = Vec3( std::cos(lat)*std::cos(lon),  std::cos(lat)*std::sin(lon), std::sin(lat));
}

// Convert az/el/r (deg,m) into an ENU vector.
// Convention (standard ENU navigation):
// - azimuth is measured clockwise from North towards East.
// - elevation is up from the local horizontal plane.
static Vec3 aer_to_enu(double az_deg, double el_deg, double r_m) {
  const double az = deg2rad(az_deg);
  const double el = deg2rad(el_deg);
  const double ce = std::cos(el);
  const double se = std::sin(el);
  const double sa = std::sin(az);
  const double ca = std::cos(az);
  const double e = r_m * ce * sa;
  const double n = r_m * ce * ca;
  const double u = r_m * se;
  return {e,n,u};
}

EcefAabb ComputeScanAabbEcefApprox(const cfg::SensorCfg& sensor, const cfg::Ownship& ownship) {
  const auto& f = sensor.scan.frustum;

  const Vec3 own(ownship.pos_x, ownship.pos_y, ownship.pos_z);

  // Choose corners: (az_min/max) x (el_min/max) x (r_min/max)
  const std::array<double,2> azs{f.az_min_deg, f.az_max_deg};
  const std::array<double,2> els{f.el_min_deg, f.el_max_deg};
  const std::array<double,2> rs {f.r_min_m,   f.r_max_m};

  std::vector<Vec3> corners;
  corners.reserve(8);

  if (sensor.scan.frame == "OWNERSHIP_BODY") {
    Vec3 ehat, nhat, uhat;
    enu_basis_from_ecef(own, ehat, nhat, uhat);

    for (double az : azs) for (double el : els) for (double r : rs) {
      Vec3 v_enu = aer_to_enu(az, el, r);
      Vec3 v_ecef = ehat * v_enu.x + nhat * v_enu.y + uhat * v_enu.z;
      corners.push_back(own + v_ecef);
    }
  } else {
    // Fallback: interpret the "AER" vector directly in ECEF axes (still deterministic).
    for (double az : azs) for (double el : els) for (double r : rs) {
      // Treat az as rotation in X-Y plane, el as Z.
      const double azr = deg2rad(az);
      const double elr = deg2rad(el);
      const double ce = std::cos(elr);
      Vec3 v(r * ce * std::cos(azr), r * ce * std::sin(azr), r * std::sin(elr));
      corners.push_back(own + v);
    }
  }

  if (corners.empty()) {
    throw std::runtime_error("ComputeScanAabbEcefApprox: no frustum corners generated");
  }

  EcefAabb a{};
  a.min_x = a.max_x = corners[0].x;
  a.min_y = a.max_y = corners[0].y;
  a.min_z = a.max_z = corners[0].z;
  for (const auto& c : corners) {
    a.min_x = std::min(a.min_x, c.x); a.max_x = std::max(a.max_x, c.x);
    a.min_y = std::min(a.min_y, c.y); a.max_y = std::max(a.max_y, c.y);
    a.min_z = std::min(a.min_z, c.z); a.max_z = std::max(a.max_z, c.z);
  }

  const double infl = sensor.scan.query_aabb_inflate_m;
  a.min_x -= infl; a.max_x += infl;
  a.min_y -= infl; a.max_y += infl;
  a.min_z -= infl; a.max_z += infl;

  return a;
}

} // namespace idx
