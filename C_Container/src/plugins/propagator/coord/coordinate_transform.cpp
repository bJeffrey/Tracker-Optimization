#include "common/coordinate_utilities/coordinate_transform.h"

#include "plugins/propagator/la.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace coord {
namespace {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

void enu_basis_from_ecef(const Vec3& p_ecef, Vec3& east, Vec3& north, Vec3& up) {
  const double lon = std::atan2(p_ecef.y, p_ecef.x);
  const double hyp = std::sqrt(p_ecef.x * p_ecef.x + p_ecef.y * p_ecef.y);
  const double lat = std::atan2(p_ecef.z, hyp);

  east  = Vec3{-std::sin(lon),  std::cos(lon), 0.0};
  north = Vec3{-std::sin(lat) * std::cos(lon),
               -std::sin(lat) * std::sin(lon),
               std::cos(lat)};
  up    = Vec3{ std::cos(lat) * std::cos(lon),
               std::cos(lat) * std::sin(lon),
               std::sin(lat)};
}

Vec3 aer_to_enu(double az_rad, double el_rad, double r_m) {
  const double ce = std::cos(el_rad);
  const double se = std::sin(el_rad);
  const double sa = std::sin(az_rad);
  const double ca = std::cos(az_rad);
  return Vec3{r_m * ce * sa, r_m * ce * ca, r_m * se};
}

} // namespace

void AerRadBatch::assert_sizes() const {
  assert(az_rad.size() == n && el_rad.size() == n && r_m.size() == n);
}

void EcefToAerBatch(const EcefBatch& ecef,
                    const cfg::Ownship& ownship,
                    AerRadBatch& out) {
  const std::size_t n = ecef.n;
  out.resize(n);
  if (n == 0) return;
  ecef.assert_sizes();

  const Vec3 own{ownship.pos_x, ownship.pos_y, ownship.pos_z};
  Vec3 east, north, up;
  enu_basis_from_ecef(own, east, north, up);

  const std::size_t cols = n;
  std::vector<double> delta(3 * cols, 0.0);
  std::vector<double> enu(3 * cols, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    delta[0 * cols + i] = ecef.x[i] - own.x;
    delta[1 * cols + i] = ecef.y[i] - own.y;
    delta[2 * cols + i] = ecef.z[i] - own.z;
  }

  double basis[9] = {
    east.x,  east.y,  east.z,
    north.x, north.y, north.z,
    up.x,    up.y,    up.z
  };

  la::MatrixView A{basis, 3, 3, 3};
  la::MatrixView B{delta.data(), 3, cols, cols};
  la::MatrixView C{enu.data(), 3, cols, cols};
  la::gemm(false, false, 1.0, A, B, 0.0, C);

  for (std::size_t i = 0; i < n; ++i) {
    const double e = enu[0 * cols + i];
    const double n_comp = enu[1 * cols + i];
    const double u = enu[2 * cols + i];
    const double r_xy = std::sqrt(e * e + n_comp * n_comp);
    out.r_m[i] = std::sqrt(r_xy * r_xy + u * u);
    out.az_rad[i] = std::atan2(e, n_comp);
    out.el_rad[i] = std::atan2(u, r_xy);
  }
}

void EcefToEnuPoint(double x_ecef,
                    double y_ecef,
                    double z_ecef,
                    const cfg::Ownship& ownship,
                    double& e_m,
                    double& n_m,
                    double& u_m) {
  const Vec3 own{ownship.pos_x, ownship.pos_y, ownship.pos_z};
  Vec3 east, north, up;
  enu_basis_from_ecef(own, east, north, up);

  const Vec3 d{x_ecef - own.x, y_ecef - own.y, z_ecef - own.z};
  e_m = east.x * d.x + east.y * d.y + east.z * d.z;
  n_m = north.x * d.x + north.y * d.y + north.z * d.z;
  u_m = up.x * d.x + up.y * d.y + up.z * d.z;
}

void EcefToEnuBatch(const EcefBatch& ecef,
                    const cfg::Ownship& ownship,
                    AerRadBatch& out) {
  const std::size_t n = ecef.n;
  out.resize(n);
  if (n == 0) return;
  ecef.assert_sizes();

  const Vec3 own{ownship.pos_x, ownship.pos_y, ownship.pos_z};
  Vec3 east, north, up;
  enu_basis_from_ecef(own, east, north, up);

  const std::size_t cols = n;
  std::vector<double> delta(3 * cols, 0.0);
  std::vector<double> enu(3 * cols, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    delta[0 * cols + i] = ecef.x[i] - own.x;
    delta[1 * cols + i] = ecef.y[i] - own.y;
    delta[2 * cols + i] = ecef.z[i] - own.z;
  }

  double basis[9] = {
    east.x,  east.y,  east.z,
    north.x, north.y, north.z,
    up.x,    up.y,    up.z
  };

  la::MatrixView A{basis, 3, 3, 3};
  la::MatrixView B{delta.data(), 3, cols, cols};
  la::MatrixView C{enu.data(), 3, cols, cols};
  la::gemm(false, false, 1.0, A, B, 0.0, C);

  for (std::size_t i = 0; i < n; ++i) {
    out.az_rad[i] = enu[0 * cols + i]; // reuse az_rad as E
    out.el_rad[i] = enu[1 * cols + i]; // reuse el_rad as N
    out.r_m[i] = enu[2 * cols + i];    // reuse r_m as U
  }
}

void AerToEnuPoint(double az_rad,
                   double el_rad,
                   double r_m,
                   double& e_m,
                   double& n_m,
                   double& u_m) {
  const Vec3 v = aer_to_enu(az_rad, el_rad, r_m);
  e_m = v.x;
  n_m = v.y;
  u_m = v.z;
}

void AerToEcefPoint(double az_rad,
                    double el_rad,
                    double r_m,
                    const cfg::Ownship& ownship,
                    double& x_ecef,
                    double& y_ecef,
                    double& z_ecef) {
  const Vec3 own{ownship.pos_x, ownship.pos_y, ownship.pos_z};
  Vec3 east, north, up;
  enu_basis_from_ecef(own, east, north, up);

  const Vec3 v_enu = aer_to_enu(az_rad, el_rad, r_m);
  const Vec3 v_ecef{
    east.x * v_enu.x + north.x * v_enu.y + up.x * v_enu.z,
    east.y * v_enu.x + north.y * v_enu.y + up.y * v_enu.z,
    east.z * v_enu.x + north.z * v_enu.y + up.z * v_enu.z
  };

  x_ecef = own.x + v_ecef.x;
  y_ecef = own.y + v_ecef.y;
  z_ecef = own.z + v_ecef.z;
}

} // namespace coord
