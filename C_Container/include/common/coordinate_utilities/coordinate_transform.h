#pragma once

#include "common/coordinate_utilities/coordinate_batches.h"
#include "config/config_types.h"

#include <cstddef>
#include <vector>

namespace coord {

struct AerRadBatch {
  std::size_t n = 0;
  std::vector<double> az_rad, el_rad, r_m;

  void resize(std::size_t n_vals) {
    n = n_vals;
    az_rad.resize(n);
    el_rad.resize(n);
    r_m.resize(n);
  }

  void assert_sizes() const;
};

// Convert ECEF positions to AER (radians/meters) relative to ownship using ENU basis.
void EcefToAerBatch(const EcefBatch& ecef,
                    const cfg::Ownship& ownship,
                    AerRadBatch& out);

void EcefToEnuPoint(double x_ecef,
                    double y_ecef,
                    double z_ecef,
                    const cfg::Ownship& ownship,
                    double& e_m,
                    double& n_m,
                    double& u_m);

void AerToEnuPoint(double az_rad,
                   double el_rad,
                   double r_m,
                   double& e_m,
                   double& n_m,
                   double& u_m);

void AerToEcefPoint(double az_rad,
                    double el_rad,
                    double r_m,
                    const cfg::Ownship& ownship,
                    double& x_ecef,
                    double& y_ecef,
                    double& z_ecef);

} // namespace coord
