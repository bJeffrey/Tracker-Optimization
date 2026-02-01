#pragma once

#include "config/config_types.h"

#include <cmath>

namespace meas {

struct AerPred {
  double range_m = 0.0;
  double az_rad = 0.0;
  double el_rad = 0.0;
};

inline AerPred PredictAerFromEnu(double e, double n, double u) {
  AerPred out{};
  const double r_xy = std::sqrt(e * e + n * n);
  out.range_m = std::sqrt(r_xy * r_xy + u * u);
  out.az_rad = std::atan2(e, n);
  out.el_rad = std::atan2(u, r_xy);
  return out;
}

struct RadarSigmas {
  double sigma_r = 0.0;
  double sigma_az = 0.0;
  double sigma_el = 0.0;
};

inline RadarSigmas RadarSigmasFromCfg(const cfg::SensorCfg& sensor) {
  const double snr_lin = std::pow(10.0, 0.1 * sensor.meas_model.radar.ref_snr_db);
  const double bw_az_rad = (sensor.beamwidth_3db_az_deg > 0.0)
                             ? (sensor.beamwidth_3db_az_deg * 3.14159265358979323846 / 180.0)
                             : 0.0;
  const double bw_el_rad = (sensor.beamwidth_3db_el_deg > 0.0)
                             ? (sensor.beamwidth_3db_el_deg * 3.14159265358979323846 / 180.0)
                             : 0.0;
  RadarSigmas out{};
  out.sigma_r = (snr_lin > 0.0) ? (sensor.meas_model.radar.ref_range_m / snr_lin) : 0.0;
  out.sigma_az = (snr_lin > 0.0) ? (bw_az_rad / snr_lin) : 0.0;
  out.sigma_el = (snr_lin > 0.0) ? (bw_el_rad / snr_lin) : 0.0;
  return out;
}

} // namespace meas
