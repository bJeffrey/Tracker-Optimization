#pragma once

#include "plugins/measurement/aer_prediction.h"

#include <cmath>

namespace gating {

inline bool PassSigmaGate(const meas::AerPred& meas,
                          const meas::AerPred& pred,
                          const meas::RadarSigmas& sigmas,
                          double k_sigma) {
  const double dr = meas.range_m - pred.range_m;
  const double daz = meas.az_rad - pred.az_rad;
  const double del = meas.el_rad - pred.el_rad;
  if (sigmas.sigma_r > 0.0 && std::fabs(dr) > k_sigma * sigmas.sigma_r) return false;
  if (sigmas.sigma_az > 0.0 && std::fabs(daz) > k_sigma * sigmas.sigma_az) return false;
  if (sigmas.sigma_el > 0.0 && std::fabs(del) > k_sigma * sigmas.sigma_el) return false;
  return true;
}

inline double AerResidualCost(const meas::AerPred& meas,
                              const meas::AerPred& pred) {
  const double dr = meas.range_m - pred.range_m;
  const double daz = meas.az_rad - pred.az_rad;
  const double del = meas.el_rad - pred.el_rad;
  return dr * dr + daz * daz + del * del;
}

} // namespace gating
