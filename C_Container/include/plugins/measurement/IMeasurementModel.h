#pragma once

#include "config/config_types.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace meas {

struct SensorMeta {
  std::string sensor_id;
  std::string frame;
  double timestamp_s = 0.0;
  double ownship_x_ecef = 0.0;
  double ownship_y_ecef = 0.0;
  double ownship_z_ecef = 0.0;
};

struct RadarMeasurement {
  std::uint64_t id = 0;
  double range_m = 0.0;
  double az_rad = 0.0;
  double el_rad = 0.0;
  double snr_db = 0.0;
  double rcs_m2 = 0.0;
  int gate_index = -1;
  std::array<double, 9> R = {0.0};
};

struct MeasurementBatch {
  SensorMeta meta{};
  cfg::RadarMeasurementType type = cfg::RadarMeasurementType::UNKNOWN;
  std::vector<RadarMeasurement> measurements;
};

} // namespace meas
