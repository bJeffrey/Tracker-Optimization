#pragma once

#include <cstdint>

namespace trk {

// Canonical interleaved state layout: [x y z vx vy vz ax ay az]
enum StateIndex : std::uint8_t {
  kX  = 0,
  kY  = 1,
  kZ  = 2,
  kVx = 3,
  kVy = 4,
  kVz = 5,
  kAx = 6,
  kAy = 7,
  kAz = 8
};

static constexpr int kStateDim = 9;
static constexpr int kStateCovN = kStateDim * kStateDim;

} // namespace trk
