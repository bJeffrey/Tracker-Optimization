#pragma once

#include <cstdint>
#include "track_status.h"   // existing root include/track_status.h

namespace trk {

struct TrackMeta
{
    uint32_t track_id = 0;

    // lifecycle / quality
    TrackStatus status{};   // your enum/struct
    float quality = 0.0f;   // arbitrary [0..1] or score; you can refine later
    uint16_t n_hits = 0;
    uint16_t n_misses = 0;

    // time bookkeeping (core for event-driven)
    double t_pred_s = 0.0;          // state+cov aged-to time (single timestamp rule)
    double t_last_update_s = 0.0;   // last measurement update time

    // optional diagnostics / policy
    uint32_t last_sensor_id = 0;
};

} // namespace trk
