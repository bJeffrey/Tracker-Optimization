#pragma once
#include "plugins/simulation/target_truth.h"
#include "plugins/track_mgmt/track_batch.h"   // tracker-owned TrackBatch (after your move/namespace)

namespace trk {

// Explicit: this is not truth generation; it seeds tracker estimates from truth.
void SeedTracksFromTruth(const sim::TargetTruth& truth,
                         trk::TrackBatch& tracks,
                         double t0_s);

} // namespace trk
