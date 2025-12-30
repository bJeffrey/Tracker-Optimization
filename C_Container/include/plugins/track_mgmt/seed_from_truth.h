#pragma once
#include "trk/target_truth.h"
#include "track_batch.h"   // tracker-owned TrackBatch (after your move/namespace)

namespace trk {

// Explicit: this is not truth generation; it seeds tracker estimates from truth.
void SeedTracksFromTruth(const sim::TargetTruth& truth,
                         trk::TrackBatch& tracks,
                         double t0_s);

} // namespace trk
