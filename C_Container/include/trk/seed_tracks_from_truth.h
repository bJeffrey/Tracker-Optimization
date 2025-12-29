#pragma once
#include "target_truth.h"
#include "trk/track_batch.h"
#include <string>

namespace trk {

// Seed tracker estimate state/cov/meta from truth.
// If you want XML-driven covariance + last-update “age”, parse those here.
void SeedTracksFromTruthXml(const sim::TargetTruth& truth,
                            const std::string& targets_xml,
                            const std::string& xsd_dir,
                            trk::TrackBatch& tracks);

} // namespace trk
