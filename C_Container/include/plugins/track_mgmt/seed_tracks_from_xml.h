#pragma once
#include <string>
#include "target_truth.h"
#include "trk/track_batch.h"
#include "config_types.h"

namespace trk {

/**
 * @brief Seed tracker TrackBatch from truth, and apply XML-driven covariance + "stale age".
 *
 * This keeps sim/truth separate while still enabling performance/plumbing tests where the
 * tracker starts with a large track set.
 *
 * Track timestamp consistency:
 * - truth is at truth.t_truth_s (= NowSeconds in XML)
 * - each track is seeded at t0 = NowSeconds - age_s
 * - state is back-propagated to t0 under CA9:
 *     v0 = v_now - a*age
 *     p0 = p_now - v_now*age + 0.5*a*age^2
 * - last_update_s and last_cov_prop_s are set to t0
 */
void SeedTracksFromTruthXml(const std::string& targets_xml,
                            const std::string& xsd_dir,
                            const cfg::Ownship& ownship,
                            const sim::TargetTruth& truth,
                            trk::TrackBatch& tracks);

} // namespace trk
