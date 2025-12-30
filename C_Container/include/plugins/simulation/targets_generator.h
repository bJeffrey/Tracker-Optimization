#pragma once
/**
 * @file targets_generator.h
 * @brief Deterministic synthetic TARGET TRUTH generation from targets_gen.xml.
 *
 * Produces truth kinematics (ECEF CA9): [x y z vx vy vz ax ay az]
 * Does NOT touch tracker containers, covariances, or track metadata.
 */

#include "target_truth.h"
#include "config_types.h"

#include <string>

namespace sim {

/**
 * @brief Populate TargetTruth with synthetic truth using targets_gen XML.
 *
 * @param targets_xml   Path to targets_gen_*.xml
 * @param xsd_dir       Directory containing targets_gen.xsd (optional; can be empty)
 * @param ownship       Ownship pose (ECEF) used for Center="ownship" models
 * @param truth         TargetTruth to fill (must already be resized)
 */
void GenerateTruthFromXml(const std::string& targets_xml,
                          const std::string& xsd_dir,
                          const cfg::Ownship& ownship,
                          sim::TargetTruth& truth);

} // namespace sim
