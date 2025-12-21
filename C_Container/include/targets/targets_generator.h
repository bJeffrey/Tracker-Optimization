#pragma once
/**
 * @file targets_generator.h
 * @brief Deterministic synthetic track generation from targets_gen.xml.
 *
 * Step 2 objective:
 *  - Fill TrackBatch.x (ECEF CA9 state) and TrackBatch metadata using a config-driven
 *    target-generation XML file (targets_gen_*.xml).
 *
 * Notes:
 *  - Output state is ECEF: [x y z vx vy vz ax ay az]
 *  - This generator is intended for performance / plumbing tests first.
 *  - It supports:
 *      - Mixture/Background and Mixture/Local components
 *      - Model types: ecef_box, global_geodetic
 *    and will throw for unsupported model types (e.g., enu_bubble) until implemented.
 */

#include "track_batch.h"
#include "config/config_types.h"

#include <string>

namespace targets {

/**
 * @brief Populate TrackBatch with synthetic state/cov/metadata using targets_gen XML.
 *
 * @param targets_xml   Path to targets_gen_*.xml
 * @param xsd_dir       Directory containing targets_gen.xsd (optional; can be empty)
 * @param ownship       Ownship pose (ECEF) used for Center="ownship" models
 * @param tb            TrackBatch to fill (must already be resized)
 */
void GenerateFromXml(const std::string& targets_xml,
                     const std::string& xsd_dir,
                     const cfg::Ownship& ownship,
                     TrackBatch& tb);

} // namespace targets
