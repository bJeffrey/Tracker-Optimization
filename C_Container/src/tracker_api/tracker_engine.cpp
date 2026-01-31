#include "tracker_api/tracker_engine.h"

#include <utility>

namespace tracker_api {

bool TrackerEngine::Initialize(const std::string& system_xml, const std::string& xsd_dir) {
  cfg_ = cfg::ConfigLoader::Load(system_xml, xsd_dir);
  initialized_ = true;
  return initialized_;
}

} // namespace tracker_api
