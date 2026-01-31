#pragma once

#include "config/config_loader.h"
#include "config/config_types.h"

#include <string>

namespace tracker_api {

// Minimal API surface for integration. This will be fleshed out with real
// measurement inputs and track outputs in later steps.
class TrackerEngine {
public:
  TrackerEngine() = default;

  // Load config and prepare the engine.
  bool Initialize(const std::string& system_xml, const std::string& xsd_dir);

  const cfg::ConfigBundle& config() const { return cfg_; }

private:
  cfg::ConfigBundle cfg_{};
  bool initialized_ = false;
};

} // namespace tracker_api
