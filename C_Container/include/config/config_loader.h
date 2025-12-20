#pragma once
/**
 * @file config_loader.h
 * @brief Loader for the tracker XML configuration set, with optional XSD validation.
 */

#include "config/config_types.h"
#include <string>

namespace cfg {

class ConfigLoader {
public:
  /**
   * @brief Load a full configuration bundle starting from system.xml.
   *
   * @param system_xml_path Path to config/system.xml (or equivalent root file).
   * @param xsd_dir Optional directory holding XSD files (system.xsd, runtime_profiles.xsd, ...).
   *                If empty, schema validation is skipped (parsing still occurs).
   *
   * @throws std::runtime_error on IO/parse/validation errors.
   */
  static ConfigBundle Load(const std::string& system_xml_path,
                           const std::string& xsd_dir = "");
};

} // namespace cfg
