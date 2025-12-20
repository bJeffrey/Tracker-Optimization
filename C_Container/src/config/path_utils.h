#pragma once
#include <string>

namespace cfg::pathu {

// Resolve href relative to base_dir and the directory containing the "owner" xml file.
std::string ResolveHref(const std::string& owner_xml_path,
                        const std::string& base_dir,
                        const std::string& href);

// Return directory portion of a path.
std::string Dirname(const std::string& path);

// Join paths in a platform-correct way.
std::string Join(const std::string& a, const std::string& b);

// Normalize (lexically) a path.
std::string Normalize(const std::string& p);

} // namespace cfg::pathu
