#include "config/path_utils.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace cfg::pathu {

std::string Dirname(const std::string& path) {
  fs::path p(path);
  return p.has_parent_path() ? p.parent_path().string() : std::string(".");
}

std::string Join(const std::string& a, const std::string& b) {
  fs::path p(a);
  p /= fs::path(b);
  return p.string();
}

std::string Normalize(const std::string& p) {
  return fs::path(p).lexically_normal().string();
}

std::string ResolveHref(const std::string& owner_xml_path,
                        const std::string& base_dir,
                        const std::string& href) {
  fs::path href_p(href);
  if (href_p.is_absolute()) {
    return Normalize(href_p.string());
  }

  fs::path owner_dir = fs::path(Dirname(owner_xml_path));

  if (!base_dir.empty()) {
    // base_dir is relative to the "owner" file's directory
    fs::path base = owner_dir / fs::path(base_dir);
    return Normalize((base / href_p).string());
  }

  return Normalize((owner_dir / href_p).string());
}

} // namespace cfg::pathu
