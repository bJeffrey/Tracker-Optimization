#pragma once

#include <string>
#include <vector>

namespace cfg::xmlu {

// Read file into libxml2 document and return opaque pointer as void* to avoid exposing libxml headers here.
void* ReadXmlDocOrThrow(const std::string& xml_path);

// Free a doc returned by ReadXmlDocOrThrow.
void FreeXmlDoc(void* doc);

// Return the root element name.
std::string RootName(void* doc);

// Validate doc against an XSD file. Throws on failure.
// Note: requires libxml2 built with schema support (standard for distro packages).
void ValidateOrThrow(void* doc, const std::string& xsd_path, const std::string& xml_path_for_errors);

// Extract helpers (first child element text, attribute, etc.)
// Paths are "Element/Child/Subchild" relative to the root; missing returns empty.
std::string GetText(void* doc, const std::string& path);
std::string GetAttr(void* doc, const std::string& path, const std::string& attr);

// Parse helpers with defaults
double GetDouble(void* doc, const std::string& path, double default_val);
int GetInt(void* doc, const std::string& path, int default_val);
bool GetBoolText(void* doc, const std::string& path, bool default_val);

// Return all nodes matching a simple path: e.g. "RuntimeProfiles/Profile" (relative to root)
// For each node, return its path index string "Profile[i]". Used for iterating.
std::vector<void*> FindNodes(void* doc, const std::string& path);

// Node-scoped helpers
std::string NodeGetAttr(void* node, const std::string& attr);
std::string NodeGetTextChild(void* node, const std::string& child_name);
double NodeGetDoubleChild(void* node, const std::string& child_name, double default_val);
int NodeGetIntChild(void* node, const std::string& child_name, int default_val);

} // namespace cfg::xmlu
