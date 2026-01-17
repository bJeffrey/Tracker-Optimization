#include "xml_utils.h"

#include <stdexcept>
#include <sstream>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xmlschemas.h>

namespace {

struct LibXmlInit {
  LibXmlInit() { xmlInitParser(); }
  ~LibXmlInit() { xmlCleanupParser(); }
};
LibXmlInit g_init;

xmlNode* root(xmlDocPtr doc) {
  return xmlDocGetRootElement(doc);
}

std::vector<std::string> split_path(const std::string& path) {
  std::vector<std::string> parts;
  std::string cur;
  for (char c : path) {
    if (c == '/') {
      if (!cur.empty()) { parts.push_back(cur); cur.clear(); }
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) parts.push_back(cur);
  return parts;
}

xmlNode* find_child(xmlNode* parent, const std::string& name) {
  for (xmlNode* n = parent ? parent->children : nullptr; n; n = n->next) {
    if (n->type == XML_ELEMENT_NODE && name == reinterpret_cast<const char*>(n->name)) {
      return n;
    }
  }
  return nullptr;
}

xmlNode* find_path(xmlDocPtr doc, const std::string& path) {
  xmlNode* n = root(doc);
  if (!n) return nullptr;

  auto parts = split_path(path);
  // If path begins with root name, skip it.
  if (!parts.empty() && parts[0] == reinterpret_cast<const char*>(n->name)) {
    parts.erase(parts.begin());
  }

  for (const auto& p : parts) {
    n = find_child(n, p);
    if (!n) return nullptr;
  }
  return n;
}

xmlNode* find_path_from(xmlNode* start, const std::string& path) {
  xmlNode* n = start;
  if (!n) return nullptr;
  auto parts = split_path(path);
  // If path begins with current node name, skip it.
  if (!parts.empty() && parts[0] == reinterpret_cast<const char*>(n->name)) {
    parts.erase(parts.begin());
  }
  for (const auto& p : parts) {
    n = find_child(n, p);
    if (!n) return nullptr;
  }
  return n;
}

std::string node_text(xmlNode* n) {
  if (!n) return "";
  xmlChar* content = xmlNodeGetContent(n);
  if (!content) return "";
  std::string s(reinterpret_cast<const char*>(content));
  xmlFree(content);
  // trim simple whitespace
  auto l = s.find_first_not_of(" \t\r\n");
  auto r = s.find_last_not_of(" \t\r\n");
  if (l == std::string::npos) return "";
  return s.substr(l, r - l + 1);
}

std::string node_attr(xmlNode* n, const std::string& attr) {
  if (!n) return "";
  xmlChar* v = xmlGetProp(n, reinterpret_cast<const xmlChar*>(attr.c_str()));
  if (!v) return "";
  std::string s(reinterpret_cast<const char*>(v));
  xmlFree(v);
  return s;
}

double to_double(const std::string& s, double defv) {
  if (s.empty()) return defv;
  try { return std::stod(s); } catch (...) { return defv; }
}
int to_int(const std::string& s, int defv) {
  if (s.empty()) return defv;
  try { return std::stoi(s); } catch (...) { return defv; }
}
bool to_bool_text(const std::string& s, bool defv) {
  if (s.empty()) return defv;
  if (s == "true" || s == "1" || s == "TRUE") return true;
  if (s == "false" || s == "0" || s == "FALSE") return false;
  return defv;
}

} // namespace

namespace cfg::xmlu {

void* ReadXmlDocOrThrow(const std::string& xml_path) {
  xmlDocPtr doc = xmlReadFile(xml_path.c_str(), nullptr, XML_PARSE_NONET);
  if (!doc) {
    std::ostringstream oss;
    oss << "Failed to parse XML: " << xml_path;
    throw std::runtime_error(oss.str());
  }
  return reinterpret_cast<void*>(doc);
}

void FreeXmlDoc(void* doc) {
  if (doc) xmlFreeDoc(reinterpret_cast<xmlDocPtr>(doc));
}

std::string RootName(void* doc) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);
  xmlNode* r = root(d);
  return r ? std::string(reinterpret_cast<const char*>(r->name)) : "";
}

void ValidateOrThrow(void* doc, const std::string& xsd_path, const std::string& xml_path_for_errors) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);

  xmlSchemaParserCtxtPtr pctx = xmlSchemaNewParserCtxt(xsd_path.c_str());
  if (!pctx) {
    throw std::runtime_error("Failed to create XSD parser context: " + xsd_path);
  }

  xmlSchemaPtr schema = xmlSchemaParse(pctx);
  xmlSchemaFreeParserCtxt(pctx);
  if (!schema) {
    throw std::runtime_error("Failed to parse XSD: " + xsd_path);
  }

  xmlSchemaValidCtxtPtr vctx = xmlSchemaNewValidCtxt(schema);
  if (!vctx) {
    xmlSchemaFree(schema);
    throw std::runtime_error("Failed to create XSD validation context: " + xsd_path);
  }

  int rc = xmlSchemaValidateDoc(vctx, d);

  xmlSchemaFreeValidCtxt(vctx);
  xmlSchemaFree(schema);

  if (rc != 0) {
    std::ostringstream oss;
    oss << "XSD validation failed for " << xml_path_for_errors << " using schema " << xsd_path
        << " (rc=" << rc << ")";
    throw std::runtime_error(oss.str());
  }
}

void SaveXmlDocOrThrow(void* doc, const std::string& xml_path) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);
  const int rc = xmlSaveFormatFileEnc(xml_path.c_str(), d, "UTF-8", 1);
  if (rc == -1) {
    throw std::runtime_error("Failed to write XML: " + xml_path);
  }
}

void EnsurePathSetAttr(void* doc,
                       const std::string& path,
                       const std::string& attr,
                       const std::string& value) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);
  xmlNode* n = root(d);
  if (!n) {
    throw std::runtime_error("EnsurePathSetAttr: XML has no root element");
  }

  auto parts = split_path(path);
  if (!parts.empty() && parts[0] == reinterpret_cast<const char*>(n->name)) {
    parts.erase(parts.begin());
  }
  for (const auto& p : parts) {
    xmlNode* c = find_child(n, p);
    if (!c) {
      c = xmlNewChild(n, nullptr, BAD_CAST p.c_str(), nullptr);
      if (!c) {
        throw std::runtime_error("EnsurePathSetAttr: failed to create node '" + p + "'");
      }
    }
    n = c;
  }

  xmlSetProp(n, BAD_CAST attr.c_str(), BAD_CAST value.c_str());
}

std::string GetText(void* doc, const std::string& path) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);
  xmlNode* n = find_path(d, path);
  return node_text(n);
}

std::string GetAttr(void* doc, const std::string& path, const std::string& attr) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);
  xmlNode* n = find_path(d, path);
  return node_attr(n, attr);
}

double GetDouble(void* doc, const std::string& path, double default_val) {
  return to_double(GetText(doc, path), default_val);
}

int GetInt(void* doc, const std::string& path, int default_val) {
  return to_int(GetText(doc, path), default_val);
}

bool GetBoolText(void* doc, const std::string& path, bool default_val) {
  return to_bool_text(GetText(doc, path), default_val);
}

std::vector<void*> FindNodes(void* doc, const std::string& path) {
  xmlDocPtr d = reinterpret_cast<xmlDocPtr>(doc);
  xmlNode* r = root(d);
  if (!r) return {};

  auto parts = split_path(path);
  // If path begins with root name, skip it.
  if (!parts.empty() && parts[0] == reinterpret_cast<const char*>(r->name)) {
    parts.erase(parts.begin());
  }

  // Walk to the parent of last part
  if (parts.empty()) return {};
  std::string leaf = parts.back();
  parts.pop_back();

  xmlNode* parent = r;
  for (const auto& p : parts) {
    parent = find_child(parent, p);
    if (!parent) return {};
  }

  std::vector<void*> out;
  for (xmlNode* n = parent->children; n; n = n->next) {
    if (n->type == XML_ELEMENT_NODE && leaf == reinterpret_cast<const char*>(n->name)) {
      out.push_back(reinterpret_cast<void*>(n));
    }
  }
  return out;
}

std::string NodeGetAttr(void* node, const std::string& attr) {
  return node_attr(reinterpret_cast<xmlNode*>(node), attr);
}

std::string NodeGetTextChild(void* node, const std::string& child_name) {
  xmlNode* n = reinterpret_cast<xmlNode*>(node);
  xmlNode* c = find_child(n, child_name);
  return node_text(c);
}

double NodeGetDoubleChild(void* node, const std::string& child_name, double default_val) {
  return to_double(NodeGetTextChild(node, child_name), default_val);
}

int NodeGetIntChild(void* node, const std::string& child_name, int default_val) {
  return to_int(NodeGetTextChild(node, child_name), default_val);
}

std::string NodeGetTextPath(void* node, const std::string& path) {
  xmlNode* n = reinterpret_cast<xmlNode*>(node);
  xmlNode* p = find_path_from(n, path);
  return node_text(p);
}

std::string NodeGetAttrPath(void* node, const std::string& path, const std::string& attr) {
  xmlNode* n = reinterpret_cast<xmlNode*>(node);
  xmlNode* p = find_path_from(n, path);
  return node_attr(p, attr);
}

double NodeGetDoublePath(void* node, const std::string& path, double default_val) {
  return to_double(NodeGetTextPath(node, path), default_val);
}

} // namespace cfg::xmlu
