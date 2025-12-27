#pragma once

#include "spatial_index_3d.h"

#include <memory>
#include <string>

namespace idx {

struct SpatialIndexConfig {
  // "sqlite_rtree" or "uniform_grid"
  std::string backend = "sqlite_rtree";

  // SQLite-specific (for future; current SqliteRTreeIndex is :memory: only).
  std::string sqlite_mode = "memory";  // "memory" | "disk"
  std::string sqlite_path = "";        // used if mode == "disk"

  // Grid-specific
  double grid_cell_m = 2000.0;
};

std::unique_ptr<ISpatialIndex3D> CreateSpatialIndex(const SpatialIndexConfig& cfg);

} // namespace idx
