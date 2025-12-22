#include "index/spatial_index_factory.h"

#include "index/sqlite_rtree_backend.h"
#include "index/uniform_grid_index.h"

#include <stdexcept>

namespace idx {

std::unique_ptr<ISpatialIndex3D> CreateSpatialIndex(const SpatialIndexConfig& cfg) {
  if (cfg.backend == "sqlite_rtree") {
    // NOTE: sqlite_mode/sqlite_path reserved for later when SqliteRTreeIndex accepts a file path.
    return std::make_unique<SqliteRTreeIndexBackend>();
  }

  if (cfg.backend == "uniform_grid") {
    auto p = std::make_unique<UniformGridIndex>(cfg.grid_cell_m);
    return p;
  }

  throw std::runtime_error("CreateSpatialIndex: unknown backend '" + cfg.backend + "'");
}

} // namespace idx
