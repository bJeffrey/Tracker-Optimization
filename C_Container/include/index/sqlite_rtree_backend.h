#pragma once

#include "index/rtree_sqlite.h"
#include "index/spatial_index_3d.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace idx {

// Adapter so SqliteRTreeIndex can be used via ISpatialIndex3D.
// Note: current SqliteRTreeIndex implementation rebuilds per BuildFromXYZ.
// Incremental updates are not supported here (degrades to no-op UpdatePoint).
class SqliteRTreeIndexBackend final : public ISpatialIndex3D {
public:
  SqliteRTreeIndexBackend();

  void Reserve(std::size_t n_tracks) override;
  void BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) override;
  void UpdatePoint(std::uint64_t id, double x, double y, double z) override;
  std::vector<std::uint64_t> QueryAabb(const EcefAabb& aabb) const override;
  bool SupportsIncrementalUpdates() const override { return false; }

private:
  SqliteRTreeIndex rtree_;
  std::size_t reserved_ = 0;
};

} // namespace idx
