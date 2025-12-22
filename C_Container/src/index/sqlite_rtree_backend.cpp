#include "index/sqlite_rtree_backend.h"

#include <stdexcept>

namespace idx {

SqliteRTreeIndexBackend::SqliteRTreeIndexBackend() = default;

void SqliteRTreeIndexBackend::Reserve(std::size_t n_tracks) {
  reserved_ = n_tracks;
  (void)reserved_;
  // SqliteRTreeIndex currently doesn't need reserve.
}

void SqliteRTreeIndexBackend::BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) {
  rtree_.BuildFromXYZ(xs, ys, zs, n);
}

void SqliteRTreeIndexBackend::UpdatePoint(std::uint64_t /*id*/, double /*x*/, double /*y*/, double /*z*/) {
  // Not supported. Caller should use IndexUpdateManager which will degrade to rebuild for non-incremental backends.
}

std::vector<std::uint64_t> SqliteRTreeIndexBackend::QueryAabb(const EcefAabb& aabb) const {
  return rtree_.QueryAabb(aabb);
}

} // namespace idx
