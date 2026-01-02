#include "sqlite_rtree_backend.h"

#include <stdexcept>

namespace idx {

SqliteRTreeIndexBackend::SqliteRTreeIndexBackend() = default;

SqliteRTreeIndexBackend::SqliteRTreeIndexBackend(const std::string& db_uri)
  : rtree_(db_uri) {}

void SqliteRTreeIndexBackend::Reserve(std::size_t n_tracks) {
  reserved_ = n_tracks;
  (void)reserved_;
  // SqliteRTreeIndex currently doesn't need reserve.
}

void SqliteRTreeIndexBackend::BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) {
  rtree_.BuildFromXYZ(xs, ys, zs, n);
}

void SqliteRTreeIndexBackend::UpdatePoint(std::uint64_t id, double x, double y, double z) {
  rtree_.UpdatePoint(id, x, y, z);
}

std::vector<std::uint64_t> SqliteRTreeIndexBackend::QueryAabb(const EcefAabb& aabb) const {
  return rtree_.QueryAabb(aabb);
}

void SqliteRTreeIndexBackend::Begin() {
  rtree_.Begin();
}

void SqliteRTreeIndexBackend::Commit() {
  rtree_.Commit();
}

void SqliteRTreeIndexBackend::Rollback() {
  rtree_.Rollback();
}

} // namespace idx
