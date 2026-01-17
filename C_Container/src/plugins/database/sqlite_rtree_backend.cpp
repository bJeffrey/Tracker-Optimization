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

void SqliteRTreeIndexBackend::EnsureMetaTable() {
  rtree_.EnsureMetaTable();
}

std::string SqliteRTreeIndexBackend::GetMeta(const std::string& key) const {
  return rtree_.GetMeta(key);
}

void SqliteRTreeIndexBackend::SetMeta(const std::string& key, const std::string& value) {
  rtree_.SetMeta(key, value);
}

void SqliteRTreeIndexBackend::EnsureTrackTables() {
  rtree_.EnsureTrackTables();
}

void SqliteRTreeIndexBackend::UpsertTrackState(std::uint64_t id,
                                               const double* x9,
                                               double t_last_update,
                                               double t_pred) {
  rtree_.UpsertTrackState(id, x9, t_last_update, t_pred);
}

void SqliteRTreeIndexBackend::UpsertTrackCovUpper(std::uint64_t id, const double* cov_upper45) {
  rtree_.UpsertTrackCovUpper(id, cov_upper45);
}

void SqliteRTreeIndexBackend::UpsertTrackMeta(std::uint64_t id, int status, double quality) {
  rtree_.UpsertTrackMeta(id, status, quality);
}

} // namespace idx
