#pragma once

#include "rtree_sqlite.h"
#include "spatial_index_3d.h"

#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>

namespace idx {

// Adapter so SqliteRTreeIndex can be used via ISpatialIndex3D.
// Note: current SqliteRTreeIndex implementation rebuilds per BuildFromXYZ.
// Incremental updates are not supported here (degrades to no-op UpdatePoint).
class SqliteRTreeIndexBackend final : public ISpatialIndex3D {
public:
  SqliteRTreeIndexBackend();
  explicit SqliteRTreeIndexBackend(const std::string& db_uri);

  void Reserve(std::size_t n_tracks) override;
  void BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) override;
  void UpdatePoint(std::uint64_t id, double x, double y, double z) override;
  std::vector<std::uint64_t> QueryAabb(const EcefAabb& aabb) const override;
  bool SupportsIncrementalUpdates() const override { return true; }

  void Begin();
  void Commit();
  void Rollback();

  void EnsureMetaTable();
  std::string GetMeta(const std::string& key) const;
  void SetMeta(const std::string& key, const std::string& value);

  void EnsureTrackTables();
  void UpsertTrackState(std::uint64_t id, const double* x9, double t_last_update, double t_pred);
  void UpsertTrackCovUpper(std::uint64_t id, const double* cov_upper45);
  void UpsertTrackMeta(std::uint64_t id, int status, double quality);

private:
  SqliteRTreeIndex rtree_;
  std::size_t reserved_ = 0;
};

} // namespace idx
