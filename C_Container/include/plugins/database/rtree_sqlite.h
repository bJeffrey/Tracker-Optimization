#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace idx {

struct EcefAabb {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
};

class SqliteRTreeIndex {
public:
  SqliteRTreeIndex();
  ~SqliteRTreeIndex();

  SqliteRTreeIndex(const SqliteRTreeIndex&) = delete;
  SqliteRTreeIndex& operator=(const SqliteRTreeIndex&) = delete;

  // Clear + (re)create schema. (Call once at init; do NOT call every scan.)
  void Reset();

  // New: fast maintenance APIs
  void Begin();      // BEGIN IMMEDIATE
  void Commit();     // COMMIT
  void Rollback();   // ROLLBACK (safe to call after errors)
  void ClearAll();   // DELETE FROM rtree;

  // Insert a point as a zero-volume box.
  void InsertPoint(std::uint64_t id, double x, double y, double z);

  // New: fast bulk insert (no ClearAll inside; caller chooses)
  void BulkInsertPointsXYZ(const double* xs, const double* ys, const double* zs, std::size_t n,
                           std::uint64_t id0 = 0);

  // Convenience: bulk rebuild from XYZ arrays (id = [id0..id0+n-1]).
  // This is what you should call per scan tick (fast path).
  void RebuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n,
                      std::uint64_t id0 = 0);

  // Keep old name for compatibility (now just calls RebuildFromXYZ with id0=0)
  void BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n);

  void BuildFromInterleavedXYZ(const double* state,
                               std::size_t n_tracks,
                               std::size_t stride,
                               std::size_t x_off = 0,
                               std::size_t y_off = 1,
                               std::size_t z_off = 2);

  std::vector<std::uint64_t> QueryAabb(const EcefAabb& aabb) const;

  std::size_t size() const;

private:
  struct Impl;
  Impl* p_;
};

} // namespace idx
