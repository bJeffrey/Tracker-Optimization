#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace idx {

// Axis-aligned bounding box in ECEF meters.
struct EcefAabb {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  double min_z = 0.0;
  double max_z = 0.0;
};

// Minimal in-memory 3D RTree index backed by SQLite R*Tree.
// Intended for coarse spatial candidate queries (AABB overlap).
class SqliteRTreeIndex {
public:
  SqliteRTreeIndex();
  ~SqliteRTreeIndex();

  SqliteRTreeIndex(const SqliteRTreeIndex&) = delete;
  SqliteRTreeIndex& operator=(const SqliteRTreeIndex&) = delete;

  // Clear + (re)create schema.
  void Reset();

  // Insert a point as a zero-volume box.
  void InsertPoint(std::uint64_t id, double x, double y, double z);

  // Convenience: bulk build from XYZ arrays (id = [0..n-1]).
  void BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n);

   // Build index from interleaved state buffer: state[i*stride + x_off/y_off/z_off]
   // stride is doubles-per-track (e.g., 9 for [pos,vel,accel]).
   void BuildFromInterleavedXYZ( const double* state,
                                 std::size_t n_tracks,
                                 std::size_t stride,
                                 std::size_t x_off = 0,
                                 std::size_t y_off = 1,
                                 std::size_t z_off = 2);

  // Query IDs whose boxes overlap the given AABB.
  std::vector<std::uint64_t> QueryAabb(const EcefAabb& aabb) const;

  std::size_t size() const;

private:
  struct Impl;
  Impl* p_;
};

} // namespace idx
