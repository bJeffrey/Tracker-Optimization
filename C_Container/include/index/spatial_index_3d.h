#pragma once

#include "index/rtree_sqlite.h" // for idx::EcefAabb

#include <cstddef>
#include <cstdint>
#include <vector>

namespace idx {

// Backend-agnostic 3D spatial index interface for coarse candidate generation.
// - Tracks are identified by uint64_t ids.
// - Typical usage is scan-tick coarse queries (AABB overlap).
// - Some backends support incremental updates; others may rebuild per scan.
class ISpatialIndex3D {
public:
  virtual ~ISpatialIndex3D() = default;

  // Optional: allow backend to size internal per-track metadata.
  // Safe to call multiple times; may reallocate.
  virtual void Reserve(std::size_t n_tracks) = 0;

  // Full rebuild from point positions (id = [0..n-1]).
  virtual void BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) = 0;

  // Incremental update of a single point (id must be in [0..n_tracks-1] after Reserve/Build).
  // If a backend doesn't support incremental updates, it should either:
  // - no-op (and rely on BuildFromXYZ), or
  // - throw (not recommended for production), or
  // - internally degrade to a rebuild strategy.
  virtual void UpdatePoint(std::uint64_t id, double x, double y, double z) = 0;

  // Query AABB overlap candidates.
  virtual std::vector<std::uint64_t> QueryAabb(const EcefAabb& aabb) const = 0;

  // Capability query.
  virtual bool SupportsIncrementalUpdates() const = 0;
};

} // namespace idx
