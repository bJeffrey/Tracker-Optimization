#pragma once

#include "index/rtree_sqlite.h"         // idx::EcefAabb
#include "index/spatial_index_3d.h"     // idx::ISpatialIndex3D

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace idx {

class UniformGridIndex final : public ISpatialIndex3D {
public:
  explicit UniformGridIndex(double cell_m = 2000.0);

  void SetCellSizeMeters(double cell_m);
  double CellSizeMeters() const { return cell_m_; }

  void Reserve(std::size_t n_tracks) override;
  void BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) override;
  void UpdatePoint(std::uint64_t id, double x, double y, double z) override;
  std::vector<std::uint64_t> QueryAabb(const EcefAabb& aabb) const override;
  bool SupportsIncrementalUpdates() const override { return true; }

  // Optional knob: when the AABB spans more than this many cells, use sparse scan.
  void SetDenseCellProbeLimit(std::uint64_t limit) { dense_cell_probe_limit_ = limit; }

private:
  struct CellKey {
    std::int32_t ix = 0;
    std::int32_t iy = 0;
    std::int32_t iz = 0;
    bool operator==(const CellKey& o) const noexcept { return ix==o.ix && iy==o.iy && iz==o.iz; }
  };

  struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const noexcept {
      std::uint64_t x = static_cast<std::uint32_t>(k.ix);
      std::uint64_t y = static_cast<std::uint32_t>(k.iy);
      std::uint64_t z = static_cast<std::uint32_t>(k.iz);
      std::uint64_t h = x * 0x9E3779B185EBCA87ULL;
      h ^= y + 0xC2B2AE3D27D4EB4FULL + (h << 6) + (h >> 2);
      h ^= z + 0x165667B19E3779F9ULL + (h << 6) + (h >> 2);
      return static_cast<std::size_t>(h);
    }
  };

  using CellList = std::vector<std::uint64_t>;

  struct Cell {
    CellList ids;
    std::size_t occ_idx = 0; // index into occupied_keys_
  };

  using CellMap = std::unordered_map<CellKey, Cell, CellKeyHash>;

  CellKey pos_to_cell(double x, double y, double z) const;
  static void swap_remove(CellList& v, std::size_t idx);

  bool key_in_range(const CellKey& k,
                    std::int32_t ix0, std::int32_t ix1,
                    std::int32_t iy0, std::int32_t iy1,
                    std::int32_t iz0, std::int32_t iz1) const noexcept {
    return (k.ix >= ix0 && k.ix <= ix1 &&
            k.iy >= iy0 && k.iy <= iy1 &&
            k.iz >= iz0 && k.iz <= iz1);
  }

  void occ_add(CellMap::iterator it);
  void occ_remove(CellMap::iterator it);

  double cell_m_ = 2000.0;

  CellMap cells_;
  std::vector<CellKey> occupied_keys_;  // only keys that currently have non-empty cells

  // Per-track metadata
  std::vector<CellKey> track_cell_;
  std::vector<std::size_t> track_slot_;
  std::vector<std::uint8_t> track_init_;

  // Query strategy
  std::uint64_t dense_cell_probe_limit_ = 200000; // default: above this, scan occupied keys instead
};

} // namespace idx