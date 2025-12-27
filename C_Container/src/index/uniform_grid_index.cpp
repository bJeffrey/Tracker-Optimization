#include "uniform_grid_index.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace idx {

static inline std::int32_t floor_to_i32(double v) {
  const double f = std::floor(v);
  if (f < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
      f > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    throw std::runtime_error("UniformGridIndex: cell coord overflow");
  }
  return static_cast<std::int32_t>(f);
}

UniformGridIndex::UniformGridIndex(double cell_m) : cell_m_(cell_m) {
  if (!(cell_m_ > 0.0)) throw std::runtime_error("UniformGridIndex: cell_m must be > 0");
}

void UniformGridIndex::SetCellSizeMeters(double cell_m) {
  if (!(cell_m > 0.0)) throw std::runtime_error("UniformGridIndex: cell_m must be > 0");
  cell_m_ = cell_m;
}

void UniformGridIndex::Reserve(std::size_t n_tracks) {
  track_cell_.resize(n_tracks);
  track_slot_.resize(n_tracks);
  track_init_.assign(n_tracks, 0);
}

UniformGridIndex::CellKey UniformGridIndex::pos_to_cell(double x, double y, double z) const {
  const double inv = 1.0 / cell_m_;
  return CellKey{floor_to_i32(x * inv), floor_to_i32(y * inv), floor_to_i32(z * inv)};
}

void UniformGridIndex::swap_remove(CellList& v, std::size_t idx) {
  const std::size_t last = v.size() - 1;
  if (idx != last) v[idx] = v[last];
  v.pop_back();
}

void UniformGridIndex::occ_add(CellMap::iterator it) {
  // called only when cell transitions 0 -> 1 ids
  it->second.occ_idx = occupied_keys_.size();
  occupied_keys_.push_back(it->first);
}

void UniformGridIndex::occ_remove(CellMap::iterator it) {
  // called only when cell transitions 1 -> 0 ids
  const std::size_t idx = it->second.occ_idx;
  const std::size_t last = occupied_keys_.size() - 1;
  if (idx != last) {
    const CellKey moved_key = occupied_keys_[last];
    occupied_keys_[idx] = moved_key;
    auto it_moved = cells_.find(moved_key);
    if (it_moved != cells_.end()) {
      it_moved->second.occ_idx = idx;
    }
  }
  occupied_keys_.pop_back();
}

void UniformGridIndex::BuildFromXYZ(const double* xs, const double* ys, const double* zs, std::size_t n) {
  if (!xs || !ys || !zs) throw std::runtime_error("UniformGridIndex::BuildFromXYZ: null pointers");

  cells_.clear();
  occupied_keys_.clear();
  Reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    const std::uint64_t id = static_cast<std::uint64_t>(i);
    const CellKey key = pos_to_cell(xs[i], ys[i], zs[i]);

    auto [it, inserted] = cells_.try_emplace(key, Cell{});
    if (inserted) {
      // cell is new and empty -> will transition to non-empty
    }

    Cell& cell = it->second;
    const bool was_empty = cell.ids.empty();
    cell.ids.push_back(id);

    if (was_empty) {
      occ_add(it);
    }

    track_cell_[i] = key;
    track_slot_[i] = cell.ids.size() - 1;
    track_init_[i] = 1;
  }
}

void UniformGridIndex::UpdatePoint(std::uint64_t id, double x, double y, double z) {
  const std::size_t i = static_cast<std::size_t>(id);
  if (i >= track_init_.size()) {
    throw std::runtime_error("UniformGridIndex::UpdatePoint: id out of range; call Reserve() first");
  }

  const CellKey new_key = pos_to_cell(x, y, z);

  if (!track_init_[i]) {
    auto [it_new, inserted] = cells_.try_emplace(new_key, Cell{});
    Cell& new_cell = it_new->second;
    const bool was_empty = new_cell.ids.empty();
    new_cell.ids.push_back(id);
    if (was_empty) occ_add(it_new);

    track_cell_[i] = new_key;
    track_slot_[i] = new_cell.ids.size() - 1;
    track_init_[i] = 1;
    return;
  }

  const CellKey old_key = track_cell_[i];
  if (new_key == old_key) return;

  auto it_old = cells_.find(old_key);
  if (it_old == cells_.end()) throw std::runtime_error("UniformGridIndex::UpdatePoint: old cell not found");

  Cell& old_cell = it_old->second;
  const std::size_t slot = track_slot_[i];
  if (slot >= old_cell.ids.size()) throw std::runtime_error("UniformGridIndex::UpdatePoint: slot out of range");

  // swap-remove from old cell
  const std::uint64_t swapped_id = old_cell.ids.back();
  swap_remove(old_cell.ids, slot);
  if (slot < old_cell.ids.size()) {
    // swapped_id moved into slot
    const std::size_t swapped_idx = static_cast<std::size_t>(swapped_id);
    if (swapped_idx < track_slot_.size()) track_slot_[swapped_idx] = slot;
  }

  // if old cell became empty, remove from occupied list and erase cell
  if (old_cell.ids.empty()) {
    occ_remove(it_old);
    cells_.erase(it_old);
  }

  // insert into new cell
  auto [it_new, inserted] = cells_.try_emplace(new_key, Cell{});
  Cell& new_cell = it_new->second;
  const bool was_empty = new_cell.ids.empty();
  new_cell.ids.push_back(id);
  if (was_empty) occ_add(it_new);

  track_cell_[i] = new_key;
  track_slot_[i] = new_cell.ids.size() - 1;
}

std::vector<std::uint64_t> UniformGridIndex::QueryAabb(const EcefAabb& aabb) const {
  const double inv = 1.0 / cell_m_;
  const std::int32_t ix0 = floor_to_i32(aabb.min_x * inv);
  const std::int32_t ix1 = floor_to_i32(aabb.max_x * inv);
  const std::int32_t iy0 = floor_to_i32(aabb.min_y * inv);
  const std::int32_t iy1 = floor_to_i32(aabb.max_y * inv);
  const std::int32_t iz0 = floor_to_i32(aabb.min_z * inv);
  const std::int32_t iz1 = floor_to_i32(aabb.max_z * inv);

  const std::uint64_t nx = static_cast<std::uint64_t>(ix1 - ix0 + 1);
  const std::uint64_t ny = static_cast<std::uint64_t>(iy1 - iy0 + 1);
  const std::uint64_t nz = static_cast<std::uint64_t>(iz1 - iz0 + 1);
  const std::uint64_t dense_probes = nx * ny * nz;

  std::vector<std::uint64_t> out;

  // Strategy 1: dense probing (good when AABB spans few cells)
  if (dense_probes <= dense_cell_probe_limit_) {
    for (std::int32_t ix = ix0; ix <= ix1; ++ix) {
      for (std::int32_t iy = iy0; iy <= iy1; ++iy) {
        for (std::int32_t iz = iz0; iz <= iz1; ++iz) {
          const CellKey key{ix, iy, iz};
          auto it = cells_.find(key);
          if (it == cells_.end()) continue;
          const auto& ids = it->second.ids;
          out.insert(out.end(), ids.begin(), ids.end());
        }
      }
    }
    return out;
  }

  // Strategy 2: sparse scan over occupied cells (good for sparse worlds / huge AABBs)
  for (const CellKey& key : occupied_keys_) {
    if (!key_in_range(key, ix0, ix1, iy0, iy1, iz0, iz1)) continue;
    auto it = cells_.find(key);
    if (it == cells_.end()) continue;
    const auto& ids = it->second.ids;
    out.insert(out.end(), ids.begin(), ids.end());
  }

  return out;
}

} // namespace idx
