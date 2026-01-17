#pragma once

#include "plugins/database/spatial_index_3d.h"
#include "common/track_batch.h"
#include "tracker_types.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace db {

struct TrackDatabaseConfig {
  std::string mode;    // HOT_ONLY / HOT_PLUS_WARM
  std::string backend; // "sqlite_rtree" or "uniform_grid"
  std::string sqlite_db_uri;
  double grid_cell_m = 2000.0;
  std::uint64_t dense_cell_probe_limit = 200000;
  double d_th_m = 500.0;
  double t_max_s = 1.0;
  std::size_t warm_commit_every_scans = 1;
  std::size_t warm_commit_after_tracks = 1000;
};

struct TrackDbTimingStats {
  double prefetch_acc_s = 0.0;
  std::size_t prefetch_calls = 0;
  double finalize_begin_acc_s = 0.0;
  std::size_t finalize_begin_calls = 0;
  double finalize_apply_acc_s = 0.0;
  double finalize_commit_acc_s = 0.0;
  std::size_t finalize_commit_calls = 0;
  std::size_t finalize_calls = 0;

  double last_prefetch_s = 0.0;
  double last_finalize_begin_s = 0.0;
  double last_finalize_apply_s = 0.0;
  double last_finalize_commit_s = 0.0;
};

class ITrackDatabase {
public:
  virtual ~ITrackDatabase() = default;

  virtual void Reserve(std::size_t n_tracks) = 0;
  virtual void Configure(double d_th_m, double t_max_s) = 0;

  // Update index entries using current positions at time t_now_s.
  virtual void UpdateTracks(double t_now_s,
                            const double* xs,
                            const double* ys,
                            const double* zs,
                            std::size_t n_tracks) = 0;

  // Persist post-scan track updates (no-op for hot-only).
  virtual void FinalizeScan(double t_now_s,
                            const trk::TrackBatch& tracks,
                            const trk::IdList* ids) = 0;

  // Query AABB overlap candidates.
  virtual trk::IdList QueryAabb(const idx::EcefAabb& aabb) const = 0;

  // Warm prefetch to define the hot working set (hot+warm only).
  virtual trk::IdList PrefetchHot(const idx::EcefAabb& aabb) = 0;

  virtual std::size_t NumUpdatedLastUpdate() const = 0;
  virtual bool SupportsIncrementalUpdates() const = 0;

  virtual TrackDbTimingStats GetTimingStats() const = 0;
  virtual void ResetTimingStats() = 0;

  // Flush any pending warm updates and wait for completion (hot+warm only).
  virtual void FlushWarmUpdates() = 0;

  // Warm DB signature support (hot+warm only).
  virtual bool WarmSignatureMatches(const std::string& signature) const = 0;
  virtual void StoreWarmSignature(const std::string& signature) = 0;
};

std::unique_ptr<ITrackDatabase> CreateTrackDatabase(const TrackDatabaseConfig& cfg);

} // namespace db
