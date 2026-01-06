#include "plugins/database/track_database.h"

#include "plugins/database/index_update_manager.h"
#include "plugins/database/spatial_index_factory.h"
#include "plugins/database/sqlite_rtree_backend.h"
#include "plugins/database/uniform_grid_index.h"

#include <algorithm>
#include <chrono>
#include <cctype>

namespace db {
namespace {

std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

class HotOnlyTrackDatabase final : public ITrackDatabase {
public:
  explicit HotOnlyTrackDatabase(const TrackDatabaseConfig& cfg)
      : cfg_(cfg) {
    idx::SpatialIndexConfig icfg;
    icfg.backend = cfg_.backend;
    icfg.grid_cell_m = cfg_.grid_cell_m;
    index_ = idx::CreateSpatialIndex(icfg);
    ApplyDenseCellProbeLimit();
  }

  void Reserve(std::size_t n_tracks) override {
    if (!index_) return;
    index_->Reserve(n_tracks);
    updater_.Reset(n_tracks);
  }

  void Configure(double d_th_m, double t_max_s) override {
    updater_.Configure(d_th_m, t_max_s);
  }

  void UpdateTracks(double t_now_s,
                    const double* xs,
                    const double* ys,
                    const double* zs,
                    std::size_t n_tracks) override {
    (void)n_tracks;
    if (!index_) return;
    updater_.Apply(t_now_s, xs, ys, zs, *index_);
  }

  void UpdateTracksSubset(double t_now_s,
                          const double* xs,
                          const double* ys,
                          const double* zs,
                          const trk::IdList& ids) {
    if (!index_) return;
    if (ids.empty()) return;
    updater_.ApplySubsetThreshold(t_now_s, xs, ys, zs, ids.data(), ids.size(), *index_);
  }

  void FinalizeScan(double /*t_now_s*/,
                    const double* /*xs*/,
                    const double* /*ys*/,
                    const double* /*zs*/,
                    std::size_t /*n_tracks*/,
                    const trk::IdList* /*ids*/) override {}

  trk::IdList QueryAabb(const idx::EcefAabb& aabb) const override {
    if (!index_) return {};
    return index_->QueryAabb(aabb);
  }

  trk::IdList PrefetchHot(const idx::EcefAabb& /*aabb*/) override { return {}; }

  std::size_t NumUpdatedLastUpdate() const override {
    return updater_.NumUpdatedLastApply();
  }

  bool SupportsIncrementalUpdates() const override {
    return index_ ? index_->SupportsIncrementalUpdates() : false;
  }

  TrackDbTimingStats GetTimingStats() const override { return stats_; }
  void ResetTimingStats() override { stats_ = TrackDbTimingStats{}; }

  void ApplyDenseCellProbeLimit() {
    if (cfg_.backend == "uniform_grid") {
      if (auto* grid = dynamic_cast<idx::UniformGridIndex*>(index_.get())) {
        grid->SetDenseCellProbeLimit(cfg_.dense_cell_probe_limit);
      }
    }
  }

private:
  TrackDatabaseConfig cfg_;
  idx::IndexUpdateManager updater_;
  std::unique_ptr<idx::ISpatialIndex3D> index_;
  TrackDbTimingStats stats_{};
};

class HotWarmTrackDatabase final : public ITrackDatabase {
public:
  explicit HotWarmTrackDatabase(const TrackDatabaseConfig& cfg)
      : cfg_(cfg),
        hot_(cfg),
        warm_index_(std::make_unique<idx::SqliteRTreeIndexBackend>(cfg_.sqlite_db_uri)) {}
  ~HotWarmTrackDatabase() override {
    if (in_transaction_) {
      if (auto* sqlite = dynamic_cast<idx::SqliteRTreeIndexBackend*>(warm_index_.get())) {
        try {
          sqlite->Commit();
        } catch (...) {
          try { sqlite->Rollback(); } catch (...) {}
        }
      }
    }
  }

  void Reserve(std::size_t n_tracks) override {
    n_tracks_ = n_tracks;
    hot_mask_.assign(n_tracks_, 0);
    hot_.Reserve(n_tracks);
    warm_index_->Reserve(n_tracks);
    warm_updater_.Reset(n_tracks);
  }

  void Configure(double d_th_m, double t_max_s) override {
    hot_.Configure(d_th_m, t_max_s);
    warm_updater_.Configure(d_th_m, t_max_s);
  }

  void UpdateTracks(double t_now_s,
                    const double* xs,
                    const double* ys,
                    const double* zs,
                    std::size_t n_tracks) override {
    (void)n_tracks;
    if (!hot_ids_.empty()) {
      hot_.UpdateTracksSubset(t_now_s, xs, ys, zs, hot_ids_);
    } else {
      hot_.UpdateTracks(t_now_s, xs, ys, zs, n_tracks);
    }
  }

  void FinalizeScan(double t_now_s,
                    const double* xs,
                    const double* ys,
                    const double* zs,
                    std::size_t n_tracks,
                    const trk::IdList* ids) override {
    (void)n_tracks;
    // TODO: warm-store prefetch/merge goes here.
    if (auto* sqlite = dynamic_cast<idx::SqliteRTreeIndexBackend*>(warm_index_.get())) {
      const bool need_begin = !in_transaction_;
      std::chrono::high_resolution_clock::time_point t0;
      std::chrono::high_resolution_clock::time_point t1;
      if (need_begin) {
        t0 = std::chrono::high_resolution_clock::now();
        sqlite->Begin();
        t1 = std::chrono::high_resolution_clock::now();
      }
      try {
        const auto t_apply0 = std::chrono::high_resolution_clock::now();
        if (ids && !ids->empty()) {
          warm_updater_.ApplySubsetThreshold(t_now_s, xs, ys, zs, ids->data(), ids->size(), *warm_index_);
        } else {
          warm_updater_.Apply(t_now_s, xs, ys, zs, *warm_index_);
        }
        const auto t_apply1 = std::chrono::high_resolution_clock::now();
        stats_.last_finalize_begin_s = need_begin ? std::chrono::duration<double>(t1 - t0).count() : 0.0;
        stats_.last_finalize_apply_s = std::chrono::duration<double>(t_apply1 - t_apply0).count();
        stats_.last_finalize_commit_s = 0.0;

        if (need_begin) {
          stats_.finalize_begin_acc_s += stats_.last_finalize_begin_s;
          ++stats_.finalize_begin_calls;
        }
        stats_.finalize_apply_acc_s += stats_.last_finalize_apply_s;
        ++stats_.finalize_calls;

        if (cfg_.warm_commit_every_scans > 1) {
          in_transaction_ = true;
          ++pending_scans_;
        } else {
          pending_scans_ = 1;
        }

        if (cfg_.warm_commit_every_scans <= 1 ||
            pending_scans_ >= cfg_.warm_commit_every_scans) {
          const auto t_commit0 = std::chrono::high_resolution_clock::now();
          sqlite->Commit();
          const auto t_commit1 = std::chrono::high_resolution_clock::now();
          stats_.last_finalize_commit_s = std::chrono::duration<double>(t_commit1 - t_commit0).count();
          stats_.finalize_commit_acc_s += stats_.last_finalize_commit_s;
          ++stats_.finalize_commit_calls;
          pending_scans_ = 0;
          in_transaction_ = false;
        }
      } catch (...) {
        sqlite->Rollback();
        in_transaction_ = false;
        pending_scans_ = 0;
        throw;
      }
    } else {
      const auto t1 = std::chrono::high_resolution_clock::now();
      if (ids && !ids->empty()) {
        warm_updater_.ApplySubsetThreshold(t_now_s, xs, ys, zs, ids->data(), ids->size(), *warm_index_);
      } else {
        warm_updater_.Apply(t_now_s, xs, ys, zs, *warm_index_);
      }
      const auto t2 = std::chrono::high_resolution_clock::now();
      stats_.last_finalize_begin_s = 0.0;
      stats_.last_finalize_apply_s = std::chrono::duration<double>(t2 - t1).count();
      stats_.last_finalize_commit_s = 0.0;
      stats_.finalize_apply_acc_s += stats_.last_finalize_apply_s;
      ++stats_.finalize_calls;
    }
  }

  trk::IdList QueryAabb(const idx::EcefAabb& aabb) const override {
    if (hot_ids_.empty()) {
      return warm_index_->QueryAabb(aabb);
    }

    auto ids = hot_.QueryAabb(aabb);
    if (ids.empty()) return ids;

    std::size_t w = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const std::size_t id = static_cast<std::size_t>(ids[i]);
      if (id < hot_mask_.size() && hot_mask_[id]) {
        ids[w++] = ids[i];
      }
    }
    ids.resize(w);
    return ids;
  }

  trk::IdList PrefetchHot(const idx::EcefAabb& aabb) override {
    // Clear previous mask.
    for (std::uint64_t id : hot_ids_) {
      if (id < hot_mask_.size()) hot_mask_[static_cast<std::size_t>(id)] = 0;
    }

    const auto t0 = std::chrono::high_resolution_clock::now();
    hot_ids_ = warm_index_->QueryAabb(aabb);
    const auto t1 = std::chrono::high_resolution_clock::now();
    stats_.last_prefetch_s = std::chrono::duration<double>(t1 - t0).count();
    stats_.prefetch_acc_s += stats_.last_prefetch_s;
    ++stats_.prefetch_calls;
    for (std::uint64_t id : hot_ids_) {
      if (id < hot_mask_.size()) hot_mask_[static_cast<std::size_t>(id)] = 1;
    }

    return hot_ids_;
  }

  std::size_t NumUpdatedLastUpdate() const override {
    return hot_.NumUpdatedLastUpdate();
  }

  bool SupportsIncrementalUpdates() const override {
    return hot_.SupportsIncrementalUpdates();
  }

  TrackDbTimingStats GetTimingStats() const override { return stats_; }
  void ResetTimingStats() override { stats_ = TrackDbTimingStats{}; }

private:
  TrackDatabaseConfig cfg_;
  HotOnlyTrackDatabase hot_;
  std::unique_ptr<idx::ISpatialIndex3D> warm_index_;
  idx::IndexUpdateManager warm_updater_;
  trk::IdList hot_ids_;
  std::vector<std::uint8_t> hot_mask_;
  std::size_t n_tracks_ = 0;
  TrackDbTimingStats stats_{};
  bool in_transaction_ = false;
  std::size_t pending_scans_ = 0;
};

} // namespace

std::unique_ptr<ITrackDatabase> CreateTrackDatabase(const TrackDatabaseConfig& cfg) {
  const std::string mode = to_upper(cfg.mode);
  if (mode == "HOT_PLUS_WARM") {
    return std::make_unique<HotWarmTrackDatabase>(cfg);
  }
  return std::make_unique<HotOnlyTrackDatabase>(cfg);
}

} // namespace db
