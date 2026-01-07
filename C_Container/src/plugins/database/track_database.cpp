#include "plugins/database/track_database.h"

#include "plugins/database/index_update_manager.h"
#include "plugins/database/spatial_index_factory.h"
#include "plugins/database/sqlite_rtree_backend.h"
#include "plugins/database/uniform_grid_index.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <mutex>
#include <thread>

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
  void FlushWarmUpdates() override {}

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
        warm_index_read_(std::make_unique<idx::SqliteRTreeIndexBackend>(cfg_.sqlite_db_uri)),
        warm_index_write_(std::make_unique<idx::SqliteRTreeIndexBackend>(cfg_.sqlite_db_uri)) {
    worker_ = std::thread(&HotWarmTrackDatabase::WorkerLoop, this);
  }
  ~HotWarmTrackDatabase() override {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stop_ = true;
      flush_requested_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void Reserve(std::size_t n_tracks) override {
    n_tracks_ = n_tracks;
    hot_mask_.assign(n_tracks_, 0);
    hot_.Reserve(n_tracks);
    warm_index_read_->Reserve(n_tracks);
    warm_index_write_->Reserve(n_tracks);
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
    if (!ids) {
      // Warm init: do synchronous rebuild so reader sees data immediately.
      if (auto* sqlite = dynamic_cast<idx::SqliteRTreeIndexBackend*>(warm_index_write_.get())) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        sqlite->Begin();
        warm_updater_.Apply(t_now_s, xs, ys, zs, *warm_index_write_);
        sqlite->Commit();
        const auto t1 = std::chrono::high_resolution_clock::now();
        const double dt = std::chrono::duration<double>(t1 - t0).count();
        std::lock_guard<std::mutex> lock(stats_mu_);
        stats_.last_finalize_begin_s = 0.0;
        stats_.last_finalize_apply_s = dt;
        stats_.last_finalize_commit_s = 0.0;
        stats_.finalize_apply_acc_s += dt;
        ++stats_.finalize_calls;
      }
      return;
    }

    if (!ids->empty()) {
      std::vector<std::uint64_t> upd_ids;
      std::vector<double> upd_xs;
      std::vector<double> upd_ys;
      std::vector<double> upd_zs;
      warm_updater_.CollectSubsetThreshold(t_now_s, xs, ys, zs, ids->data(), ids->size(),
                                           upd_ids, upd_xs, upd_ys, upd_zs);
      if (upd_ids.empty()) return;

      std::lock_guard<std::mutex> lock(mu_);
      pending_ids_.insert(pending_ids_.end(), upd_ids.begin(), upd_ids.end());
      pending_xs_.insert(pending_xs_.end(), upd_xs.begin(), upd_xs.end());
      pending_ys_.insert(pending_ys_.end(), upd_ys.begin(), upd_ys.end());
      pending_zs_.insert(pending_zs_.end(), upd_zs.begin(), upd_zs.end());
      ++pending_scans_;
      pending_tracks_ += upd_ids.size();

      if ((pending_scans_ >= cfg_.warm_commit_every_scans) ||
          (pending_tracks_ >= cfg_.warm_commit_after_tracks)) {
        flush_requested_ = true;
      }
    }
    cv_.notify_all();
  }

  trk::IdList QueryAabb(const idx::EcefAabb& aabb) const override {
    if (hot_ids_.empty()) {
      return warm_index_read_->QueryAabb(aabb);
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
    hot_ids_ = warm_index_read_->QueryAabb(aabb);
    const auto t1 = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> lock(stats_mu_);
      stats_.last_prefetch_s = std::chrono::duration<double>(t1 - t0).count();
      stats_.prefetch_acc_s += stats_.last_prefetch_s;
      ++stats_.prefetch_calls;
    }
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

  TrackDbTimingStats GetTimingStats() const override {
    std::lock_guard<std::mutex> lock(stats_mu_);
    return stats_;
  }
  void ResetTimingStats() override {
    std::lock_guard<std::mutex> lock(stats_mu_);
    stats_ = TrackDbTimingStats{};
  }
  void FlushWarmUpdates() override {
    std::size_t req_id = 0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      flush_requested_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(flush_mu_);
      req_id = ++flush_request_id_;
    }
    cv_.notify_all();
    std::unique_lock<std::mutex> lock(flush_mu_);
    flush_cv_.wait(lock, [&]() { return flush_completed_id_ >= req_id; });
  }

private:
  void WorkerLoop() {
    while (true) {
      std::vector<std::uint64_t> ids;
      std::vector<double> xs, ys, zs;
      {
        std::unique_lock<std::mutex> lock(mu_);
        cv_.wait(lock, [&]() {
          return stop_ || flush_requested_ || !pending_ids_.empty();
        });
        if (stop_ && pending_ids_.empty()) {
          break;
        }
        if (!pending_ids_.empty() &&
            (flush_requested_ || stop_ ||
             pending_scans_ >= cfg_.warm_commit_every_scans ||
             pending_tracks_ >= cfg_.warm_commit_after_tracks)) {
          ids.swap(pending_ids_);
          xs.swap(pending_xs_);
          ys.swap(pending_ys_);
          zs.swap(pending_zs_);
          pending_scans_ = 0;
          pending_tracks_ = 0;
          flush_requested_ = false;
        } else if (flush_requested_ && pending_ids_.empty()) {
          flush_requested_ = false;
          std::lock_guard<std::mutex> lock_flush(flush_mu_);
          flush_completed_id_ = flush_request_id_;
          flush_cv_.notify_all();
          continue;
        } else {
          continue;
        }
      }

      if (ids.empty()) {
        if (stop_) break;
        continue;
      }

      if (auto* sqlite = dynamic_cast<idx::SqliteRTreeIndexBackend*>(warm_index_write_.get())) {
        const auto t_begin0 = std::chrono::high_resolution_clock::now();
        sqlite->Begin();
        const auto t_begin1 = std::chrono::high_resolution_clock::now();

        const auto t_apply0 = std::chrono::high_resolution_clock::now();
        for (std::size_t i = 0; i < ids.size(); ++i) {
          warm_index_write_->UpdatePoint(ids[i], xs[i], ys[i], zs[i]);
        }
        const auto t_apply1 = std::chrono::high_resolution_clock::now();

        const auto t_commit0 = std::chrono::high_resolution_clock::now();
        sqlite->Commit();
        const auto t_commit1 = std::chrono::high_resolution_clock::now();

        std::lock_guard<std::mutex> lock(stats_mu_);
        stats_.last_finalize_begin_s = std::chrono::duration<double>(t_begin1 - t_begin0).count();
        stats_.last_finalize_apply_s = std::chrono::duration<double>(t_apply1 - t_apply0).count();
        stats_.last_finalize_commit_s = std::chrono::duration<double>(t_commit1 - t_commit0).count();
        stats_.finalize_begin_acc_s += stats_.last_finalize_begin_s;
        stats_.finalize_apply_acc_s += stats_.last_finalize_apply_s;
        stats_.finalize_commit_acc_s += stats_.last_finalize_commit_s;
        ++stats_.finalize_begin_calls;
        ++stats_.finalize_commit_calls;
        ++stats_.finalize_calls;
      }

      {
        std::lock_guard<std::mutex> lock_flush(flush_mu_);
        flush_completed_id_ = flush_request_id_;
      }
      flush_cv_.notify_all();
    }
  }

  TrackDatabaseConfig cfg_;
  HotOnlyTrackDatabase hot_;
  std::unique_ptr<idx::ISpatialIndex3D> warm_index_read_;
  std::unique_ptr<idx::ISpatialIndex3D> warm_index_write_;
  idx::IndexUpdateManager warm_updater_;
  trk::IdList hot_ids_;
  std::vector<std::uint8_t> hot_mask_;
  std::size_t n_tracks_ = 0;
  TrackDbTimingStats stats_{};
  std::mutex mu_;
  std::condition_variable cv_;
  bool stop_ = false;
  bool flush_requested_ = false;
  std::size_t pending_scans_ = 0;
  std::size_t pending_tracks_ = 0;
  std::vector<std::uint64_t> pending_ids_;
  std::vector<double> pending_xs_;
  std::vector<double> pending_ys_;
  std::vector<double> pending_zs_;
  std::thread worker_;
  mutable std::mutex stats_mu_;
  std::mutex flush_mu_;
  std::condition_variable flush_cv_;
  std::size_t flush_request_id_ = 0;
  std::size_t flush_completed_id_ = 0;
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
