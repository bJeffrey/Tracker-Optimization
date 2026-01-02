#include "plugins/database/track_database.h"

#include "plugins/database/index_update_manager.h"
#include "plugins/database/spatial_index_factory.h"
#include "plugins/database/sqlite_rtree_backend.h"
#include "plugins/database/uniform_grid_index.h"

#include <algorithm>
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

  void FinalizeScan(double /*t_now_s*/,
                    const double* /*xs*/,
                    const double* /*ys*/,
                    const double* /*zs*/,
                    std::size_t /*n_tracks*/) override {}

  trk::IdList QueryAabb(const idx::EcefAabb& aabb) const override {
    if (!index_) return {};
    return index_->QueryAabb(aabb);
  }

  std::size_t NumUpdatedLastUpdate() const override {
    return updater_.NumUpdatedLastApply();
  }

  bool SupportsIncrementalUpdates() const override {
    return index_ ? index_->SupportsIncrementalUpdates() : false;
  }

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
};

class HotWarmTrackDatabase final : public ITrackDatabase {
public:
  explicit HotWarmTrackDatabase(const TrackDatabaseConfig& cfg)
      : cfg_(cfg),
        hot_(cfg),
        warm_index_(std::make_unique<idx::SqliteRTreeIndexBackend>(cfg_.sqlite_db_uri)) {}

  void Reserve(std::size_t n_tracks) override {
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
    hot_.UpdateTracks(t_now_s, xs, ys, zs, n_tracks);
  }

  void FinalizeScan(double t_now_s,
                    const double* xs,
                    const double* ys,
                    const double* zs,
                    std::size_t n_tracks) override {
    // TODO: warm-store prefetch/merge goes here.
    if (auto* sqlite = dynamic_cast<idx::SqliteRTreeIndexBackend*>(warm_index_.get())) {
      sqlite->Begin();
      try {
        warm_updater_.Apply(t_now_s, xs, ys, zs, *warm_index_);
        sqlite->Commit();
      } catch (...) {
        sqlite->Rollback();
        throw;
      }
    } else {
      warm_updater_.Apply(t_now_s, xs, ys, zs, *warm_index_);
    }
  }

  trk::IdList QueryAabb(const idx::EcefAabb& aabb) const override {
    return warm_index_->QueryAabb(aabb);
  }

  std::size_t NumUpdatedLastUpdate() const override {
    return hot_.NumUpdatedLastUpdate();
  }

  bool SupportsIncrementalUpdates() const override {
    return hot_.SupportsIncrementalUpdates();
  }

private:
  TrackDatabaseConfig cfg_;
  HotOnlyTrackDatabase hot_;
  std::unique_ptr<idx::ISpatialIndex3D> warm_index_;
  idx::IndexUpdateManager warm_updater_;
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
