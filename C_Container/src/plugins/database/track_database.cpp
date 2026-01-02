#include "plugins/database/track_database.h"

#include "plugins/database/index_update_manager.h"
#include "plugins/database/spatial_index_factory.h"
#include "plugins/database/uniform_grid_index.h"

namespace db {
namespace {

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

} // namespace

std::unique_ptr<ITrackDatabase> CreateTrackDatabase(const TrackDatabaseConfig& cfg) {
  return std::make_unique<HotOnlyTrackDatabase>(cfg);
}

} // namespace db
