// include/motion/scenario_loop.hpp
// Scenario loop skeleton with:
// - dt tick (propagation step)
// - scan tick callback at scan_rate_hz
//
// This is the "glue" between motion propagation and your index / scan volume / association pipeline.
//
// Intended usage:
// - Maintain TrackBatchSoA + ModelBuckets.
// - Each dt tick:
//     * optional model selection update
//     * propagate by bucket: CA bucket, CT bucket, etc.
// - Each scan tick:
//     * build/update your spatial index from SoA position columns (or x/y/z)
//     * query candidates for the current scan volume (AABB), etc.

#pragma once

#include <cassert>

#include "motion/track_batch_soa.hpp"
#include "motion/model_buckets.hpp"
#include "motion/motion_models.hpp"

namespace motion {

struct ScenarioConfig {
    double T_run_s = 10.0;
    double dt_s = 0.02;
    double scan_rate_hz = 1.0;
};

template <class ScanTickFn>
inline void run_scenario_loop(TrackBatchSoA& tb,
                              ModelBuckets& buckets,
                              const IMotionModel& model_ca,
                              const IMotionModel& model_ct,
                              const ModelSelector& selector,
                              const ScenarioConfig& cfg,
                              ScanTickFn&& on_scan_tick)
{
    assert(cfg.dt_s > 0.0);
    assert(cfg.scan_rate_hz > 0.0);

    MotionContext ctx{};
    double next_scan_time = 0.0;

    for (double t = 0.0; t < cfg.T_run_s; t += cfg.dt_s) {
      ctx.t_s = t;

      // 0) scan tick(s) due at time t (state is "at t" here)
      while ((next_scan_time <= cfg.T_run_s + 1e-12) && (t + 1e-12 >= next_scan_time)) {
          tb.sync_pos_from_state();          // ensure query columns match the state at time t
          on_scan_tick(tb, next_scan_time);  // pass the scan time
          next_scan_time += 1.0 / cfg.scan_rate_hz;
      }

      // 1) optional model update
      selector.update_models(tb, buckets);

      // 2) propagate by bucket from t -> t+dt
      if (!buckets.bucket_ca.empty()) model_ca.propagate(tb, buckets.bucket_ca, cfg.dt_s, ctx);
      if (!buckets.bucket_ct.empty()) model_ct.propagate(tb, buckets.bucket_ct, cfg.dt_s, ctx);

      // 3) (optional) keep query columns synced after propagation too
      // tb.sync_pos_from_state();
   }

}

} // namespace motion
