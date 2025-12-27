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
//     * build/update your spatial index from SoA x/y/z (slab) and query candidates (AABB), etc.

#pragma once

#include <cassert>
#include <cmath>

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
    assert(cfg.T_run_s >= 0.0);

    constexpr double eps = 1e-12;

    MotionContext ctx{};
    const double scan_dt = 1.0 / cfg.scan_rate_hz;
    double next_scan_time = 0.0;

    // Helper: emit all scans due up to time t_now (inclusive w/ epsilon)
    auto emit_scans_up_to = [&](double t_now) {
        while (next_scan_time <= cfg.T_run_s + eps && next_scan_time <= t_now + eps) {
            // State is assumed to already represent "current time t_now"
            // (in this loop, scans are emitted before propagation, so state is at time t)
            on_scan_tick(tb, next_scan_time);
            next_scan_time += scan_dt;
        }
    };

    // Main loop: state is at time t at loop top.
    for (double t = 0.0; t < cfg.T_run_s - eps; t += cfg.dt_s) {
        ctx.t_s = t;

        // 0) scan tick(s) due at time t (state is "at t" here)
        emit_scans_up_to(t);

        // 1) optional model update (still at time t)
        selector.update_models(tb, buckets);

        // 2) propagate by bucket from t -> t+dt
        if (!buckets.bucket_ca.empty()) model_ca.propagate(tb, buckets.bucket_ca, cfg.dt_s, ctx);
        if (!buckets.bucket_ct.empty()) model_ct.propagate(tb, buckets.bucket_ct, cfg.dt_s, ctx);

        // (No sync_pos_from_state needed in slab SoA; index reads x/y/z directly)
    }

    // Final flush: ensure we emit scans that occur after the last dt tick but <= T_run_s.
    // State here is effectively at (approximately) T_run_s, but even if slightly under,
    // the scan time passed into callback is authoritative.
    emit_scans_up_to(cfg.T_run_s);
}

} // namespace motion
