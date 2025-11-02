/**
 * @file main.cpp
 * @brief Demo driver for Δt-scaled batch covariance propagation.
 *
 * @details
 *   This program initializes a batch of track covariances P_i (row-major),
 *   constructs a stationary process model (F = I), and propagates the batch
 *   forward one step using a per-second process noise Q_per_sec scaled by dt:
 *
 *       P_i ← P_i + (Q_per_sec * dt)
 *
 *   It times only the propagation (not data generation), then prints a simple
 *   checksum so different backends (STD/Eigen/MKL) can be sanity-checked.
 *
 *   Notes:
 *    - Row-major layout is assumed (ld = n, stride = n*n).
 *    - The “fast stationary” path avoids F P Fᵀ when F = I.
 *    - All code is C++98-compatible.
 */

#include "la.h"
#include <vector>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>  // POSIX wall-clock timer (gettimeofday)

/**
 * @brief Get wall-clock time in seconds (POSIX).
 * @return Current time as a double (seconds).
 *
 * @note Uses gettimeofday for portability to C++98-era toolchains.
 *       Measures wall time (not CPU time), suitable for multithreaded code.
 */
static double now_sec() {
    struct timeval tv; gettimeofday(&tv, 0);
    return double(tv.tv_sec) + double(tv.tv_usec) * 1e-6;
}

/**
 * @brief Uniform random deviate in [-1, +1].
 * @return Pseudorandom double in [-1, 1].
 *
 * @note Uses std::rand() to remain C++98-compatible; good enough for demo noise.
 */
static double urand() {
    return 2.0 * (double)std::rand() / (double)RAND_MAX - 1.0;
}

/**
 * @brief Entry point: build demo data and time a single batch propagation step.
 *
 * Steps:
 *  1) Build F = I (kept general via MatrixView).
 *  2) Build diagonal Q_per_sec (variance/sec for each state).
 *  3) Initialize P_i with PD-ish values across many tracks.
 *  4) Time only the Δt-scaled random-walk update:
 *         P_i ← P_i + (Q_per_sec * dt)
 *  5) Optionally symmetrize P for numeric hygiene.
 *  6) Print timing and a checksum to validate equivalence across backends.
 */
int main() {
    std::size_t n = 6;           // state dimension per track
    std::size_t tracks = 1000;   // number of tracks
    double dt = 0.1;             // seconds between propagations

    // (1) Identity F (stationary ESM random-walk), kept as a general F for API parity
    std::vector<double> Fbuf(n * n, 0.0);
    for (std::size_t i = 0; i < n; ++i) Fbuf[i * n + i] = 1.0;
    la::MatrixView F; F.ptr = &Fbuf[0]; F.rows = n; F.cols = n; F.stride = n;

    // (2) Q_per_sec: diagonal per-second noise (e.g., sigma^2 per second per state)
    std::vector<double> Qps_buf(n * n, 0.0);
    for (std::size_t i = 0; i < n; ++i) Qps_buf[i * n + i] = 0.05; // variance/sec on each state
    la::MatrixView Qps; Qps.ptr = &Qps_buf[0]; Qps.rows = n; Qps.cols = n; Qps.stride = n;

    // (3) P batch storage (row-major, contiguous across tracks)
    std::vector<double> Pblock(tracks * n * n, 0.0);
    for (std::size_t t = 0; t < tracks; ++t) {
        double* Pi = &Pblock[t * n * n];
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t c = 0; c < n; ++c)
                Pi[r * n + c] = (r == c) ? 1.0 : 0.05 * urand(); // PD-ish start
    }
    la::BatchMat Pbatch;
    Pbatch.base   = &Pblock[0];
    Pbatch.n      = n;
    Pbatch.ld     = n;
    Pbatch.stride = n * n;
    Pbatch.count  = tracks;

    // Optional: add a tiny diagonal bump once (e.g., model fudge / numeric cushion)
    la::add_diag_noise_batch(Pbatch, 1e-6);

    // ---- (4) Timed propagation (Δt-scaled random walk) ----
    double t0 = now_sec();

    // Fast stationary ESM propagation (no F P Fᵀ when F = I):  P_i += Q_per_sec * dt
    la::cov_predict_rw_fullQ_dt(Qps, dt, Pbatch);

    // If you wanted the full general form instead (handles non-identity F):
    // la::cov_predict_batch_dt(F, Qps, dt, Pbatch);

    // (5) Optional: symmetrize for numeric hygiene (P ≈ Pᵀ)
    la::symmetrize_batch(Pbatch);

    double t1 = now_sec();

    // (6) Simple checksum (useful for backend/regression checks)
    double sum = 0.0;
    for (std::size_t i = 0; i < Pblock.size(); ++i) sum += Pblock[i];

    std::cout << "Batch RW propagation time (dt=" << dt << "): "
              << (t1 - t0) << " s, checksum " << sum << std::endl;

    return 0;
}
