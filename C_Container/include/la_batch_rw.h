#pragma once
/**
 * @file la_batch_rw.h
 * @brief Backend-selected batched random-walk covariance propagation.
 *
 * This is the "fast path" used in the Matrix Optimization demo:
 *
 *    P_i <- P_i + (Q_per_sec * dt)
 *
 * Layout:
 *  - P_batch: row-major, contiguous batch of (n x n) matrices
 *    stride = n*n doubles per track
 *  - Q_per_sec: row-major (n x n)
 *
 * Backend selection remains compile-time (Eigen / MKL / STD) via CMake options.
 */

#include <cstddef>

namespace la {

/**
 * @brief Add scaled process noise to a batch of covariances: P += Q_per_sec * dt
 * @param P_batch    [in/out] batch of covariances, row-major, stride=n*n
 * @param Q_per_sec  [in]     process noise per second, row-major (n x n)
 * @param dt_s       [in]     time step in seconds
 * @param n          [in]     matrix dimension (e.g., 9 for CA9 full covariance)
 * @param batch      [in]     number of track covariances in the batch
 */
void rw_add_qdt_batch(double* P_batch,
                      const double* Q_per_sec,
                      double dt_s,
                      int n,
                      std::size_t batch);

} // namespace la
