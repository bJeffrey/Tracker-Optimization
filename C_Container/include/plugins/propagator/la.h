/**
 * @file la.h
 * @brief Backend-agnostic linear algebra interface (C++98-compatible).
 *
 * This header defines a minimal abstraction for matrix operations and batch
 * covariance propagation used in tracking applications. Implementations exist
 * for multiple backends (Eigen, Intel MKL, and a C++98 baseline) while keeping
 * a single call surface for client code.
 *
 * Conventions:
 *  - Row-major storage.
 *  - `MatrixView::stride` and `BatchMat::ld` are leading dimensions in elements.
 *  - Batch operations treat each covariance P_i independently (no cross-covariances).
 *
 * Typical use:
 *  - Wrap raw buffers in MatrixView/BatchMat.
 *  - Call `cov_predict_*` per cycle to propagate P.
 *  - (Optionally) call `symmetrize_batch` to mitigate numeric asymmetry.
 */

#pragma once
#include <cstddef>

namespace la {

/**
 * @struct MatrixView
 * @brief Lightweight row-major view over a 2D double matrix.
 *
 * No ownership/lifetime management: caller retains responsibility for the
 * pointed-to memory. `stride` is the leading dimension in elements (for
 * row-major it is typically the number of columns).
 */
struct MatrixView {
    double* ptr;        ///< Pointer to the first element (row-major).
    std::size_t rows;   ///< Number of rows.
    std::size_t cols;   ///< Number of columns.
    std::size_t stride; ///< Leading dimension in elements (usually = cols).
};

/**
 * @struct BatchMat
 * @brief Descriptor for a batch of square matrices laid out sequentially.
 *
 * Represents `count` independent n×n matrices. Each matrix starts at
 * `base + b*stride` (elements), uses leading dimension `ld`, and has size n×n.
 * Common layout is tight packing with `ld = n` and `stride = n*n`.
 */
struct BatchMat {
    double*      base;    ///< Pointer to the first matrix in the batch.
    std::size_t  n;       ///< Matrix order (rows == cols).
    std::size_t  ld;      ///< Leading dimension (elements), usually n.
    std::size_t  stride;  ///< Element distance between consecutive matrices.
    std::size_t  count;   ///< Number of matrices in the batch.
};

/**
 * @brief General matrix multiply (row-major):
 *        C ← alpha * op(A) * op(B) + beta * C
 *
 * @param transA If true, use A^T; otherwise use A.
 * @param transB If true, use B^T; otherwise use B.
 * @param alpha  Scalar multiplier for the product term.
 * @param A      Left operand matrix view.
 * @param B      Right operand matrix view.
 * @param beta   Scalar multiplier for the existing C.
 * @param C      Output matrix view (updated in place).
 *
 * @note Implementations may assume dimension consistency; callers should ensure
 *       that op(A).cols == op(B).rows and C has shape (op(A).rows × op(B).cols).
 */
void gemm(bool transA, bool transB,
          double alpha, const MatrixView& A, const MatrixView& B,
          double beta,        MatrixView& C);

/**
 * @brief Add a diagonal bump to each matrix in the batch:
 *        ∀i: P_i ← P_i + q * I
 *
 * @param Pbatch Batch descriptor for covariances.
 * @param q      Diagonal increment applied to every P_i.
 */
void add_diag_noise_batch(const BatchMat& Pbatch, double q);

/**
 * @brief Batch covariance propagation with fixed Q (absolute, not per-second):
 *        ∀i: P_i ← F * P_i * F^T + Q
 *
 * @param F      Shared state transition matrix.
 * @param Q      Shared process noise covariance (absolute).
 * @param Pbatch Batch of covariances (updated in place).
 */
void cov_predict_batch(const MatrixView& F,
                       const MatrixView& Q,
                       const BatchMat&   Pbatch);

/**
 * @brief Δt-scaled random-walk propagation with diagonal Q:
 *        ∀i: P_i ← P_i + (q_per_sec * dt) * I
 *
 * @param Pbatch    Batch of covariances (updated in place).
 * @param q_per_sec Diagonal variance growth rate [units^2 / s].
 * @param dt        Time step [s].
 */
void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt);

/**
 * @brief Δt-scaled random-walk propagation with full Q:
 *        ∀i: P_i ← P_i + (Q_per_sec * dt)
 *
 * @param Q_per_sec Full per-second process noise covariance.
 * @param dt        Time step [s].
 * @param Pbatch    Batch of covariances (updated in place).
 */
void cov_predict_rw_fullQ_dt(const MatrixView& Q_per_sec,
                             double dt,
                             const BatchMat& Pbatch);

/**
 * @brief General Δt-scaled batch propagation:
 *        ∀i: P_i ← F * P_i * F^T + (Q_per_sec * dt)
 *
 * @param F         Shared state transition matrix.
 * @param Q_per_sec Per-second process noise covariance.
 * @param dt        Time step [s].
 * @param Pbatch    Batch of covariances (updated in place).
 */
void cov_predict_batch_dt(const MatrixView& F,
                          const MatrixView& Q_per_sec,
                          double dt,
                          const BatchMat& Pbatch);

/**
 * @brief Symmetrize each covariance numerically:
 *        ∀i: P_i ← 0.5 * (P_i + P_i^T)
 *
 * @param Pbatch Batch of covariances (updated in place).
 * @note Useful after floating-point operations to reduce asymmetry.
 *       Does not enforce positive semi-definiteness by itself.
 */
void symmetrize_batch(const BatchMat& Pbatch);

} // namespace la
