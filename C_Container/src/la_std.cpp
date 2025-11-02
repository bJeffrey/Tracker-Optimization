/**
 * @file la_std.cpp
 * @brief Baseline C++98 backend (no external libs) for linear algebra.
 *
 * @details
 *   Reference implementation of the la.h API using straightforward
 *   nested-loop math. This is portable, dependency-free, and useful as a
 *   correctness baseline (and for fully offline/minimal systems).
 *
 *   Conventions:
 *     - Row-major storage.
 *     - MatrixView::stride and BatchMat::ld are leading dimensions (elements).
 *     - BatchMat::stride is the element distance between consecutive n×n blocks.
 *     - Batch ops treat each covariance independently (no cross-covariances).
 */

#include "la.h"
#include <cassert>
#include <cstring> // memset

namespace la {

/**
 * @brief Row-major element accessor with arbitrary leading dimension.
 *
 * @param base Pointer to matrix base.
 * @param ld   Leading dimension (elements).
 * @param r    Row index.
 * @param c    Column index.
 * @return Element at (r,c).
 */
static inline double get_el(const double* base, std::size_t ld,
                            std::size_t r, std::size_t c)
{
    return base[r*ld + c];
}

/**
 * @brief Row-major element setter with arbitrary leading dimension.
 */
static inline void set_el(double* base, std::size_t ld,
                          std::size_t r, std::size_t c, double v)
{
    base[r*ld + c] = v;
}

/**
 * @brief General matrix multiply (row-major):
 *        C ← alpha * op(A) * op(B) + beta * C
 *
 * @param tA    If true, use A^T; else use A.
 * @param tB    If true, use B^T; else use B.
 * @param alpha Scalar for product term.
 * @param A,B   Input matrices (views).
 * @param beta  Scalar for existing C.
 * @param C     Output matrix (updated in place).
 *
 * @note Triple-loop reference GEMM; adequate for small/medium problems or baseline.
 */
void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    const std::size_t M = tA ? A.cols : A.rows;
    const std::size_t N = tB ? B.rows : B.cols;
    const std::size_t K = tA ? A.rows : A.cols;

    assert(C.rows == M && C.cols == N);

    // Simple i-j-k ordering (cache-friendly enough for tiny n).
    for (std::size_t i = 0; i < M; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            double acc = 0.0;
            for (std::size_t k = 0; k < K; ++k) {
                // a_ik and b_kj with optional transposes
                const double a = tA ? get_el(A.ptr, A.stride, k, i)
                                    : get_el(A.ptr, A.stride, i, k);
                const double b = tB ? get_el(B.ptr, B.stride, j, k)
                                    : get_el(B.ptr, B.stride, k, j);
                acc += a * b;
            }
            const double cij = get_el(C.ptr, C.stride, i, j);
            set_el(C.ptr, C.stride, i, j, alpha*acc + beta*cij);
        }
    }
}

/**
 * @brief Add diagonal bump to every covariance in the batch:
 *        ∀b: P_b ← P_b + q * I
 */
void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    for (std::size_t b = 0; b < Pbatch.count; ++b) {
        double* P = Pbatch.base + b * Pbatch.stride;
        for (std::size_t i = 0; i < Pbatch.n; ++i) {
            P[i * Pbatch.ld + i] += q;
        }
    }
}

/**
 * @brief Batch covariance prediction with absolute Q (not Δt-scaled):
 *        ∀b: P_b ← F * P_b * Fᵀ + Q
 *
 * @param Fv     Shared state transition (n×n).
 * @param Qv     Shared process noise (n×n, absolute).
 * @param Pbatch Batch of covariances (updated in place).
 *
 * @note Uses a single n×n scratch (FP) reused per matrix for simplicity.
 */
void cov_predict_batch(const MatrixView& Fv,
                       const MatrixView& Qv,
                       const BatchMat&   Pbatch)
{
    assert(Fv.rows == Fv.cols);
    assert(Qv.rows == Qv.cols);
    assert(Fv.rows == Qv.rows);
    assert(Pbatch.n  == Fv.rows);

    const std::size_t n = Pbatch.n;

    // Workspace for FP per track (tight n×n); reused inside loop.
    double* FP = new double[n * n];

    MatrixView F = Fv;
    MatrixView Q = Qv;

    for (std::size_t b = 0; b < Pbatch.count; ++b) {
        double* Pptr = Pbatch.base + b * Pbatch.stride;

        // FP = F * P
        {
            MatrixView A; A.ptr = F.ptr;    A.rows = n; A.cols = n; A.stride = F.stride;
            MatrixView B; B.ptr = Pptr;     B.rows = n; B.cols = n; B.stride = Pbatch.ld;
            MatrixView C; C.ptr = FP;       C.rows = n; C.cols = n; C.stride = n;
            gemm(false, false, 1.0, A, B, 0.0, C);
        }

        // P = FP * F^T
        {
            MatrixView A; A.ptr = FP;       A.rows = n; A.cols = n; A.stride = n;
            MatrixView B; B.ptr = F.ptr;    B.rows = n; B.cols = n; B.stride = F.stride;
            MatrixView C; C.ptr = Pptr;     C.rows = n; C.cols = n; C.stride = Pbatch.ld;
            gemm(false, true, 1.0, A, B, 0.0, C);
        }

        // P += Q
        for (std::size_t r = 0; r < n; ++r) {
            double* prow = Pptr + r * Pbatch.ld;
            const double* qrow = Q.ptr + r * Q.stride;
            for (std::size_t c = 0; c < n; ++c) {
                prow[c] += qrow[c];
            }
        }
    }

    delete [] FP;
}

/**
 * @brief Δt-scaled random-walk with diagonal noise:
 *        ∀b: P_b ← P_b + (q_per_sec * dt) * I
 */
void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt)
{
    add_diag_noise_batch(Pbatch, q_per_sec*dt);
}

/**
 * @brief Δt-scaled random-walk with full Q:
 *        ∀b: P_b ← P_b + (Q_per_sec * dt)
 */
void cov_predict_rw_fullQ_dt(const MatrixView& Qps,
                             double dt,
                             const BatchMat& Pbatch)
{
    assert(Qps.rows==Qps.cols && Qps.rows==Pbatch.n);
    const std::size_t n = Pbatch.n;
    for (std::size_t b=0; b<Pbatch.count; ++b) {
        double* P = Pbatch.base + b * Pbatch.stride;
        for (std::size_t r=0; r<n; ++r) {
            double* prow = P + r*Pbatch.ld;
            const double* qrow = Qps.ptr + r*Qps.stride;
            for (std::size_t c=0; c<n; ++c)
                prow[c] += qrow[c] * dt;
        }
    }
}

/**
 * @brief General Δt-scaled batch propagation:
 *        ∀b: P_b ← F * P_b * Fᵀ + (Q_per_sec * dt)
 */
void cov_predict_batch_dt(const MatrixView& Fv,
                          const MatrixView& Qps,
                          double dt,
                          const BatchMat& Pbatch)
{
    const std::size_t n = Pbatch.n;
    double* FP = new double[n*n];

    for (std::size_t b=0; b<Pbatch.count; ++b) {
        double* Pptr = Pbatch.base + b*Pbatch.stride;

        // FP = F * P
        MatrixView A=Fv, B, C;
        B.ptr=Pptr; B.rows=n; B.cols=n; B.stride=Pbatch.ld;
        C.ptr=FP;   C.rows=n; C.cols=n; C.stride=n;
        gemm(false,false,1.0, A,B, 0.0, C);

        // P = FP * F^T
        MatrixView A2; A2.ptr=FP;   A2.rows=n; A2.cols=n; A2.stride=n;
        MatrixView B2=Fv;
        MatrixView C2; C2.ptr=Pptr; C2.rows=n; C2.cols=n; C2.stride=Pbatch.ld;
        gemm(false,true,1.0, A2,B2, 0.0, C2);

        // P += Q_per_sec * dt
        for (std::size_t r=0; r<n; ++r) {
            double* prow = Pptr + r*Pbatch.ld;
            const double* qrow = Qps.ptr + r*Qps.stride;
            for (std::size_t c=0; c<n; ++c) prow[c] += qrow[c] * dt;
        }
    }
    delete [] FP;
}

/**
 * @brief Numerically enforce symmetry:
 *        ∀b: P_b ← 0.5 * (P_b + P_bᵀ)
 *
 * @note Mitigates floating-point asymmetry; does not guarantee PSD.
 */
void symmetrize_batch(const BatchMat& Pbatch)
{
    const std::size_t n = Pbatch.n;
    for (std::size_t b=0; b<Pbatch.count; ++b) {
        double* P = Pbatch.base + b*Pbatch.stride;
        for (std::size_t r=0; r<n; ++r) {
            for (std::size_t c=r+1; c<n; ++c) {
                const double a  = P[r*Pbatch.ld + c];
                const double bt = P[c*Pbatch.ld + r];
                const double m  = 0.5*(a + bt);
                P[r*Pbatch.ld + c] = m;
                P[c*Pbatch.ld + r] = m;
            }
        }
    }
}

} // namespace la
