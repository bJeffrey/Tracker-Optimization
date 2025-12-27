/**
 * @file la_mkl.cpp
 * @brief Intel MKL backend for linear-algebra routines (row-major, C++98).
 *
 * @details
 *   Implements the la.h API using Intel MKL BLAS for high performance on Intel
 *   CPUs. Functions cover single GEMM, batch covariance propagation, Δt-scaled
 *   random-walk updates (diagonal/full Q), and numeric symmetrization.
 *
 *   Conventions:
 *     - Row-major storage throughout.
 *     - MatrixView::stride and BatchMat::ld are leading dimensions (elements).
 *     - BatchMat::stride is the element distance between consecutive n×n blocks.
 *     - Batch ops process covariances independently (no cross-covariances).
 *
 *   Notes:
 *     - Uses cblas_dgemm_batch_strided where available (one F reused across batch).
 *     - Falls back to cblas_dgemm_batch in cov_predict_batch_dt for portability.
 *     - OpenMP is applied to outer batch loops where appropriate. The large
 *       MKL batched GEMM calls remain single calls for efficiency; control
 *       MKL/Eigen threading at runtime (e.g., MKL_NUM_THREADS=1) to avoid oversubscription.
 */

#include "la.h"
#include <mkl.h>
#include <cassert>

namespace la {

/**
 * @brief Matrix multiply (row-major): C ← alpha * op(A) * op(B) + beta * C
 *
 * @param tA   If true, use A^T; else use A.
 * @param tB   If true, use B^T; else use B.
 * @param alpha Scalar multiplier for product term.
 * @param A,B   Input matrices (row-major views).
 * @param beta  Scalar multiplier for existing C.
 * @param C     Output matrix (row-major view), updated in place.
 */
void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    const CBLAS_TRANSPOSE TA = tA ? CblasTrans : CblasNoTrans;
    const CBLAS_TRANSPOSE TB = tB ? CblasTrans : CblasNoTrans;

    const MKL_INT m = (MKL_INT)(tA ? A.cols : A.rows);
    const MKL_INT n = (MKL_INT)(tB ? B.rows : B.cols);
    const MKL_INT k = (MKL_INT)(tA ? A.rows : A.cols);

    cblas_dgemm(CblasRowMajor, TA, TB, m, n, k,
                alpha, A.ptr, (MKL_INT)A.stride,
                        B.ptr, (MKL_INT)B.stride,
                beta,  C.ptr, (MKL_INT)C.stride);
}

/**
 * @brief Add a diagonal bump to every covariance in the batch:
 *        ∀b: P_b ← P_b + q * I
 *
 * @param Pbatch Batch descriptor (n×n, ld, stride, count).
 * @param q      Diagonal increment applied to each P_b.
 */
void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    #pragma omp parallel for schedule(static)
    for (long long b = 0; b < (long long)Pbatch.count; ++b) {
        double* P = Pbatch.base + (std::size_t)b * Pbatch.stride;
        for (std::size_t i = 0; i < Pbatch.n; ++i)
            P[i * Pbatch.ld + i] += q;
    }
}

/**
 * @brief Batch covariance prediction with absolute Q (not Δt-scaled):
 *        ∀b: P_b ← F * P_b * Fᵀ + Q
 *
 * Assumes tight row-major for F, Q, and each P (ld = n, stride = n*n).
 * Uses strided batched GEMM to compute FP and then FP * Fᵀ.
 */
void cov_predict_batch(const MatrixView& Fv,
                       const MatrixView& Qv,
                       const BatchMat&   Pbatch)
{
    assert(Fv.rows == Fv.cols);
    assert(Qv.rows == Qv.cols);
    assert(Fv.rows == Qv.rows);
    assert(Pbatch.n == Fv.rows);
    assert(Pbatch.ld == Pbatch.n);          // assume tight row-major P
    assert(Fv.stride == Fv.cols);           // tight F
    assert(Qv.stride == Qv.cols);           // tight Q

    const MKL_INT n = (MKL_INT)Pbatch.n;
    const MKL_INT batch = (MKL_INT)Pbatch.count;

    // Scratch FP block: contiguous storage of n*n per batch item.
    const std::size_t mat_elems = (std::size_t)n * (std::size_t)n;
    double* FP = (double*)mkl_malloc(sizeof(double) * mat_elems * batch, 64);
    if (!FP) { /* Optional: fall back to per-item dgemm here */ return; }

    // (1) FP_b = F * P_b   (re-use same F for all b via strideA=0)
    cblas_dgemm_batch_strided(
        CblasRowMajor, CblasNoTrans, CblasNoTrans,
        n, n, n,
        1.0,
        Fv.ptr, (MKL_INT)Fv.stride, /*strideA=*/0,
        Pbatch.base, (MKL_INT)Pbatch.ld, (MKL_INT)Pbatch.stride,
        0.0,
        FP, n, (MKL_INT)mat_elems,
        batch);

    // (2) P_b = FP_b * Fᵀ   (re-use same F as B with TransB)
    cblas_dgemm_batch_strided(
        CblasRowMajor, CblasNoTrans, CblasTrans,
        n, n, n,
        1.0,
        FP, n, (MKL_INT)mat_elems,
        Fv.ptr, (MKL_INT)Fv.stride, /*strideB=*/0,
        0.0,
        Pbatch.base, (MKL_INT)Pbatch.ld, (MKL_INT)Pbatch.stride,
        batch);

    // (3) P_b += Q   (parallelize across b)
    #pragma omp parallel for schedule(static)
    for (long long b = 0; b < (long long)Pbatch.count; ++b) {
        double* Pi = Pbatch.base + (std::size_t)b * Pbatch.stride;
        for (MKL_INT r = 0; r < n; ++r) {
            const double* qrow = Qv.ptr + r * (MKL_INT)Qv.stride;
            double*       prow = Pi    + r * (MKL_INT)Pbatch.ld;
            for (MKL_INT c = 0; c < n; ++c)
                prow[c] += qrow[c];
        }
    }

    mkl_free(FP);
}

/**
 * @brief Δt-scaled random-walk with diagonal noise:
 *        ∀b: P_b ← P_b + (q_per_sec * dt) * I
 */
void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt)
{
    add_diag_noise_batch(Pbatch, q_per_sec * dt);
}

/**
 * @brief Δt-scaled random-walk with full Q:
 *        ∀b: P_b ← P_b + (Q_per_sec * dt)
 */
void cov_predict_rw_fullQ_dt(const MatrixView& Qps,
                             double dt,
                             const BatchMat& Pbatch)
{
    assert(Qps.rows == Qps.cols && Qps.rows == Pbatch.n);
    const MKL_INT n   = (MKL_INT)Pbatch.n;
    const MKL_INT ldQ = (MKL_INT)Qps.stride;

    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)Pbatch.count; ++i) {
        double* Pi = Pbatch.base + (std::size_t)i * Pbatch.stride;
        for (MKL_INT r = 0; r < n; ++r) {
            double*       prow = Pi      + r * (MKL_INT)Pbatch.ld;
            const double* qrow = Qps.ptr + r * ldQ;
            for (MKL_INT c = 0; c < n; ++c) prow[c] += qrow[c] * dt;
        }
    }
}

/**
 * @brief General Δt-scaled batch propagation:
 *        ∀b: P_b ← F * P_b * Fᵀ + (Q_per_sec * dt)
 *
 * Uses cblas_dgemm_batch (pointer arrays) for portability, then adds Q*dt.
 */
void cov_predict_batch_dt(const MatrixView& Fv,
                          const MatrixView& Qps,
                          double dt,
                          const BatchMat& Pbatch)
{
    // Dimension checks
    assert(Fv.rows == Fv.cols && Qps.rows == Qps.cols &&
           Fv.rows == Qps.rows && Pbatch.n == Fv.rows);

    const MKL_INT n   = (MKL_INT)Fv.rows;
    const MKL_INT ldF = (MKL_INT)Fv.stride;
    const MKL_INT ldP = (MKL_INT)Pbatch.ld;

    const std::size_t mat_elems = (std::size_t)Pbatch.n * (std::size_t)Pbatch.n;
    double* FP = (double*)mkl_malloc(sizeof(double) * mat_elems * Pbatch.count, 64);
    if (!FP) return;

    // (1) FP = F * P (batched pointer arrays)
    {
        const MKL_INT group_count = 1;
        MKL_INT m_arr[1]   = { n }, n_arr[1] = { n }, k_arr[1] = { n };
        MKL_INT lda_arr[1] = { ldF }, ldb_arr[1] = { ldP }, ldc_arr[1] = { (MKL_INT)Pbatch.n };
        CBLAS_TRANSPOSE ta_arr[1] = { CblasNoTrans }, tb_arr[1] = { CblasNoTrans };
        double alpha_arr[1] = { 1.0 }, beta_arr[1] = { 0.0 };
        MKL_INT group_size[1] = { (MKL_INT)Pbatch.count };

        const double** A_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        const double** B_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        double**       C_array = (double**)      mkl_malloc(sizeof(double*) * Pbatch.count, 64);

        for (std::size_t i = 0; i < Pbatch.count; ++i) {
            A_array[i] = Fv.ptr;                              // same F for all
            B_array[i] = Pbatch.base + i * Pbatch.stride;     // P_i
            C_array[i] = FP           + i * mat_elems;        // FP_i
        }

        cblas_dgemm_batch(CblasRowMajor,
                          ta_arr, tb_arr,
                          m_arr, n_arr, k_arr,
                          alpha_arr,
                          A_array, lda_arr,
                          B_array, ldb_arr,
                          beta_arr,
                          C_array, ldc_arr,
                          group_count, group_size);

        mkl_free(A_array); mkl_free(B_array); mkl_free(C_array);
    }

    // (2) P = FP * Fᵀ (batched pointer arrays)
    {
        const MKL_INT group_count = 1;
        MKL_INT m_arr[1]   = { n }, n_arr[1] = { n }, k_arr[1] = { n };
        MKL_INT lda_arr[1] = { (MKL_INT)Pbatch.n }, ldb_arr[1] = { ldF }, ldc_arr[1] = { ldP };
        CBLAS_TRANSPOSE ta_arr[1] = { CblasNoTrans }, tb_arr[1] = { CblasTrans };
        double alpha_arr[1] = { 1.0 }, beta_arr[1] = { 0.0 };
        MKL_INT group_size[1] = { (MKL_INT)Pbatch.count };

        const double** A_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        const double** B_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        double**       C_array = (double**)      mkl_malloc(sizeof(double*) * Pbatch.count, 64);

        for (std::size_t i = 0; i < Pbatch.count; ++i) {
            A_array[i] = FP           + i * mat_elems;        // FP_i
            B_array[i] = Fv.ptr;                               // F
            C_array[i] = Pbatch.base  + i * Pbatch.stride;     // P_i
        }

        cblas_dgemm_batch(CblasRowMajor,
                          ta_arr, tb_arr,
                          m_arr, n_arr, k_arr,
                          alpha_arr,
                          A_array, lda_arr,
                          B_array, ldb_arr,
                          beta_arr,
                          C_array, ldc_arr,
                          group_count, group_size);

        mkl_free(A_array); mkl_free(B_array); mkl_free(C_array);
    }

    // (3) Add Q_per_sec * dt
    const MKL_INT ldQ = (MKL_INT)Qps.stride;
    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)Pbatch.count; ++i) {
        double* Pi = Pbatch.base + (std::size_t)i * Pbatch.stride;
        for (MKL_INT r = 0; r < n; ++r) {
            double*       prow = Pi      + r * (MKL_INT)Pbatch.ld;
            const double* qrow = Qps.ptr + r * ldQ;
            for (MKL_INT c = 0; c < n; ++c) prow[c] += qrow[c] * dt;
        }
    }

    mkl_free(FP);
}

/**
 * @brief Numerically enforce symmetry for each covariance:
 *        P_b ← 0.5 * (P_b + P_bᵀ)
 *
 * @note This mitigates floating-point asymmetry; it does not guarantee PSD.
 */
void symmetrize_batch(const BatchMat& Pbatch)
{
    const std::size_t n = Pbatch.n;
    #pragma omp parallel for schedule(static)
    for (long long b = 0; b < (long long)Pbatch.count; ++b) {
        double* P = Pbatch.base + (std::size_t)b * Pbatch.stride;
        for (std::size_t r = 0; r < n; ++r) {
            for (std::size_t c = r + 1; c < n; ++c) {
                const double a    = P[r * Pbatch.ld + c];
                const double bval = P[c * Pbatch.ld + r];
                const double m    = 0.5 * (a + bval);
                P[r * Pbatch.ld + c] = m;
                P[c * Pbatch.ld + r] = m;
            }
        }
    }
}

} // namespace la
