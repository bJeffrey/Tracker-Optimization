/**
 * @file la_eigen.cpp
 * @brief Eigen backend implementation of core linear-algebra routines (row-major).
 *
 * @details
 *   Implements all functions declared in la.h using the Eigen3 library.
 *   This backend supports efficient matrix operations with Eigen’s
 *   vectorization and (optionally) OpenMP-based parallelism across batches.
 *
 *   Usage notes:
 *     - All matrices are treated as row-major.
 *     - Batch operations iterate over a set of n×n covariances, each independent.
 *     - For best performance, build in Release mode and consider enabling OpenMP.
 *     - You can control threading at runtime (outer OpenMP vs Eigen’s own threading).
 *
 *   Threading guidance:
 *     - We parallelize *batch loops* with OpenMP (#pragma omp parallel for).
 *     - If you enable OpenMP in CMake (ENABLE_OPENMP=ON), set OMP_NUM_THREADS
 *       at runtime to control thread count, e.g.:
 *         OMP_NUM_THREADS=8 ./demo
 *     - Avoid oversubscription: if you use outer OpenMP on batch loops, prefer
 *       single-threaded Eigen at the inner level (default is fine for most small n).
 */

#include "la.h"
#include <Eigen/Dense>
#include <cassert>

namespace la {

/**
 * @brief General matrix multiply (row-major):
 *        C ← alpha * op(A) * op(B) + beta * C
 *
 * @param tA   If true, use Aᵀ; otherwise A.
 * @param tB   If true, use Bᵀ; otherwise B.
 * @param alpha Scalar multiplier for product term.
 * @param A,B   Input matrices (views).
 * @param beta  Scalar multiplier for existing C.
 * @param C     Output matrix (updated in place).
 *
 * @note Uses Eigen::Map to avoid copies. We materialize optional transposes
 *       into temporaries (Al, Bl) to keep types consistent for C++17.
 */
void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;
    typedef Eigen::Map<Mat> MapMat;

    MapMat Am(A.ptr, A.rows, A.cols);
    MapMat Bm(B.ptr, B.rows, B.cols);
    MapMat Cm(C.ptr, C.rows, C.cols);

    // Materialize optional transposes into same-typed temporaries.
    Mat Al(Am);
    if (tA) {
        Al = Am.transpose();
    }
    Mat Bl(Bm);
    if (tB) {
        Bl = Bm.transpose();
    }

    // Eigen handles vectorization internally.
    Cm = alpha * (Al * Bl) + beta * Cm;
}

/**
 * @brief Add a diagonal noise bump to every covariance in the batch:
 *        ∀i: P_i ← P_i + q * I
 *
 * @param Pbatch Batch descriptor (row-major blocks).
 * @param q      Diagonal increment.
 */
void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;

    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)Pbatch.count; ++i) {
        double* Pi = Pbatch.base + (std::size_t)i * Pbatch.stride;
        Eigen::Map<Mat> Pm(Pi, Pbatch.n, Pbatch.n);
        Pm.diagonal().array() += q;
    }
}

/**
 * @brief Full covariance prediction with absolute Q:
 *        ∀i: P_i ← F * P_i * Fᵀ + Q
 *
 * @param Fv     Shared transition matrix (n×n).
 * @param Qv     Shared process noise (n×n, absolute; not Δt-scaled).
 * @param Pbatch Batch of covariances (updated in place).
 */
void cov_predict_batch(const MatrixView& Fv,
                       const MatrixView& Qv,
                       const BatchMat&   Pbatch)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;

    assert(Fv.rows == Fv.cols);
    assert(Qv.rows == Qv.cols);
    assert(Fv.rows == Qv.rows);
    assert(Pbatch.n  == Fv.rows);

    const Eigen::Map<const Mat> F(Fv.ptr, Fv.rows, Fv.cols);
    const Eigen::Map<const Mat> Q(Qv.ptr, Qv.rows, Qv.cols);
    const Mat Ft = F.transpose();

    #pragma omp parallel for schedule(static)
    for (long long bi = 0; bi < (long long)Pbatch.count; ++bi) {
        double* Pi = Pbatch.base + (std::size_t)bi * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);

        // P ← F P Fᵀ + Q
        Mat FP = F * P;
        P.noalias() = FP * Ft;
        P += Q;
    }
}

/**
 * @brief Δt-scaled random-walk with diagonal noise:
 *        ∀i: P_i ← P_i + (q_per_sec * dt) * I
 */
void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt)
{
    const double q = q_per_sec * dt;
    add_diag_noise_batch(Pbatch, q);
}

/**
 * @brief Δt-scaled random-walk with full Q:
 *        ∀i: P_i ← P_i + (Q_per_sec * dt)
 */
void cov_predict_rw_fullQ_dt(const MatrixView& Qps,
                             double dt,
                             const BatchMat& Pbatch)
{
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Mat;
    assert(Qps.rows == Qps.cols && Qps.rows == Pbatch.n);

    const Eigen::Map<const Mat> Qs(Qps.ptr, Qps.rows, Qps.cols);
    const Mat Q = Qs * dt; // scale once outside the loop

    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)Pbatch.count; ++i) {
        double* Pi = Pbatch.base + (std::size_t)i * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);
        P += Q;
    }
}

/**
 * @brief General Δt-scaled propagation:
 *        ∀i: P_i ← F * P_i * Fᵀ + (Q_per_sec * dt)
 */
void cov_predict_batch_dt(const MatrixView& Fv,
                          const MatrixView& Qps,
                          double dt,
                          const BatchMat& Pbatch)
{
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Mat;
    assert(Fv.rows == Fv.cols && Qps.rows == Qps.cols &&
           Fv.rows == Qps.rows && Pbatch.n == Fv.rows);

    const Eigen::Map<const Mat> F(Fv.ptr, Fv.rows, Fv.cols);
    const Eigen::Map<const Mat> Qs(Qps.ptr, Qps.rows, Qps.cols);
    const Mat Ft = F.transpose();
    const Mat Q  = Qs * dt;

    #pragma omp parallel for schedule(static)
    for (long long bi = 0; bi < (long long)Pbatch.count; ++bi) {
        double* Pi = Pbatch.base + (std::size_t)bi * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);

        Mat FP = F * P;
        P.noalias() = FP * Ft;
        P += Q;
    }
}

/**
 * @brief Enforce symmetry numerically for every covariance:
 *        ∀i: P_i ← 0.5 * (P_i + P_iᵀ)
 *
 * @note This mitigates floating-point asymmetry; it does not guarantee PSD.
 */
void symmetrize_batch(const BatchMat& Pbatch)
{
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Mat;

    #pragma omp parallel for schedule(static)
    for (long long i = 0; i < (long long)Pbatch.count; ++i) {
        double* Pi = Pbatch.base + (std::size_t)i * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);
        P = 0.5 * (P + P.transpose());
    }
}

} // namespace la
