/**
 * @file la_eigen.cpp
 * @brief Eigen backend implementation of core linear-algebra routines.
 *
 * @details
 *   Implements all functions declared in la.h using the Eigen3 library.
 *   This backend supports efficient matrix operations with vectorization
 *   and optional OpenMP multithreading. All matrices are assumed row-major.
 *
 *   The API supports batch covariance propagation and Δt-scaled random-walk
 *   updates typical in radar/ESM tracking pipelines.
 */

#include "la.h"
#include <Eigen/Dense>
#include <cassert>

namespace la {

/**
 * @brief General matrix multiply (row-major):
 *        C ← alpha * op(A) * op(B) + beta * C
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

    // Handle optional transpositions.
    Mat Al(Am);  // start as A
    Mat Bl(Bm);  // start as B
    if (tA)
        Al = Am.transpose();
    if (tB)
        Bl = Bm.transpose();

    // Eigen handles SIMD/vectorization internally.
    Cm = alpha * (Al * Bl) + beta * Cm;
}

/**
 * @brief Add a diagonal noise bump to every covariance in the batch:
 *        P_i ← P_i + q * I
 */
void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;

    for (std::size_t i = 0; i < Pbatch.count; ++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        Eigen::Map<Mat> Pm(Pi, Pbatch.n, Pbatch.n);
        Pm.diagonal().array() += q;
    }
}

/**
 * @brief Full covariance prediction:
 *        P_i ← F * P_i * Fᵀ + Q
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

    for (std::size_t i = 0; i < Pbatch.count; ++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);

        // P ← F P Fᵀ + Q
        Mat FP = F * P;
        P.noalias() = FP * Ft;
        P += Q;
    }
}

/**
 * @brief Δt-scaled random-walk with diagonal noise:
 *        P_i ← P_i + (q_per_sec * dt) * I
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
 *        P_i ← P_i + (Q_per_sec * dt)
 */
void cov_predict_rw_fullQ_dt(const MatrixView& Qps,
                             double dt,
                             const BatchMat& Pbatch)
{
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Mat;
    assert(Qps.rows == Qps.cols && Qps.rows == Pbatch.n);

    const Eigen::Map<const Mat> Qs(Qps.ptr, Qps.rows, Qps.cols);
    const Mat Q = Qs * dt; // scale once outside loop

    for (std::size_t i = 0; i < Pbatch.count; ++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);
        P += Q;
    }
}

/**
 * @brief General Δt-scaled propagation:
 *        P_i ← F * P_i * Fᵀ + (Q_per_sec * dt)
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

    for (std::size_t i = 0; i < Pbatch.count; ++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);
        Mat FP = F * P;
        P.noalias() = FP * Ft;
        P += Q;
    }
}

/**
 * @brief Enforce symmetry numerically:
 *        P_i ← 0.5 * (P_i + P_iᵀ)
 */
void symmetrize_batch(const BatchMat& Pbatch)
{
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Mat;

    for (std::size_t i = 0; i < Pbatch.count; ++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        Eigen::Map<Mat> P(Pi, Pbatch.n, Pbatch.n);
        P = 0.5 * (P + P.transpose());
    }
}

} // namespace la
