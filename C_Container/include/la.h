#pragma once
#include <cstddef>

namespace la {

// Simple row-major matrix view
struct MatrixView {
    double* ptr;
    std::size_t rows;
    std::size_t cols;
    std::size_t stride; // leading dimension in elements (usually cols for row-major)
};

// Strided batch descriptor: a set of n x n matrices laid out one after another
struct BatchMat {
    double*      base;    // pointer to first matrix
    std::size_t  n;       // rows == cols
    std::size_t  ld;      // leading dimension (elements), usually n
    std::size_t  stride;  // distance (elements) between consecutive matrices, usually n*n
    std::size_t  count;   // number of matrices
};

// Single GEMM wrapper (row-major):
//   C = alpha * op(A) * op(B) + beta * C
void gemm(bool transA, bool transB,
          double alpha, const MatrixView& A, const MatrixView& B,
          double beta,        MatrixView& C);

// Batch diagonal noise bump: for all i,  P_i <- P_i + q I
void add_diag_noise_batch(const BatchMat& Pbatch, double q);

// Batch covariance prediction (same F, Q for all tracks):
//   For all i, P_i <- F * P_i * F^T + Q
void cov_predict_batch(const MatrixView& F,
                       const MatrixView& Q,
                       const BatchMat&   Pbatch);

// Random-walk, diagonal Q:    P_i <- P_i + (q_per_sec * dt) * I
void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt);
         
// Random-walk, full Q:        P_i <- P_i + (Q_per_sec * dt)
void cov_predict_rw_fullQ_dt(const MatrixView& Q_per_sec,
                             double dt,
                             const BatchMat& Pbatch);

// General Δt-scaled prediction:
// P_i <- F P_i F^T + (Q_per_sec * dt)
void cov_predict_batch_dt(const MatrixView& F,
                          const MatrixView& Q_per_sec,
                          double dt,
                          const BatchMat& Pbatch);

// Optional: numerically re-symmetrize P_i ← 0.5*(P_i + P_i^T)
void symmetrize_batch(const BatchMat& Pbatch);
         
} // namespace la
