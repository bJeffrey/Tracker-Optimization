#pragma once
#include <cstddef>

namespace la {

struct MatrixView {
    double* ptr;
    std::size_t rows, cols, stride; // row-major
};

// Existing:
void gemm(bool transA, bool transB,
          double alpha, const MatrixView& A, const MatrixView& B,
          double beta,        MatrixView& C);

// NEW 1: Add scalar process noise to the diagonal of many P's: P += q * I
void add_diag_noise_batch(MatrixView* P_list,
                          std::size_t count,
                          double q);

// NEW 2: Covariance prediction in batch: P <- F P F^T + Q (same F,Q for all)
void cov_predict_batch(const MatrixView& F,
                       const MatrixView& Q,
                       MatrixView* P_list,
                       std::size_t count);

} // namespace la
