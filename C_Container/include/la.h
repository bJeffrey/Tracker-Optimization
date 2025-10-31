// include/la.h
#pragma once
#include <cstddef>

namespace la {

struct MatrixView {            // row-major view
    double* ptr;
    std::size_t rows, cols, stride; // stride in elements
};

// C = alpha*A*B + beta*C
void gemm(bool transA, bool transB,
          double alpha, const MatrixView& A, const MatrixView& B,
          double beta,        MatrixView& C);

} // namespace la
