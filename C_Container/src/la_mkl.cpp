// src/la_mkl.cpp
#include "la.h"
#include <mkl.h>

namespace la {
void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    CBLAS_TRANSPOSE TA = tA ? CblasTrans : CblasNoTrans;
    CBLAS_TRANSPOSE TB = tB ? CblasTrans : CblasNoTrans;
    const MKL_INT m = (MKL_INT)(tA ? A.cols : A.rows);
    const MKL_INT n = (MKL_INT)(tB ? B.rows : B.cols);
    const MKL_INT k = (MKL_INT)(tA ? A.rows : A.cols);

    cblas_dgemm(CblasRowMajor, TA, TB, m, n, k,
                alpha, A.ptr, (MKL_INT)A.stride,
                        B.ptr, (MKL_INT)B.stride,
                beta,  C.ptr, (MKL_INT)C.stride);
}
} // namespace la
