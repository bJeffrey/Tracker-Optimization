#include "la.h"
#include <mkl.h>
#include <cassert>

namespace la {

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

void add_diag_noise_batch(MatrixView* P_list,
                          std::size_t count,
                          double q)
{
    for (std::size_t idx = 0; idx < count; ++idx) {
        MatrixView& P = P_list[idx];
        assert(P.rows == P.cols);
        const std::size_t n = P.rows;
        double* base = P.ptr;
        // diagonal elements are at base[i*stride + i]
        for (std::size_t i = 0; i < n; ++i) {
            base[i*P.stride + i] += q;
        }
    }
}

void cov_predict_batch(const MatrixView& F,
                       const MatrixView& Q,
                       MatrixView* P_list,
                       std::size_t count)
{
    assert(F.rows == F.cols);
    assert(Q.rows == Q.cols);
    assert(F.rows == Q.rows);

    const MKL_INT n = (MKL_INT)F.rows;
    const MKL_INT lda = (MKL_INT)F.stride;
    const MKL_INT ldq = (MKL_INT)Q.stride;

    // We'll need F^T for each multiply; compute once into a temp buffer.
    // To keep things simple and C++98-friendly, allocate a temporary matrix on heap.
    double* Ft_buf = new double[(std::size_t)n * (std::size_t)n];
    // Copy-transpose F -> Ft_buf
    for (MKL_INT r = 0; r < n; ++r)
        for (MKL_INT c = 0; c < n; ++c)
            Ft_buf[c*n + r] = F.ptr[r*lda + c];

    for (std::size_t t = 0; t < count; ++t) {
        MatrixView& Pv = P_list[t];
        assert(Pv.rows == Pv.cols);
        assert((MKL_INT)Pv.rows == n);

        // Temp FP = F * P
        double* FP = new double[(std::size_t)n * (std::size_t)n];

        // FP = 1.0 * F * P + 0.0 * FP
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
                    n, n, n,
                    1.0,
                    F.ptr,  lda,
                    Pv.ptr, (MKL_INT)Pv.stride,
                    0.0,
                    FP,     n);

        // P_new = FP * F^T + Q  (store back into P)
        cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
                    n, n, n,
                    1.0,
                    FP, n,
                    Ft_buf, n,
                    0.0,
                    Pv.ptr, (MKL_INT)Pv.stride);

        // Add Q: P += Q
        for (MKL_INT r = 0; r < n; ++r) {
            const double* qrow = Q.ptr + r*ldq;
            double* prow = Pv.ptr + r*(MKL_INT)Pv.stride;
            for (MKL_INT c = 0; c < n; ++c) prow[c] += qrow[c];
        }

        delete [] FP;
    }

    delete [] Ft_buf;
}

} // namespace la
