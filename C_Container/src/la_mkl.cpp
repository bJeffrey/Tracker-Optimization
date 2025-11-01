#include "la.h"
#include <mkl.h>
#include <cassert>

namespace la {

// Single GEMM
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

// Add q*I to each P
void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    for (std::size_t b = 0; b < Pbatch.count; ++b) {
        double* P = Pbatch.base + b * Pbatch.stride;
        for (std::size_t i = 0; i < Pbatch.n; ++i)
            P[i * Pbatch.ld + i] += q;
    }
}

// P_i <- F * P_i * F^T + Q (same F,Q; row-major; ld=n; stride=n*n)
void cov_predict_batch(const MatrixView& Fv,
                       const MatrixView& Qv,
                       const BatchMat&   Pbatch)
{
    assert(Fv.rows == Fv.cols);
    assert(Qv.rows == Qv.cols);
    assert(Fv.rows == Qv.rows);
    assert(Pbatch.n == Fv.rows);
    assert(Pbatch.ld == Pbatch.n);          // assume tight row-major
    assert(Fv.stride == Fv.cols);           // tight F
    assert(Qv.stride == Qv.cols);           // tight Q

    const MKL_INT n = (MKL_INT)Pbatch.n;
    const MKL_INT batch = (MKL_INT)Pbatch.count;

    // Scratch FP block, contiguous (n*n per batch)
    const std::size_t mat_elems = (std::size_t)n * (std::size_t)n;
    double* FP = (double*)mkl_malloc(sizeof(double) * mat_elems * batch, 64);
    if (!FP) { /* fallback: do per-track dgemm if desired */ return; }

    // 1) FP_i = F * P_i       (strided batched)
    cblas_dgemm_batch_strided(
        CblasRowMajor, CblasNoTrans, CblasNoTrans,
        n, n, n,
        1.0,
        Fv.ptr, (MKL_INT)Fv.stride, /*strideA=*/0,              // same F for all
        Pbatch.base, (MKL_INT)Pbatch.ld, (MKL_INT)Pbatch.stride,
        0.0,
        FP, n, (MKL_INT)mat_elems,
        batch);

    // 2) P_i = FP_i * F^T     (strided batched; use TransB)
    cblas_dgemm_batch_strided(
        CblasRowMajor, CblasNoTrans, CblasTrans,
        n, n, n,
        1.0,
        FP, n, (MKL_INT)mat_elems,
        Fv.ptr, (MKL_INT)Fv.stride, /*strideB=*/0,             // reuse F as B^T
        0.0,
        Pbatch.base, (MKL_INT)Pbatch.ld, (MKL_INT)Pbatch.stride,
        batch);

    // 3) P_i += Q  (simple loop; tiny n)
    for (std::size_t b = 0; b < Pbatch.count; ++b) {
        double* Pi = Pbatch.base + b * Pbatch.stride;
        for (MKL_INT r = 0; r < n; ++r) {
            const double* qrow = Qv.ptr + r * (MKL_INT)Qv.stride;
            double* prow = Pi + r * (MKL_INT)Pbatch.ld;
            for (MKL_INT c = 0; c < n; ++c)
                prow[c] += qrow[c];
        }
    }

    mkl_free(FP);
}

} // namespace la
