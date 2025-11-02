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

void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt)
{
    add_diag_noise_batch(Pbatch, q_per_sec * dt);
}

void cov_predict_rw_fullQ_dt(const MatrixView& Qps,
                             double dt,
                             const BatchMat& Pbatch)
{
    assert(Qps.rows==Qps.cols && Qps.rows==Pbatch.n);
    const MKL_INT n   = (MKL_INT)Pbatch.n;
    const MKL_INT ldQ = (MKL_INT)Qps.stride;

    for (std::size_t i=0;i<Pbatch.count;++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        for (MKL_INT r=0;r<n;++r) {
            double* prow = Pi + r * (MKL_INT)Pbatch.ld;
            const double* qrow = Qps.ptr + r * ldQ;
            for (MKL_INT c=0;c<n;++c) prow[c] += qrow[c] * dt;
        }
    }
}

void cov_predict_batch_dt(const MatrixView& Fv,
                          const MatrixView& Qps,
                          double dt,
                          const BatchMat& Pbatch)
{
    // same as cov_predict_batch, but add Qps*dt
    assert(Fv.rows==Fv.cols && Qps.rows==Qps.cols && Fv.rows==Qps.rows && Pbatch.n==Fv.rows);

    const MKL_INT n   = (MKL_INT)Fv.rows;
    const MKL_INT ldF = (MKL_INT)Fv.stride;
    const MKL_INT ldP = (MKL_INT)Pbatch.ld;

    const std::size_t mat_elems = (std::size_t)Pbatch.n * (std::size_t)Pbatch.n;
    double* FP = (double*)mkl_malloc(sizeof(double) * mat_elems * Pbatch.count, 64);
    if (!FP) return;

    // FP = F * P (batched)
    {
        const MKL_INT group_count = 1;
        MKL_INT m_arr[1]   = { n }, n_arr[1] = { n }, k_arr[1] = { n };
        MKL_INT lda_arr[1] = { ldF }, ldb_arr[1] = { ldP }, ldc_arr[1] = { (MKL_INT)Pbatch.n };
        CBLAS_TRANSPOSE ta_arr[1] = { CblasNoTrans }, tb_arr[1] = { CblasNoTrans };
        double alpha_arr[1] = { 1.0 }, beta_arr[1] = { 0.0 };
        MKL_INT group_size[1] = { (MKL_INT)Pbatch.count };

        const double** A_array = (const double**)mkl_malloc(sizeof(double*)*Pbatch.count,64);
        const double** B_array = (const double**)mkl_malloc(sizeof(double*)*Pbatch.count,64);
        double**       C_array = (double**)      mkl_malloc(sizeof(double*)*Pbatch.count,64);

        for (std::size_t i=0;i<Pbatch.count;++i) {
            A_array[i] = Fv.ptr;
            B_array[i] = Pbatch.base + i * Pbatch.stride;
            C_array[i] = FP           + i * mat_elems;
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

    // P = FP * F^T (batched)
    {
        const MKL_INT group_count = 1;
        MKL_INT m_arr[1]   = { n }, n_arr[1] = { n }, k_arr[1] = { n };
        MKL_INT lda_arr[1] = { (MKL_INT)Pbatch.n }, ldb_arr[1] = { ldF }, ldc_arr[1] = { ldP };
        CBLAS_TRANSPOSE ta_arr[1] = { CblasNoTrans }, tb_arr[1] = { CblasTrans };
        double alpha_arr[1] = { 1.0 }, beta_arr[1] = { 0.0 };
        MKL_INT group_size[1] = { (MKL_INT)Pbatch.count };

        const double** A_array = (const double**)mkl_malloc(sizeof(double*)*Pbatch.count,64);
        const double** B_array = (const double**)mkl_malloc(sizeof(double*)*Pbatch.count,64);
        double**       C_array = (double**)      mkl_malloc(sizeof(double*)*Pbatch.count,64);

        for (std::size_t i=0;i<Pbatch.count;++i) {
            A_array[i] = FP           + i * mat_elems;
            B_array[i] = Fv.ptr;
            C_array[i] = Pbatch.base  + i * Pbatch.stride;
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

    // Add Qps*dt
    const MKL_INT ldQ = (MKL_INT)Qps.stride;
    for (std::size_t i=0;i<Pbatch.count;++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        for (MKL_INT r=0;r<n;++r) {
            double* prow = Pi + r * (MKL_INT)Pbatch.ld;
            const double* qrow = Qps.ptr + r * ldQ;
            for (MKL_INT c=0;c<n;++c) prow[c] += qrow[c] * dt;
        }
    }

    mkl_free(FP);
}

void symmetrize_batch(const BatchMat& Pbatch)
{
    const std::size_t n = Pbatch.n;
    for (std::size_t b=0;b<Pbatch.count;++b) {
        double* P = Pbatch.base + b * Pbatch.stride;
        // only need to average upper/lower
        for (std::size_t r=0;r<n;++r) {
            for (std::size_t c=r+1;c<n;++c) {
                const double a = P[r*Pbatch.ld + c];
                const double bval = P[c*Pbatch.ld + r];
                const double m = 0.5*(a + bval);
                P[r*Pbatch.ld + c] = m;
                P[c*Pbatch.ld + r] = m;
            }
        }
    }
}

} // namespace la
