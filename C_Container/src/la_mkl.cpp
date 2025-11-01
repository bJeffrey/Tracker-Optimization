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

void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    // Simple and cache-friendly diagonal bump
    for (std::size_t idx = 0; idx < Pbatch.count; ++idx) {
        double* P = Pbatch.base + idx * Pbatch.stride;
        for (std::size_t i = 0; i < Pbatch.n; ++i) {
            P[i * Pbatch.ld + i] += q;
        }
    }
}

void cov_predict_batch(const MatrixView& Fv,
                       const MatrixView& Qv,
                       const BatchMat&   Pbatch)
{
    assert(Fv.rows == Fv.cols);
    assert(Qv.rows == Qv.cols);
    assert(Fv.rows == Qv.rows);
    assert(Pbatch.n == Fv.rows);

    const MKL_INT n   = (MKL_INT)Fv.rows;
    const MKL_INT ldF = (MKL_INT)Fv.stride;
    const MKL_INT ldP = (MKL_INT)Pbatch.ld;

    // Workspace: FP for each batch matrix (contiguous block)
    const std::size_t mat_elems = Pbatch.n * Pbatch.n;
    double* FP_block = new double[mat_elems * Pbatch.count];

    // --- Batched GEMM #1: FP_i = F * P_i ---
    {
        const MKL_INT group_count = 1;
        MKL_INT m_arr[1]   = { n };
        MKL_INT n_arr[1]   = { n };
        MKL_INT k_arr[1]   = { n };
        MKL_INT lda_arr[1] = { ldF };
        MKL_INT ldb_arr[1] = { ldP };
        MKL_INT ldc_arr[1] = { (MKL_INT)Pbatch.n }; // FP is stored tightly: ld = n

        CBLAS_TRANSPOSE ta_arr[1] = { CblasNoTrans };
        CBLAS_TRANSPOSE tb_arr[1] = { CblasNoTrans };
        double alpha_arr[1] = { 1.0 };
        double beta_arr[1]  = { 0.0 };
        MKL_INT group_size[1] = { (MKL_INT)Pbatch.count };

        const double** A_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        const double** B_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        double**       C_array = (double**)      mkl_malloc(sizeof(double*) * Pbatch.count, 64);

        for (std::size_t i = 0; i < Pbatch.count; ++i) {
            A_array[i] = Fv.ptr;                                   // same F for all
            B_array[i] = Pbatch.base + i * Pbatch.stride;          // P_i
            C_array[i] = FP_block + i * mat_elems;                 // FP_i
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

    // --- Batched GEMM #2: P_i = FP_i * F^T ---
    {
        const MKL_INT group_count = 1;
        MKL_INT m_arr[1]   = { n };
        MKL_INT n_arr[1]   = { n };
        MKL_INT k_arr[1]   = { n };
        MKL_INT lda_arr[1] = { (MKL_INT)Pbatch.n }; // FP ld = n
        MKL_INT ldb_arr[1] = { ldF };
        MKL_INT ldc_arr[1] = { ldP };

        CBLAS_TRANSPOSE ta_arr[1] = { CblasNoTrans };
        CBLAS_TRANSPOSE tb_arr[1] = { CblasTrans }; // use F^T via trans flag
        double alpha_arr[1] = { 1.0 };
        double beta_arr[1]  = { 0.0 };
        MKL_INT group_size[1] = { (MKL_INT)Pbatch.count };

        const double** A_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        const double** B_array = (const double**)mkl_malloc(sizeof(double*) * Pbatch.count, 64);
        double**       C_array = (double**)      mkl_malloc(sizeof(double*) * Pbatch.count, 64);

        for (std::size_t i = 0; i < Pbatch.count; ++i) {
            A_array[i] = FP_block + i * mat_elems;                 // FP_i
            B_array[i] = Fv.ptr;                                   // same F for all, use TB = Trans
            C_array[i] = Pbatch.base + i * Pbatch.stride;          // P_i
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

    // P_i += Q (shared Q); keep it simple/robust
    {
        const MKL_INT ldQ = (MKL_INT)Qv.stride;
        for (std::size_t i = 0; i < Pbatch.count; ++i) {
            double* Pi = Pbatch.base + i * Pbatch.stride;
            for (MKL_INT r = 0; r < n; ++r) {
                double* prow = Pi + r * (MKL_INT)Pbatch.ld;
                const double* qrow = Qv.ptr + r * ldQ;
                for (MKL_INT c = 0; c < n; ++c) {
                    prow[c] += qrow[c];
                }
            }
        }
    }

    delete [] FP_block;
}

} // namespace la
