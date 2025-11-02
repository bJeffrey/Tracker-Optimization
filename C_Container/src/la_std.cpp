#include "la.h"
#include <cassert>
#include <cstring> // memset

namespace la {

// Row-major element access with arbitrary leading dimension (ld = stride)
static inline double get_el(const double* base, std::size_t ld,
                            std::size_t r, std::size_t c)
{
    return base[r*ld + c];
}
static inline void set_el(double* base, std::size_t ld,
                          std::size_t r, std::size_t c, double v)
{
    base[r*ld + c] = v;
}

// C = alpha * op(A) * op(B) + beta * C
void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    const std::size_t M = tA ? A.cols : A.rows;
    const std::size_t N = tB ? B.rows : B.cols;
    const std::size_t K = tA ? A.rows : A.cols;

    assert(C.rows == M && C.cols == N);

    // Simple, cache-friendly-ish triple loop (i-j-k). For tiny N it’s fine.
    for (std::size_t i = 0; i < M; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            double acc = 0.0;
            for (std::size_t k = 0; k < K; ++k) {
                // a_ik
                double a = tA ? get_el(A.ptr, A.stride, k, i)
                              : get_el(A.ptr, A.stride, i, k);
                // b_kj
                double b = tB ? get_el(B.ptr, B.stride, j, k)
                              : get_el(B.ptr, B.stride, k, j);
                acc += a * b;
            }
            double cij = get_el(C.ptr, C.stride, i, j);
            set_el(C.ptr, C.stride, i, j, alpha*acc + beta*cij);
        }
    }
}

void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    for (std::size_t b = 0; b < Pbatch.count; ++b) {
        double* P = Pbatch.base + b * Pbatch.stride;
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
    assert(Pbatch.n  == Fv.rows);

    const std::size_t n = Pbatch.n;

    // Workspace for FP per track (tight n x n)
    // For simplicity, reuse one FP buffer inside the loop.
    double* FP = new double[n * n];

    MatrixView F = Fv;
    MatrixView Q = Qv;

    for (std::size_t b = 0; b < Pbatch.count; ++b) {
        double* Pptr = Pbatch.base + b * Pbatch.stride;

        // FP = F * P
        {
            MatrixView A; A.ptr = F.ptr;    A.rows = n; A.cols = n; A.stride = F.stride;
            MatrixView B; B.ptr = Pptr;     B.rows = n; B.cols = n; B.stride = Pbatch.ld;
            MatrixView C; C.ptr = FP;       C.rows = n; C.cols = n; C.stride = n;
            // C = 1*F*P + 0*C
            gemm(false, false, 1.0, A, B, 0.0, C);
        }

        // P = FP * F^T
        {
            MatrixView A; A.ptr = FP;       A.rows = n; A.cols = n; A.stride = n;
            MatrixView B; B.ptr = F.ptr;    B.rows = n; B.cols = n; B.stride = F.stride;
            MatrixView C; C.ptr = Pptr;     C.rows = n; C.cols = n; C.stride = Pbatch.ld;
            // C = 1*FP*F^T + 0*C
            gemm(false, true, 1.0, A, B, 0.0, C);
        }

        // P += Q
        for (std::size_t r = 0; r < n; ++r) {
            double* prow = Pptr + r * Pbatch.ld;
            const double* qrow = Q.ptr + r * Q.stride;
            for (std::size_t c = 0; c < n; ++c) {
                prow[c] += qrow[c];
            }
        }
    }

    delete [] FP;
}

void cov_predict_rw_diag_dt(const BatchMat& Pbatch,
                            double q_per_sec,
                            double dt)
{
    add_diag_noise_batch(Pbatch, q_per_sec*dt);
}

void cov_predict_rw_fullQ_dt(const MatrixView& Qps,
                             double dt,
                             const BatchMat& Pbatch)
{
    assert(Qps.rows==Qps.cols && Qps.rows==Pbatch.n);
    const std::size_t n = Pbatch.n;
    for (std::size_t b=0;b<Pbatch.count;++b) {
        double* P = Pbatch.base + b * Pbatch.stride;
        for (std::size_t r=0;r<n;++r) {
            double* prow = P + r*Pbatch.ld;
            const double* qrow = Qps.ptr + r*Qps.stride;
            for (std::size_t c=0;c<n;++c)
                prow[c] += qrow[c] * dt;
        }
    }
}

void cov_predict_batch_dt(const MatrixView& Fv,
                          const MatrixView& Qps,
                          double dt,
                          const BatchMat& Pbatch)
{
    const std::size_t n = Pbatch.n;
    double* FP = new double[n*n];
    for (std::size_t b=0;b<Pbatch.count;++b) {
        double* Pptr = Pbatch.base + b*Pbatch.stride;

        MatrixView A=Fv, B, C;
        B.ptr=Pptr; B.rows=n; B.cols=n; B.stride=Pbatch.ld;
        C.ptr=FP;   C.rows=n; C.cols=n; C.stride=n;
        gemm(false,false,1.0, A,B, 0.0, C);

        MatrixView A2; A2.ptr=FP;   A2.rows=n; A2.cols=n; A2.stride=n;
        MatrixView B2=Fv;
        MatrixView C2; C2.ptr=Pptr; C2.rows=n; C2.cols=n; C2.stride=Pbatch.ld;
        gemm(false,true,1.0, A2,B2, 0.0, C2);

        for (std::size_t r=0;r<n;++r) {
            double* prow = Pptr + r*Pbatch.ld;
            const double* qrow = Qps.ptr + r*Qps.stride;
            for (std::size_t c=0;c<n;++c) prow[c] += qrow[c] * dt;
        }
    }
    delete [] FP;
}

void symmetrize_batch(const BatchMat& Pbatch)
{
    const std::size_t n = Pbatch.n;
    for (std::size_t b=0;b<Pbatch.count;++b) {
        double* P = Pbatch.base + b*Pbatch.stride;
        for (std::size_t r=0;r<n;++r) {
            for (std::size_t c=r+1;c<n;++c) {
                const double a = P[r*Pbatch.ld + c];
                const double bt= P[c*Pbatch.ld + r];
                const double m = 0.5*(a+bt);
                P[r*Pbatch.ld + c] = m;
                P[c*Pbatch.ld + r] = m;
            }
        }
    }
}

} // namespace la
