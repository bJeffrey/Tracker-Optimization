#include "la.h"
#include <Eigen/Dense>
#include <cassert>

namespace la {

void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;
    typedef Eigen::Map<Mat> MapMat;

    MapMat Am(A.ptr, A.rows, A.cols);
    MapMat Bm(B.ptr, B.rows, B.cols);
    MapMat Cm(C.ptr, C.rows, C.cols);

    Mat Al, Bl;
    if (tA) Al = Am.transpose(); else Al = Am;
    if (tB) Bl = Bm.transpose(); else Bl = Bm;

    // Let Eigen handle vectorization
    Cm = alpha * (Al * Bl) + beta * Cm;
}

void add_diag_noise_batch(const BatchMat& Pbatch, double q)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;
    for (std::size_t i = 0; i < Pbatch.count; ++i) {
        double* Pi = Pbatch.base + i * Pbatch.stride;
        Eigen::Map<Mat> Pm(Pi, Pbatch.n, Pbatch.n);
        Pm.diagonal().array() += q;
    }
}

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

        // P <- F P F^T + Q
        Mat FP = F * P;
        P.noalias() = FP * Ft;
        P += Q;
    }
}

} // namespace la
