// src/la_eigen.cpp
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

    Cm = alpha * (Al * Bl) + beta * Cm;
}

void add_diag_noise_batch(MatrixView* P_list,
                          std::size_t count,
                          double q)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;
    for (std::size_t i = 0; i < count; ++i) {
        MatrixView& Pv = P_list[i];
        assert(Pv.rows == Pv.cols);
        Eigen::Map<Mat> Pm(Pv.ptr, Pv.rows, Pv.cols);
        Pm.diagonal().array() += q;
    }
}

void cov_predict_batch(const MatrixView& Fv,
                       const MatrixView& Qv,
                       MatrixView* P_list,
                       std::size_t count)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;

    assert(Fv.rows == Fv.cols);
    assert(Qv.rows == Qv.cols);
    assert(Fv.rows == Qv.rows);

    Eigen::Map<const Mat> F(Fv.ptr, Fv.rows, Fv.cols);
    Eigen::Map<const Mat> Q(Qv.ptr, Qv.rows, Qv.cols);
    Mat Ft = F.transpose();

    for (std::size_t i = 0; i < count; ++i) {
        MatrixView& Pv = P_list[i];
        assert(Pv.rows == Pv.cols);
        assert(Pv.rows == (std::size_t)F.rows());

        Eigen::Map<Mat> P(Pv.ptr, Pv.rows, Pv.cols);

        Mat FP = F * P;      // FP = F * P
        P = FP * Ft;         // P  = FP * F^T
        P += Q;              // P += Q
    }
}

} // namespace la
