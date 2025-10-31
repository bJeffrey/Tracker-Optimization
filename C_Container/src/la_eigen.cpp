// src/la_eigen.cpp
#include "la.h"
#include <Eigen/Dense>

namespace la {
void gemm(bool tA, bool tB, double alpha,
          const MatrixView& A, const MatrixView& B,
          double beta, MatrixView& C)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat;
    Eigen::Map<Mat> Am(A.ptr, A.rows, A.cols);
    Eigen::Map<Mat> Bm(B.ptr, B.rows, B.cols);
    Eigen::Map<Mat> Cm(C.ptr, C.rows, C.cols);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Al, Bl;

    if (tA) Al = Am.transpose(); else Al = Am;
    if (tB) Bl = Bm.transpose(); else Bl = Bm;

    Cm = alpha * (Al * Bl) + beta * Cm;
}
} // namespace la
