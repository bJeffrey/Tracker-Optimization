#include "la_batch_rw.h"

#include <Eigen/Dense>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace la {

void rw_add_qdt_batch(double* P_batch,
                      const double* Q_per_sec,
                      double dt_s,
                      int n,
                      std::size_t batch)
{
  using MatRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);

  Eigen::Map<const MatRM> Q(Q_per_sec, n, n);
  MatRM Qdt = Q * dt_s;

  #ifdef _OPENMP
  #pragma omp parallel for
  #endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(batch); ++i) {
    double* Pi = P_batch + static_cast<std::size_t>(i) * nn;
    Eigen::Map<MatRM> P(Pi, n, n);
    P.noalias() += Qdt;
  }
}

} // namespace la
