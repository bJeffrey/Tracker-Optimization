#include "la_batch_rw.h"

#include <cstddef>

#include <mkl.h>

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
  const MKL_INT nn = static_cast<MKL_INT>(n) * static_cast<MKL_INT>(n);

  #ifdef _OPENMP
  #pragma omp parallel for
  #endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(batch); ++i) {
    double* Pi = P_batch + static_cast<std::size_t>(i) * static_cast<std::size_t>(nn);
    // Pi += dt_s * Q_per_sec
    cblas_daxpy(nn, dt_s, Q_per_sec, 1, Pi, 1);
  }
}

} // namespace la
