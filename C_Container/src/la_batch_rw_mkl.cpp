#include "la_batch_rw.h"

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
  if (!P_batch || !Q_per_sec || n <= 0 || batch == 0) return;

  const MKL_INT nn = static_cast<MKL_INT>(n) * static_cast<MKL_INT>(n);

#ifdef _OPENMP
  #pragma omp parallel for schedule(static)
#endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(batch); ++i) {
    double* Pi = P_batch + static_cast<std::size_t>(i) * static_cast<std::size_t>(nn);
    // Pi += dt_s * Q_per_sec
    cblas_daxpy(nn, dt_s, Q_per_sec, 1, Pi, 1);
  }
}

void rw_add_qdt_subset(double* P_batch,
                       const double* Q_per_sec,
                       double dt_s,
                       int n,
                       const std::uint64_t* ids,
                       std::size_t n_ids)
{
  if (!P_batch || !Q_per_sec || !ids || n <= 0 || n_ids == 0) return;

  const MKL_INT nn = static_cast<MKL_INT>(n) * static_cast<MKL_INT>(n);

  // Intentionally single-threaded: typical candidate set sizes are small,
  // and OpenMP overhead (plus potential MKL oversubscription) can dominate.
  for (std::size_t k = 0; k < n_ids; ++k) {
    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P_batch + i * static_cast<std::size_t>(nn);
    cblas_daxpy(nn, dt_s, Q_per_sec, 1, Pi, 1);
  }
}

} // namespace la
