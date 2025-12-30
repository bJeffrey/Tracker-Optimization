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
  const MKL_INT nn = static_cast<MKL_INT>(n) * static_cast<MKL_INT>(n);

  #ifdef _OPENMP
  #pragma omp parallel for
  #endif
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(batch); ++i) {
    double* Pi = P_batch + static_cast<std::size_t>(i) * static_cast<std::size_t>(nn);
    cblas_daxpy(nn, dt_s, Q_per_sec, 1, Pi, 1);  // Pi += dt_s * Q
  }
}

void rw_add_qdt_subset(double* P,
                       const double* Q,
                       double dt_s,
                       int n,
                       const std::uint64_t* ids,
                       std::size_t n_ids)
{
  if (!P || !Q || !ids || n <= 0 || n_ids == 0) return;
  if (dt_s <= 0.0) return;

  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
  const MKL_INT len = static_cast<MKL_INT>(nn);

  // Intentionally single-threaded: ids are sparse/random and candidate sets are usually small.
  for (std::size_t k = 0; k < n_ids; ++k) {
    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P + i * nn;
    cblas_daxpy(len, dt_s, Q, 1, Pi, 1);  // Pi += dt_s * Q
  }
}

void rw_add_qdt_subset_var_dt(double* P_batch,
                             const double* Q_per_sec,
                             int n,
                             const std::uint64_t* ids,
                             const double* dt_s_per_id,
                             std::size_t n_ids)
{
  if (!P_batch || !Q_per_sec || !ids || !dt_s_per_id || n <= 0 || n_ids == 0) return;

  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
  const MKL_INT len = static_cast<MKL_INT>(nn);

  // Intentionally single-threaded: ids are sparse/random and candidate sets are usually small.
  for (std::size_t k = 0; k < n_ids; ++k) {
    const double dt = dt_s_per_id[k];
    if (dt <= 0.0) continue;

    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P_batch + i * nn;
    cblas_daxpy(len, dt, Q_per_sec, 1, Pi, 1);  // Pi += dt * Q
  }
}

} // namespace la
