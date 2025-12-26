#include "la_batch_rw.h"

namespace la {

void rw_add_qdt_batch(double* P_batch,
                      const double* Q_per_sec,
                      double dt_s,
                      int n,
                      std::size_t batch)
{
  if (!P_batch || !Q_per_sec || n <= 0 || batch == 0) return;

  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);

  for (std::size_t i = 0; i < batch; ++i) {
    double* P = P_batch + i * nn;
    for (std::size_t k = 0; k < nn; ++k) {
      P[k] += Q_per_sec[k] * dt_s;
    }
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

  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);

  for (std::size_t k = 0; k < n_ids; ++k) {
    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P_batch + i * nn;
    for (std::size_t j = 0; j < nn; ++j) {
      Pi[j] += Q_per_sec[j] * dt_s;
    }
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

  for (std::size_t k = 0; k < n_ids; ++k) {
    const double dt = dt_s_per_id[k];
    if (dt <= 0.0) continue;

    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P_batch + i * nn;
    for (std::size_t j = 0; j < nn; ++j) {
      Pi[j] += Q_per_sec[j] * dt;
    }
  }
}


} // namespace la
