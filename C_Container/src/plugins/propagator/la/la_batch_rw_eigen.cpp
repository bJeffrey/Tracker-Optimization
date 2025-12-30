#include "la_batch_rw.h"

#include <Eigen/Dense>

namespace la {

void rw_add_qdt_batch(double* P_batch,
                      const double* Q_per_sec,
                      double dt_s,
                      int n,
                      std::size_t batch)
{
  if (!P_batch || !Q_per_sec || n <= 0 || batch == 0) return;

  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);

  // Treat each covariance as a flat vector (row-major contiguous).
  Eigen::Map<const Eigen::VectorXd> Q(Q_per_sec, static_cast<Eigen::Index>(nn));

  for (std::size_t i = 0; i < batch; ++i) {
    double* Pi = P_batch + i * nn;
    Eigen::Map<Eigen::VectorXd> P(Pi, static_cast<Eigen::Index>(nn));
    P.noalias() += dt_s * Q;
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

  Eigen::Map<const Eigen::VectorXd> Qv(Q, static_cast<Eigen::Index>(nn));

  for (std::size_t k = 0; k < n_ids; ++k) {
    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P + i * nn;
    Eigen::Map<Eigen::VectorXd> Pv(Pi, static_cast<Eigen::Index>(nn));
    Pv.noalias() += dt_s * Qv;
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

  Eigen::Map<const Eigen::VectorXd> Q(Q_per_sec, static_cast<Eigen::Index>(nn));

  // Intentionally single-threaded: ids are sparse/random and candidate sets are usually small.
  for (std::size_t k = 0; k < n_ids; ++k) {
    const double dt = dt_s_per_id[k];
    if (dt <= 0.0) continue;

    const std::size_t i = static_cast<std::size_t>(ids[k]);
    double* Pi = P_batch + i * nn;
    Eigen::Map<Eigen::VectorXd> P(Pi, static_cast<Eigen::Index>(nn));
    P.noalias() += dt * Q;
  }
}

} // namespace la
