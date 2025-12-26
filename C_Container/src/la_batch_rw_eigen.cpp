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

  Eigen::Map<const Eigen::VectorXd> q(Q_per_sec, static_cast<Eigen::Index>(nn));
  const double scale = dt_s;

  for (std::size_t i = 0; i < batch; ++i) {
    Eigen::Map<Eigen::VectorXd> p(P_batch + i * nn, static_cast<Eigen::Index>(nn));
    p.noalias() += scale * q;
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

  Eigen::Map<const Eigen::VectorXd> q(Q_per_sec, static_cast<Eigen::Index>(nn));
  const double scale = dt_s;

  for (std::size_t k = 0; k < n_ids; ++k) {
    const std::size_t i = static_cast<std::size_t>(ids[k]);
    Eigen::Map<Eigen::VectorXd> p(P_batch + i * nn, static_cast<Eigen::Index>(nn));
    p.noalias() += scale * q;
  }
}

} // namespace la
