#include "la_batch_rw.h"

namespace la {

void rw_add_qdt_batch(double* P_batch,
                      const double* Q_per_sec,
                      double dt_s,
                      int n,
                      std::size_t batch)
{
  const std::size_t nn = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
  const double* Q = Q_per_sec;

  for (std::size_t i = 0; i < batch; ++i) {
    double* P = P_batch + i * nn;
    for (std::size_t k = 0; k < nn; ++k) {
      P[k] += Q[k] * dt_s;
    }
  }
}

} // namespace la
