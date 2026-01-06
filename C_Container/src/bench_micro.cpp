/**
 * @file bench_micro.cpp
 * @brief Microbench harness for common tracker hot-path computations.
 */

#include "plugins/propagator/la_batch_rw.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Args {
  std::size_t n = 50000;
  std::size_t iters = 200;
  std::size_t ids_count = 5000;
  std::string bench = "all"; // all|aer|rdot|cov
};

std::size_t parse_size(const char* s, std::size_t def) {
  if (!s || !*s) return def;
  return static_cast<std::size_t>(std::stoull(s));
}

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--n" && i + 1 < argc) {
      a.n = parse_size(argv[++i], a.n);
    } else if (k == "--iters" && i + 1 < argc) {
      a.iters = parse_size(argv[++i], a.iters);
    } else if (k == "--ids-count" && i + 1 < argc) {
      a.ids_count = parse_size(argv[++i], a.ids_count);
    } else if (k == "--bench" && i + 1 < argc) {
      a.bench = argv[++i];
    }
  }
  return a;
}

template <typename Fn>
double time_it(const char* label, std::size_t iters, Fn&& fn) {
  const auto t0 = std::chrono::high_resolution_clock::now();
  for (std::size_t i = 0; i < iters; ++i) {
    fn();
  }
  const auto t1 = std::chrono::high_resolution_clock::now();
  const double s = std::chrono::duration<double>(t1 - t0).count();
  std::cout << label << "_total_s=" << s
            << " avg_s=" << (s / static_cast<double>(iters)) << "\n";
  return s;
}

struct AerOut {
  std::vector<double> az_deg;
  std::vector<double> el_deg;
  std::vector<double> r_m;
};

void bench_aer(const std::vector<double>& x,
               const std::vector<double>& y,
               const std::vector<double>& z,
               AerOut& out) {
  const std::size_t n = x.size();
  out.az_deg.resize(n);
  out.el_deg.resize(n);
  out.r_m.resize(n);

  double acc = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = x[i];
    const double dy = y[i];
    const double dz = z[i];
    const double r = std::sqrt(dx*dx + dy*dy + dz*dz);
    const double az = std::atan2(dy, dx);
    const double el = (r > 0.0) ? std::asin(dz / r) : 0.0;
    out.r_m[i] = r;
    out.az_deg[i] = az * 57.29577951308232;
    out.el_deg[i] = el * 57.29577951308232;
    acc += r + az + el;
  }
  if (acc == 0.0) {
    std::cerr << "acc=0\n";
  }
}

void bench_rdot(const std::vector<double>& x,
                const std::vector<double>& y,
                const std::vector<double>& z,
                const std::vector<double>& vx,
                const std::vector<double>& vy,
                const std::vector<double>& vz,
                std::vector<double>& rdot) {
  const std::size_t n = x.size();
  rdot.resize(n);

  double acc = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = x[i];
    const double dy = y[i];
    const double dz = z[i];
    const double r = std::sqrt(dx*dx + dy*dy + dz*dz);
    const double dot = dx*vx[i] + dy*vy[i] + dz*vz[i];
    const double rd = (r > 0.0) ? (dot / r) : 0.0;
    rdot[i] = rd;
    acc += rd;
  }
  if (acc == 0.0) {
    std::cerr << "acc=0\n";
  }
}

void bench_cov(std::size_t n, std::size_t ids_count) {
  const int dim = 9;
  const std::size_t nn = static_cast<std::size_t>(dim) * static_cast<std::size_t>(dim);
  std::vector<double> P(n * nn, 0.1);
  std::vector<double> Q(nn, 0.01);
  std::vector<std::uint64_t> ids(ids_count);
  std::vector<double> dt(ids_count, 1.0);

  const std::size_t stride = std::max<std::size_t>(1, n / ids_count);
  for (std::size_t i = 0; i < ids_count; ++i) {
    ids[i] = static_cast<std::uint64_t>(std::min(n - 1, i * stride));
  }

  la::rw_add_qdt_subset_var_dt(P.data(), Q.data(), dim, ids.data(), dt.data(), ids.size());
}

} // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  std::cout << "bench=n" << args.n
            << " iters=" << args.iters
            << " ids_count=" << args.ids_count
            << " bench=" << args.bench << "\n";

  std::vector<double> x(args.n), y(args.n), z(args.n);
  std::vector<double> vx(args.n), vy(args.n), vz(args.n);
  for (std::size_t i = 0; i < args.n; ++i) {
    const double t = static_cast<double>(i) * 0.001;
    x[i] = 1.0e6 + std::sin(t) * 1.0e3;
    y[i] = -4.7e6 + std::cos(t) * 1.0e3;
    z[i] = 4.1e6 + std::sin(t * 0.7) * 1.0e3;
    vx[i] = std::cos(t) * 50.0;
    vy[i] = std::sin(t) * 50.0;
    vz[i] = std::cos(t * 0.3) * 5.0;
  }

  if (args.bench == "all" || args.bench == "aer") {
    AerOut out;
    time_it("aer", args.iters, [&]() { bench_aer(x, y, z, out); });
  }

  if (args.bench == "all" || args.bench == "rdot") {
    std::vector<double> rdot;
    time_it("rdot", args.iters, [&]() { bench_rdot(x, y, z, vx, vy, vz, rdot); });
  }

  if (args.bench == "all" || args.bench == "cov") {
    time_it("cov", args.iters, [&]() { bench_cov(args.n, args.ids_count); });
  }

  return 0;
}
