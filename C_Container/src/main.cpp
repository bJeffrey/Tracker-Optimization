// src/main.cpp
#include "la.h"
#include <vector>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>  // for wall-time timer (POSIX)

// simple wall clock
static double now_sec() {
    struct timeval tv; gettimeofday(&tv, 0);
    return double(tv.tv_sec) + double(tv.tv_usec)*1e-6;
}

static double urand() {
    return 2.0 * (double)std::rand() / (double)RAND_MAX - 1.0;
}

int main() {
    std::size_t n = 6;           // state dimension per track
    std::size_t tracks = 1000;   // number of tracks
    double dt = 0.1;             // seconds between propagations

    // Identity F (stationary ESM random-walk), but we keep a general F for completeness
    std::vector<double> Fbuf(n*n, 0.0);
    for (std::size_t i=0;i<n;++i) Fbuf[i*n + i] = 1.0;
    la::MatrixView F; F.ptr=&Fbuf[0]; F.rows=n; F.cols=n; F.stride=n;

    // Choose Q_per_sec: here full-diagonal per-second noise (e.g., sigma^2 per sec)
    std::vector<double> Qps_buf(n*n, 0.0);
    for (std::size_t i=0;i<n;++i) Qps_buf[i*n + i] = 0.05; // variance/sec on each state
    la::MatrixView Qps; Qps.ptr=&Qps_buf[0]; Qps.rows=n; Qps.cols=n; Qps.stride=n;

    // P batch storage (row-major, contiguous)
    std::vector<double> Pblock(tracks * n * n, 0.0);
    for (std::size_t t=0;t<tracks;++t) {
        double* Pi = &Pblock[t * n * n];
        for (std::size_t r=0;r<n;++r)
            for (std::size_t c=0;c<n;++c)
                Pi[r*n + c] = (r==c) ? 1.0 : 0.05*urand(); // PD-ish start
    }
    la::BatchMat Pbatch;
    Pbatch.base=&Pblock[0]; Pbatch.n=n; Pbatch.ld=n; Pbatch.stride=n*n; Pbatch.count=tracks;

    // Optional: add a tiny diagonal bump once (e.g., model fudge)
    la::add_diag_noise_batch(Pbatch, 1e-6);

    // ---- Timed propagation (Δt-scaled random walk) ----
    double t0 = now_sec();

    // Fast stationary ESM propagation (no FPF^T): P_i += Q_per_sec * dt
    la::cov_predict_rw_fullQ_dt(Qps, dt, Pbatch);

    // If you wanted full general form instead:
    // la::cov_predict_batch_dt(F, Qps, dt, Pbatch);

    // Optional: symmetrize for numeric hygiene
    la::symmetrize_batch(Pbatch);

    double t1 = now_sec();

    // Simple checksum
    double sum = 0.0;
    for (std::size_t i=0;i<Pblock.size();++i) sum += Pblock[i];

    std::cout << "Batch RW propagation time (dt=" << dt << "): "
              << (t1 - t0) << " s, checksum " << sum << std::endl;

    return 0;
}
