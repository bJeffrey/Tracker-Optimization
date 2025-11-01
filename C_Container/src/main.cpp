// src/main.cpp
#include "la.h"
#include <vector>
#include <iostream>
#include <cstdlib>   // rand, RAND_MAX
#include <cstring>   // memset

static double urand() {
    return 2.0 * (double)std::rand() / (double)RAND_MAX - 1.0;
}

int main() {
    std::size_t n = 6;              // state dim
    std::size_t tracks = 8;         // number of tracks

    // Build F = I (for demo), Q = 0.01*I
    std::vector<double> Fbuf(n*n, 0.0), Qbuf(n*n, 0.0);
    for (std::size_t i=0;i<n;++i) {
        Fbuf[i*n+i] = 1.0;
        Qbuf[i*n+i] = 0.01;
    }
    la::MatrixView F; F.ptr=&Fbuf[0]; F.rows=n; F.cols=n; F.stride=n;
    la::MatrixView Q; Q.ptr=&Qbuf[0]; Q.rows=n; Q.cols=n; Q.stride=n;

    // Batch of P_i (row-major, contiguous; stride = n*n, ld = n)
    std::vector<double> Pblock(tracks * n * n, 0.0);
    for (std::size_t t=0; t<tracks; ++t) {
        double* Pi = &Pblock[t * n * n];
        // seed each P with a random SPD-ish matrix (diagonal-dominant)
        for (std::size_t r=0;r<n;++r) {
            for (std::size_t c=0;c<n;++c) {
                Pi[r*n + c] = (r==c) ? 1.0 : 0.1*urand();
            }
        }
    }
    la::BatchMat Pbatch;
    Pbatch.base   = &Pblock[0];
    Pbatch.n      = n;
    Pbatch.ld     = n;
    Pbatch.stride = n * n;
    Pbatch.count  = tracks;

    // 1) P_i += q I (batch)
    la::add_diag_noise_batch(Pbatch, 0.05);

    // 2) P_i <- F P_i F^T + Q (batch)
    la::cov_predict_batch(F, Q, Pbatch);

    // quick checksum
    double sum = 0.0;
    for (std::size_t i=0;i<Pblock.size();++i) sum += Pblock[i];
    std::cout << "batch sum(P) = " << sum << std::endl;
    return 0;
}
