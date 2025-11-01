// src/main.cpp
#include "la.h"
#include <vector>
#include <cstdlib>
#include <iostream>
// ... existing includes ...
#include <cstring> // memset

int main() {
    const std::size_t n = 6;
    const std::size_t tracks = 4;

    // Allocate Ps (row-major, stride=n)
    std::vector< std::vector<double> > Pbuf(tracks, std::vector<double>(n*n, 0.0));
    std::vector<la::MatrixView> Pviews(tracks);
    for (std::size_t i=0;i<tracks;++i) {
        // simple PD-ish seed
        for (std::size_t r=0;r<n;++r)
          for (std::size_t c=0;c<n;++c)
            Pbuf[i][r*n + c] = (r==c) ? 1.0 : 0.1;

        Pviews[i].ptr = &Pbuf[i][0];
        Pviews[i].rows = n; Pviews[i].cols = n; Pviews[i].stride = n;
    }

    // Build F (constant across tracks)
    std::vector<double> Fbuf(n*n, 0.0);
    for (std::size_t i=0;i<n;++i) Fbuf[i*n + i] = 1.0; // identity for demo
    la::MatrixView F; F.ptr=&Fbuf[0]; F.rows=n; F.cols=n; F.stride=n;

    // Build Q (process noise)
    std::vector<double> Qbuf(n*n, 0.0);
    for (std::size_t i=0;i<n;++i) Qbuf[i*n + i] = 0.01; // small diag noise
    la::MatrixView Q; Q.ptr=&Qbuf[0]; Q.rows=n; Q.cols=n; Q.stride=n;

    // 1) Bump diagonals: P += q I (batch)
    la::add_diag_noise_batch(&Pviews[0], tracks, 0.05);

    // 2) Full prediction: P <- F P F^T + Q (batch)
    la::cov_predict_batch(F, Q, &Pviews[0], tracks);

    // print a quick checksum
    double sum=0.0;
    for (std::size_t i=0;i<tracks;++i)
      for (std::size_t j=0;j<n*n;++j)
        sum += Pbuf[i][j];

    std::cout << "batch sum(P) = " << sum << std::endl;
    return 0;
}
