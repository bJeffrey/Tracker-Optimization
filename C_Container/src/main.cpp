// src/main.cpp
#include "la.h"
#include <vector>
#include <cstdlib>
#include <iostream>

int main() {
    const std::size_t M=4, K=3, N=5;
    std::vector<double> A(M*K), B(K*N), C(M*N, 0.0);

    for (std::size_t i=0;i<A.size();++i) A[i] = (double)std::rand()/RAND_MAX*2.0 - 1.0;
    for (std::size_t i=0;i<B.size();++i) B[i] = (double)std::rand()/RAND_MAX*2.0 - 1.0;

    la::MatrixView Am; Am.ptr=&A[0]; Am.rows=M; Am.cols=K; Am.stride=K;
    la::MatrixView Bm; Bm.ptr=&B[0]; Bm.rows=K; Bm.cols=N; Bm.stride=N;
    la::MatrixView Cm; Cm.ptr=&C[0]; Cm.rows=M; Cm.cols=N; Cm.stride=N;

    la::gemm(false,false,1.0,Am,Bm,0.0,Cm);

    double sum=0.0; for (std::size_t i=0;i<C.size();++i) sum+=C[i];
    std::cout << "sum(C)=" << sum << std::endl;
    return 0;
}
