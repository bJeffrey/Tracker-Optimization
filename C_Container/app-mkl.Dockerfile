# Offline app build using the MKL dev base
FROM devbase:mkl
WORKDIR /src

# Expect project files:
#   CMakeLists.txt
#   include/la.h
#   src/la_eigen.cpp
#   src/la_mkl.cpp
#   src/main.cpp
COPY . /src

# Build (MKL backend only, C++98)
#RUN cmake -S . -B /build -G Ninja \
#    -DUSE_MKL=ON -DCMAKE_CXX_STANDARD=98 \
#    -DCMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake && \
#    cmake --build /build -j

RUN cmake -S . -B /build -G Ninja \
    -DUSE_MKL=ON -DCMAKE_CXX_STANDARD=98 -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake && \
    cmake --build /build -j


ENTRYPOINT ["/build/demo"]
