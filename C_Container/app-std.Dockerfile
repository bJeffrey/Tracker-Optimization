# app-std.Dockerfile
FROM devbase:std
WORKDIR /src

# Your project files (CMakeLists.txt, include/, src/, etc.)
COPY . /src

# Build STD backend (no Eigen/MKL), C++98, Release
RUN cmake -S . -B /build -G Ninja \
      -DUSE_STD=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=98 && \
    cmake --build /build -j

ENTRYPOINT ["/build/demo"]
