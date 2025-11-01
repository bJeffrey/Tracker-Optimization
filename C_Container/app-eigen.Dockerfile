# Offline app build using the Eigen dev base
FROM devbase:eigen
WORKDIR /src

# Expect project files:
#   CMakeLists.txt
#   include/la.h
#   src/la_eigen.cpp
#   src/la_mkl.cpp
#   src/main.cpp
COPY . /src

# Build (Eigen backend only, C++98)
#RUN cmake -S . -B /build -G Ninja -DUSE_MKL=OFF -DCMAKE_CXX_STANDARD=98 && \
#    cmake --build /build -j

RUN cmake -S . -B /build -G Ninja -DUSE_MKL=OFF -DCMAKE_CXX_STANDARD=98 -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /build -j


ENTRYPOINT ["/build/demo"]
