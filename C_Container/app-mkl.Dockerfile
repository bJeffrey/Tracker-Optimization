# syntax=docker/dockerfile:1.6
# CentOS Stream 9 (RHEL 9–compatible)
FROM quay.io/centos/centos:stream9

ENV LANG=C.UTF-8
WORKDIR /src

# Enable CRB and install base toolchain + utilities + OpenMP runtime
RUN dnf -y install dnf-plugins-core && \
    dnf -y config-manager --set-enabled crb && \
    dnf -y install \
        gcc gcc-c++ make \
        cmake ninja-build \
        doxygen graphviz \
        pkgconf-pkg-config \
        curl-minimal ca-certificates \
        libgomp \
    && dnf clean all

# Intel oneAPI MKL YUM repo (EL9) with correct GPG key location
RUN printf '%s\n' \
      '[intel-oneapi]' \
      'name=Intel oneAPI Repository' \
      'baseurl=https://yum.repos.intel.com/oneapi' \
      'enabled=1' \
      'gpgcheck=1' \
      'repo_gpgcheck=1' \
      'gpgkey=https://yum.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2023.PUB' \
    > /etc/yum.repos.d/intel-oneapi.repo \
 && rpm --import https://yum.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2023.PUB \
 && dnf -y makecache \
 && dnf -y install intel-oneapi-mkl-devel \
 && dnf clean all

# Helpful envs (MKLROOT is provided by the RPMs)
ENV MKLROOT=/opt/intel/oneapi/mkl/latest
# Add MKL & (if present) compiler runtimes to the loader path
ENV LD_LIBRARY_PATH=$MKLROOT/lib/intel64:/opt/intel/oneapi/compiler/latest/linux/lib:$LD_LIBRARY_PATH
# Help CMake find MKLConfig.cmake (note the /mkl suffix)
ENV CMAKE_PREFIX_PATH=$MKLROOT/lib/cmake/mkl${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}

# Threading policy:
# - Use GCC's OpenMP runtime with MKL (avoids mismatch with Intel iomp)
# - Keep MKL single-threaded by default; let outer OpenMP (your batch loops) drive parallelism
ENV MKL_THREADING_LAYER=GNU
ENV MKL_NUM_THREADS=1
# You control outer threads at run time with OMP_NUM_THREADS
# e.g.: docker run --rm -e OMP_NUM_THREADS=8 la-demo:mkl

# Copy sources
COPY . /src

# Build (MKL backend), C++17, Release, OpenMP enabled
RUN cmake -S . -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=17 \
      -DUSE_MKL=ON -DUSE_STD=OFF \
      -DENABLE_OPENMP=ON && \
    cmake --build /build -j

# Generate Doxygen docs to /src/docs/html
RUN doxygen -g /src/Doxyfile && \
    sed -i 's|^OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = /src/docs|' /src/Doxyfile && \
    sed -i 's|^RECURSIVE.*|RECURSIVE = YES|' /src/Doxyfile && \
    sed -i 's|^INPUT .*|INPUT = include src|' /src/Doxyfile && \
    sed -i 's|^GENERATE_LATEX.*|GENERATE_LATEX = NO|' /src/Doxyfile && \
    sed -i 's|^HAVE_DOT.*|HAVE_DOT = YES|' /src/Doxyfile && \
    sed -i 's|^DOT_IMAGE_FORMAT.*|DOT_IMAGE_FORMAT = svg|' /src/Doxyfile && \
    doxygen /src/Doxyfile

ENTRYPOINT ["/build/demo"]
