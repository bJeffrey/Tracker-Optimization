# syntax=docker/dockerfile:1.6
# CentOS Stream 9 (RHEL 9–compatible)

# From online only, no base image
# FROM quay.io/centos/centos:stream9

# From base image - offline enabled
ARG BASE_IMAGE=quay.io/centos/centos:stream9   # default for online
FROM ${BASE_IMAGE}

ENV LANG=C.UTF-8
WORKDIR /src

# Enable CRB and EPEL/EPEL-Next, then install toolchain + deps (incl. OpenMP runtime)
# dnf install not needed since we built an for offline use
# RUN dnf -y install dnf-plugins-core && \
#     dnf -y config-manager --set-enabled crb && \
#     dnf -y install epel-release epel-next-release && \
#     dnf -y install \
#         gcc gcc-c++ make \
#         cmake ninja-build \
#         eigen3-devel \
#         doxygen graphviz \
#         pkgconfig \
#         libgomp \
#     && dnf clean all

# Copy project sources
COPY . /src

# Build (Eigen backend), C++17, Release, OpenMP enabled
RUN cmake -S . -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=17 \
      -DUSE_STD=OFF -DUSE_MKL=OFF \
      -DENABLE_OPENMP=ON \
    && cmake --build /build -j

# Generate Doxygen docs to /src/docs/html
RUN doxygen -g /src/Doxyfile && \
    sed -i 's|^OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = /src/docs|' /src/Doxyfile && \
    sed -i 's|^RECURSIVE.*|RECURSIVE = YES|' /src/Doxyfile && \
    sed -i 's|^INPUT .*|INPUT = include src|' /src/Doxyfile && \
    sed -i 's|^GENERATE_LATEX.*|GENERATE_LATEX = NO|' /src/Doxyfile && \
    sed -i 's|^HAVE_DOT.*|HAVE_DOT = YES|' /src/Doxyfile && \
    sed -i 's|^DOT_IMAGE_FORMAT.*|DOT_IMAGE_FORMAT = svg|' /src/Doxyfile && \
    doxygen /src/Doxyfile

# Tip: control outer batch parallelism at runtime (avoid oversubscription)
# e.g. docker run --rm -e OMP_NUM_THREADS=8 la-demo:eigen
ENTRYPOINT ["/build/demo"]
