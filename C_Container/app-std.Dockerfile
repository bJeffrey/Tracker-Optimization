# syntax=docker/dockerfile:1.6

# From online only, no base image
# FROM quay.io/centos/centos:stream9

# From base image - offline enabled
ARG BASE_IMAGE=quay.io/centos/centos:stream9   # default for online
FROM ${BASE_IMAGE}

ENV LANG=C.UTF-8
WORKDIR /src

# --- Base dev tools + OpenMP + Doxygen ---
# dnf install not needed since we built an for offline use
RUN dnf -y install dnf-plugins-core && \
    dnf -y config-manager --set-enabled crb && \
    dnf -y install \
      gcc gcc-c++ make \
      cmake ninja-build \
      doxygen graphviz \
      pkgconfig \
      libgomp \
    && dnf clean all

# Copy sources
COPY . /src

# --- Build (STD backend), C++17, Release ---
RUN cmake -S . -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=17 \
      -DUSE_STD=ON -DUSE_MKL=OFF \
      -DENABLE_OPENMP=ON && \
    cmake --build /build -j

# --- Generate Doxygen docs to /src/docs/html ---
RUN doxygen -g /src/Doxyfile && \
    sed -i 's|^OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = /src/docs|' /src/Doxyfile && \
    sed -i 's|^RECURSIVE.*|RECURSIVE = YES|' /src/Doxyfile && \
    sed -i 's|^INPUT .*|INPUT = include src|' /src/Doxyfile && \
    sed -i 's|^GENERATE_LATEX.*|GENERATE_LATEX = NO|' /src/Doxyfile && \
    sed -i 's|^HAVE_DOT.*|HAVE_DOT = YES|' /src/Doxyfile && \
    sed -i 's|^DOT_IMAGE_FORMAT.*|DOT_IMAGE_FORMAT = svg|' /src/Doxyfile && \
    doxygen /src/Doxyfile

ENTRYPOINT ["/build/demo"]
