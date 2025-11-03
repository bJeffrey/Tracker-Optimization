# syntax=docker/dockerfile:1.6
# CentOS Stream 9 (RHEL 9–compatible)
FROM quay.io/centos/centos:stream9

ENV LANG=C.UTF-8
WORKDIR /src

# Enable CRB and EPEL/EPEL-Next, then install toolchain + deps
RUN dnf -y install dnf-plugins-core && \
    dnf -y config-manager --set-enabled crb && \
    dnf -y install epel-release epel-next-release && \
    dnf -y install \
        gcc gcc-c++ make \
        cmake ninja-build \
        eigen3-devel \
        doxygen graphviz \
        pkgconfig && \
    dnf clean all

# Copy project sources
COPY . /src

# Build (Eigen backend), C++17, Release
RUN cmake -S . -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=17 \
      -DUSE_STD=OFF -DUSE_MKL=OFF && \
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

# Run the demo by default
ENTRYPOINT ["/build/demo"]
