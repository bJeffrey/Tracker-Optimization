# =========================
# Multi-stage Dockerfile
# Option 1: one backend per image
#   - Build Eigen image: docker build -t la-demo:eigen .
#   - Build MKL   image: docker build -t la-demo:mkl --build-arg USE_MKL=1 .
# =========================

# -------- stage 1: build --------
FROM ubuntu:22.04 AS build

ARG DEBIAN_FRONTEND=noninteractive
ARG USE_MKL=0

# Base tools + Eigen + build tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config libeigen3-dev \
    ca-certificates curl gnupg && \
    rm -rf /var/lib/apt/lists/*

# (Optional) Intel oneAPI MKL for build stage
# Uses current Intel key URL and signed-by repo entry
RUN if [ "$USE_MKL" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
      mkdir -p /usr/share/keyrings && \
      curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
        | gpg --dearmor -o /usr/share/keyrings/oneapi-archive-keyring.gpg && \
      echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
        > /etc/apt/sources.list.d/oneAPI.list && \
      apt-get update && apt-get install -y --no-install-recommends \
        intel-oneapi-mkl intel-oneapi-mkl-devel && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

WORKDIR /src
# Expecting:
#   CMakeLists.txt
#   include/la.h
#   src/la_eigen.cpp
#   src/la_mkl.cpp
#   src/main.cpp
COPY . /src

# Configure & build exactly one backend (C++98)
RUN if [ "$USE_MKL" = "1" ]; then \
      cmake -S . -B /build -G Ninja \
        -DUSE_MKL=ON \
        -DCMAKE_CXX_STANDARD=98 \
        -DCMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake && \
      cmake --build /build -j ; \
    else \
      cmake -S . -B /build -G Ninja \
        -DUSE_MKL=OFF \
        -DCMAKE_CXX_STANDARD=98 && \
      cmake --build /build -j ; \
    fi

# -------- stage 2: runtime --------
FROM ubuntu:22.04 AS runtime

ARG DEBIAN_FRONTEND=noninteractive
ARG USE_MKL=0

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# (Optional) Minimal MKL runtime if this image was built with MKL
RUN if [ "$USE_MKL" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
      mkdir -p /usr/share/keyrings && \
      curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
        | gpg --dearmor -o /usr/share/keyrings/oneapi-archive-keyring.gpg && \
      echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
        > /etc/apt/sources.list.d/oneAPI.list && \
      apt-get update && apt-get install -y --no-install-recommends \
        intel-oneapi-mkl && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

# --- Docs generation (online) ---
# Set to 0 to skip docs at build time.
ARG GENERATE_DOCS=1

RUN if [ "$GENERATE_DOCS" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends doxygen graphviz && \
      rm -rf /var/lib/apt/lists/* && \
      doxygen -g /src/Doxyfile && \
      sed -i 's|^OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = /src/docs|' /src/Doxyfile && \
      sed -i 's|^RECURSIVE.*|RECURSIVE = YES|' /src/Doxyfile && \
      sed -i 's|^INPUT .*|INPUT = include src|' /src/Doxyfile && \
      sed -i 's|^GENERATE_LATEX.*|GENERATE_LATEX = NO|' /src/Doxyfile && \
      sed -i 's|^HAVE_DOT.*|HAVE_DOT = YES|' /src/Doxyfile && \
      sed -i 's|^DOT_IMAGE_FORMAT.*|DOT_IMAGE_FORMAT = svg|' /src/Doxyfile && \
      doxygen /src/Doxyfile ; \
    fi

# Copy the built app
COPY --from=build /build/demo /usr/local/bin/demo

ENTRYPOINT ["/usr/local/bin/demo"]
