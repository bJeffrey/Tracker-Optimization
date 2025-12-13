# syntax=docker/dockerfile:1.7
# =========================
# Unified Multi-stage Dockerfile (eigen/std/mkl)
#
# Build:
#   Eigen: docker build -t la-demo:eigen --build-arg BACKEND=eigen .
#    Std : docker build -t la-demo:std   --build-arg BACKEND=std   .
#    MKL : docker build -t la-demo:mkl   --build-arg BACKEND=mkl   .
#
# Optional docs:
#   --build-arg GENERATE_DOCS=1
# =========================

# -------- stage 1: build --------
FROM ubuntu:22.04 AS build

ARG DEBIAN_FRONTEND=noninteractive
ARG BACKEND=eigen        # eigen | std | mkl
ARG GENERATE_DOCS=0

ENV CCACHE_DIR=/ccache
ENV CCACHE_MAXSIZE=2G

# Base tools + plumbing deps + ccache
# (Eigen installed here for simplicity; you can conditionalize later if you want)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build pkg-config \
      libeigen3-dev \
      ccache \
      sqlite3 libsqlite3-dev \
      libxml2-utils \
      zstd lz4 \
      ca-certificates curl gnupg && \
    rm -rf /var/lib/apt/lists/*

# Verify SQLite was built with R*Tree enabled
RUN sqlite3 ":memory:" "pragma compile_options;" | grep -i ENABLE_RTREE

# Optional: Intel oneAPI MKL (only when BACKEND=mkl)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    bash -lc 'set -e; \
      if [ "$BACKEND" = "mkl" ]; then \
        mkdir -p /usr/share/keyrings && \
        curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
          | gpg --dearmor -o /usr/share/keyrings/oneapi-archive-keyring.gpg && \
        echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
          > /etc/apt/sources.list.d/oneAPI.list && \
        apt-get update && apt-get install -y --no-install-recommends \
          intel-oneapi-mkl intel-oneapi-mkl-devel && \
        rm -rf /var/lib/apt/lists/*; \
      fi'

WORKDIR /src
COPY . /src

# Configure & build one backend using your CMake options (C++17 comes from CMakeLists.txt)
RUN --mount=type=cache,target=/ccache,sharing=locked \
    bash -lc 'set -e; \
      case "$BACKEND" in \
        eigen) CMAKE_ARGS="-DUSE_MKL=OFF -DUSE_STD=OFF" ;; \
        std)   CMAKE_ARGS="-DUSE_MKL=OFF -DUSE_STD=ON"  ;; \
        mkl)   CMAKE_ARGS="-DUSE_MKL=ON  -DUSE_STD=OFF -DCMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake" ;; \
        *) echo "ERROR: BACKEND must be eigen|std|mkl (got: $BACKEND)"; exit 2 ;; \
      esac; \
      cmake -S . -B /build -G Ninja \
        $CMAKE_ARGS \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache; \
      cmake --build /build -j'

# -------- stage 2: docs (optional) --------
FROM ubuntu:22.04 AS docs

ARG DEBIAN_FRONTEND=noninteractive
ARG GENERATE_DOCS=0

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    bash -lc 'set -e; \
      if [ "$GENERATE_DOCS" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends doxygen graphviz ca-certificates && \
        rm -rf /var/lib/apt/lists/*; \
      else \
        apt-get update && apt-get install -y --no-install-recommends ca-certificates && \
        rm -rf /var/lib/apt/lists/*; \
      fi'

WORKDIR /src
COPY --from=build /src/include /src/include
COPY --from=build /src/src     /src/src

RUN mkdir -p /src/docs && \
    if [ "$GENERATE_DOCS" = "1" ]; then \
      doxygen -g /src/Doxyfile && \
      sed -i 's|^OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = /src/docs|' /src/Doxyfile && \
      sed -i 's|^RECURSIVE.*|RECURSIVE = YES|' /src/Doxyfile && \
      sed -i 's|^INPUT .*|INPUT = include src|' /src/Doxyfile && \
      sed -i 's|^GENERATE_LATEX.*|GENERATE_LATEX = NO|' /src/Doxyfile && \
      sed -i 's|^HAVE_DOT.*|HAVE_DOT = YES|' /src/Doxyfile && \
      sed -i 's|^DOT_IMAGE_FORMAT.*|DOT_IMAGE_FORMAT = svg|' /src/Doxyfile && \
      doxygen /src/Doxyfile ; \
    fi

# -------- stage 3: runtime --------
FROM ubuntu:22.04 AS runtime

ARG DEBIAN_FRONTEND=noninteractive
ARG BACKEND=eigen

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      libgomp1 \
      sqlite3 libsqlite3-0 \
      libxml2-utils \
      zstd lz4 \
      curl gnupg && \
    rm -rf /var/lib/apt/lists/*

RUN sqlite3 ":memory:" "pragma compile_options;" | grep -i ENABLE_RTREE

# Optional MKL runtime (only when BACKEND=mkl)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    bash -lc 'set -e; \
      if [ "$BACKEND" = "mkl" ]; then \
        mkdir -p /usr/share/keyrings && \
        curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
          | gpg --dearmor -o /usr/share/keyrings/oneapi-archive-keyring.gpg && \
        echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
          > /etc/apt/sources.list.d/oneAPI.list && \
        apt-get update && apt-get install -y --no-install-recommends intel-oneapi-mkl && \
        rm -rf /var/lib/apt/lists/*; \
      fi'

COPY --from=build /build/demo /usr/local/bin/demo
COPY --from=docs  /src/docs   /usr/local/share/la-demo/docs

ENTRYPOINT ["/usr/local/bin/demo"]
