# syntax=docker/dockerfile:1.6

# Default to Ubuntu when online. Override with: --build-arg BASE_IMAGE=devbase:std
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /src

# If the base doesn't already include toolchains (online case), install them.
# In the offline case (BASE_IMAGE=devbase:std), this block is skipped because cmake exists.
RUN if ! command -v cmake >/dev/null 2>&1; then \
      apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build pkg-config ca-certificates && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

# Project sources
COPY . /src

# Build (STD backend), C++98, Release
RUN cmake -S . -B /build -G Ninja \
      -DUSE_STD=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=98 && \
    cmake --build /build -j

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


ENTRYPOINT ["/build/demo"]
