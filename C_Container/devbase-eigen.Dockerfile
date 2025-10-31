# Dev base with toolchain + Eigen (no internet needed later)
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config libeigen3-dev \
    ca-certificates && \
    rm -rf /var/lib/apt/lists/*
