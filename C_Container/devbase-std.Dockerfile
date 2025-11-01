# devbase-std.Dockerfile
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

# Toolchain + CMake + Ninja
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config ca-certificates && \
    rm -rf /var/lib/apt/lists/*
