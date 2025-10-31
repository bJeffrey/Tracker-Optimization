# Dev base with toolchain + Intel oneAPI MKL (no internet needed later)
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

# Add Intel oneAPI APT repo & install MKL (do this while online)
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
    mkdir -p /usr/share/keyrings && \
    curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
      | gpg --dearmor -o /usr/share/keyrings/oneapi-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
      > /etc/apt/sources.list.d/oneAPI.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build pkg-config \
      intel-oneapi-mkl intel-oneapi-mkl-devel \
      ca-certificates && \
    rm -rf /var/lib/apt/lists/*
