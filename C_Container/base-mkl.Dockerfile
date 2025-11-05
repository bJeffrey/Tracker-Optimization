# base-mkl.Dockerfile
FROM quay.io/centos/centos:stream9
ENV LANG=C.UTF-8

# Toolchain
RUN dnf -y install dnf-plugins-core && \
    dnf -y config-manager --set-enabled crb && \
    dnf -y install \
        gcc gcc-c++ make \
        cmake ninja-build \
        doxygen graphviz \
        pkgconf-pkg-config \
        ca-certificates && \
    dnf clean all

# Intel oneAPI MKL repo + install
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

# Helpful envs (set in one block to avoid UndefinedVar warnings)
# Helpful envs — define first, then append to avoid UndefinedVar warnings
ENV MKLROOT=/opt/intel/oneapi/mkl/latest
ENV LD_LIBRARY_PATH=""
ENV CMAKE_PREFIX_PATH=""

ENV LD_LIBRARY_PATH=/opt/intel/oneapi/mkl/latest/lib/intel64:/opt/intel/oneapi/compiler/latest/linux/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake:${CMAKE_PREFIX_PATH}
