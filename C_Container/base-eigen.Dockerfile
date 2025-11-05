# base-eigen.Dockerfile
FROM quay.io/centos/centos:stream9
ENV LANG=C.UTF-8
RUN dnf -y install dnf-plugins-core && \
    dnf -y config-manager --set-enabled crb && \
    dnf -y install epel-release epel-next-release && \
    dnf -y install \
        gcc gcc-c++ make \
        cmake ninja-build \
        eigen3-devel \
        doxygen graphviz \
        pkgconf-pkg-config \
        ca-certificates && \
    dnf clean all
