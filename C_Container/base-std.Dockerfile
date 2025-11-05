# base-std.Dockerfile
FROM quay.io/centos/centos:stream9
ENV LANG=C.UTF-8
RUN dnf -y install dnf-plugins-core && \
    dnf -y config-manager --set-enabled crb && \
    dnf -y install \
        gcc gcc-c++ make \
        cmake ninja-build \
        doxygen graphviz \
        pkgconf-pkg-config \
        ca-certificates && \
    dnf clean all
