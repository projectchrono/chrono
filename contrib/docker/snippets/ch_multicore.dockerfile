# SPDX-License-Identifier: MIT
# Chrono::Multicore uses OpenMP and Thrust's host backend; it needs no GPU.
# Use CUDA's Thrust headers when available. CPU-only and AMD/HIP images need
# a stand-alone Thrust, since Chrono does not use rocThrust for host targets.

ARG PACKAGE_DIR
ARG THRUST_VERSION="1.17.2"

RUN if [ ! -f /usr/local/cuda/include/thrust/version.h ] && \
       [ ! -f /usr/local/cuda/include/cccl/thrust/version.h ]; then \
        thrust_tmp="$(mktemp -d)" && \
        wget -qO "${thrust_tmp}/thrust.tar.gz" \
            "https://github.com/NVIDIA/thrust/archive/refs/tags/${THRUST_VERSION}.tar.gz" && \
        mkdir -p "${PACKAGE_DIR}/thrust" && \
        tar -xzf "${thrust_tmp}/thrust.tar.gz" --strip-components=1 -C "${PACKAGE_DIR}/thrust" && \
        rm -rf "${thrust_tmp}"; \
    fi

# The fallback prefix is searched first; otherwise FindThrust uses the CUDA toolkit.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MULTICORE=ON \
    -DCH_ENABLE_OPENMP=ON \
    -DThrust_ROOT=${PACKAGE_DIR}/thrust"
