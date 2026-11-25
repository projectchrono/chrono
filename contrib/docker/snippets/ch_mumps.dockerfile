# SPDX-License-Identifier: MIT
# Build static MUMPS libraries against Intel MKL. Include ch_pardisomkl first.

ARG CHRONO_DIR
ARG PACKAGE_DIR

RUN sudo apt update && \
    sudo apt install --no-install-recommends -y gfortran && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Customize a temporary copy of the repository helper for a Release-only build.
# bash -e propagates build failures; all downloaded and generated files are
# removed in this RUN so they do not remain in the image layer.
RUN SETVARS_ARGS=--force . /opt/intel/oneapi/setvars.sh && \
    mumps_tmp="$(mktemp -d)" && \
    sed 's/^BUILD_DEBUG=ON$/BUILD_DEBUG=OFF/' \
        "${CHRONO_DIR}/contrib/build-scripts/linux/buildMUMPS.sh" > "${mumps_tmp}/buildMUMPS.sh" && \
    grep -q '^BUILD_DEBUG=OFF$' "${mumps_tmp}/buildMUMPS.sh" && \
    cd "${mumps_tmp}" && \
    bash -e ./buildMUMPS.sh "${PACKAGE_DIR}/mumps" && \
    test -f "${PACKAGE_DIR}/mumps/cmake/MUMPSConfig.cmake" && \
    cd / && \
    rm -rf "${mumps_tmp}"

# Use the installed config directory directly so relative library paths resolve.
# MUMPS is static; its library directory needs no LD_LIBRARY_PATH entry.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MUMPS=ON \
    -DMUMPS_DIR=${PACKAGE_DIR}/mumps/cmake"
