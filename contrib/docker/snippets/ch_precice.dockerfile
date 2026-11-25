# SPDX-License-Identifier: MIT
# Install preCICE for Chrono's partitioned multi-physics adapters.

ARG PACKAGE_DIR
ARG PRECICE_VERSION="3.4.0"

# Eigen and Python development files are installed by chrono.dockerfile.
# Preserve preCICE's MPI, PETSc mapping, and Python actions support.
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y \
        libboost-log-dev \
        libboost-program-options-dev \
        libboost-thread-dev \
        libboost-test-dev \
        libxml2-dev \
        libopenmpi-dev \
        openmpi-bin \
        libpetsc-real-dev \
        pkg-config && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Chrono's installation guide records testing with preCICE 3.4.0.
# Keep downloads and build files out of the resulting image layer.
RUN precice_tmp="$(mktemp -d)" && \
    wget -nv -O "${precice_tmp}/precice.tar.gz" \
        "https://github.com/precice/precice/archive/refs/tags/v${PRECICE_VERSION}.tar.gz" && \
    mkdir "${precice_tmp}/src" && \
    tar -xzf "${precice_tmp}/precice.tar.gz" --strip-components=1 -C "${precice_tmp}/src" && \
    cmake -S "${precice_tmp}/src" -B "${precice_tmp}/build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${PACKAGE_DIR}/precice" \
        -DCMAKE_INSTALL_LIBDIR=lib \
        -DEigen3_DIR=/usr/share/eigen3/cmake \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
        -DPRECICE_FEATURE_MPI_COMMUNICATION=ON \
        -DPRECICE_FEATURE_PETSC_MAPPING=ON \
        -DPRECICE_FEATURE_PYTHON_ACTIONS=ON \
        -DBUILD_TESTING=OFF && \
    cmake --build "${precice_tmp}/build" && \
    cmake --install "${precice_tmp}/build" && \
    test -s "${PACKAGE_DIR}/precice/lib/cmake/precice/preciceConfig.cmake" && \
    test -s "${PACKAGE_DIR}/precice/lib/libprecice.so" && \
    rm -rf "${precice_tmp}"

ENV PATH="${PACKAGE_DIR}/precice/bin:${PATH}"
ENV LD_LIBRARY_PATH="${PACKAGE_DIR}/precice/lib:${LD_LIBRARY_PATH}"

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PRECICE=ON \
    -Dprecice_DIR=${PACKAGE_DIR}/precice/lib/cmake/precice"
