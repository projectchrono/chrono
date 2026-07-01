# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::MUMPS
# Builds MUMPS via projectchrono fork of scivision/mumps.
# Requires: gfortran and Intel MKL (include ch_pardisomkl.dockerfile before this snippet).
#
# Note on MUMPS build and discovery:
# The Chrono::MUMPS installation guide says that MUMPS does not provide its own
# CMake-based installation system and recommends the Chrono utility scripts
# buildMUMPS.bat/buildMUMPS.sh/buildMUMPS_Mac.sh. The Linux script installs the
# CMake package configuration files under the chosen MUMPS install prefix, and
# Chrono should then be configured with MUMPS_DIR pointing at that config
# directory.
#
# This mirrors the current Chrono CMake code path:
#
#   - src/chrono_mumps/CMakeLists.txt:
#       * checks that a Fortran compiler is available
#       * calls find_package(MUMPS QUIET REQUIRED CONFIG)
#   - cmake/ChronoConfig.cmake.in:
#       * propagates MUMPS_DIR to Chrono consumers
#   - contrib/build-scripts/linux/buildMUMPS.sh:
#       * builds the projectchrono/mumps CMake wrapper
#       * requires a Fortran compiler and Intel oneAPI/MKL
#       * installs MUMPSConfig.cmake under the MUMPS install prefix
#       * recommends updating LD_LIBRARY_PATH for the installed MUMPS libraries
#
# Use gfortran in this image rather than Intel ifx: it keeps the Docker build
# independent from the oneAPI compiler toolchain while still satisfying Chrono's
# Fortran compiler requirement. Source oneAPI before running buildMUMPS.sh so
# the script can discover MKL from the ch_pardisomkl snippet. Finally, normalize
# the discovered MUMPSConfig.cmake directory to ${PACKAGE_DIR}/mumps/cmake so
# the final Chrono configure step can use a stable MUMPS_DIR value.

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

# Fortran compiler required by MUMPS
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y gfortran && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Build MUMPS (sources MKL env so MUMPS finds it via Intel oneAPI).
# After install, verify the CMake package and normalize its directory to
# ${PACKAGE_DIR}/mumps/cmake so the final Chrono configure step can use a
# plain, stable path.
RUN . /opt/intel/oneapi/setvars.sh --force > /dev/null 2>&1 && \
    cd ${CHRONO_DIR}/contrib/build-scripts/linux/ && \
    bash buildMUMPS.sh ${PACKAGE_DIR}/mumps && \
    mumps_config="$(find ${PACKAGE_DIR}/mumps \( -name MUMPSConfig.cmake -o -name mumps-config.cmake \) -print -quit)" && \
    if [ -z "$mumps_config" ]; then \
        echo "ERROR: MUMPS CMake package not produced. Tree of ${PACKAGE_DIR}/mumps:"; \
        ls -R ${PACKAGE_DIR}/mumps; \
        exit 1; \
    fi && \
    mumps_cmake_dir="$(dirname "$mumps_config")" && \
    if [ "$mumps_cmake_dir" != "${PACKAGE_DIR}/mumps/cmake" ]; then \
        rm -rf ${PACKAGE_DIR}/mumps/cmake && \
        ln -s "$mumps_cmake_dir" ${PACKAGE_DIR}/mumps/cmake; \
    fi && \
    echo "MUMPS CMake package: ${PACKAGE_DIR}/mumps/cmake"

# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${PACKAGE_DIR}/mumps/lib" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MUMPS=ON \
    -DMUMPS_DIR=${PACKAGE_DIR}/mumps/cmake"
