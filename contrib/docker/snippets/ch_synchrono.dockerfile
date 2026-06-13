# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::SynChrono
# Includes MPI and (optionally) a standalone FastDDS build that is safe
# to use alongside a ROS 2 installation.

ARG USERHOME
ARG PACKAGE_DIR="${USERHOME}/packages"
ARG FASTDDS_INSTALL_DIR="${PACKAGE_DIR}/fastrtps-2.4.0"

# --- MPI (required) --------------------------------------------------------

RUN sudo apt-get update && \
    sudo apt-get install --no-install-recommends -y libopenmpi-dev && \
    sudo apt-get clean && sudo apt-get autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# --- Standalone FastDDS 2.4.0 (optional DDS backend) -----------------------
# Built in a clean sub-shell so that any sourced ROS 2 environment variables
# (AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH, LD_LIBRARY_PATH) cannot cause CMake
# to pick up the ROS-bundled FastDDS/FastCDR.
# CMAKE_POLICY_VERSION_MINIMUM and "-include cstdint -Wno-error" keep these
# 2021-era sources building with CMake >= 4 / GCC >= 13; drop them when
# SynChrono moves to a current Fast-DDS.

RUN env -i HOME="${USERHOME}" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    bash -c ' \
    set -e && \
    INSTALL_DIR="'"${FASTDDS_INSTALL_DIR}"'" && \
    TMPDIR=$(mktemp -d) && cd "$TMPDIR" && \
    \
    echo "=== Building foonathan_memory ===" && \
    git clone -c advice.detachedHead=false --depth 1 --branch v0.7-3 \
        https://github.com/foonathan/memory.git && \
    cmake -S memory -B memory/build -G Ninja \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DCMAKE_CXX_FLAGS="-include cstdint -Wno-error" \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DFOONATHAN_MEMORY_BUILD_TESTS=OFF \
        -DFOONATHAN_MEMORY_BUILD_TOOLS=ON \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build memory/build -j"$(nproc)" && \
    cmake --install memory/build && \
    \
    echo "=== Building Fast-CDR ===" && \
    git clone -c advice.detachedHead=false --depth 1 --branch v1.0.24 \
        https://github.com/eProsima/Fast-CDR.git && \
    cmake -S Fast-CDR -B Fast-CDR/build -G Ninja \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DCMAKE_CXX_FLAGS="-include cstdint -Wno-error" \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release && \
    cmake --build Fast-CDR/build -j"$(nproc)" && \
    cmake --install Fast-CDR/build && \
    \
    echo "=== Building Fast-DDS ===" && \
    git clone -c advice.detachedHead=false --depth 1 --branch v2.4.0 \
        https://github.com/eProsima/Fast-DDS.git && \
    cmake -S Fast-DDS -B Fast-DDS/build -G Ninja \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DCMAKE_CXX_FLAGS="-include cstdint -Wno-error" \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
        -DCMAKE_PREFIX_PATH="$INSTALL_DIR" \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_BUILD_TYPE=Release \
        -DTHIRDPARTY=ON \
        -DSECURITY=OFF && \
    cmake --build Fast-DDS/build -j"$(nproc)" && \
    cmake --install Fast-DDS/build && \
    \
    rm -rf "$TMPDIR" '

# Update CMake options
# SynChrono uses the MPI backend here: this image also enables Chrono::ROS,
# and the standalone Fast-DDS 2.4.0 built above conflicts at configure time
# with the Fast-DDS 3.x that ROS Lyrical brings in (shared imported target
# eProsima_atomic). The DDS backend stays off until SynChrono is ported to
# ROS's Fast-DDS version. To re-enable it (e.g. a ROS-free image), uncomment
# the Fast-DDS options below (the *_DIR pins give them highest find_package
# priority so they win over the ROS-bundled copies).
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_SYNCHRONO=ON"
#   -DCH_USE_SYNCHRONO_FASTDDS=ON \
#   -DFastDDS_ROOT=${FASTDDS_INSTALL_DIR} \
#   -Dfastrtps_DIR=${FASTDDS_INSTALL_DIR}/share/fastrtps/cmake \
#   -Dfastcdr_DIR=${FASTDDS_INSTALL_DIR}/share/fastcdr/cmake \
#   -Dfoonathan_memory_DIR=${FASTDDS_INSTALL_DIR}/lib/foonathan_memory/cmake
