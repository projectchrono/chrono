# SPDX-License-Identifier: MIT
# This snippet installs OpenCASCADE for Chrono::Cascade (STEP/IGES CAD import).
#
# Note on OpenCASCADE version and discovery:
# The Chrono::Cascade installation guide states that the Chrono API is
# compatible with OpenCASCADE OCCT 7.9.2 and 7.9.3, and asks users to set
# OpenCASCADE_DIR to the directory containing OpenCASCADEConfig.cmake if CMake
# cannot find it automatically. Distribution packages can lag behind the
# versions supported by the Chrono guide, so this snippet builds a supported
# upstream OCCT tag instead of relying on the version provided by the base
# image.
#
# This mirrors the current Chrono CMake code path:
#
#   - src/chrono_cascade/CMakeLists.txt:
#       * calls find_package(OpenCASCADE REQUIRED CONFIG)
#       * consumes the package variables exported by OpenCASCADEConfig.cmake
#   - contrib/build-scripts/linux/buildChrono.sh:
#       * uses a supported OpenCASCADE 7.9.x install directory
#       * passes OpenCASCADE_DIR=<prefix>/<lib>/cmake/opencascade
#
# Default to OCCT 7.9.3, the newest supported maintenance release listed by
# the Chrono::Cascade guide, while keeping the tag overridable for reproducible
# rebuilds against 7.9.2 if needed.
#
# Install OCCT under ${PACKAGE_DIR}/opencascade, pass its package config
# directory explicitly, and add its shared-library directory to LD_LIBRARY_PATH
# as recommended by the Chrono::Cascade guide for Linux runtime use.

ARG PACKAGE_DIR
ARG USERSHELLPROFILE
ARG OPENCASCADE_TAG="V7_9_3"

RUN sudo apt update && \
    sudo apt install --no-install-recommends -y \
        libfreetype-dev \
        libx11-dev \
        libxext-dev \
        libxmu-dev \
        libxi-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

RUN tmpdir="$(mktemp -d)" && \
    git clone --depth 1 --branch ${OPENCASCADE_TAG} https://github.com/Open-Cascade-SAS/OCCT.git "$tmpdir/occt" && \
    cmake -S "$tmpdir/occt" -B "$tmpdir/occt-build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${PACKAGE_DIR}/opencascade \
        -DBUILD_MODULE_Draw=OFF \
        -DBUILD_SAMPLES_QT=OFF \
        -DBUILD_Inspector=OFF \
        -DUSE_TBB=OFF && \
    cmake --build "$tmpdir/occt-build" && \
    cmake --install "$tmpdir/occt-build" && \
    rm -rf "$tmpdir" && \
    test -f ${PACKAGE_DIR}/opencascade/lib/cmake/opencascade/OpenCASCADEConfig.cmake

RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${PACKAGE_DIR}/opencascade/lib" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_CASCADE=ON \
    -DOpenCASCADE_DIR=${PACKAGE_DIR}/opencascade/lib/cmake/opencascade"
