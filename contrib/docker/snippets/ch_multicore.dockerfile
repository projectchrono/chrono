# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Multicore
# Builds Blaze (header-only linear algebra library) from source.
#
# Note on Blaze CMake variables:
# The public Chrono::Multicore installation guide may refer to Blaze_ROOT_DIR.
# In this source tree, however, the active finder documents and consumes the
# lower-case variables below:
#
#   - cmake/FindBlaze.cmake:
#       * blaze_DIR: directory containing the Blaze package config script
#       * blaze_INCLUDE_DIR: directory containing the subdirectory "blaze/"
#   - src/chrono_multicore/CMakeLists.txt:
#       * calls find_package(Blaze REQUIRED)
#       * reports that users should set blaze_INCLUDE_DIR or blaze_DIR if
#         Blaze cannot be found
#   - contrib/build-scripts/linux/buildBlaze.sh:
#       * installs Blaze's CMake package files under share/blaze/cmake
#       * installs Blaze headers under include/blaze
#
# Pass both the package config directory and the include directory explicitly.
# This follows the current CMake code path and keeps configure-time discovery
# deterministic even if the documentation still mentions the older variable.

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

# Build Blaze
RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux/ && \
    bash buildBlaze.sh ${PACKAGE_DIR}/blaze

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MULTICORE=ON \
    -Dblaze_DIR=${PACKAGE_DIR}/blaze/share/blaze/cmake \
    -Dblaze_INCLUDE_DIR=${PACKAGE_DIR}/blaze/include"
