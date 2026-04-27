# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Modal
# Builds Spectra (eigenvalue problem solver) from source.
# Chrono uses the Krylov-Schur solver, available only on the Spectra develop branch.
#
# Note on Spectra CMake variables:
# The public Chrono::Modal installation guide may refer to SpectraINCLUDE_DIR.
# In this source tree, however, the active finder documents and consumes the
# lower-case variables below:
#
#   - cmake/FindSpectra.cmake:
#       * spectra_DIR: directory containing the Spectra package config script
#       * spectra_INCLUDE_DIR: directory containing the subdirectory "Spectra/"
#   - src/chrono_modal/CMakeLists.txt:
#       * calls find_package(Spectra REQUIRED)
#       * reports that users should set spectra_INCLUDE_DIR or spectra_DIR if
#         Spectra cannot be found
#   - contrib/build-scripts/linux/buildSpectra.sh:
#       * installs Spectra's CMake package files under share/spectra/cmake
#       * installs Spectra headers under include/Spectra
#
# Pass both the package config directory and the include directory explicitly.
# This follows the current CMake code path and keeps configure-time discovery
# deterministic even if the documentation still mentions the older variable.

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

# Build Spectra
RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux/ && \
    bash buildSpectra.sh ${PACKAGE_DIR}/spectra

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_MODAL=ON \
    -Dspectra_DIR=${PACKAGE_DIR}/spectra/share/spectra/cmake \
    -Dspectra_INCLUDE_DIR=${PACKAGE_DIR}/spectra/include"
