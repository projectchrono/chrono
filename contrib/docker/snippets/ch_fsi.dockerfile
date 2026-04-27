# SPDX-License-Identifier: MIT
# This snippet enables Chrono::FSI (Fluid-Solid Interaction) including SPH and TDPF backends.
# Requires CUDA (provided by cuda.dockerfile, which must be included before this snippet).
#
# Note on FSI dependencies:
# The Chrono::FSI installation guide describes two submodules:
# Chrono::FSI-SPH and Chrono::FSI-TDPF. For FSI-SPH, the guide requires a CUDA
# installation and currently specifies CUDA 12.9, with CUDA 13 not yet supported.
# For FSI-TDPF, the guide requires HDF5 support and CH_ENABLE_HDF5=ON.
#
# This mirrors the current Chrono CMake code path:
#
#   - src/chrono_fsi/CMakeLists.txt:
#       * builds the generic Chrono_fsi interface library
#       * configures the sph and tdpf subdirectories
#   - src/chrono_fsi/sph/CMakeLists.txt:
#       * enables CH_ENABLE_MODULE_FSI_SPH when CH_ENABLE_MODULE_FSI is ON
#       * requires Eigen >= 3.3.6 and CHRONO_CUDA_FOUND
#       * optionally enables CH_USE_SPH_DOUBLE
#   - src/chrono_fsi/tdpf/CMakeLists.txt:
#       * enables CH_ENABLE_MODULE_FSI_TDPF when CH_ENABLE_MODULE_FSI is ON
#       * requires Eigen >= 3.3.6 and HDF5_FOUND
#   - cmake/ChronoConfig.cmake.in:
#       * reports CUDA/Thrust requirements to Chrono::FSI_SPH consumers
#       * propagates Chrono_fsi, Chrono_fsisph, and Chrono_fsitdpf libraries
#
# Keep both submodules enabled in this image and install libhdf5-dev for TDPF.
# The CUDA toolkit version is selected by the shared cuda.dockerfile snippet;
# when FSI-SPH is enabled, use CUDA_VERSION=12-9 to match the installation
# guide.
#
# Do not set CH_USE_SPH_DOUBLE here. The installation guide describes double
# precision as optional and notes that the single-precision FSI solver has been
# tested to provide a similar accuracy level with nearly a 2x performance
# improvement. Leaving CH_USE_SPH_DOUBLE at its default OFF value is therefore
# the better default for a general-purpose Docker image.

# Verify cuda is installed, exit if not
RUN if [ ! -d "/usr/local/cuda" ]; then echo "CUDA is required for Chrono::FSI."; exit 1; fi

# HDF5 is required by the TDPF submodule
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y libhdf5-dev && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_FSI=ON \
    -DCH_ENABLE_MODULE_FSI_SPH=ON \
    -DCH_ENABLE_MODULE_FSI_TDPF=ON \
    -DCH_ENABLE_HDF5=ON"
