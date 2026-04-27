# SPDX-License-Identifier: MIT
# This snippet enables Chrono::DEM (GPU-based discrete element method).
# Requires CUDA (provided by cuda.dockerfile, which must be included before this snippet).
#
# Note on DEM CUDA requirements:
# The Chrono::DEM installation guide requires CUDA to build applications based
# on this module and an NVIDIA GPU at runtime. It reports testing on Linux and
# Windows with CUDA 12.3 and 12.8. This Docker image uses the shared
# cuda.dockerfile snippet and currently defaults to CUDA 12.9 because
# Chrono::FSI-SPH, enabled in the same full-featured image, documents CUDA 12.9
# as its required toolkit version.
#
# This mirrors the current Chrono CMake code path:
#
#   - src/chrono_dem/CMakeLists.txt:
#       * enables the module with CH_ENABLE_MODULE_DEM
#       * requires CHRONO_CUDA_FOUND
#       * requires Eigen >= 3.3.6
#       * builds the optional DEM VSG interface when Chrono::VSG is enabled
#   - src/CMakeLists.txt:
#       * detects CUDAToolkit and sets CHRONO_CUDA_FOUND
#       * propagates CHRONO_CUDA_VERSION to generated configuration headers
#   - cmake/ChronoConfig.cmake.in:
#       * reports CUDA requirements to Chrono::DEM consumers
#       * propagates Chrono_dem and, when available, Chrono_dem_vsg
#
# Do not patch DEM CUDA sources here. NVIDIA CCCL/libcu++ documents
# cuda::std::terminate() as the supported device-side termination API and maps
# it to __trap(). The CCCL migration guide also directs users to replace legacy
# CUB/Thrust trap helpers with cuda::std::terminate(). If a future Chrono
# revision or CUDA toolkit exposes a source/API mismatch, the fix should be made
# in the DEM source or the CUDA version pin, rather than rewriting cloned source
# files from this Docker snippet.

# Verify cuda is installed, exit if not
RUN if [ ! -d "/usr/local/cuda" ]; then echo "CUDA is required for Chrono::DEM."; exit 1; fi

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_DEM=ON"
