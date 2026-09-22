# SPDX-License-Identifier: MIT
# This snippet enables Chrono::DEM.
#
# The module needs a GPU backend and takes either one: src/chrono_dem/CMakeLists.txt declares
# REQUIRES CUDA_OR_HIP with HIP_PLATFORMS amd nvidia, using hipCUB on ROCm and CUB on CUDA.
# Its GPU dependencies come from cuda.dockerfile or rocm.dockerfile, so include one of those
# first; without either, Chrono::DEM reports itself unavailable and turns itself off.

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} -DCH_ENABLE_MODULE_DEM=ON"
