# SPDX-License-Identifier: MIT
# Enable Chrono::DEM with a GPU backend (CUDA or HIP).
# CMake selects an available backend and disables the module if none is found.

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_DEM=ON"
