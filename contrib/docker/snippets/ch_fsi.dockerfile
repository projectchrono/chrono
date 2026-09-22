# SPDX-License-Identifier: MIT
# This snippet enables Chrono::FSI.
#
# Chrono::FSI::SPH needs a GPU backend and takes either one: src/chrono_fsi/sph/CMakeLists.txt
# declares REQUIRES CUDA_OR_HIP with HIP_PLATFORMS amd nvidia. Include cuda.dockerfile or
# rocm.dockerfile first; without either, the SPH solver reports itself unavailable and turns
# itself off, leaving the rest of Chrono::FSI intact.

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} -DCH_ENABLE_MODULE_FSI=ON"
