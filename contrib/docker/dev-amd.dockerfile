# syntax = devthefuture/dockerfile-x
# The INCLUDE directive is provided by the devthefuture/dockerfile-x project
#
# AMD / ROCm development image, the counterpart to dev.dockerfile.
#
# Chrono::DEM, Chrono::FSI::SPH and the Chrono::Vehicle SCM GPU backend all declare
# REQUIRES CUDA_OR_HIP, so they build here through HIP. Chrono::Sensor builds too: its
# default backend is Vulkan RT, which is vendor-neutral. Only the OptiX renderer is missing,
# since OptiX is NVIDIA-only.

# Will copy in the base configuration for the build
INCLUDE ./common/base.dockerfile

# GPU toolkit. cuda.dockerfile and rocm.dockerfile are alternatives; include exactly one.
INCLUDE ./snippets/rocm.dockerfile

# Chrono sources, shared dependencies, and the vendor-neutral module snippets
INCLUDE ./snippets/chrono.dockerfile

# Modules that require a GPU backend and accept either CUDA or HIP
INCLUDE ./snippets/ch_dem.dockerfile
INCLUDE ./snippets/ch_fsi.dockerfile

# Configure, build and install. Must come after every snippet that appends to CMAKE_OPTIONS.
INCLUDE ./snippets/chrono_build.dockerfile

# Will copy in other common configurations for this build
INCLUDE ./common/common.dockerfile

# Complete the build
INCLUDE ./common/final.dockerfile
