# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Sensor.
#
# Nothing here is vendor-specific. Chrono::Sensor's default rendered-sensor backend is
# Vulkan RT, which builds and runs on AMD as well as NVIDIA; the CUDA-only OptiX renderer
# lives in ch_sensor_optix.dockerfile.

# Vulkan headers and glslangValidator are needed at configure time:
# CH_USE_SENSOR_VULKAN_RT_GPU defaults ON and is a hard FATAL_ERROR when glslangValidator is
# missing. ch_vsg.dockerfile installs a superset of this and currently runs first, but that is
# an ordering accident -- Chrono::Sensor has to be buildable without Chrono::VSG. apt is
# idempotent, so the overlap with ch_vsg.dockerfile is harmless.
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    libvulkan-dev \
    glslang-tools && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} -DCH_ENABLE_MODULE_SENSOR=ON"
