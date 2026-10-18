# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Sensor

ARG OPTIX_SCRIPT

# Verify cuda is installed, exit if not
RUN if [ ! -d "/usr/local/cuda" ]; then echo "CUDA is required Chrono::Sensor."; exit 1; fi

# Verify OPTIX_SCRIPT has been set
RUN if [ -z "${OPTIX_SCRIPT}" ]; then echo "OPTIX_SCRIPT must be set to install Chrono::Sensor."; exit 1; fi

# Chrono::Sensor's default rendered-sensor backend is Vulkan RT, not OptiX, and it needs
# Vulkan headers plus glslangValidator at configure time: CH_USE_SENSOR_VULKAN_RT_GPU
# defaults ON and is a hard FATAL_ERROR when glslangValidator is missing. ch_vsg.dockerfile
# installs a superset of this and currently runs first, but that is an ordering accident --
# Chrono::Sensor has to be buildable without Chrono::VSG. apt is idempotent, so the overlap
# with ch_vsg.dockerfile is harmless.
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    libvulkan-dev \
    glslang-tools && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# The OptiX AI denoiser additionally needs the driver's weight blob, /usr/share/nvidia/nvoptix.bin,
# and nothing is done about it here on purpose. The blob ships with the host's NVIDIA driver and is
# proprietary and driver-version specific, so it cannot be baked into an image -- and since
# nvidia-container-toolkit v1.14.4 / v1.15.0 it need not be, because the toolkit mounts it into the
# container itself (NVIDIA/nvidia-container-toolkit issue #127). On an older toolkit it is simply
# absent and denoiser sensors fail at run time; that is left to surface as a plain runtime error
# rather than guessed at during the build. docker-compose.yml carries a commented-out bind mount
# for that case.

# Get optix script at https://developer.nvidia.com/designworks/optix/downloads/legacy
# OptiX
COPY ${OPTIX_SCRIPT} /tmp/optix.sh
RUN sudo chmod +x /tmp/optix.sh && \
    mkdir -p ${PACKAGE_DIR}/optix && \
    /tmp/optix.sh --prefix=${PACKAGE_DIR}/optix --skip-license && \
    sudo rm /tmp/optix.sh

# Update CMake options
#
# CH_USE_SENSOR_OPTIX must be requested explicitly. Chrono::Sensor now selects between an
# OptiX, a Vulkan RT and a Metal RT backend, and the OptiX one defaults to OFF, so without
# this the image installs the OptiX SDK above and then configures "Chrono::Sensor with NO
# OptiX support", leaving OptiX_INSTALL_DIR unused.
#
# CH_USE_SENSOR_NVRTC is the cache option; USE_CUDA_NVRTC is only the compile definition
# CMake derives from it, so passing that name reaches nothing and CMake reports it as a
# manually-specified variable that was not used. OptiX_INCLUDE_DIR was dead for the same
# reason: FindOptiX.cmake takes OptiX_INSTALL_DIR and publishes OptiX_INCLUDE itself.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_SENSOR=ON \
    -DCH_USE_SENSOR_OPTIX=ON \
    -DOptiX_INSTALL_DIR=${PACKAGE_DIR}/optix \
    -DCH_USE_SENSOR_NVRTC=ON"