# SPDX-License-Identifier: MIT
# This snippet adds the OptiX renderer to Chrono::Sensor. It is separate from
# ch_sensor.dockerfile because OptiX is a CUDA-and-NVIDIA-only FEATURE of a module that is
# otherwise vendor-neutral: Chrono::Sensor's default backend is Vulkan RT, which builds and
# runs on AMD. src/chrono_sensor/CMakeLists.txt makes the same split deliberately, and warns
# that attaching "requires CUDA" to the module rather than to the feature would wrongly
# disable the whole module on AMD hardware that can run most of it.
#
# Include this only after ch_sensor.dockerfile, and only on a CUDA image.

ARG OPTIX_SCRIPT

# Verify cuda is installed, exit if not
RUN if [ ! -d "/usr/local/cuda" ]; then echo "CUDA is required for the Chrono::Sensor OptiX renderer."; exit 1; fi

# Verify OPTIX_SCRIPT has been set
RUN if [ -z "${OPTIX_SCRIPT}" ]; then echo "OPTIX_SCRIPT must be set to install the Chrono::Sensor OptiX renderer."; exit 1; fi

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
# CH_USE_SENSOR_OPTIX must be requested explicitly. Chrono::Sensor selects between an OptiX,
# a Vulkan RT and a Metal RT backend, and the OptiX one defaults to OFF, so without this the
# image installs the OptiX SDK above and then configures "Chrono::Sensor with NO OptiX
# support", leaving OptiX_INSTALL_DIR unused.
#
# CH_USE_SENSOR_NVRTC is the cache option; USE_CUDA_NVRTC is only the compile definition
# CMake derives from it, so passing that name reaches nothing and CMake reports it as a
# manually-specified variable that was not used. OptiX_INCLUDE_DIR was dead for the same
# reason: FindOptiX.cmake takes OptiX_INSTALL_DIR and publishes OptiX_INCLUDE itself.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_USE_SENSOR_OPTIX=ON \
    -DOptiX_INSTALL_DIR=${PACKAGE_DIR}/optix \
    -DCH_USE_SENSOR_NVRTC=ON"
