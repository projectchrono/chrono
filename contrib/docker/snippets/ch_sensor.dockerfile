# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::VSG

ARG OPTIX_SCRIPT

# Verify cuda is installed, exit if not
RUN if [ ! -d "/usr/local/cuda" ]; then echo "CUDA is required Chrono::Sensor."; exit 1; fi

# Verify OPTIX_SCRIPT has been set
RUN if [ -z "${OPTIX_SCRIPT}" ]; then echo "OPTIX_SCRIPT must be set to install Chrono::Sensor."; exit 1; fi

# TODO: need to add mapping to the /usr/share/nvidia/nvoptix.bin for denoisers.
# OptiX
COPY ${OPTIX_SCRIPT} /tmp/optix.sh
RUN sudo chmod +x /tmp/optix.sh && \
    mkdir -p ${PACKAGE_DIR}/optix && \
    /tmp/optix.sh --prefix=${PACKAGE_DIR}/optix --skip-license && \
    sudo rm /tmp/optix.sh

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_SENSOR=ON \
    -DOptiX_INSTALL_DIR=${PACKAGE_DIR}/optix \
    -DOptiX_INCLUDE_DIR=${PACKAGE_DIR}/optix/include \
    -DNUMPY_INCLUDE_DIR=$(python3 -c 'import numpy; print(numpy.get_include())') \
    -DUSE_CUDA_NVRTC=ON"