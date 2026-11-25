# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::VSG
# (Vulkan from the Ubuntu archive; the LunarG apt repository is discontinued)

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

RUN sudo apt update && sudo apt install -y --no-install-recommends \
    libvulkan-dev \
    vulkan-tools \
    mesa-vulkan-drivers \
    vulkan-validationlayers \
    spirv-tools \
    glslang-tools \
    glslang-dev \
    libshaderc-dev && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Keep dependency sources and build files out of the image layer.
RUN vsg_tmp="$(mktemp -d)" && \
    cd "${vsg_tmp}" && \
    bash -e "${CHRONO_DIR}/contrib/build-scripts/linux/buildVSG.sh" "${PACKAGE_DIR}/vsg" && \
    cd / && \
    rm -rf "${vsg_tmp}"

# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${PACKAGE_DIR}/vsg/lib" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_VSG=ON \
    -Dvsg_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsg \
    -DvsgImGui_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsgImGui \
    -DvsgXchange_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsgXchange"
