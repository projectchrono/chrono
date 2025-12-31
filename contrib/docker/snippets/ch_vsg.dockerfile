# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::VSG

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

# Check the image is Ubuntu, error out if not
# RUN if [ ! -f /etc/os-release ] || ! grep -q 'ID=ubuntu' /etc/os-release; then echo "This snippet can only be used with an Ubuntu image."; exit 1; fi

# Vulkan
RUN wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | sudo tee /etc/apt/trusted.gpg.d/lunarg.asc && \
    UBUNTU_CODENAME=$(lsb_release -cs) && \
    sudo wget -qO /etc/apt/sources.list.d/lunarg-vulkan.list "http://packages.lunarg.com/vulkan/lunarg-vulkan-${UBUNTU_CODENAME}.list" && \
    sudo apt update && sudo apt install vulkan-sdk -y && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Build the VSG dependencies
RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux/ && \
    bash buildVSG.sh ${PACKAGE_DIR}/vsg/

# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${PACKAGE_DIR}/vsg/lib" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_VSG=ON \
    -Dvsg_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsg \
    -DvsgImGui_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsgImGui \
    -DvsgXchange_DIR=${PACKAGE_DIR}/vsg/lib/cmake/vsgXchange"