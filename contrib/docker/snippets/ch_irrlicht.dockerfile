# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Irrlicht

RUN sudo apt update && \
        sudo apt install --no-install-recommends -y \
        libirrlicht-dev && \
        sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_IRRLICHT=ON"