# SPDX-License-Identifier: MIT
# This snippet fetches Chrono and its module dependencies into ${CHRONO_DIR}.
# It includes other snippets, where specific modules can be added or removed based on need.
#
# It does NOT configure or build: chrono_build.dockerfile does that, and is included
# separately so the top-level dockerfile can slot vendor-specific snippets (a GPU toolkit,
# the OptiX renderer) in between. INCLUDE is textual, so anything that has to vary per GPU
# vendor has to be composed at that level rather than chosen here.

ARG CHRONO_BRANCH="main"
ARG CHRONO_REPO="https://github.com/projectchrono/chrono.git"
ARG CHRONO_DIR="${USERHOME}/chrono"
ARG CHRONO_INSTALL_DIR="${USERHOME}/packages/chrono"
ARG PACKAGE_DIR="${USERHOME}/packages"
RUN mkdir -p ${PACKAGE_DIR}

# Install Chrono dependencies that are required for all modules (or some but are fairly small)
RUN sudo apt update && \
        sudo apt install --no-install-recommends -y \
        libirrlicht-dev \
        libeigen3-dev \
        git \
        cmake \
        build-essential \
        ninja-build \
        swig \
        libxxf86vm-dev \
        python3-numpy \
        python3-dev \
        libglu1-mesa-dev \
        libglew-dev \
        libglfw3-dev \
        libblas-dev \
        liblapack-dev \
        wget \
        xorg-dev && \
        (sudo apt install --no-install-recommends -y libglut-dev || \
         sudo apt install --no-install-recommends -y freeglut3-dev) && \
        sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Clone Chrono before running the snippets
RUN git clone --recursive -b ${CHRONO_BRANCH} ${CHRONO_REPO} ${CHRONO_DIR}

# Include the snippets which install shared dependencies
# These can be commented out or removed if they are no longer needed
#
# The GPU toolkit is NOT included here: cuda.dockerfile and rocm.dockerfile are alternatives,
# so the top-level dockerfile picks one.
INCLUDE ./ros.dockerfile

# Then include the snippets for the modules you want to install
INCLUDE ./ch_ros.dockerfile
INCLUDE ./ch_vsg.dockerfile
INCLUDE ./ch_irrlicht.dockerfile
INCLUDE ./ch_vehicle.dockerfile
INCLUDE ./ch_sensor.dockerfile
INCLUDE ./ch_parser.dockerfile
INCLUDE ./ch_python.dockerfile
# SynChrono disabled in this image: its Fast-DDS backend conflicts with the
# ROS 2 Fast-DDS (see ch_synchrono.dockerfile) and its flatbuffers dependency
# does not build under GCC 15. Not needed for Chrono::ROS. Re-enable by
# uncommenting once SynChrono is updated for the new toolchain.
# INCLUDE ./ch_synchrono.dockerfile
