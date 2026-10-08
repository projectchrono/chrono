# SPDX-License-Identifier: MIT
# This snippet install Chrono in ${PACKAGE_DIR}/chrono
# It includes other snippets, where specific modules can be added or removed based on need

ARG CHRONO_BRANCH="main"
ARG CHRONO_REPO="https://github.com/projectchrono/chrono.git"
ARG CHRONO_DIR="${USERHOME}/chrono"
ARG CHRONO_INSTALL_DIR="${USERHOME}/packages/chrono"
ARG PACKAGE_DIR="${USERHOME}/packages"
RUN mkdir -p ${PACKAGE_DIR}

# This variable will be used by snippets to add cmake options
ENV CMAKE_OPTIONS=""
# This variable is used before building (but in the same RUN command)
# This is useful for setting environment variables that are used in the build process
ENV PRE_BUILD_SCRIPTS=""

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
INCLUDE ./cuda.dockerfile
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


# Install Chrono
#
# CHRONO_CUDA_ARCHITECTURES is declared here rather than with the other ARGs at the top of
# the file on purpose: a build arg invalidates the build cache from its declaration onward,
# even for instructions that never read it, and everything above this point (the CUDA
# toolkit, ROS, the VSG build and the OptiX SDK) is expensive to rebuild.
#
# `docker build` runs with no GPU visible, so CMake resolves the vendor from the installed
# SDK, reports a cross-target build and falls back to a fat binary covering every major
# architecture. Empty keeps that default, which is the portable choice and costs little
# build time; what it costs is binary size. Set this to a concrete compute capability to
# target one GPU. Never set it to "native" -- with no GPU visible that is a FATAL_ERROR.
ARG CHRONO_CUDA_ARCHITECTURES=""
RUN ${PRE_BUILD_SCRIPTS} && \
    # Evaluate the cmake options to expand any $(...) commands or variables
    eval "_CMAKE_OPTIONS=\"${CMAKE_OPTIONS}\"" && \
    mkdir ${CHRONO_DIR}/build && \
    cd ${CHRONO_DIR}/build && \
    cmake ../ -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_DEMOS=OFF \
        -DBUILD_BENCHMARKING=OFF \
        -DBUILD_TESTING=OFF \
        -DCMAKE_LIBRARY_PATH=$(find /usr/local/cuda/ -type d -name stubs) \
        -DEigen3_DIR=/usr/share/eigen3/cmake \
        -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_DIR} \
        -DCHRONO_CUDA_ARCHITECTURES="${CHRONO_CUDA_ARCHITECTURES}" \
        ${_CMAKE_OPTIONS} \
        && \
    ninja && ninja install


# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${CHRONO_INSTALL_DIR}/lib" >> ${USERSHELLPROFILE}
