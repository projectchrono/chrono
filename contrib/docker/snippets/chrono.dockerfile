# SPDX-License-Identifier: MIT
# This snippet install Chrono in ${PACKAGE_DIR}/chrono
# It includes other snippets, where specific modules can be added or removed based on need

ARG CHRONO_BRANCH="feature/modern_cmake"
ARG CHRONO_REPO="https://github.com/projectchrono/chrono.git"
ARG CHRONO_DIR="${USERHOME}/chrono"
ARG CHRONO_INSTALL_DIR="${USERHOME}/packages/chrono"
ARG PACKAGE_DIR="${USERHOME}/packages"
RUN mkdir -p ${PACKAGE_DIR}

# This variable will be used by snippets to add cmake options
ENV CMAKE_OPTIONS=""
# This variable is used before building (but in the same RUN command)
# This is useful for setting environment variables that are used in the build process
ENV PRE_BUILD_COMMANDS=""

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
        freeglut3-dev \
        python3-numpy \
        libglu1-mesa-dev \
        libglew-dev \
        libglfw3-dev \
        libblas-dev \
        liblapack-dev \
        wget \
        xorg-dev && \
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

# Install Chrono
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
        -DEigen3_DIR=/usr/lib/cmake/eigen3 \
        -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_DIR} \
        -DNUMPY_INCLUDE_DIR=$(python3 -c 'import numpy; print(numpy.get_include())') \
        ${_CMAKE_OPTIONS} \
        && \
    ninja && ninja install


# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${CHRONO_INSTALL_DIR}/lib" >> ${USERSHELLPROFILE}