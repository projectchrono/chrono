# SPDX-License-Identifier: MIT
# This snippet configures, builds and installs Chrono into ${CHRONO_INSTALL_DIR}.
# Include it last, after chrono.dockerfile and after every snippet that appends to
# CMAKE_OPTIONS -- it consumes the accumulated value.

# The architecture lists are declared here rather than with the other ARGs at the top of
# chrono.dockerfile on purpose: a build arg invalidates the build cache from its declaration
# onward, even for instructions that never read it, and everything above this point (the GPU
# toolkit, ROS, the VSG build and the OptiX SDK) is expensive to rebuild.
#
# `docker build` runs with no GPU visible, so CMake reports a cross-target build and cannot
# detect what to compile for. The two backends differ in what that costs:
#
#   CUDA  Empty falls back to a fat binary covering every major architecture -- portable, and
#         it costs binary size rather than build time.
#   HIP   There is no all-major equivalent. Empty leaves CMAKE_HIP_ARCHITECTURES unset and the
#         result is whatever the toolchain defaults to, so set this to the target's gfx name.
#
# Never set either to "native": with no GPU visible that is a configure-time FATAL_ERROR.
ARG CHRONO_CUDA_ARCHITECTURES=""
ARG CHRONO_HIP_ARCHITECTURES=""
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
        -DEigen3_DIR=/usr/share/eigen3/cmake \
        -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_DIR} \
        -DCHRONO_CUDA_ARCHITECTURES="${CHRONO_CUDA_ARCHITECTURES}" \
        -DCHRONO_HIP_ARCHITECTURES="${CHRONO_HIP_ARCHITECTURES}" \
        ${_CMAKE_OPTIONS} \
        && \
    ninja && ninja install


# Update shell config
RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${CHRONO_INSTALL_DIR}/lib" >> ${USERSHELLPROFILE}
