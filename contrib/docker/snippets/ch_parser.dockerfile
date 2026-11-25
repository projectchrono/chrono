# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Parser

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

# Keep dependency sources and build files out of the image layer.
RUN urdf_tmp="$(mktemp -d)" && \
    cd "${urdf_tmp}" && \
    bash -e "${CHRONO_DIR}/contrib/build-scripts/linux/buildURDF.sh" "${PACKAGE_DIR}/urdf" && \
    cd / && \
    rm -rf "${urdf_tmp}"

# Update shell config
RUN echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:${PACKAGE_DIR}/urdf/lib" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PARSERS=ON \
    -DCH_ENABLE_YAML=ON \
    -Durdfdom_DIR=${PACKAGE_DIR}/urdf/lib/urdfdom/cmake \
    -Durdfdom_headers_DIR=${PACKAGE_DIR}/urdf/lib/urdfdom_headers/cmake \
    -Dconsole_bridge_DIR=${PACKAGE_DIR}/urdf/lib/console_bridge/cmake \
    -Dtinyxml2_DIR=${PACKAGE_DIR}/urdf/CMake"