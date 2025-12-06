# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Parser

ARG CHRONO_DIR
ARG PACKAGE_DIR
ARG USERSHELLPROFILE

# Build the URDF dependencies
RUN cd ${CHRONO_DIR}/contrib/build-scripts/linux/ && \
    bash buildURDF.sh ${PACKAGE_DIR}/urdf

# Update shell config
RUN echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:${PACKAGE_DIR}/urdf/lib" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PARSERS=ON \
    -Durdfdom_DIR=${PACKAGE_DIR}/urdf/lib/urdfdom/cmake \
    -Durdfdom_headers_DIR=${PACKAGE_DIR}/urdf/lib/urdfdom_headers/cmake \
    -Dconsole_bridge_DIR=${PACKAGE_DIR}/urdf/lib/console_bridge/cmake \
    -Dtinyxml2_DIR=${PACKAGE_DIR}/urdf/CMake"