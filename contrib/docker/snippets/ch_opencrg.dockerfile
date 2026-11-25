# SPDX-License-Identifier: MIT
# Add OpenCRG road support to Chrono::Vehicle. Include ch_vehicle first.

ARG CHRONO_DIR
ARG PACKAGE_DIR

RUN sudo apt update && \
    sudo apt install --no-install-recommends -y unzip && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Keep the helper's Release and Debug static libraries. Run it in a temporary
# directory so downloaded sources and build files do not remain in the layer.
RUN opencrg_tmp="$(mktemp -d)" && \
    cd "${opencrg_tmp}" && \
    bash -e "${CHRONO_DIR}/contrib/build-scripts/linux/buildOpenCRG.sh" "${PACKAGE_DIR}/opencrg" && \
    test -s "${PACKAGE_DIR}/opencrg/include/crgBaseLib.h" && \
    test -s "${PACKAGE_DIR}/opencrg/lib/libOpenCRG.1.1.2.a" && \
    test -s "${PACKAGE_DIR}/opencrg/lib/libOpenCRG.1.1.2_d.a" && \
    cd / && \
    rm -rf "${opencrg_tmp}"

# OpenCRG has no CMake package config; provide the header and library paths.
# The libraries are static, so no runtime library path entry is needed.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_OPENCRG=ON \
    -DOpenCRG_INCLUDE_DIR=${PACKAGE_DIR}/opencrg/include \
    -DOpenCRG_LIBRARY=${PACKAGE_DIR}/opencrg/lib/libOpenCRG.1.1.2.a"
