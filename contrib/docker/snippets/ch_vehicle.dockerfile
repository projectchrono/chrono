# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::Vehicle

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_VEHICLE=ON"