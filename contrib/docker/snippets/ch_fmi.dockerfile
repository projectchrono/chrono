# SPDX-License-Identifier: MIT
# Enable Chrono::FMI import support. FMU export requires a separate static
# Chrono build, so keep it disabled in this shared-library image.

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_FMI=ON \
    -DCH_ENABLE_FMU_EXPORT=OFF"
