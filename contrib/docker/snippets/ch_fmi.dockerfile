# SPDX-License-Identifier: MIT
# This snippet enables Chrono::FMI (FMI 2.0/3.0 co-simulation interface).
#
# Note on FMI support in this Docker image:
# The Chrono::FMI installation guide explains that full FMU export support is
# easiest to satisfy with static Chrono libraries. It explicitly instructs users
# to configure Chrono with BUILD_SHARED_LIBRARIES=OFF and currently recommends
# not enabling GPU-based Chrono modules in an FMI export build. This Docker image
# is a full-featured shared-library image and also enables CUDA-based modules
# such as Chrono::Sensor, Chrono::FSI-SPH, and Chrono::DEM, so full FMU export
# support is intentionally left disabled here.
#
# This mirrors the current Chrono CMake code path:
#
#   - src/chrono_fmi/CMakeLists.txt:
#       * enables the module with CH_ENABLE_MODULE_FMI
#       * disables FMU_EXPORT_SUPPORT automatically when BUILD_SHARED_LIBS is ON
#       * uses the in-tree src/chrono_thirdparty/fmu-forge by default
#       * still builds the Chrono_fmi library for FMU import support
#   - src/CMakeLists.txt:
#       * defaults BUILD_SHARED_LIBS to ON
#       * adds chrono_fmi before the other optional modules
#   - cmake/ChronoConfig.cmake.in:
#       * exposes Chrono_FMI_AVAILABLE and fmu-forge headers to consumers
#
# Make the shared-library intent explicit by setting FMU_EXPORT_SUPPORT=OFF.
# Users who need FMU export should use a separate static Chrono configuration
# with the module set recommended by the Chrono::FMI installation guide.

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_FMI=ON \
    -DFMU_EXPORT_SUPPORT=OFF"
