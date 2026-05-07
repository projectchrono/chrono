#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building a standalone FastDDS 2.4.0 installation safe for use with Chrono::SynChrono,
# even when ROS 2 is installed on the same machine (or sourced in the current shell).
#
# Builds three components in order:
#   1. foonathan_memory  v0.7-3
#   2. Fast-CDR          v1.0.24
#   3. Fast-DDS          v2.4.0
#
# All three are installed to a single prefix (FASTDDS_INSTALL_DIR).
# The build is performed inside a clean sub-shell (env -i) so that ROS 2
# environment variables (AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH, LD_LIBRARY_PATH)
# cannot cause CMake to discover the ROS-bundled FastDDS/FastCDR.
#
# Usage:
#   sh ./buildFastDDS.sh [INSTALL_DIR]
#
# The resulting install directory is what you pass to Chrono's CMake:
#   -DFastDDS_ROOT=/path/to/fastrtps-2.4.0
#
# Notes:
# - Requires cmake, git, ninja-build, and a C++ compiler.
# - The script accepts 1 optional argument to override the install directory.
# - This script is intentionally compatible with ROS 2 Humble, Jazzy, etc.
#   being sourced at the time of invocation.
# -------------------------------------------------------------------------------------------------------

DOWNLOAD=ON

FASTDDS_INSTALL_DIR="$HOME/Packages/fastrtps-2.4.0"

BUILDSYSTEM="Ninja"

if [ ${DOWNLOAD} = OFF ]
then
    FOONATHAN_SOURCE_DIR="$HOME/Sources/foonathan_memory"
    FASTCDR_SOURCE_DIR="$HOME/Sources/Fast-CDR"
    FASTDDS_SOURCE_DIR="$HOME/Sources/Fast-DDS"
fi

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument

if [ $# -eq 1 ]
then
    FASTDDS_INSTALL_DIR=$1
fi

# ------------------------------------------------------------------------
# Sanitize environment: strip any ROS / ament paths from key variables so
# that CMake cannot accidentally find the ROS-bundled FastDDS.  We preserve
# the rest of the user's environment (compiler, locale, etc.).

_clean_path() {
    # Remove path components containing /opt/ros/ or ament
    echo "$1" | tr ':' '\n' | grep -v -E '/opt/ros/|ament' | paste -sd ':' -
}

export CMAKE_PREFIX_PATH="$FASTDDS_INSTALL_DIR"
export PATH="$(_clean_path "$PATH")"
# Keep LD_LIBRARY_PATH clean for the build; runtime RPATH handles loading.
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH

echo "============================================"
echo "Building standalone FastDDS 2.4.0"
echo "Install directory: ${FASTDDS_INSTALL_DIR}"
echo "============================================"

# ------------------------------------------------------------------------

if [ ${DOWNLOAD} = ON ]
then
    echo "Download sources from GitHub"

    rm -rf download_fastdds
    mkdir download_fastdds

    echo "  ... foonathan_memory"
    git clone -c advice.detachedHead=false --depth 1 --branch v0.7-3 \
        "https://github.com/foonathan/memory.git" "download_fastdds/foonathan_memory"
    FOONATHAN_SOURCE_DIR="download_fastdds/foonathan_memory"

    echo "  ... Fast-CDR"
    git clone -c advice.detachedHead=false --depth 1 --branch v1.0.24 \
        "https://github.com/eProsima/Fast-CDR.git" "download_fastdds/Fast-CDR"
    FASTCDR_SOURCE_DIR="download_fastdds/Fast-CDR"

    echo "  ... Fast-DDS"
    git clone -c advice.detachedHead=false --depth 1 --branch v2.4.0 \
        "https://github.com/eProsima/Fast-DDS.git" "download_fastdds/Fast-DDS"
    FASTDDS_SOURCE_DIR="download_fastdds/Fast-DDS"
else
    echo "Using provided source directories"
fi

echo -e "\nSources in:"
echo "  " ${FOONATHAN_SOURCE_DIR}
echo "  " ${FASTCDR_SOURCE_DIR}
echo "  " ${FASTDDS_SOURCE_DIR}
echo ""

# ------------------------------------------------------------------------

rm -rf ${FASTDDS_INSTALL_DIR}
mkdir -p ${FASTDDS_INSTALL_DIR}

# --- foonathan_memory ---------------------------------------------------

echo "============================================"
echo "Building foonathan_memory..."
echo "============================================"

cmake -G "${BUILDSYSTEM}" -B "${FOONATHAN_SOURCE_DIR}/build" -S "${FOONATHAN_SOURCE_DIR}" \
    -DCMAKE_INSTALL_PREFIX:PATH="${FASTDDS_INSTALL_DIR}" \
    -DFOONATHAN_MEMORY_BUILD_TESTS:BOOL=OFF \
    -DFOONATHAN_MEMORY_BUILD_TOOLS:BOOL=ON \
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON \
    -DCMAKE_BUILD_TYPE:STRING=Release

cmake --build "${FOONATHAN_SOURCE_DIR}/build" -j$(nproc)
cmake --install "${FOONATHAN_SOURCE_DIR}/build"

# --- Fast-CDR -----------------------------------------------------------

echo "============================================"
echo "Building Fast-CDR..."
echo "============================================"

cmake -G "${BUILDSYSTEM}" -B "${FASTCDR_SOURCE_DIR}/build" -S "${FASTCDR_SOURCE_DIR}" \
    -DCMAKE_INSTALL_PREFIX:PATH="${FASTDDS_INSTALL_DIR}" \
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON \
    -DCMAKE_BUILD_TYPE:STRING=Release

cmake --build "${FASTCDR_SOURCE_DIR}/build" -j$(nproc)
cmake --install "${FASTCDR_SOURCE_DIR}/build"

# --- Fast-DDS (fastrtps) ------------------------------------------------

echo "============================================"
echo "Building Fast-DDS..."
echo "============================================"

cmake -G "${BUILDSYSTEM}" -B "${FASTDDS_SOURCE_DIR}/build" -S "${FASTDDS_SOURCE_DIR}" \
    -DCMAKE_INSTALL_PREFIX:PATH="${FASTDDS_INSTALL_DIR}" \
    -DCMAKE_PREFIX_PATH:PATH="${FASTDDS_INSTALL_DIR}" \
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DTHIRDPARTY:BOOL=ON \
    -DSECURITY:BOOL=OFF

cmake --build "${FASTDDS_SOURCE_DIR}/build" -j$(nproc)
cmake --install "${FASTDDS_SOURCE_DIR}/build"

# ------------------------------------------------------------------------

echo ""
echo "============================================"
echo "FastDDS 2.4.0 installed to: ${FASTDDS_INSTALL_DIR}"
echo ""
echo "To use with Chrono::SynChrono, add to your CMake configuration:"
echo "  -DCH_ENABLE_MODULE_SYNCHRONO=ON"
echo "  -DCH_USE_SYNCHRONO_FASTDDS=ON"
echo "  -DFastDDS_ROOT=${FASTDDS_INSTALL_DIR}"
echo "============================================"
