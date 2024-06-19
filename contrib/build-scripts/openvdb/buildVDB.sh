#!/bin/bash
# ---------------------------------------------------------------------------------------------------------
# Bash script for building OpenVDB from sources on Linux.
# ---------------------------------------------------------------------------------------------------------

# REQUIRED PACKAGES to install from apt
# sudo apt install libboost-iostreams-dev libtbb-dev libboost-iostreams-dev libblosc-dev

DOWNLOAD=ON

OPENVDB_INSTALL_DIR="$HOME/Packages/openvdb"
OPENVDB_DEPENDENCIES_DIR="$HOME/Packages/openvdb/openvdb-deps"

BUILDSHARED=ON
BUILDDEBUG=OFF

BUILDSYSTEM="Ninja Multi-Config"

# Allow overriding installation directory through command line argument
if [ "$1" != "" ]; then
   OPENVDB_INSTALL_DIR=$1
fi

if [ "$DOWNLOAD" = "ON" ]; then
    rm -rf download_openvdb 2>/dev/null
    mkdir -p download_openvdb

    # Download OpenVDB
    echo "Downloading OpenVDB 11.0.0 release from GitHub"
    wget https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v11.0.0.zip -O download_openvdb/openvdb.zip
    unzip download_openvdb/openvdb.zip -d download_openvdb
    OPENVDB_SOURCE_DIR="download_openvdb/openvdb-11.0.0"
fi

# Clean install directory
rm -rf ${OPENVDB_INSTALL_DIR} ${OPENVDB_DEPENDENCIES_DIR}
mkdir -p ${OPENVDB_INSTALL_DIR} ${OPENVDB_DEPENDENCIES_DIR}

# Build OpenVDB
rm -rf build_openvdb
mkdir build_openvdb
cmake -G "${BUILDSYSTEM}" -B build_openvdb -S ${OPENVDB_SOURCE_DIR} -DCMAKE_INSTALL_PREFIX=${OPENVDB_INSTALL_DIR} -DCMAKE_BUILD_TYPE=Release \
-DCMAKE_CXX_STANDARD=17 \
-DOPENVDB_BUILD_CORE=ON \
-DOPENVDB_CORE_SHARED=ON \
-DOPENVDB_BUILD_NANOVDB=ON \
-DUSE_NANOVDB=ON \
-DNANOVDB_USE_OPENVDB=ON \
-DNANOVDB_USE_CUDA=ON \
-DNANOVDB_CUDA_KEEP_PTX=ON \
-DNANOVDB_USE_TBB=ON \
-DNANOVDB_USE_ZLIB=ON \
-DUSE_BLOSC=ON \
-DUSE_TBB=ON \
-DUSE_ZLIB=OFF \
-DUSE_STATIC_DEPENDENCIES=OFF \
-DNANOVDB_BUILD_EXAMPLES=ON \

cmake --build build_openvdb --config Release
cmake --install build_openvdb --config Release

# Move NanoVDB Cuda header files to utils/cuda directory
UTIL_DIR="$HOME/Packages/openvdb/OpenVDB/include/nanovdb/util"
DEST_DIR="$HOME/Packages/openvdb/OpenVDB/include/nanovdb/util/cuda"

mv ${UTIL_DIR}/*.cuh ${DEST_DIR}
mv ${UTIL_DIR}/GpuTimer.h ${DEST_DIR}

# Delete downloaded sources and build directories
echo "Deleting build directories"
rm -rf build_tbb
rm -rf build_zlib
rm -rf build_blosc
rm -rf build_openvdb