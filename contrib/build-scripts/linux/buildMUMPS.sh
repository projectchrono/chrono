#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building MUMPS using the scivision/mumps CMake project.
# - Requires git, cmake, a Fortran compiler, and the Intel openAPI HPC toolkit (for MKL).
#   NOTE: Intel oneAPI 2025 does *not* work with Eigen! Use version 2023.
# - Place in an arbitrary temporary directory.
# - Specify the install directory.
# - Run the script (sh ./buildMUMPS.sh).
# - The install directory will contain (under subdirectories of MUMPS_INSTALL_DIR/cmake) the MUMPS CMake
#   project configuration script required to configure Chrono with the Chrono::MUMPS module enabled.
# - After installation, run `sudo ldconfig ${MUMPS_INSTALL_DIR}/lib` or else update the LD_LIBRARY_PATH
#   environment variable.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - We suggest using Ninja (ninja-build.org/) and the "Ninja Multi-Config" CMake generator.
#   (otherwise, you will need to explicitly set the CMAKE_BUILD_TYPE variable)
# - This script builds MUMPS with settings appropriate for use in Chrono:
#   - single and double precision support, no complex support
#   - build static MUMPS libraries
#   - OpenMP support enabled
#   - no MPI support
#   Modify the CMake configuration below to enable other options (e.g., alternative ordering algorithms).
# -------------------------------------------------------------------------------------------------------

MUMPS_INSTALL_DIR="$HOME/Packages/mumps"

# Build MUMPS debug libraries
BUILD_DEBUG=ON

BUILD_SYSTEM="Ninja Multi-Config"

# Allow overriding installation directory through command line argument
if [ $# -eq 1 ]
then
    MUMPS_INSTALL_DIR=$1
fi

echo "----------------------------- Download sources from GitHub"

rm -rf download_mumps
mkdir download_mumps

# Note: use custom fork of scivision/mumps
git clone "https://github.com/projectchrono/mumps.git" download_mumps

rm -rf ${MUMPS_INSTALL_DIR}
mkdir ${MUMPS_INSTALL_DIR}

echo -e "\n------------------------ Configure mumps\n"
rm -rf build_mumps
cmake -G "${BUILD_SYSTEM}" -B build_mumps -S download_mumps \
      -DBUILD_SINGLE=on \
      -DBUILD_DOUBLE=on \
      -DBUILD_SHARED_LIBS=OFF \
      -DMUMPS_openmp=on \
      -DMUMPS_parallel=off \
      -DCMAKE_DEBUG_POSTFIX=_d \
      --install-prefix ${MUMPS_INSTALL_DIR}

echo -e "\n------------------------ Build and install mumps\n"
cmake --build build_mumps --config Release
cmake --install build_mumps --config Release --prefix ${MUMPS_INSTALL_DIR}
if [ ${BUILD_DEBUG} = ON ]
then
    cmake --build build_mumps --config Debug
    cmake --install build_mumps --config Debug --prefix ${MUMPS_INSTALL_DIR}
else
    echo "No Debug build of mumps"
fi
