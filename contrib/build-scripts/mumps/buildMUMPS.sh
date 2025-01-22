#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building MUMPS using the scivision/mumps CMake project.
# - Requires git, cmake, and the Intel openAPI HPC toolkit.
#   NOTE: Intel oneAPI 2025 does *not* work with Eigen!
# - Place in an arbitrary temporary directory.
# - Specify the install directory.
# - Decide whether to build shared or static libraries and whether to also build debug libraries.
# - Run the script (sh ./buildMUMPS.sh).
# - The install directory will contain (under subdirectories of MUMPS_INSTALL_DIR/cmake) the MUMPS CMake
#   project configuration script required to configure Chrono with the Chrono::MUMPS module enabled.
# - After installation, run `sudo ldconfig` (if installing to a system directory) or else update the
#   LD_LIBRARY_PATH (if installing to a custom directory).
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - We suggest using Ninja (ninja-build.org/) and the "Ninja Multi-Config" CMake generator.
#   (otherwise, you will need to explicitly set the CMAKE_BUILD_TYPE variable)
# - This script build MUMPS with settings appropriate for use in Chrono
#   (in particular, with OpenMP support enabled and no MPI support).
#   Modify the CMake configuration below to enable other options (e.g., alternative ordering algorithms).
# -------------------------------------------------------------------------------------------------------

MUMPS_INSTALL_DIR="/usr/local/mumps"

BUILDSHARED=ON
BUILDDEBUG=ON
BUILDSYSTEM="Ninja Multi-Config"

# Allow overriding installation directory through command line argument
if [ $# -eq 1 ]
then
    MUMPS_INSTALL_DIR=$1
fi

echo "----------------------------- Download sources from GitHub"

rm -rf download_mumps
mkdir download_mumps

git clone "https://github.com/scivision/mumps.git" download_mumps

echo -e "\n------------------------ Configure mumps\n"
rm -rf build_mumps
cmake -G "${BUILDSYSTEM}" -B build_mumps -S download_mumps \
      -DBUILD_SINGLE=on \
      -DBUILD_DOUBLE=on \
      -DBUILD_SHARED_LIBS=${BUILDSHARED} \
      -DMUMPS_openmp=on \
      -DMUMPS_parallel=off \
      -DCMAKE_DEBUG_POSTFIX=_d \
      --install-prefix ${MUMPS_INSTALL_DIR}

echo -e "\n------------------------ Build and install mumps\n"
cmake --build build_mumps --config Release
sudo cmake --install build_mumps --config Release --prefix ${MUMPS_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_mumps --config Debug
    sudo cmake --install build_mumps --config Debug --prefix ${MUMPS_INSTALL_DIR}
else
    echo "No Debug build of mumps"
fi
