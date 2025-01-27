#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building MUMPS using the scivision/mumps CMake project on MacOS
# - Requires git, cmake, ninja, gfortran
# - Place in an arbitrary temporary directory.
# - Specify the install directory.
# - Decide whether to build shared or static libraries and whether to also build debug libraries.
# - Run the script (sh ./buildMUMPS_Mac.sh).
# - The install directory will contain (under subdirectories of MUMPS_INSTALL_DIR/cmake) the MUMPS CMake
#   project configuration script required to configure Chrono with the Chrono::MUMPS module enabled.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - We suggest using Ninja (ninja-build.org/) and the "Ninja" CMake generator.
#   We take two separate configurations, id desired to allow the MUMPS test routines to run. All tests
#   should run properly!
# - This script build MUMPS with settings appropriate for use in Chrono
#   (in particular, with OpenMP support enabled and no MPI support).
#   Modify the CMake configuration below to enable other options (e.g., alternative ordering algorithms).
# -------------------------------------------------------------------------------------------------------

MUMPS_INSTALL_DIR="/usr/local/mumps"

if [ "$(uname)" = "Darwin" -a -x "$(which gfortran)" ] ; then
	echo "System and Fortran Compiler ok."
else
	echo "Your Mac doesn't have a gfortran compiler."
	echo "You can install gfortran whith homebrew like:"
	echo "brew install gfortran"
	exit 1
fi

BUILDSHARED=ON
BUILDDEBUG=ON
BUILDSYSTEM="Ninja"

# Allow overriding installation directory through command line argument
if [ $# -eq 1 ]
then
    MUMPS_INSTALL_DIR=$1
fi

SOURCE=download_mumps
BUILD=build_mumps

rm -rf ${BUILD} ${SOURCE}

git clone "https://github.com/scivision/mumps.git" ${SOURCE}

cmake -S ${SOURCE} -B${BUILD} -G "${BUILDSYSTEM}" \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=${MUMPS_INSTALL_DIR} \
	-DMUMPS_parallel:BOOL=OFF \
	-DMUMPS_openmp:BOOL=OFF \
	-DBUILD_COMPLEX:BOOL=ON \
	-DBUILD_COMPLEX16:BOOL=ON \
	-DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} 

cmake --build ${BUILD} 
cmake --build ${BUILD} --target test
sudo cmake --install ${BUILD}

if [ "${BUILDDEBUG}" = "ON" ] ; then
	rm -rf ${BUILD}
	cmake -S ${SOURCE} -B${BUILD} -G "${BUILDSYSTEM}" \
		-DCMAKE_BUILD_TYPE=Debug \
		-DCMAKE_INSTALL_PREFIX=${MUMPS_INSTALL_DIR} \
		-DMUMPS_parallel:BOOL=OFF \
		-DMUMPS_openmp:BOOL=OFF \
		-DBUILD_COMPLEX:BOOL=ON \
		-DBUILD_COMPLEX16:BOOL=ON \
		-DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} \
		-DCMAKE_DEBUG_POSTFIX=_d
	
	cmake --build ${BUILD} 
	cmake --build ${BUILD} --target test
	sudo cmake --install ${BUILD}
fi