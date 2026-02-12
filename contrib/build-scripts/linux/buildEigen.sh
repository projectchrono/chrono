#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building and installing Eigen3.
# - Requires git, cmake, and ninja.
# - Place in an arbitrary temporary directory.
# - Specify the install directory.
# - Run the script (sh ./buildEigen.sh).
# - The install directory will contain subdirectories for all necessary dependencies.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - If ninja is not available, set BUILD_SYSTEM="Unix Makefiles" or some other generator
# -------------------------------------------------------------------------------------------------------

EIGEN_INSTALL_DIR="$HOME/Packages/eigen"
EIGEN_VERSION="5.0.0"
#EIGEN_VERSION="3.4.0"

BUILD_SYSTEM="Ninja"

# Allow overriding installation directory through command line argument
if [ $# -eq 1 ]
then
    EIGEN_INSTALL_DIR=$1
fi

echo "----------------------------- Download sources from GitLab"

rm -rf download_eigen
mkdir download_eigen

git clone -c advice.detachedHead=false --depth 1 --branch ${EIGEN_VERSION} "https://gitlab.com/libeigen/eigen.git" "download_eigen"

rm -rf ${EIGEN_INSTALL_DIR}
mkdir ${EIGEN_INSTALL_DIR}

echo -e "\n------------------------ Configure Eigen\n"
rm -rf build_eigen
cmake -G "${BUILD_SYSTEM}" -B build_eigen -S download_eigen

echo -e "\n------------------------ Build and install Eigen\n"
cmake --build build_eigen --config Release
cmake --install build_eigen --config Release --prefix ${EIGEN_INSTALL_DIR}
