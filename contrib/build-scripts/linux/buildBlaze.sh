#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building and installing Blaze.
# - Requires git, cmake, and ninja.
# - Place in an arbitrary temporary directory.
# - Specify the install directory.
# - Run the script (sh ./buildBlaze.sh).
# - The install directory will contain subdirectories for all necessary dependencies.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - If ninja is not available, set BUILD_SYSTEM="Unix Makefiles" or some other generator
# -------------------------------------------------------------------------------------------------------

BLAZE_INSTALL_DIR="$HOME/Packages/blaze"

BUILD_SYSTEM="Ninja"

# Allow overriding installation directory through command line argument
if [ $# -eq 1 ]
then
    BLAZE_INSTALL_DIR=$1
fi

echo "----------------------------- Download sources from Bitbucket"

rm -rf download_blaze
mkdir download_blaze

git clone -c advice.detachedHead=false --depth 1 --branch v3.8.2 "https://bitbucket.org/blaze-lib/blaze.git" "download_blaze"

rm -rf ${BLAZE_INSTALL_DIR}
mkdir ${BLAZE_INSTALL_DIR}

echo -e "\n------------------------ Configure Blaze\n"
rm -rf build_blaze
cmake -G "${BUILD_SYSTEM}" -B build_blaze -S download_blaze

echo -e "\n------------------------ Build and install Blaze\n"
# cmake --build build_blaze --config Release
cmake --install build_blaze --config Release --prefix ${BLAZE_INSTALL_DIR}
