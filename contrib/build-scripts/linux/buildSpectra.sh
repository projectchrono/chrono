#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building and installing Spectra.
# - Requires git, cmake, and ninja.
# - Place in an arbitrary temporary directory.
# - Specify the install directory.
# - Run the script (sh ./buildSpectra.sh).
# - The install directory will contain subdirectories for all necessary dependencies.
#
# Notes:
# - Chrono uses the Krylov-Schur solver from Spectra, available *only* in the Spectra development branch.
# - The script accepts 1 optional argument to override the install directory.
# - If ninja is not available, set BUILD_SYSTEM="Unix Makefiles" or some other generator
# -------------------------------------------------------------------------------------------------------

SPECTRA_INSTALL_DIR="$HOME/Packages/spectra"

BUILD_SYSTEM="Ninja"

# Allow overriding installation directory through command line argument
if [ $# -eq 1 ]
then
    SPECTRA_INSTALL_DIR=$1
fi

echo "----------------------------- Download sources from GitHub"

rm -rf download_spectra
mkdir download_spectra

git clone -c advice.detachedHead=false --depth 1 --branch develop "https://github.com/yixuan/spectra.git" "download_spectra"

rm -rf ${SPECTRA_INSTALL_DIR}
mkdir ${SPECTRA_INSTALL_DIR}

echo -e "\n------------------------ Configure Spectra\n"
rm -rf build_spectra
cmake -G "${BUILD_SYSTEM}" -B build_spectra -S download_spectra

echo -e "\n------------------------ Build and install Spectra\n"
# cmake --build build_spectra --config Release
cmake --install build_spectra --config Release --prefix ${SPECTRA_INSTALL_DIR}
