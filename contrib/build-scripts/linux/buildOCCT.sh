#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building OpenCASCADE based on the last official releases.
# - Requires cmake, wget, and unzip
# - Place in an arbitrary temporary directory.
# - Specify the locations for the OpenCASCADE sources OR indicate that these should be downloaded.
# - Specify the install directory.
# - Decide whether to build shared or static libraries and whether to also build debug libraries.
# - Run the script (sh ./buildOCCT.sh).
# - The install directory will contain (under subdirectories of CASCADE_INSTALL_DIR/lib/shared) all VSG CMake
#   project configuration scripts required to configure Chrono with the Chorno::VSG module enabled.
#
# -------------------------------------------------------------------------------------------------------

DOWNLOAD=ON

CASCADE_INSTALL_DIR="$HOME/Packages/opencascade-7.4.0"

BUILDSHARED=ON
BUILDDEBUG=OFF
BUILDSYSTEM="Ninja Multi-Config"

if [ ${DOWNLOAD} = OFF ]
then    
    OCCT_SOURCE_DIR="$HOME/Sources/opencascade"
fi

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument

if [ $# -eq 1 ]
then
    CASCADE_INSTALL_DIR=$1
fi

# ------------------------------------------------------------------------

if [ ${DOWNLOAD} = ON ]
then
    echo "Download sources from GitHub"

    rm -rf OCCT-7_4_0
    mkdir OCCT-7_4_0

    wget -q https://codeload.github.com/Open-Cascade-SAS/OCCT/tar.gz/refs/tags/V7_4_0 -O OCCT-7.4.0.tar.gz
    tar xzf OCCT-7.4.0.tar.gz
    OCCT_SOURCE_DIR=OCCT-7_4_0

else
    echo "Using provided source directories"
fi

echo -e "\nSources in:"
echo "  "  ${OCCT_SOURCE_DIR}

# ------------------------------------------------------------------------

rm -rf ${CASCADE_INSTALL_DIR}
mkdir ${CASCADE_INSTALL_DIR}

# --- OCCT ----------------------------------------------------------------

echo -e "\n------------------------ Configure OCCT\n"
rm -rf build_occt
cmake -G "${BUILDSYSTEM}" \
      -B build_occt \
      -S ${OCCT_SOURCE_DIR} \
      -DCMAKE_INSTALL_PREFIX=${CASCADE_INSTALL_DIR}

echo -e "\n------------------------ Build and install OCCT\n"
cmake --build build_occt --config Release
cmake --install build_occt --config Release
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_occt --config Debug
    cmake --install build_occt --config Debug
else
    echo "No Debug build of OCCT"
fi

# Fix the bug mentioned here: https://github.com/clearlinux/distribution/issues/3116
sed -i -e 's/\\\${OCCT_INSTALL_BIN_LETTER}//' ${CASCADE_INSTALL_DIR}/lib/cmake/opencascade/*