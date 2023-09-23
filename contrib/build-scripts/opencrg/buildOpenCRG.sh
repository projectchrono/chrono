#!/bin/bash

# ------------------------------------------------------------------------
# Windows batch script for building OpenCRG from 1.1.2 source.
# - Place in an arbitrary temporary directory.
# - Edit the lines below to point to the location of the OpenCRG-1.1.2
#   source and the desired install directory.
# - Run the script (sh ./buildOpenCRG.sh).
# - As provided, this script generates both Release and Debug libraries.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# ------------------------------------------------------------------------

DOWNLOAD=ON

CRG_INSTALL_DIR="$HOME/Packages/openCRG"

if [ ${DOWNLOAD} = OFF ]
then    
    CRG_SOURCE_DIR="$HOME/Sources/OpenCRG-1.1.2"
    REV=1.1.2
fi

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument

if [ $# -eq 1 ]
then
    CRG_INSTALL_DIR=$1
fi

# ------------------------------------------------------------------------

# Download sources
if [ ${DOWNLOAD} = ON ]
then
    echo "Download source from GitHub"

    rm -rf download_crg
    mkdir download_crg
    
    wget https://github.com/hlrs-vis/opencrg/archive/refs/tags/v1.1.2.zip -O download_crg/crg.zip
    unzip -q download_crg/crg.zip -d download_crg

    CRG_SOURCE_DIR="../download_crg/opencrg-1.1.2"
    REV=1.1.2
else
    echo "Using provided source directories"
fi

echo "Sources in: " ${CRG_SOURCE_DIR}

# ------------------------------------------------------------------------

# Prepare install directory
rm -rf ${CRG_INSTALL_DIR}
mkdir ${CRG_INSTALL_DIR}
mkdir ${CRG_INSTALL_DIR}/include
mkdir ${CRG_INSTALL_DIR}/lib

# ------------------------------------------------------------------------

COMP=gcc
AR=ar
VERBOSE=

# Prepare build directory
rm -rf build_crg
mkdir build_crg

cd build_crg

# Release build
rm -f *.o
echo -e "${COMP} ${VERBOSE} -Wall -O3 -fPIC -I${CRG_SOURCE_DIR}/inc -c ${CRG_SOURCE_DIR}/src/*.c"
${COMP} ${VERBOSE} -Wall -O3 -fPIC -I${CRG_SOURCE_DIR}/inc -c ${CRG_SOURCE_DIR}/src/*.c
${AR} ${VERBOSE} -r ${CRG_INSTALL_DIR}/lib/libOpenCRG.${REV}.a *.o

# Debug build
rm -f *.o
echo -e "${COMP} ${VERBOSE} -Wall -ggdb -fPIC -I${CRG_SOURCE_DIR}/inc -c ${CRG_SOURCE_DIR}/src/*.c"
${COMP} ${VERBOSE} -Wall -ggdb -fPIC -I${CRG_SOURCE_DIR}/inc -c ${CRG_SOURCE_DIR}/src/*.c
${AR} ${VERBOSE} -r ${CRG_INSTALL_DIR}/lib/libOpenCRG.${REV}_d.a *.o

# Copy headers
cp -r ${CRG_SOURCE_DIR}/inc/* ${CRG_INSTALL_DIR}/include/

cd ..
