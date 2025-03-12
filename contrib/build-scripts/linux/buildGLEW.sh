#!/bin/bash

# ---------------------------------------------------------------------------------------------------------
# Shell batch script for building GLEW from source.
# - Requires cmake, wget, and unzip
# - Place in an arbitrary temporary directory.
# - Specify the locations for the GLEW source OR indicate that it should be downloaded.
# - Specify the install directory.
# - Decide whether to build shared or static libraries and whether to also build debug libraries.
# - Run the script (sh ./buildGLEW.sh)
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - Do *not* use GLEW from its GitHub repository. It is not properly set up to work with CMake. Use instead
#   the source archive available on SourceForge.
# - This was tested with the following version:
#      GLEW (glew.sourceforge.net/):     Version 2.2.0                  
# - We suggest using Ninja (ninja-build.org/) and the "Ninja Multi-Config" CMake generator.
#   (otherwise, you will need to explicitly set the CMAKE_BUILD_TYPE variable)
# ---------------------------------------------------------------------------------------------------------

DOWNLOAD=ON

GL_INSTALL_DIR="$HOME/Packages/gl"

BUILDSHARED=ON
BUILDDEBUG=OFF
BUILDSYSTEM="Ninja Multi-Config"

if [ ${DOWNLOAD} = OFF ]
then    
    GLEW_SOURCE_DIR="$HOME/Sources/glew"
fi

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument

if [ $# -eq 1 ]
then
    GL_INSTALL_DIR=$1
fi

# ------------------------------------------------------------------------

if [ ${DOWNLOAD} = ON ]
then
    echo "Download source from sourceforge.net"

    rm -rf download_gl
    mkdir download_gl
    
    echo "  ... GLEW"
    wget https://sourceforge.net/projects/glew/files/glew/2.2.0/glew-2.2.0.zip -O download_gl/glew.zip
    unzip -q download_gl/glew.zip -d download_gl
    GLEW_SOURCE_DIR="download_gl/glew-2.2.0"
else
    echo "Using provided source directory"
fi

echo -e "\nSource in:"
echo "  "  ${GLEW_SOURCE_DIR}

# ------------------------------------------------------------------------

rm -rf ${GL_INSTALL_DIR}
mkdir ${GL_INSTALL_DIR}

# --- GLEW -------------------------------------------------------------------

echo -e "\n------------------------ Configure GLEW\n"
rm -rf build_glew
mkdir build_glew
cmake -G "${BUILDSYSTEM}" -B build_glew -S ${GLEW_SOURCE_DIR}/build/cmake \
      -DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} \
      -DCMAKE_DEBUG_POSTFIX=_d

echo -e "\n------------------------ Build and install GLEW\n"
cmake --build build_glew --config Release
cmake --install build_glew --config Release --prefix ${GL_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_glew --config Debug
    cmake --install build_glew --config Debug --prefix ${GL_INSTALL_DIR}
else
    echo "No Debug build"
fi
