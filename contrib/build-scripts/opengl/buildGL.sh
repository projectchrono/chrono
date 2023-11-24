#!/bin/bash

# ---------------------------------------------------------------------------------------------------------
# Windows batch script for building GL from sources.
# - Requires cmake, wget, and unzip
# - Place in an arbitrary temporary directory.
# - Specify the locations for the GL sources OR indicate that these should be downloaded.
# - Specify the install directory.
# - Decide whether to build shared or static libraries and whether to also build debug libraries.
# - Run the script (sh ./buildGL.sh)
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - Do *not* use GLEW from its GitHub repository. It is not properly set up to work with CMake. Use instead
#   the source archive available on SourceForge.
# - The sources for GLM and GLFW can be obtained either from GitHub or from SourceForge.
# - This was tested with the following versions of VSG libraries:
#      GLEW (glew.sourceforge.net/):     Version 2.1.0
#      GLFW (github.com/glfw/glfw):      Commit (#8f470597)
#      GLM (github.com/g-truc/glm.git):  Commit (#efec5db0)                  
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
    GLM_SOURCE_DIR="$HOME/Sources/glm"
    GLEW_SOURCE_DIR="$HOME/Sources/glew"
    GLFW_SOURCE_DIR="$HOME/Sources/glfw"
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
    echo "Download sources from sourceforge.net"

    rm -rf download_gl
    mkdir download_gl
    
    echo "  ... GLEW"
    wget https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.zip -O download_gl/glew.zip
    unzip -q download_gl/glew.zip -d download_gl
    GLEW_SOURCE_DIR="download_gl/glew-2.1.0"

    echo "  ... GLFW"
    wget https://sourceforge.net/projects/glfw/files/glfw/3.3.8/glfw-3.3.8.zip -O download_gl/glfw.zip
    unzip -q download_gl/glfw.zip -d download_gl
    GLFW_SOURCE_DIR="download_gl/glfw-3.3.8"
    
    echo "  ... GLM"
    wget https://sourceforge.net/projects/glm.mirror/files/0.9.9.8/glm-0.9.9.8.zip -O download_gl/glm.zip
    unzip -q download_gl/glm.zip -d download_gl
    GLM_SOURCE_DIR="download_gl/glm"
else
    echo "Using provided source directories"
fi

echo -e "\nSources in:"
echo "  "  ${GLEW_SOURCE_DIR}
echo "  "  ${GLFW_SOURCE_DIR}
echo "  "  ${GLM_SOURCE_DIR}

# ------------------------------------------------------------------------

rm -rf ${GL_INSTALL_DIR}
mkdir ${GL_INSTALL_DIR}

# --- GLM --------------------------------------------------------------------

echo -e "\n------------------------ Install GLM\n"
mkdir ${GL_INSTALL_DIR}/include
cp -r ${GLM_SOURCE_DIR}/glm ${GL_INSTALL_DIR}/include/

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

# --- GLFW -------------------------------------------------------------------

echo -e "\n------------------------ Configure GLFW\n"
rm -rf build_glfw
mkdir build_glfw
cmake -G "${BUILDSYSTEM}" -B build_glfw -S ${GLFW_SOURCE_DIR} \
      -DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} \
      -DCMAKE_DEBUG_POSTFIX=_d

echo -e "\n------------------------ Build and install GLFW\n"
cmake --build build_glfw --config Release
cmake --install build_glfw --config Release --prefix ${GL_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_glfw --config Debug
    cmake --install build_glfw --config Debug --prefix ${GL_INSTALL_DIR}
else
    echo "No Debug build"
fi
