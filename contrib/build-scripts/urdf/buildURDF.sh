#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Bash script for building and installing URDF parser dependencies
# - Place in an arbitrary temporary directory.
# - Specify the locations for the URDF sources OR indicate that these should be downloaded.
# - Specify the install directory.
# - Run the script (sh ./buildURDF.sh).
# - The install directory will contain subdirectories for all necessary dependencies.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# -------------------------------------------------------------------------------------------------------

DOWNLOAD=ON

URDF_INSTALL_DIR="$HOME/Packages/urdf"

BUILDDEBUG=ON
BUILDSYSTEM="Ninja Multi-Config"

if [ ${DOWNLOAD} = OFF ]
then
    TINYXML2_SOURCE_DIR="$HOME/Sources/tinyxml2"
    CONSOLE_BRIDGE_SOURCE_DIR="$HOME/Sources/console_bridge"
    URDFDOM_HEADERS_SOURCE_DIR="$HOME/Sources/urdfdom_headers"
    URDFDOM_SOURCE_DIR="$HOME/Sources/urdfdom"
fi

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument

if [ $# -eq 1 ]
then
    URDF_INSTALL_DIR=$1
fi

# ------------------------------------------------------------------------

if [ ${DOWNLOAD} = ON ]
then
    echo "Download sources from GitHub"

    rm -rf download_urdf
    mkdir download_urdf

    echo "  ... tinyxml2"
    git clone "https://github.com/leethomason/tinyxml2.git" "download_urdf/tinyxml2"
    TINYXML2_SOURCE_DIR="download_urdf/tinyxml2"

    echo "  ... console_bridge"
    git clone "https://github.com/ros/console_bridge.git" "download_urdf/console_bridge"
    CONSOLE_BRIDGE_SOURCE_DIR="download_urdf/console_bridge"

    echo "  ... urdfdom_headers"
    git clone "https://github.com/ros/urdfdom_headers.git" "download_urdf/urdfdom_headers"
    URDFDOM_HEADERS_SOURCE_DIR="download_urdf/urdfdom_headers"
    
    echo "  ... urdfdom"
    git clone -c advice.detachedHead=false --depth 1 --branch scpeters/tinyxml2 "https://github.com/rserban/urdfdom.git" "download_urdf/urdfdom"
    #git clone "https://github.com/ros/urdfdom.git" "download_urdf/urdfdom"
    URDFDOM_SOURCE_DIR="download_urdf/urdfdom"
else
    echo "Using provided source directories"
fi

echo -e "\nSources in:"
echo "  "  ${TINYXML2_SOURCE_DIR}
echo "  "  ${CONSOLE_BRIDGE_SOURCE_DIR}
echo "  "  ${URDFDOM_HEADERS_SOURCE_DIR}
echo "  "  ${URDFDOM_SOURCE_DIR}

# ------------------------------------------------------------------------

rm -rf ${URDF_INSTALL_DIR}
mkdir ${URDF_INSTALL_DIR}

# --- tinyxml2 -------------------------------------------------------------
#
# Note that tinixml2 cannot be built with "Ninja Multi-Config".
# Only configure and build a Release tinixml2 library.
# Pass "-fPIC" to allow linking in a shared library.
#

echo -e "\n------------------------ Configure tinyxml2\n"
rm -rf build_tinyxml2
cmake -E env CXXFLAGS="-fPIC" cmake -G "Ninja" -B build_tinyxml2 -S ${TINYXML2_SOURCE_DIR} \
      -DCMAKE_BUILD_TYPE="Release" \
      -Dtinyxml2_INSTALL_CMAKEDIR:PATH="CMake"

echo -e "\n------------------------ Build and install tinyxml2\n"
cmake --build build_tinyxml2
cmake --install build_tinyxml2 --prefix ${URDF_INSTALL_DIR}

# --- console_bridge ----------------------------------------------------------------

echo -e "\n------------------------ Configure console_bridge\n"
rm -rf build_console_bridge
cmake -G "${BUILDSYSTEM}" -B build_console_bridge -S ${CONSOLE_BRIDGE_SOURCE_DIR}  \
      -DCMAKE_BUILD_TYPE="Release" \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd

echo -e "\n------------------------ Build and install console_bridge\n"
cmake --build build_console_bridge --config Release
cmake --install build_console_bridge --config Release --prefix ${URDF_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_console_bridge --config Debug
    cmake --install build_console_bridge --config Debug --prefix ${URDF_INSTALL_DIR}
else
    echo "No Debug build of console_bridge"
fi

# --- urdfdom_headers ---------------------------------------------------------

echo -e "\n------------------------ Configure urdfdom_headers\n"
rm -rf build_urdfdom_headers
cmake  -G "${BUILDSYSTEM}" -B build_urdfdom_headers -S ${URDFDOM_HEADERS_SOURCE_DIR}  \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd

echo -e "\n------------------------ Build and install urdfdom_headers\n"
cmake --build build_urdfdom_headers --config Release
cmake --install build_urdfdom_headers --config Release --prefix ${URDF_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_urdfdom_headers --config Debug
    cmake --install build_urdfdom_headers --config Debug --prefix ${URDF_INSTALL_DIR}
else
    echo "No Debug build of urdfdom_headers"
fi

# --- urdfdom -----------------------------------------------------------

echo -e "\n------------------------ Configure urdfdom\n"
rm -rf build_urdfdom
cmake -G "${BUILDSYSTEM}" -B build_urdfdom -S ${URDFDOM_SOURCE_DIR} \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd \
      -DDISABLE_TINYXML_SUPPORT:BOOL=ON \
      -Dtinyxml2_DIR:PATH=${URDF_INSTALL_DIR}/CMake \
      -Dconsole_bridge_DIR:PATH=${URDF_INSTALL_DIR}/lib/console_bridge/cmake \
      -Durdfdom_headers_DIR:PATH=${URDF_INSTALL_DIR}/lib/urdfdom_headers/cmake

echo -e "\n------------------------ Build and install urdfdom\n"
cmake --build build_urdfdom --config Release
cmake --install build_urdfdom --config Release --prefix ${URDF_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_urdfdom --config Debug
    cmake --install build_urdfdom --config Debug --prefix ${URDF_INSTALL_DIR}
else
    echo "No Debug build of urdfdom"
fi
