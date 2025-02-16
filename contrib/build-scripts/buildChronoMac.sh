#!/bin/bash

# -----------------------------------------------------------------------------------------
# Bash script for configuring and building Chrono on MacOS
# -----------------------------------------------------------------------------------------
# The structure of the install directories for VSG and GL libraries is assumed to be as
# created by running the buildVSG and buildGL scripts. If using alternative installations
# of these libraries, modify as appropriate the CMake variables with paths to the various
# project configuration scripts.
#
# On the Mac, some libraries are better installed by the homebrew packet manager:
#     - eigen
#     - irrlicht
#     - libomp
#
# The following libraries should *NOT* be installed by homebrew for compatibility reasons:
#     - opencascade
#     - spectra
#
# Tools:
#     - gnuplot (optional)
#     - ninja (optional)
#     - cmake
#
# Unavailable on the Mac:
#     - mkl (Apple Silicon)
#     - fastrtps
#     - cuda
#     - optix
#     - csharp
#     - GPU
#     - FSI
#     - CSHARP
#     - COSIMULATION
#     - SENSOR
#     - SYNCHRONO
#     - MATLAB (Apple Silicon)
#     - MODAL
#     - PARDISO_MKL (Apple Silicon)
#     - PARDISO_PROJECT (Apple Silicon)
#     - OPENGL (broken due to unsupported shader programs)
# 
# -----------------------------------------------------------------------------------------
# 1. As needed, use the provided scripts to build and install various dependencies.
# 2. Edit this script to specify the installation directories for the various dependencies.
#    Dependencies for Chrono modules that are disabled are ignored.
# 3. As needed, modify the CMake generator (BUILDSYSTEM). We suggest using Ninja
#    (ninja-build.org/) and the "Ninja Multi-Config" CMake generator. Otherwise, you will
#    need to explicitly set the CMAKE_BUILD_TYPE variable).
# 4. Edit the CMake command to disable selected Chrono modules.
# 5. Run the script (sh ./buildChrono.sh).
# -----------------------------------------------------------------------------------------

SOURCE_DIR="$HOME/Source/chrono"
BUILD_DIR="$HOME/Build/chrono"
INSTALL_DIR="$HOME/Install/chrono"

EIGEN3_INSTALL_DIR="$HOMEBREW_PREFIX/include/eigen3"
IRRLICHT_ROOT="$HOMEBREW_PREFIX/include/irrlicht"

BLAZE_ROOT="$HOME/Packages/blaze-3.8"
THRUST_INCLUDE_DIR="$HOME/Packages/thrust"
CRG_INSTALL_DIR="$HOME/Packages/OpenCRG"
VSG_INSTALL_DIR="$HOME/Packages/vsg"

URDF_INSTALL_DIR="C:/Packages/urdf"

CASCADE_INSTALL_DIR="/usr/local/include/opencascade"
SPECTRA_INCLUDE_DIR="/usr/local/include"

SWIG_EXE="swig"

# ------------------------------------------------------------------------

BUILDSYSTEM="Ninja Multi-Config"
#BUILDSYSTEM=Xcode

# ------------------------------------------------------------------------

rm -rf $BUILD_DIR

cmake -G "${BUILDSYSTEM}" -B ${BUILD_DIR} -S ${SOURCE_DIR} \
      -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR} \
      -DBUILD_BENCHMARKING:BOOL=ON \
      -DBUILD_TESTING:BOOL=ON \
      -DCH_ENABLE_MODULE_POSTPROCESS:BOOL=ON \
      -DCH_ENABLE_MODULE_IRRLICHT:BOOL=ON \
      -DCH_ENABLE_MODULE_MULTICORE:BOOL=ON \
      -DCH_ENABLE_MODULE_MODAL:BOOL=ON \
      -DCH_ENABLE_MODULE_VEHICLE:BOOL=ON \
      -DCH_ENABLE_MODULE_VSG:BOOL=OFF \
      -DCH_ENABLE_MODULE_CASCADE:BOOL=ON \
      -DCH_ENABLE_MODULE_MODAL:BOOL=ON \
      -DCH_ENABLE_MODULE_PYTHON:BOOL=ON \
      -DCH_ENABLE_MODULE_CSHARP:BOOL=ON \
      -DCH_ENABLE_OPENCRG:BOOL=ON \
      -DOpenMP_CXX_FLAGS:STRING="-Xclang -fopenmp -I$HOMEBREW_PREFIX/Cellar/libomp/15.0.7/include" \
      -DOpenMP_C_FLAGS:STRING="-Xclang -fopenmp" \
      -DOpenMP_C_INCLUDE_DIR:PATH="-L$HOMEBREW_PREFIX/opt/libomp/lib" \
      -DOpenMP_CXX_INCLUDE_DIR:PATH="$HOMEBREW_PREFIX/include" \
      -DOpenMP_C_LIB_NAMES:STRING=libomp \
      -DOpenMP_CXX_LIB_NAMES:STRING=libomp \
      -DOpenMP_libomp_LIBRARY:FILEPATH="$HOMEBREW_PREFIX/opt/libomp/lib/libomp.dylib" \
      -DTHRUST_INCLUDE_DIR:PATH=${THRUST_INCLUDE_DIR} \
      -DBlaze_ROOT:PATH=${BLAZE_ROOT} \
      -DIrrlicht_ROOT:PATH=${IRRLICHT_ROOT} \
      -DOpenCRG_INCLUDE_DIR:PATH=${CRG_INSTALL_DIR}/include \
      -DOpenCRG_LIBRARY:FILEPATH=${CRG_INSTALL_DIR}/lib/libOpenCRG.a \
      -DOpenCASCADE_DIR:PATH=${CASCADE_INSTALL_DIR}/lib/cmake/opencascade \
      -DSpectra_INCLUDE_DIR:PATH=${SPECTRA_INCLUDE_DIR} \
      -Durdfdom_DIR:PATH=${URDF_INSTALL_DIR}/CMake \
      -Durdfdom_headers_DIR:PATH=${URDF_INSTALL_DIR}/CMake \
      -Dconsole_bridge_DIR:PATH=${URDF_INSTALL_DIR}/CMake \
      -DPYTHON_EXECUTABLE:PATH=$HOMEBREW_PREFIX/bin \
      -DPYTHON_INCLUDE_DIR:PATH=$HOMEBREW_PREFIX/Cellar/python@$BREW_PY_MAJOR/$BREW_PY_VER/Frameworks/Python.framework/Versions/$BREW_PY_MAJOR/include/python$BREW_PY_MAJOR \
      -DPYTHON_LIBRARY:PATH=$HOMEBREW_PREFIX/Cellar/python@$BREW_PY_MAJOR/$BREW_PY_VER/Frameworks/Python.framework/Versions/$BREW_PY_MAJOR/lib/python$BREW_PY_MAJOR/config-$BREW_PY_MAJOR-darwin/libpython$BREW_PY_MAJOR.dylib

 
# ------------------------------------------------------------------------

# --- Build Chrono modules (command line)

#cmake --build ${BUILD_DIR} --config Release
#cmake --build ${BUILD_DIR} --config Debug

# --- Build Chrono modules (IDE users)
# --- open Xcode IDE and build from there, select scheme "ALL_BUILD"

#cd ${BUILD_DIR}
#open Chrono.xcodeproj

