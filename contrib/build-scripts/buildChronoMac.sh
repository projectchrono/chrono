#!/bin/bash

# -----------------------------------------------------------------------------------------
# Bash script for configuring and building Chrono on MacOS
# -----------------------------------------------------------------------------------------
# The structure of the install directories for VSG and GL libraries is assumed to be as
# created by running the buildVSG and buildGL scripts. If using alternative installations
# of these libraries, modify as appropriate the CMake variables with paths to the various
# project configuration scripts.
#
# On the Mac some libraries should be better installed by the homebrew packet manager:
#  Libraries:
#     - eigen
#     - irrlicht
#     - libomp
#
# Following library should ***NOT*** be installed by homebrew for compatibility reasons:
#     - opencascade
#     - spectra
#
# Tools:
#     - gnuplot
#     - ninja (optional)
#     - cmake
#
# Unavailable on the Mac:
#     - mkl
#     - fastrtps
#     - cuda
#     - optix
#     - csharp
#     - GPU
#     - FSI
#     - CSHARP
#     - COSIMULATION
#     - DISTRIBUTED
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
#    Dependencies for Chrono modules that are disabled need not be set.
# 3. As needed, modify the CMake generator (BUILDSYSTEM). We suggest using Ninja
#    (ninja-build.org/) and the "Ninja Multi-Config" CMake generator. Otherwise, you will
#    need to explicitly set the CMAKE_BUILD_TYPE variable).
# 4. Edit the CMake command to disable selected Chrono modules.
# 5. Run the script (sh ./buildChrono.sh).
# -----------------------------------------------------------------------------------------

SOURCE_DIR="."
BUILD_DIR="./build"
INSTALL_DIR="$HOME/install/chrono"

EIGEN3_INSTALL_DIR="$HOMEBREW_PREFIX/include/eigen3"
BLAZE_INSTALL_DIR="/Users/builder/dependencies/blaze-3.8"
THRUST_INSTALL_DIR="/Users/builder/dependencies/thrust"
CASCADE_INSTALL_DIR="/usr/local/include/opencascade"
# Spectra directory is located inside /usr/local/include
SPECTRA_INSTALL_DIR="/usr/local/include"
CRG_INSTALL_DIR="/Users/builder/dependencies/OpenCRG"

IRRLICHT_INSTALL_DIR="$HOMEBREW_PREFIX/include/irrlicht"
VSG_INSTALL_DIR="$HOME/Soft/Packs/vsg"

SWIG_EXE="swig"

rm -rf $BUILD_DIR

# ------------------------------------------------------------------------

BUILDSYSTEM="Ninja Multi-Config"
#BUILDSYSTEM=Xcode

# ------------------------------------------------------------------------

cmake -G "${BUILDSYSTEM}" -B ${BUILD_DIR} -S ${SOURCE_DIR} \
      -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR} \
      -DBUILD_BENCHMARKING:BOOL=OFF \
      -DBUILD_TESTING:BOOL=ON \
      -DENABLE_MODULE_POSTPROCESS:BOOL=ON \
      -DENABLE_MODULE_IRRLICHT:BOOL=ON \
      -DENABLE_MODULE_MULTICORE:BOOL=ON \
      -DENABLE_MODULE_VEHICLE:BOOL=ON \
      -DENABLE_OPENCRG:BOOL=ON \
      -DOpenMP_CXX_FLAGS:STRING="-Xclang -fopenmp -I$HOMEBREW_PREFIX/Cellar/libomp/15.0.7/include" \
      -DOpenMP_C_FLAGS:STRING="-Xclang -fopenmp" \
      -DOpenMP_C_INCLUDE_DIR:PATH="-L$HOMEBREW_PREFIX/opt/libomp/lib" \
      -DOpenMP_CXX_INCLUDE_DIR:PATH="$HOMEBREW_PREFIX/include" \
      -DOpenMP_C_LIB_NAMES:STRING=libomp \
      -DOpenMP_CXX_LIB_NAMES:STRING=libomp \
      -DOpenMP_libomp_LIBRARY:FILEPATH="$HOMEBREW_PREFIX/opt/libomp/lib/libomp.dylib" \
      -DTHRUST_INCLUDE_DIR:PATH=${THRUST_INSTALL_DIR} \
      -DBLAZE_INSTALL_DIR:PATH=${BLAZE_INSTALL_DIR} \
      -DENABLE_MODULE_VSG:BOOL=OFF \
      -DENABLE_MODULE_CASCADE:BOOL=ON \
      -DENABLE_MODULE_MODAL:BOOL=ON \
      -DIRRLICHT_INSTALL_DIR:PATH=${IRRLICHT_INSTALL_DIR} \
      -DOpenCRG_INCLUDE_DIR:PATH=${CRG_INSTALL_DIR}/include \
      -DOpenCRG_LIBRARY:FILEPATH=${CRG_INSTALL_DIR}/lib/libOpenCRG.a \
      -DOpenCASCADE_DIR:PATH=${CASCADE_INSTALL_DIR}/lib/cmake/opencascade \
      -DSPECTRA_INCLUDE_DIR:PATH=${SPECTRA_INSTALL_DIR} \
      -DENABLE_MODULE_PYTHON:BOOL=ON \
      -DPYTHON_EXECUTABLE:PATH=HOMEBREW_PREFIX/bin \
	-DPYTHON_INCLUDE_DIR:PATH=$HOMEBREW_PREFIX/Cellar/python@$BREW_PY_MAJOR/$BREW_PY_VER/Frameworks/Python.framework/Versions/$BREW_PY_MAJOR/include/python$BREW_PY_MAJOR \
	-DPYTHON_LIBRARY:PATH=$HOMEBREW_PREFIX/Cellar/python@$BREW_PY_MAJOR/$BREW_PY_VER/Frameworks/Python.framework/Versions/$BREW_PY_MAJOR/lib/python$BREW_PY_MAJOR/config-$BREW_PY_MAJOR-darwin/libpython$BREW_PY_MAJOR.dylib \
      -DENABLE_MODULE_CSHARP:BOOL=ON
 
# ------------------------------------------------------------------------

# --- Build Chrono modules (command line)

#cmake --build ${BUILD_DIR} --config Release
#cmake --build ${BUILD_DIR} --config Debug

# --- Build Chrono modules (IDE users)
# --- open Xcode IDE and build from there, select scheme "ALL_BUILD"

#cd ${BUILD_DIR}
#open Chrono.xcodeproj

