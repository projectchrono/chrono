#!/bin/bash

# -----------------------------------------------------------------------------------------
# Bash script for configuring and building Chrono.
# -----------------------------------------------------------------------------------------
# The structure of the install directories for VSG and GL libraries is assumed to be as
# created by running the buildVSG and buildGL scripts. If using alternative installations
# of these libraries, modify as appropriate the CMake variables with paths to the various
# project configuration scripts.
# -----------------------------------------------------------------------------------------
# 1. As needed, use the provided scripts to build and install various dependencies.
# 2. Copy this script in an arbitrary location, outside the Chrono source tree.
# 3. Edit this script to specify the installation directories for the various dependencies.
#    Dependencies for disabled Chrono modules are ignored.
# 4. As needed, modify the CMake generator (BUILDSYSTEM). We suggest using Ninja
#    (ninja-build.org/) and the "Ninja Multi-Config" CMake generator. Otherwise, you will
#    need to explicitly set the CMAKE_BUILD_TYPE variable).
# 5. Edit the CMake command to enable/disable Chrono modules.
# 6. Run the script (sh ./buildChrono.sh).
# -----------------------------------------------------------------------------------------

SOURCE_DIR="$HOME/Source/chrono"
BUILD_DIR="$HOME/Build/chrono"
INSTALL_DIR="$HOME/Install/chrono"

EIGEN3_INSTALL_DIR="$HOME/Packages/eigen-3.4.0"
BLAZE_ROOT="$HOME/Packages/blaze-3.8"
CASCADE_INSTALL_DIR="$HOME/Packages/opencascade-7.4.0"
SPECTRA_INCLUDE_DIR="$HOME/Packages/spectra/include"

CRG_INCLUDE_DIR="${HOME}/Packages/OpenCRG/include"
CRG_LIBRARY="$HOME/Packages/OpenCRG/lib/libOpenCRG.1.1.2.a"

IRRLICHT_ROOT="$HOME/Packages/irrlicht-1.8.5"
VSG_INSTALL_DIR="$HOME/Packages/vsg"
GL_INSTALL_DIR="$HOME/Packages/gl"

URDF_INSTALL_DIR="C:/Packages/urdf"

OPTIX_INSTALL_DIR="$HOME/Packages/optix-7.5.0"
FASTDDS_INSTALL_DIR="$HOME/Packages/fastrtps-2.4.0"

SWIG_EXE="swig"

# ------------------------------------------------------------------------

BUILDSYSTEM="Ninja Multi-Config"

# ------------------------------------------------------------------------

cmake -G ${BUILDSYSTEM} -B ${BUILD_DIR} -S ${SOURCE_DIR} \
      -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR} \
      -DCH_ENABLE_MODULE_IRRLICHT:BOOL=ON \
      -DCH_ENABLE_MODULE_VSG:BOOL=OFF \
      -DCH_ENABLE_MODULE_OPENGL:BOOL=ON \
      -DCH_ENABLE_MODULE_VEHICLE:BOOL=ON \
      -DIOMP5_LIBRARY=${IOMP5_DIR} \
      -DCH_ENABLE_MODULE_POSTPROCESS:BOOL=ON \
      -DCH_ENABLE_MODULE_MULTICORE:BOOL=ON \
      -DCH_ENABLE_MODULE_FSI:BOOL=ON \
      -DCH_ENABLE_MODULE_GPU:BOOL=ON \
      -DCH_ENABLE_MODULE_PARDISO_MKL:BOOL=ON \
      -DCH_ENABLE_MODULE_CASCADE:BOOL=ON \
      -DCH_ENABLE_MODULE_COSIMULATION:BOOL=ON \
      -DCH_ENABLE_MODULE_SENSOR:BOOL=ON \
      -DCH_ENABLE_MODULE_MODAL:BOOL=ON \
      -DCH_ENABLE_MODULE_MATLAB:BOOL=OFF \
      -DCH_ENABLE_MODULE_CSHARP:BOOL=ON \
      -DCH_ENABLE_MODULE_PYTHON:BOOL=ON \
      -DCH_ENABLE_MODULE_SYNCHRONO:BOOL=OFF \
      -DBUILD_BENCHMARKING:BOOL=ON \
      -DBUILD_TESTING:BOOL=ON \
      -DCH_ENABLE_OPENCRG:BOOL=ON \
      -DCH_USE_CUDA_NVRTC:BOOL=OFF \
      -DCH_USE_FAST_DDS:BOOL=ON \
      -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INSTALL_DIR} \
      -DIrrlicht_ROOT:PATH=${IRRLICHT_ROOT} \
      -DBlaze_ROOT:PATH=${BLAZE_ROOT} \
      -DOptiX_INSTALL_DIR:PATH=${OPTIX_INSTALL_DIR} \
      -FastDDS_ROOT:PATH=${FASTDDS_INSTALL_DIR} \
      -DGLEW_DIR=${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glew \
      -Dglfw3_DIR=${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glfw3 \
      -DGLM_INCLUDE_DIR:PATH=${GL_INSTALL_DIR}/include \
      -DOpenCRG_INCLUDE_DIR:PATH=${CRG_INCLUDE_DIR} \
      -DOpenCRG_LIBRARY:FILEPATH=${CRG_LIBRARY} \
      -DOpenCASCADE_DIR:PATH=${CASCADE_INSTALL_DIR}/${LIB_DIR}/cmake/opencascade \
      -DSpectra_INCLUDE_DIR:PATH=${SPECTRA_INCLUDE_DIR}/include \
      -DMATLAB_SDK_ROOT:PATH=${MATLAB_INSTALL_DIR}/extern \
      -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsg \
      -DvsgImGui_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsgImGui \
      -DvsgXchange_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsgXchange \
      -Durdfdom_DIR:PATH=${URDF_INSTALL_DIR}/lib/urdfdom/cmake \
      -Durdfdom_headers_DIR:PATH=${URDF_INSTALL_DIR}/lib/urdfdom_heders/cmake \
      -Dconsole_bridge_DIR:PATH=${URDF_INSTALL_DIR}/lib/console_bridge/cmake \
      -Dtinyxml2_DIR:PATH=${URDF_INSTALL_DIR}/CMake \
      -DSWIG_EXECUTABLE:FILEPATH=${SWIG_EXE} \
      -DCMAKE_BUILD_TYPE="Release"

# ------------------------------------------------------------------------

# cmake --build ${BUILD_DIR} --config Release
# cmake --build ${BUILD_DIR} --config Debug
