#!/bin/bash
mkdir -p $HOME/Desktop
cd $HOME/Desktop
git clone https://github.com/projectchrono/chrono.git
cd chrono
git submodule init
git submodule update

# ------------------------------------------------------------------------

SOURCE_DIR="$HOME/Desktop/chrono"
BUILD_DIR="$HOME/Desktop/chrono/build"
INSTALL_DIR="$HOME/Desktop/chrono/install"

EIGEN3_INSTALL_DIR="/Packages/eigen-3.4.0"
BLAZE_INSTALL_DIR="/Packages/blaze-3.8"
SPECTRA_INSTALL_DIR="/Packages/spectra"

CRG_INCLUDE_DIR="/Packages/OpenCRG/include"
CRG_LIBRARY="/Packages/OpenCRG/lib/libOpenCRG.1.1.2.a"

IRRLICHT_INSTALL_DIR="/Packages/irrlicht-1.8.5"
VSG_INSTALL_DIR="/Packages/vsg"
GL_INSTALL_DIR="/Packages/gl"

OPTIX_INSTALL_DIR="/Packages/optix-7.5.0"
FASTRTPS_INSTALL_DIR="/Packages/fastrtps-2.4.0"

SWIG_EXE="swig"

# ------------------------------------------------------------------------

BUILDSYSTEM="Ninja Multi-Config"

# ------------------------------------------------------------------------

cmake -G ${BUILDSYSTEM} -B ${BUILD_DIR} -S ${SOURCE_DIR} \
      -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR} \
      -DENABLE_MODULE_IRRLICHT:BOOL=ON \
      -DENABLE_MODULE_VSG:BOOL=OFF \
      -DENABLE_MODULE_OPENGL:BOOL=ON \
      -DENABLE_MODULE_VEHICLE:BOOL=ON \
      -DIOMP5_LIBRARY=${IOMP5_DIR} \
      -DENABLE_MODULE_POSTPROCESS:BOOL=ON \
      -DENABLE_MODULE_MULTICORE:BOOL=ON \
      -DENABLE_MODULE_FSI:BOOL=ON \
      -DENABLE_MODULE_GPU:BOOL=ON \
      -DENABLE_MODULE_DISTRIBUTED:BOOL=ON \
      -DENABLE_MODULE_PARDISO_MKL:BOOL=OFF \
      -DENABLE_MODULE_CASCADE:BOOL=OFF \
      -DENABLE_MODULE_COSIMULATION:BOOL=ON \
      -DENABLE_MODULE_SENSOR:BOOL=ON \
      -DENABLE_MODULE_MODAL:BOOL=ON \
      -DENABLE_MODULE_MATLAB:BOOL=OFF \
      -DENABLE_MODULE_CSHARP:BOOL=ON \
      -DENABLE_MODULE_PYTHON:BOOL=ON \
      -DENABLE_MODULE_SYNCHRONO:BOOL=OFF \
      -DBUILD_BENCHMARKING:BOOL=OFF \
      -DBUILD_TESTING:BOOL=OFF \
      -DENABLE_OPENCRG:BOOL=ON \
      -DUSE_CUDA_NVRTC:BOOL=ON \
      -DUSE_FAST_DDS:BOOL=ON \
      -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INSTALL_DIR} \
      -DBLAZE_INSTALL_DIR:PATH=${BLAZE_INSTALL_DIR} \
      -DOptiX_INSTALL_DIR:PATH=${OPTIX_INSTALL_DIR} \
      -Dfastrtps_INSTALL_DIR:PATH=${FASTRTPS_INSTALL_DIR} \
      -DGLEW_DIR=${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glew \
      -Dglfw3_DIR=${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glfw3 \
      -DGLM_INCLUDE_DIR:PATH=${GL_INSTALL_DIR}/include \
      -DOpenCRG_INCLUDE_DIR:PATH=${CRG_INCLUDE_DIR} \
      -DOpenCRG_LIBRARY:FILEPATH=${CRG_LIBRARY} \
      -DOpenCASCADE_DIR:PATH=${CASCADE_INSTALL_DIR}/adm \
      -DSPECTRA_INCLUDE_DIR:PATH=${SPECTRA_INSTALL_DIR}/include \
      -DMATLAB_SDK_ROOT:PATH=${MATLAB_INSTALL_DIR}/extern \
      -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsg \
      -DvsgImGui_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsgImGui \
      -DvsgXchange_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsgXchange \
      -DSWIG_EXECUTABLE:FILEPATH=${SWIG_EXE} \
      -DCMAKE_BUILD_TYPE="Release" \
      -DCUDA_ARCH_NAME=All

cmake --build ${BUILD_DIR} --config Release
cd build
ninja install
