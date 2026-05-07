
echo Started build.sh

OS_NAME=$(uname)
if [ -d "./build" ]; then
    rm -rf ./build
fi
mkdir ./build
cd ./build


# ROS_SETUP_SCRIPT="$HOME/Packages/ros_ws/install/setup.sh"
# if [ -f "$ROS_SETUP_SCRIPT" ]; then
#   source $ROS_SETUP_SCRIPT
# fi


# Due to issues in https://github.com/conda-forge/vsgimgui-feedstock/issues/6, we need to build vsgImGui from source and link it statically.
# This is a workaround until the issue is resolved.
VSGIMGUI_SOURCE_DIR="$SRC_DIR/download_vsg/vsgImGui"
VSG_INSTALL_DIR="$SRC_DIR/contrib/build-scripts/vsg_build"

git clone -c advice.detachedHead=false --depth 1 --branch v0.7.0 "https://github.com/vsg-dev/vsgImGui" "$VSGIMGUI_SOURCE_DIR"
cmake -G "Ninja" -B build_vsgImGui -S ${VSGIMGUI_SOURCE_DIR} -DBUILD_SHARED_LIBS:BOOL=OFF
cmake --build build_vsgImGui --config Release
cmake --install build_vsgImGui --config Release --prefix ${VSG_INSTALL_DIR}


CONFIGURATION=Release

if [ "$(uname -m)" = x86_64 ]; then
    PARDISO_MKL_ENABLE="ON"
else
    PARDISO_MKL_ENABLE="OFF"
fi

CMAKE_PLATFORM_ARGS=""

if [ "$OS_NAME" = Darwin ]; then
    CMAKE_PLATFORM_ARGS="
     -DCMAKE_C_COMPILER=$(which clang)
     -DCMAKE_CXX_COMPILER=$(which clang++)
     -DCH_ENABLE_MODULE_SENSOR=OFF
    "

else
    CMAKE_PLATFORM_ARGS="
     -DCH_ENABLE_MODULE_FSI=ON
     -DCH_ENABLE_MODULE_FSI_SPH=ON
     -DCH_ENABLE_MODULE_FSI_TDPF=OFF
     -DCH_ENABLE_MODULE_SENSOR=ON
     -DCH_ENABLE_MODULE_ROS=OFF
     -DCH_ENABLE_MODULE_PARSERS=ON
     -DCH_USE_SENSOR_NVRTC=OFF
     -DCH_USE_SENSOR_OPTIX=ON
     -DCUDA_ARCH_NAME=Manual
     -DCHRONO_CUDA_ARCHITECTURES=75
     -DOptiX_INSTALL_DIR=$HOME/Packages/optix-dev-9.1.0
    "
fi


# Configure step
cmake -G "Ninja" -DCMAKE_INSTALL_PREFIX=$PREFIX \
 -DCMAKE_PREFIX_PATH=$PREFIX \
 -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
 -DCH_CONDA_INSTALL=ON \
 -DCH_INSTALL_PYTHON_PACKAGE=$SP_DIR \
 -DCH_PYCHRONO_DATA_PATH=../../../../share/chrono/data \
 -DCH_PYCHRONO_SHADER_PATH=../../../../share/chrono/sensor_shaders/ \
 -DPython3_ROOT_DIR=$PREFIX \
 -DCMAKE_BUILD_TYPE=$CONFIGURATION \
 -DCH_ENABLE_MODULE_IRRLICHT=ON \
 -DCH_ENABLE_MODULE_VSG=ON \
 -DCH_ENABLE_MODULE_POSTPROCESS=ON \
 -DCH_ENABLE_MODULE_VEHICLE=ON \
 -DCH_ENABLE_MODULE_PYTHON=ON \
 -DCH_ENABLE_MODULE_PARSERS=ON \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DCH_ENABLE_MODULE_CASCADE=OFF \
 -DCH_ENABLE_MODULE_PARDISO_MKL=$PARDISO_MKL_ENABLE \
 -DIrrlicht_ROOT=$PREFIX/include/irrlicht \
 -DvsgImGui_DIR=$VSG_INSTALL_DIR/lib/cmake/vsgImGui/ \
 $CMAKE_PLATFORM_ARGS ./..

#  -DCASCADE_INCLUDE_DIR=$HOME/miniconda3/include/opencascade \
#  -DCASCADE_LIBDIR=$HOME/miniconda3/lib \

# Build & Install
ninja
ninja install
