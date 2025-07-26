
echo Started build.sh
if [ -d "./build" ]; then
    rm -rf ./build
fi
mkdir ./build
cd ./build

# Python libraries are different file types for MacOS and linux
# TODO: Check if this is needed since MacOS has its own deployment script

if [ `uname` == Darwin ]; then
    PY_LIB="libpython${PY_VER}.dylib"
else
    PY_LIB="libpython${PY_VER}.so"
fi

ROS_SETUP_SCRIPT="$HOME/Packages/ros_ws/install/setup.sh"
if [ -f "$ROS_SETUP_SCRIPT" ]; then
  source $ROS_SETUP_SCRIPT
fi

# set MKL vars
export MKL_INTERFACE_LAYER=LP64
export MKL_THREADING_LAYER=INTEL

CONFIGURATION=Release
# Configure step
cmake -G "Ninja" -DCMAKE_INSTALL_PREFIX=$PREFIX \
 -DCMAKE_PREFIX_PATH=$PREFIX \
 -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
 -DCH_CONDA_INSTALL=ON \
 -DCH_INSTALL_PYTHON_PACKAGE=$SP_DIR \
 -DCH_PYCHRONO_DATA_PATH=../../../../share/chrono/data \
 -DCH_PYCHRONO_SHADER_PATH=../../../../lib/sensor_ptx \
 -DPython3_ROOT_DIR=$PREFIX \
 -DCMAKE_BUILD_TYPE=$CONFIGURATION \
 -DCH_ENABLE_MODULE_IRRLICHT=ON \
 -DCH_ENABLE_MODULE_POSTPROCESS=ON \
 -DCH_ENABLE_MODULE_VEHICLE=ON \
 -DCH_ENABLE_MODULE_PYTHON=ON \
 -DCH_ENABLE_MODULE_SENSOR=ON \
 -DCH_ENABLE_MODULE_ROS=ON \
 -DCH_ENABLE_MODULE_PARSERS=ON \
 -DCH_USE_CUDA_NVRTC=OFF \
 -DCUDA_ARCH_NAME=Manual \
 -DCHRONO_CUDA_ARCHITECTURES=60 \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DCH_ENABLE_MODULE_CASCADE=ON \
 -DCH_ENABLE_MODULE_PARDISO_MKL=ON \
 -DOptiX_INSTALL_DIR=$HOME/Packages/optix-7.7.0 \
 ./..

#  -DCASCADE_INCLUDE_DIR=$HOME/miniconda3/include/opencascade \
#  -DCASCADE_LIBDIR=$HOME/miniconda3/lib \

# Build & Install
ninja
ninja install
