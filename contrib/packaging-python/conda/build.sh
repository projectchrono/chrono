
echo Started build.sh
mkdir ./build
cd ./build
export NP_INCL=$(python3 -c "import numpy; print(numpy.get_include())")
echo $NP_INCL

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
 -DPYTHON_EXECUTABLE:FILEPATH=$PYTHON \
 -DPYTHON_INCLUDE_DIR:PATH=$PREFIX/include/python${PY_VER} \
 -DPYTHON_LIBRARY:FILEPATH=$PREFIX/lib/${PY_LIB} \
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
 -DCUDA_ARCH_PTX=52 \
 -DCUDA_ARCH_BIN=5.2 \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DCH_ENABLE_MODULE_CASCADE=ON \
 -DOpenCASCADE_DIR=$HOME/Packages/opencascade-7.4.0/lib/cmake/opencascade \
 -DCH_ENABLE_MODULE_PARDISO_MKL=ON \
 -DMKL_INCLUDE_DIR=$BUILD_PREFIX/include \
 -DMKL_RT_LIBRARY=$BUILD_PREFIX/lib/libmkl_rt.so \
 -DEIGEN3_INCLUDE_DIR=$HOME/Packages/eigen-3.4.0 \
 -DIRRLICHT_INSTALL_DIR=$HOME/Packages/irrlicht-1.8.5 \
 -DOptiX_INSTALL_DIR=$HOME/Packages/optix-7.7.0 \
 -DNUMPY_INCLUDE_DIR=$NP_INCL \
 -Durdfdom_DIR=$HOME/Packages/urdf/lib/urdfdom/cmake \
 -Durdfdom_headers_DIR=$HOME/Packages/urdf/lib/urdfdom_headers/cmake \
 -Dconsole_bridge_DIR=$HOME/Packages/urdf/lib/console_bridge/cmake \
 -Dtinyxml2_DIR=$HOME/Packages/urdf/CMake \
 ./..

#  -DCASCADE_INCLUDE_DIR=$HOME/miniconda3/include/opencascade \
#  -DCASCADE_LIBDIR=$HOME/miniconda3/lib \

# Build & Install
ninja
ninja install
