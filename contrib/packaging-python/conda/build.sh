
echo Started build.sh
mkdir ./build
cd ./build
echo $CI_PROJECT_DIR
export NP_INCL=$(python $CI_PROJECT_DIR/contrib/packaging-python/conda/setvarnumpy.py )

# Python libraries are different file types for MacOS and linux
# TODO: Check if this is needed since MacOS has its own deployment script

if [ `uname` == Darwin ]; then
    PY_LIB="libpython${PY_VER}.dylib"
else
    PY_LIB="libpython${PY_VER}.so"
fi

# set MKL vars
export MKL_INTERFACE_LAYER=LP64
export MKL_THREADING_LAYER=INTEL
CONFIGURATION=Release
# Configure step
cmake -DCMAKE_INSTALL_PREFIX=$PREFIX \
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
 -DENABLE_MODULE_IRRLICHT=ON \
 -DENABLE_MODULE_POSTPROCESS=ON \
 -DENABLE_MODULE_VEHICLE=ON \
 -DENABLE_MODULE_PYTHON=ON \
 -DENABLE_MODULE_SENSOR=ON \
 -DUSE_CUDA_NVRTC=OFF \
 -DCUDA_ARCH_NAME=Manual \
 -DCUDA_ARCH_PTX=52 \
 -DCUDA_ARCH_BIN=5.2 \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DENABLE_MODULE_CASCADE=ON \
 -DCASCADE_INCLUDE_DIR=$HOME/miniconda3/include/opencascade \
 -DCASCADE_LIBDIR=$HOME/miniconda3/lib \
 -DENABLE_MODULE_PARDISO_MKL=ON \
 -DMKL_INCLUDE_DIR=$BUILD_PREFIX/include \
 -DMKL_RT_LIBRARY=$BUILD_PREFIX/lib/libmkl_rt.so \
 -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
 -DOptiX_INSTALL_DIR=/opt/optix/7.2.0 \
 -DNUMPY_INCLUDE_DIR=$NP_INCL \
 ./..

# Build step
# on linux travis, limit the number of concurrent jobs otherwise
# gcc gets out of memory
cmake --build . --config "$CONFIGURATION"

cmake --build . --config "$CONFIGURATION" --target install
