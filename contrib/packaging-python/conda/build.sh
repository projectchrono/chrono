
mkdir ./build
cd ./build
# in py <= 3.7, headers are in $PREFIX/include/python3.xm/, while since python 3.8 they are in $PREFIX/include/python3.8/ go figure.
if [ "$PY3K" == "1" ] && [ "$PY_VER" != "3.8" ] ; then
    MY_PY_VER="${PY_VER}m"
else
    MY_PY_VER="${PY_VER}"
fi

if [ `uname` == Darwin ]; then
    PY_LIB="libpython${MY_PY_VER}.dylib"
else
    PY_LIB="libpython${MY_PY_VER}.so"
fi

# set MKL vars
export MKL_INTERFACE_LAYER=LP64
export MKL_THREADING_LAYER=INTEL
CONFIGURATION=Release
# Configure step
cmake -DCMAKE_INSTALL_PREFIX=$PREFIX \
 -DCMAKE_PREFIX_PATH=$PREFIX \
 -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
 -DCH_INSTALL_PYTHON_PACKAGE=$SP_DIR \
 -DPYTHON_EXECUTABLE:FILEPATH=$PYTHON \
 -DPYTHON_INCLUDE_DIR:PATH=$PREFIX/include/python$MY_PY_VER \
 -DPYTHON_LIBRARY:FILEPATH=$PREFIX/lib/${PY_LIB} \
 -DCMAKE_BUILD_TYPE=$CONFIGURATION \
 -DENABLE_MODULE_IRRLICHT=ON \
 -DENABLE_MODULE_POSTPROCESS=ON \
 -DENABLE_MODULE_VEHICLE=ON \
 -DENABLE_MODULE_PYTHON=ON \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DENABLE_MODULE_CASCADE=ON \
 -DCASCADE_INCLUDE_DIR=$HOME/miniconda3/include/opencascade \
 -DCASCADE_LIBDIR=$HOME/miniconda3/lib \
 -DENABLE_MODULE_PARDISO_MKL=ON \
 -DMKL_INCLUDE_DIR=$HOME/miniconda3/include \
 -DMKL_RT_LIBRARY=$HOME/miniconda3/lib/libmkl_rt.so \
 -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
 -DPYCHRONO_DATA_PATH=../../../../../../share/chrono/data \
 ./..
# Build step
# on linux travis, limit the number of concurrent jobs otherwise
# gcc gets out of memory
cmake --build . --config "$CONFIGURATION"

cmake --build . --config "$CONFIGURATION" --target install



