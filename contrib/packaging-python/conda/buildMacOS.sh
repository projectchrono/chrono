
mkdir -p ./build
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

if [ `uname` == Darwin ]; then
    sed -i '' 's/${PYTHON_LIBRARY}//g' $SRC_DIR/src/chrono_swig/chrono_python/CMakeLists.txt
fi
export LDFLAGS="-Wl,-undefined,dynamic_lookup $LDFLAGS"

#Setting MKL directory values
export MKL_INCLUDE_DIR=`cd $HOME/miniconda/pkgs/mkl-include-*/; pwd`
export MKL_LIB_DIR=`cd $HOME/miniconda/pkgs/mkl-2022*/; pwd`
CONFIGURATION=Release

export PARDISO_MKL_ENABLE="OFF"
if [`uname -m` == x86_64]; then
    PARDISO_MKL_ENABLE="ON"

# Configure step
cmake -DCMAKE_INSTALL_PREFIX=$PREFIX \
 -DCMAKE_C_COMPILER=$(which clang) \
 -DCMAKE_CXX_COMPILER=$(which clang++) \
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
 -DENABLE_MODULE_CASCADE=OFF \
 -DCASCADE_INCLUDE_DIR=$HOME/miniconda/include/opencascade \
 -DCASCADE_LIBDIR=$HOME/miniconda/lib \
 -DENABLE_MODULE_PARDISO_MKL=$(PARDISO_MKL_ENABLE) \
 -DMKL_INCLUDE_DIR=$MKL_INCLUDE_DIR/include \
 -DMKL_RT_LIBRARY=$MKL_LIB_DIR/lib/libmkl_rt.dylib \
 -DEIGEN3_INCLUDE_DIR="/usr/local/include/eigen3" "$HOME/miniconda/include" \
 -DPYCHRONO_DATA_PATH=../../../../../../share/chrono/data/ \
 ./..
# Build step
# on linux travis, limit the number of concurrent jobs otherwise
# gcc gets out of memory
cmake --build . --config "$CONFIGURATION"

cmake --build . --config "$CONFIGURATION" --target install



