# make an in source build do to some problems with install

if [ "$PY3K" == "1" ]; then
    MY_PY_VER="${PY_VER}m"
else
    MY_PY_VER="${PY_VER}"
fi

if [ `uname` == Darwin ]; then
    PY_LIB="libpython${MY_PY_VER}.dylib"
else
    PY_LIB="libpython${MY_PY_VER}.so"
fi

# Configure step
cmake -G "%CMAKE_GENERATOR%" \
 -DCMAKE_INSTALL_PREFIX="%PREFIX%" \
 -DCH_INSTALL_PYTHON_PACKAGE="." \
 -DPYTHON_EXECUTABLE:FILEPATH="%PYTHON%" \
 -DPYTHON_INCLUDE_DIR:PATH="%PREFIX%"/include \
 -DPYTHON_LIBRARY:FILEPATH="%PREFIX%"/libs/python%MY_PY_VER%.lib \
 --config "%CONFIGURATION%" \
 -H"C:\projects\chrono" \
 -B"C:\projects\build" \
 -DENABLE_MODULE_IRRLICHT=ON \
 -DENABLE_MODULE_FEA=OFF \
 -DENABLE_MODULE_POSTPROCESS=OFF \
 -DENABLE_MODULE_PYTHON=ON \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 .
# Build step
# on linux travis, limit the number of concurrent jobs otherwise
# gcc gets out of memory
ninja -j 6 install

# fix rpaths
#if [ `uname` == Darwin ]; then
#    for lib in `ls $SP_DIR/OCC/_*.so`; do
#      install_name_tool -rpath $PREFIX/lib @loader_path/../../../ $lib
#    done
#fi