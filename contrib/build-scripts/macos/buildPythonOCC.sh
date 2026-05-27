# python site package dir(s)
# python -m site
#
# https://github.com/tpaviot/pythonocc-core.git
rm -rf download_pythonocc
git clone -c advice.detachedHead=false --depth 1 --branch 7.9.3 "https://github.com/tpaviot/pythonocc-core.git" "download_pythonocc"
rm -rf buildocc
cmake -S download_pythonocc -B buildocc -G Ninja \
	-DCMAKE_CXX_FLAGS="-I/Library/Frameworks/Python.framework/Versions/3.14/lib/python3.14/site-packages/numpy/_core/include -I${HOME}/Source/chrono/src/chrono_thirdparty" \
	-DOpenCASCADE_DIR:PATH=${HOME}/Packages/Cascade/lib/cmake/opencascade \
	-DPYTHONOCC_INSTALL_DIRECTORY:PATH=/Library/Frameworks/Python.framework/Versions/3.14/lib/python3.14/site-packages/OCC
cmake --build buildocc
sudo cmake --install buildocc
#export CFLAGS=-I/usr/lib/python2.7/site-packages/numpy/core/include/
