#
# Cascade allows the visualization of CAD files (STEP, IGES, ..) with chrono
#
# cascade depends on Tcl, Tk and freetype, so be sure to have it installed before you use this script!
#

PREFIX=${HOME}/Packages/Cascade
CAS_VERSION=V7_9_3

BUILDDEBUG=ON
BUILD=build_cascade

rm -rf build_cascade ${PREFIX} download_cascade

git clone -c advice.detachedHead=false --depth 1 --branch ${CAS_VERSION} "https://github.com/Open-Cascade-SAS/OCCT.git" "download_cascade"

cmake -B ${BUILD} -G "Ninja Multi-Config" -S download_cascade -DINSTALL_DIR:PATH=${PREFIX} -DCMAKE_DEBUG_POSTFIX="_d"

cmake --build ${BUILD} --config Release
cmake --install ${BUILD} --config Release

if [ ${BUILDDEBUG} = ON ]
then
	cmake --build ${BUILD} --config Debug
	cmake --install ${BUILD} --config Debug
fi
