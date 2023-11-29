#!/bin/bash
# installs vsg and dependencies as bleeding edge versions, sources are not retained
# Assimp may exist on the local machine or not - it will be built anyway
# Should work on MacOS and Linux
PREFIX=${HOME}/Packages/VSG
SHARED="ON"
rm -rf $PREFIX build 
GENERATOR="Ninja Multi-Config"
rm -rf _tmp_Sources build
git clone https://github.com/assimp/assimp.git _tmp_Sources/assimp
cmake -G "${GENERATOR}" -B build -S _tmp_Sources/assimp -DBUILD_SHARED_LIBS:BOOL=${SHARED} -DCMAKE_INSTALL_PREFIX=$PREFIX \
	-DASSIMP_BUILD_DRACO:BOOL=ON
cmake --build build --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rm -rf build
git clone --recurse-submodules https://github.com/vsg-dev/VulkanSceneGraph.git _tmp_Sources/vsg
cmake -G "${GENERATOR}" -B build -S _tmp_Sources/vsg -DBUILD_SHARED_LIBS:BOOL=${SHARED} -DCMAKE_INSTALL_PREFIX=$PREFIX
cmake --build build --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rm -rf build 
git clone --recurse-submodules https://github.com/vsg-dev/vsgXchange.git _tmp_Sources/vsgXchange
cmake -G "${GENERATOR}" -B build -S _tmp_Sources/vsgXchange -DBUILD_SHARED_LIBS:BOOL=${SHARED} -DCMAKE_INSTALL_PREFIX=$PREFIX \
	-Dassimp_DIR:PATH=${PREFIX}/lib/cmake/assimp-5.3
cmake --build build  --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rm -rf build
git clone --recurse-submodules https://github.com/vsg-dev/vsgImGui.git _tmp_Sources/vsgImGui
cmake -G "${GENERATOR}" -B build -S _tmp_Sources/vsgImGui -DBUILD_SHARED_LIBS:BOOL=${SHARED} -DCMAKE_INSTALL_PREFIX=$PREFIX
cmake --build build  --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rm -rf build 
git clone --recurse-submodules https://github.com/vsg-dev/vsgExamples.git _tmp_Sources/vsgExamples
cmake -G "${GENERATOR}" -B build -S _tmp_Sources/vsgExamples -DBUILD_SHARED_LIBS:BOOL=${SHARED} -DCMAKE_INSTALL_PREFIX=$PREFIX
cmake --build build  --config Release
cmake --install build  --config Release

rm -rf build _tmp_Sources
