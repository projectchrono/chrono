@rem installs vsg and dependencies as bleeding edge versions, sources are not retained
@rem  Assimp may exist on the local machine or not - it will be built anyway
@rem  Should work on Windows, used only for development!!
set PREFIX="C:\Packages\vsg"
set GENERATOR="Ninja Multi-Config"
set SHARED="ON"
rmdir /S/Q %PREFIX% build 
rmdir /S/Q _tmp_Sources

git clone https://github.com/assimp/assimp.git _tmp_Sources/assimp
cmake -G %GENERATOR% -B build -S _tmp_Sources/assimp -DBUILD_SHARED_LIBS:BOOL=%SHARED% -DCMAKE_INSTALL_PREFIX=%PREFIX% ^
	-DASSIMP_BUILD_DRACO:BOOL=ON
cmake --build build --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rmdir /S/Q build 
git clone --recurse-submodules https://github.com/vsg-dev/VulkanSceneGraph.git _tmp_Sources/vsg
cmake -G %GENERATOR% -B build -S _tmp_Sources/vsg -DBUILD_SHARED_LIBS:BOOL=%SHARED% -DCMAKE_INSTALL_PREFIX=%PREFIX%
cmake --build build --config Release
cmake --install build  --config Release
make --build build --config Debug
cmake --install build  --config Debug

rmdir /S/Q build 
git clone --recurse-submodules https://github.com/vsg-dev/vsgXchange.git _tmp_Sources/vsgXchange
cmake -G %GENERATOR% -B build -S _tmp_Sources/vsgXchange -DBUILD_SHARED_LIBS:BOOL=%SHARED% -DCMAKE_INSTALL_PREFIX=%PREFIX% ^
	-Dassimp_DIR:PATH=%PREFIX%/lib/cmake/assimp-5.3
cmake --build build  --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rmdir /S/Q build
git clone --recurse-submodules https://github.com/vsg-dev/vsgImGui.git _tmp_Sources/vsgImGui
cmake -G %GENERATOR% -B build -S _tmp_Sources/vsgImGui -DBUILD_SHARED_LIBS:BOOL=%SHARED% -DCMAKE_INSTALL_PREFIX=%PREFIX%
cmake --build build  --config Release
cmake --install build  --config Release
cmake --build build --config Debug
cmake --install build  --config Debug

rmdir /S/Q build 
git clone --recurse-submodules https://github.com/vsg-dev/vsgExamples.git _tmp_Sources/vsgExamples
cmake -G %GENERATOR% -B build -S _tmp_Sources/vsgExamples -DBUILD_SHARED_LIBS:BOOL=%SHARED% -DCMAKE_INSTALL_PREFIX=%PREFIX%
cmake --build build  --config Release
cmake --install build  --config Release

rmdir /S/Q build _tmp_Sources
