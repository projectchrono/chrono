@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building OpenVDB from sources.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set OPENVDB_INSTALL_DIR="C:/Packages/openvdb"
set OPENVDB_DEPENDENCIES_DIR="C:/Packages/openvdb/openvdb-deps"

set BUILDSHARED=ON
set BUILDDEBUG=ON

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
set OPENVDB_INSTALL_DIR=%1
)

@if %DOWNLOAD% EQU ON (
rmdir /S/Q download_openvdb 2>nul
mkdir download_openvdb

@rem Download dependencies
echo "Cloning TBB from GitHub"
powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2020.3.zip -OutFile download_openvdb/tbb.zip"
powershell -Command "Expand-Archive -Force download_openvdb/tbb.zip download_openvdb"
git clone https://github.com/oneapi-src/oneTBB.git download_openvdb/oneTBB
set TBB_SOURCE_DIR="download_openvdb/oneTBB"

echo "Downloading boost 1.80.0 release from SourceForge"
powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://sourceforge.net/projects/boost/files/boost/1.80.0/boost_1_80_0.zip -OutFile download_openvdb/boost.zip"
powershell -Command "Expand-Archive -Force download_openvdb/boost.zip download_openvdb"
set BOOST_SOURCE_DIR="download_openvdb/boost_1_80_0"

echo "Downloading latest zlib release from SourceForge"
powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://www.zlib.net/zlib131.zip -OutFile download_openvdb/zlib.zip"
powershell -Command "Expand-Archive -Force download_openvdb/zlib.zip download_openvdb"
set ZLIB_SOURCE_DIR="download_openvdb/zlib-1.3.1"

echo "Downloading latest blosc release from GitHub"
powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/Blosc/c-blosc/archive/refs/tags/v1.21.5.zip -OutFile download_openvdb/blosc.zip"
powershell -Command "Expand-Archive -Force download_openvdb/blosc.zip download_openvdb"
set BLOSC_SOURCE_DIR="download_openvdb/c-blosc-1.21.5"

echo "Downloading OpenVDB 11.0.0 release from GitHub"
powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v11.0.0.zip -OutFile download_openvdb/openvdb.zip"
powershell -Command "Expand-Archive -Force download_openvdb/openvdb.zip download_openvdb"
set OPENVDB_SOURCE_DIR="download_openvdb/openvdb-11.0.0"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %OPENVDN_INSTALL_DIR% 2>nul
rmdir /S/Q %OPENVDB_DEPENDENCIES_DIR% 2>nul

@rem --- TBB ----------------------------------------------------------------
cmake  -B build_tbb -S %TBB_SOURCE_DIR% -DCMAKE_INSTALL_PREFIX=%OPENVDB_DEPENDENCIES_DIR%/TBB -DTBB_TEST=OFF
cmake --build build_tbb --config Release
cmake --install build_tbb --config Release
if %BUILDDEBUG% EQU ON (
cmake --build build_tbb --config Debug
cmake --install build_tbb --config Debug
) else (
echo "Skipping TBB Debug build"
)

@rem --- BOOST --------------------------------------------------------------
cd %BOOST_SOURCE_DIR%
call bootstrap.bat
call .\b2 variant=release address-model=64 link=static,shared
if %BUILDDEBUG% EQU ON (
call .\b2 variant=debug address-model=64 link=static,shared
) else (
echo "Skipping BOOST Debug build"
)
cd ../../
xcopy /S/E/I/Q %BOOST_SOURCE_DIR% %OPENVDB_DEPENDENCIES_DIR%\boost_1_80_0


@rem --- ZLIB ---------------------------------------------------------------
cmake -B build_zlib -S %ZLIB_SOURCE_DIR% -DCMAKE_INSTALL_PREFIX=%OPENVDB_DEPENDENCIES_DIR%/zlib
cmake --build build_zlib --config Release
cmake --install build_zlib --config Release
if %BUILDDEBUG% EQU ON (
cmake --build build_zlib --config Debug
cmake --install build_zlib --config Debug
) else (
echo "Skipping ZLIB Debug build"
)

@rem --- BLOSC --------------------------------------------------------------
cmake -B build_blosc -S %BLOSC_SOURCE_DIR% -DCMAKE_INSTALL_PREFIX=%OPENVDB_DEPENDENCIES_DIR%/blosc
cmake --build build_blosc --config Release
cmake --install build_blosc --config Release
if %BUILDDEBUG% EQU ON (
cmake --build build_blosc --config Debug
cmake --install build_blosc --config Debug
) else (
echo "Skipping BLOSC Debug build"
)

@rem--- OPENVDB ------------------------------------------------------------
cmake -B build_openvdb -S %OPENVDB_SOURCE_DIR% -DCMAKE_INSTALL_PREFIX=%OPENVDB_INSTALL_DIR% -DCMAKE_BUILD_TYPE=Release ^
-DCMAKE_CXX_STANDARD=17 ^
-DOPENVDB_BUILD_CORE=ON ^
-DOPENVDB_CORE_SHARED=ON ^
-DOPENVDB_BUILD_NANOVDB=ON ^
-DUSE_NANOVDB=ON ^
-DNANOVDB_USE_OPENVDB=ON ^
-DNANOVDB_USE_CUDA=ON ^
-DNANOVDB_CUDA_KEEP_PTX=ON ^
-DNANOVDB_USE_TBB=ON ^
-DNANOVDB_USE_ZLIB=ON ^
-DUSE_BLOSC=ON ^
-DUSE_TBB=ON ^
-DUSE_ZLIB=ON ^
-DUSE_STATIC_DEPENDENCIES=OFF ^
-DNANOVDB_BUILD_EXAMPLES=ON ^
-DBlosc_INCLUDE_DIR="%OPENVDB_DEPENDENCIES_DIR%/blosc/include" ^
-DBlosc_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/blosc/lib/blosc.lib" ^
-DBlosc_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/blosc/lib/blosc.lib" ^
-DBoost_INCLUDE_DIR="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/" ^
-DBoost_IOSTREAMS_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/stage/lib/libboost_iostreams-vc142-mt-gd-x64-1_80.lib" ^
-DBoost_IOSTREAMS_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/stage/lib/boost_iostreams-vc142-mt-x64-1_80.lib" ^
-DBoost_LIBRARY_DIR_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/stage/lib" ^
-DBoost_LIBRARY_DIR_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/stage/lib" ^
-DBoost_REGEX_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/stage/lib/boost_regex-vc142-mt-gd-x64-1_80.lib" ^
-DBoost_REGEX_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/boost_1_80_0/stage/lib/boost_regex-vc142-mt-x64-1_80.lib" ^
-DTbb_INCLUDE_DIR="%OPENVDB_DEPENDENCIES_DIR%/TBB/include" ^
-DTbb_tbb_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/TBB/lib/tbb12_debug.lib" ^
-DTbb_tbb_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/TBB/lib/tbb12.lib" ^
-DTbb_tbbmalloc_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/TBB/lib/tbbmalloc_debug.lib" ^
-DTbb_tbbmalloc_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/TBB/lib/tbbmalloc.lib" ^
-DTbb_tbbmalloc_proxy_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/TBB/lib/tbbmalloc_proxy_debug.lib" ^
-DTbb_tbbmalloc_proxy_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/TBB/lib/tbbmalloc_proxy.lib" ^
-DZLIB_INCLUDE_DIR="%OPENVDB_DEPENDENCIES_DIR%/zlib/include" ^
-DZLIB_LIBRARY_DEBUG="%OPENVDB_DEPENDENCIES_DIR%/zlib/lib/zlibd.lib" ^
-DZLIB_LIBRARY_RELEASE="%OPENVDB_DEPENDENCIES_DIR%/zlib/lib/zlib.lib" ^


cmake --build build_openvdb --config Release
cmake --install build_openvdb --config Release

echo "Done building OpenVDB"
@rem --- Move NanoVDB Cuda header files to utils/cuda directory
@echo off
SET "UTIL_DIR=C:/Packages/openvdb/OpenVDB/include/nanovdb/util"
SET DEST_DIR="C:/Packages/openvdb/OpenVDB/include/nanovdb/util/cuda"


mkdir %DEST_DIR%

echo %UTIL_DIR%/*.cuh
powershell -Command "mv %UTIL_DIR%/*.cuh %DEST_DIR%"
powershell -Command "mv %UTIL_DIR%/GpuTimer.h %DEST_DIR%"

echo "Done moving NanoVDB Cuda header files"

@rem ---- Delete downloaded sources
rmdir /S/Q download_openvdb

@rem --- Delete build directories
rmdir /S/Q build_tbb
rmdir /S/Q build_zlib
rmdir /S/Q build_blosc
rmdir /S/Q build_openvdb
