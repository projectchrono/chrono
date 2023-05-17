@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building VSG based on the last official releases.
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the VSG sources OR indicate that these should be downloaded
@rem - Specify the install directory
@rem - Optionally set the path to the doxygen and dot executables
@rem - Decide whether to build shared or static libraries, whether to also build debug libraries,
@rem   and whether to build the VSG documentation.
@rem - Run the script (.\buildVSG.bat).
@rem - The install directory will contain (under subdirectories of VSG_INSTALL_DIR/lib/shared) all VSG CMake
@rem   project configuration scripts required to configure Chrono with the Chrono::VSG module enabled.
@rem
@rem Notes:
@rem - This script uses the latest versions of the various codes from their respective repositories, with the
@rem   only exception being assimp (set at version 5.2.5). This means that pushes to any of the VSG repositories
@rem   may break the generation of the dependencies for Chrono::VSG.
@rem - This script was tested with the following versions of VSG libraries:
@rem      VulkanSceneGraph (github.com/vsg-dev/VulkanSceneGraph.git): Commit #c87b4cca
@rem      vsgXchange (github.com/vsg-dev/vsgXchange.git):             Commit #883f887
@rem      vsgImGui (github.com/vsg-dev/vsgImGui.git):                 Commit #d9261b1
@rem      vsgExamples (github.com/vsg-dev/vsgExamples.git):           Commit #7ac782b
@rem      assimp (github.com/assimp/assimp):                          Tag v5.2.5
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set VSG_INSTALL_DIR="C:/Packages/vsg"

set DOXYGEN_EXE="doxygen.exe"
set DOT_EXE="dot.exe"

set BUILDSHARED=ON
set BUILDDOCS=OFF
set BUILDDEBUG=ON

@if %DOWNLOAD% EQU OFF (
    set VSG_SOURCE_DIR="C:/Sources/VulkanSceneGraph"
    set VSGXCHANGE_SOURCE_DIR="C:/Sources/vsgXchange"
    set VSGIMGUI_SOURCE_DIR="C:/Sources/vsgImGui"
    set VSGEXAMPLES_SOURCE_DIR="C:/Sources/vsgExamples"
    set ASSIMP_SOURCE_DIR="C:/Sources/assimp"  
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading sources from GitHub"

    rmdir /S/Q download_vsg 2>null
    mkdir download_vsg

    echo "  ... VulkanSceneGraph"
    rem git clone -c advice.detachedHead=false --depth 1 --branch VulkanSceneGraph-1.0.5 "https://github.com/vsg-dev/VulkanSceneGraph" "download_vsg/vsg"
    git clone "https://github.com/vsg-dev/VulkanSceneGraph" "download_vsg/vsg"
    set VSG_SOURCE_DIR="download_vsg/vsg"

    echo "  ... vsgXchange"    
    rem git clone -c advice.detachedHead=false --depth 1 --branch vsgXchange-1.0.2 "https://github.com/vsg-dev/vsgXchange" "download_vsg/vsgXchange"
    git clone "https://github.com/vsg-dev/vsgXchange" "download_vsg/vsgXchange"
    set VSGXCHANGE_SOURCE_DIR="download_vsg/vsgXchange"

    echo "  ... vsgImGui"
    git clone "https://github.com/vsg-dev/vsgImGui" "download_vsg/vsgImGui"
    set VSGIMGUI_SOURCE_DIR="download_vsg/vsgImGui"

    echo "  ... vsgExamples"
    rem git clone -c advice.detachedHead=false --depth 1 --branch vsgExamples-1.0.3 "https://github.com/vsg-dev/vsgExamples" "download_vsg/vsgExamples"
    git clone "https://github.com/vsg-dev/vsgExamples" "download_vsg/vsgExamples"
    set VSGEXAMPLES_SOURCE_DIR="download_vsg/vsgExamples"

    echo "  ... assimp"
    git clone -c advice.detachedHead=false --depth 1 --branch v5.2.5 "https://github.com/assimp/assimp" "download_vsg/assimp"
    set ASSIMP_SOURCE_DIR="download_vsg/assimp"
) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %VSG_INSTALL_DIR% 2>null

rem --- assimp -------------------------------------------------------------

rmdir /S/Q build_assimp 2>null
cmake -B build_assimp -S %ASSIMP_SOURCE_DIR%  ^
      -DBUILD_SHARED_LIBS:BOOL=OFF ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -DASSIMP_BUILD_TESTS:BOOL=OFF  ^
      -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF ^
      -DASSIMP_BUILD_ZLIB:BOOL=ON ^
      -DASSIMP_BUILD_DRACO:BOOL=ON

cmake --build build_assimp --config Release
cmake --install build_assimp --config Release --prefix %VSG_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_assimp --config Debug
    cmake --install build_assimp --config Debug --prefix %VSG_INSTALL_DIR%
) else (
    echo "No Debug build"
)

rem --- vsg ----------------------------------------------------------------

rmdir /S/Q build_vsg 2>null
if %BUILDDOCS% EQU ON (
cmake -B build_vsg -S %VSG_SOURCE_DIR%  ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -DDOXYGEN_EXECUTABLE:FILEPATH=%DOXYGEN_EXE% ^
      -DDOXYGEN_DOT_EXECUTABLE:FILEPATH=%DOT_EXE%
) else (
cmake -B build_vsg -S %VSG_SOURCE_DIR%  ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd
)    

cmake --build build_vsg --config Release
cmake --install build_vsg --config Release --prefix %VSG_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_vsg --config Debug
    cmake --install build_vsg --config Debug --prefix %VSG_INSTALL_DIR%
) else (
    echo "No Debug build"
)
if %BUILDDOCS% EQU ON (
    cmake --build build_vsg --config Release --target docs
    mkdir %VSG_INSTALL_DIR%\html
    xcopy /S/E build_vsg\html %VSG_INSTALL_DIR%\html
)

rem --- vsgXchange ---------------------------------------------------------

rmdir /S/Q build_vsgXchange 2>null
cmake -B build_vxgXchange -S %VSGXCHANGE_SOURCE_DIR%  ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -Dvsg_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsg
cmake --build build_vxgXchange --config Release
cmake --install build_vxgXchange --config Release --prefix %VSG_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_vxgXchange --config Debug
    cmake --install build_vxgXchange --config Debug --prefix %VSG_INSTALL_DIR%
) else (
    echo "No Debug build"
)

@rem del /S/Q red_teapot.vsgt
@rem vsgconv red_teapot.obj red_teapot.vsgt
@rem if EXIST red_teapot.vsgt (
@rem     echo "seems to work."
@rem )

rem --- vsgImGui -----------------------------------------------------------

rmdir /S/Q build_vsgImGui 2>null
cmake -B build_vsgImGui -S %VSGIMGUI_SOURCE_DIR% ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -Dvsg_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsg
cmake --build build_vsgImGui --config Release
cmake --install build_vsgImGui --config Release --prefix %VSG_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_vsgImGui --config Debug
    cmake --install build_vsgImGui --config Debug --prefix %VSG_INSTALL_DIR%
) else (
    echo "No Debug build"
)

rem --- vsgExamples --------------------------------------------------------

rmdir /S/Q build_vsgExamples 2>null
cmake -B build_vsgExamples -S %VSGEXAMPLES_SOURCE_DIR% ^
      -Dvsg_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsg
cmake --build build_vsgExamples --config Release
cmake --install build_vsgExamples --config Release --prefix %VSG_INSTALL_DIR%

rem --- VSG_FILE_PATH ------------------------------------------------------

set "VSG_INSTALL_DIR=%VSG_INSTALL_DIR:/=\%"
setx VSG_FILE_PATH "%VSG_INSTALL_DIR%\share\vsgExamples"
