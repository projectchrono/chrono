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
@rem - This was tested with the following versions of VSG libraries:
@rem      VulkanSceneGraph (github.com/vsg-dev/VulkanSceneGraph.git): Commit #aa28d62
@rem      vsgXchange (github.com/vsg-dev/vsgXchange.git):             Commit #b37861b
@rem      vsgImGui (github.com/vsg-dev/vsgImGui.git):                 Commit #0ec0cd4
@rem      vsgExamples (github.com/vsg-dev/vsgExamples.git):           Commit #c789753
@rem      assimp (github.com/assimp/assimp):                          Commit #8c6b3fe
@rem ---------------------------------------------------------------------------------------------------------

set VSG_SOURCE_DIR="E:/Repositories/VulkanSceneGraph"
set VSGXCHANGE_SOURCE_DIR="E:/Repositories/vsgXchange"
set VSGIMGUI_SOURCE_DIR="E:/Repositories/vsgImGui"
set VSGEXAMPLES_SOURCE_DIR="E:/Repositories/vsgExamples"
set ASSIMP_SOURCE_DIR="E:/Repositories/assimp"

set DOWNLOAD=ON

set VSG_INSTALL_DIR="E:/Packages/vsg"

set DOXYGEN_EXE="doxygen.exe"
set DOT_EXE="dot.exe"

set BUILDSHARED=ON
set BUILDDOCS=OFF
set BUILDDEBUG=ON

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading sources from GitHub"

    rmdir /S/Q download_vsg 2>null
    mkdir download_vsg

    echo "  ... VulkanSceneGraph"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/vsg-dev/VulkanSceneGraph/archive/refs/heads/master.zip -OutFile download_vsg/vsg.zip"
    powershell -Command "Expand-Archive -Force download_vsg/vsg.zip download_vsg"
    set VSG_SOURCE_DIR="download_vsg/VulkanSceneGraph-master"

    echo "  ... vsgXchange"    
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/vsg-dev/vsgXchange/archive/refs/heads/master.zip -OutFile download_vsg/vsgXchange.zip"
    powershell -Command "Expand-Archive -Force download_vsg/vsgXchange.zip download_vsg"
    set VSGXCHANGE_SOURCE_DIR="download_vsg/vsgXchange-master"

    echo "  ... vsgImGui"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/vsg-dev/vsgImGui/archive/refs/heads/master.zip -OutFile download_vsg/vsgImGui.zip"
    powershell -Command "Expand-Archive -Force download_vsg/vsgImGui.zip download_vsg"
    set VSGIMGUI_SOURCE_DIR="download_vsg/vsgImGui-master"

    echo "  ... ImGui"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/ocornut/imgui/archive/refs/heads/master.zip -OutFile download_vsg/imgui.zip"
    powershell -Command "Expand-Archive -Force download_vsg/imgui.zip download_vsg"
    xcopy /S/E/Y download_vsg\imgui-master\* download_vsg\vsgImGui-master\src\imgui\

    echo "  ... ImPlot"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/epezent/implot/archive/refs/heads/master.zip -OutFile download_vsg/implot.zip"
    powershell -Command "Expand-Archive -Force download_vsg/implot.zip download_vsg"
    xcopy /S/E/Y download_vsg\implot-master\* download_vsg\vsgImGui-master\src\implot\

    echo "  ... vsgExamples"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/vsg-dev/vsgExamples/archive/refs/heads/master.zip -OutFile download_vsg/vsgExamples.zip"
    powershell -Command "Expand-Archive -Force download_vsg/vsgExamples.zip download_vsg"
    set VSGEXAMPLES_SOURCE_DIR="download_vsg/vsgExamples-master"

    echo "  ... assimp"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/assimp/assimp/archive/refs/heads/master.zip -OutFile download_vsg/assimp.zip"
    powershell -Command "Expand-Archive -Force download_vsg/assimp.zip download_vsg"
    set ASSIMP_SOURCE_DIR="download_vsg/assimp-master"
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
cmake -B build_vsg -S %VSG_SOURCE_DIR%  ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -DDOXYGEN_EXECUTABLE:FILEPATH=%DOXYGEN_EXE% ^
      -DDOXYGEN_DOT_EXECUTABLE:FILEPATH=%DOT_EXE%
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
