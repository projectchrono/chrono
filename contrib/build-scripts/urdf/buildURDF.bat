@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building and installing URDF parser dependencies
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the URDF sources OR indicate that these should be downloaded.
@rem - Specify the install directory.
@rem - Run the script (.\buildURDF.bat).
@rem - The install directory will contain subdirectories for all necessary dependencies.
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set URDF_INSTALL_DIR="C:/Packages/urdf"

set BUILDDEBUG=ON

@if %DOWNLOAD% EQU OFF (
    set TINYXML2_SOURCE_DIR="C:/Sources/tinyxml2"
    set CONSOLE_BRIDGE_SOURCE_DIR="C:/Sources/console_bridge"
    set URDFDOM_HEADERS_SOURCE_DIR="C:/Sources/urdfdom_headers"
    set URDFDOM_SOURCE_DIR="C:/Sources/urdfdom"
)

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set URDF_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading sources from GitHub"

    rmdir /S/Q download_urdf 2>null
    mkdir download_urdf

    echo "  ... tinyxml2"
    git clone "https://github.com/leethomason/tinyxml2.git" "download_urdf/tinyxml2"
    set TINYXML2_SOURCE_DIR="download_urdf/tinyxml2"

    echo "  ... console_bridge"    
    git clone "https://github.com/ros/console_bridge.git" "download_urdf/console_bridge"
    set CONSOLE_BRIDGE_SOURCE_DIR="download_urdf/console_bridge"

    echo "  ... urdfdom_headers"
    git clone "https://github.com/ros/urdfdom_headers.git" "download_urdf/urdfdom_headers"
    set URDFDOM_HEADERS_SOURCE_DIR="download_urdf/urdfdom_headers"

    echo "  ... urdfdom"
    git clone -c advice.detachedHead=false --depth 1 --branch scpeters/tinyxml2 "https://github.com/rserban/urdfdom.git" "download_urdf/urdfdom"
    @rem git clone "https://github.com/ros/urdfdom.git" "download_urdf/urdfdom"
    set URDFDOM_SOURCE_DIR="download_urdf/urdfdom"
) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %URDF_INSTALL_DIR% 2>null

rem --- tinyxml2 ------------------------------------------------------------

rmdir /S/Q build_tinyxml2 2>null
cmake -B build_tinyxml2 -S %TINYXML2_SOURCE_DIR% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -Dtinyxml2_INSTALL_CMAKEDIR:PATH="CMake"

cmake --build build_tinyxml2 --config Release
cmake --install build_tinyxml2 --config Release --prefix %URDF_INSTALL_DIR%

if %BUILDDEBUG% EQU ON (
    cmake --build build_tinyxml2 --config Debug
    cmake --install build_tinyxml2 --config Debug --prefix %URDF_INSTALL_DIR%
) else (
    echo "No Debug build"
)

rem --- console_bridge ------------------------------------------------------

rmdir /S/Q build_console_bridge 2>null
cmake -B build_console_bridge -S %CONSOLE_BRIDGE_SOURCE_DIR% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd

cmake --build build_console_bridge --config Release
cmake --install build_console_bridge --config Release --prefix %URDF_INSTALL_DIR%

if %BUILDDEBUG% EQU ON (
    cmake --build build_console_bridge --config Debug
    cmake --install build_console_bridge --config Debug --prefix %URDF_INSTALL_DIR%
) else (
    echo "No Debug build"
)

rem --- urdfdom_headers --------------------------------------------------------

rmdir /S/Q build_urdfdom_headers 2>null
cmake -B build_urdfdom_headers -S %URDFDOM_HEADERS_SOURCE_DIR% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd

cmake --build build_urdfdom_headers --config Release
cmake --install build_urdfdom_headers --config Release --prefix %URDF_INSTALL_DIR%

if %BUILDDEBUG% EQU ON (
    cmake --build build_urdfdom_headers --config Debug
    cmake --install build_urdfdom_headers --config Debug --prefix %URDF_INSTALL_DIR%
) else (
    echo "No Debug build"
)

rem --- urdfdom -------------------------------------------------------------

rmdir /S/Q build_urdfdom 2>null
cmake -B build_urdfdom -S %URDFDOM_SOURCE_DIR% ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -DDISABLE_TINYXML_SUPPORT:BOOL=ON ^
      -Dtinyxml2_DIR:PATH=%URDF_INSTALL_DIR%/CMake ^
      -Dconsole_bridge_DIR:PATH=%URDF_INSTALL_DIR%/CMake ^
      -Durdfdom_headers_DIR:PATH=%URDF_INSTALL_DIR%/CMake

cmake --build build_urdfdom --config Release
cmake --install build_urdfdom --config Release --prefix %URDF_INSTALL_DIR%

if %BUILDDEBUG% EQU ON (
    cmake --build build_urdfdom --config Debug
    cmake --install build_urdfdom --config Debug --prefix %URDF_INSTALL_DIR%
) else (
    echo "No Debug build"
)

