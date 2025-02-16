
@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building GL from sources.
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the GL sources OR indicate that these should be downloaded.
@rem - Specify the install directory.
@rem - Decide whether to build shared or static libraries and whether to also build debug libraries.
@rem - Run the script (.\buildGL.bat) from a *VS developer console*.
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem - Do *not* use GLEW from its GitHub repository. It is not properly set up to work with CMake. Use instead
@rem   the source archive available on SourceForge.
@rem - The sources for GLM and GLFW can be obtained either from GitHub or from SourceForge.
@rem - This was tested with the following versions of GL libraries:
@rem      GLEW (glew.sourceforge.net/):     Version 2.2.0
@rem      GLFW (github.com/glfw/glfw):      Version 3.3.10
@rem      GLM (github.com/g-truc/glm.git):  Version 1.0.1                  
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set GL_INSTALL_DIR="C:/Packages/gl"

set BUILDSHARED=ON
set BUILDDEBUG=OFF

@if %DOWNLOAD% EQU OFF (
    set GLM_SOURCE_DIR="C:/Sources/glm"
    set GLEW_SOURCE_DIR="C:/Sources/glew"
    set GLFW_SOURCE_DIR="C:/Sources/glfw"
)    

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set GL_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading sources from sourceforge.net"

    rmdir /S/Q download_gl 2>nul
    mkdir download_gl

    echo "  ... GLEW"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://sourceforge.net/projects/glew/files/glew/2.2.0/glew-2.2.0.zip -OutFile download_gl/glew.zip"
    powershell -Command "Expand-Archive -Force download_gl/glew.zip download_gl"
    set GLEW_SOURCE_DIR="download_gl/glew-2.2.0"

    echo "  ... GLFW"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://sourceforge.net/projects/glfw/files/glfw/3.3.10/glfw-3.3.10.zip -OutFile download_gl/glfw.zip"
    powershell -Command "Expand-Archive -Force download_gl/glfw.zip download_gl"
    set GLFW_SOURCE_DIR="download_gl/glfw-3.3.10"

    echo "  ... GLM"
    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/g-truc/glm/archive/refs/tags/1.0.1.zip -OutFile download_gl/glm.zip"
    powershell -Command "Expand-Archive -Force download_gl/glm.zip download_gl"
    set GLM_SOURCE_DIR="download_gl/glm-1.0.1"
) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %GL_INSTALL_DIR% 2>nul

rem --- GLM ----------------------------------------------------------------

rmdir /S/Q build_glm 2>nul
cmake -S %GLM_SOURCE_DIR% ^
    -DGLM_BUILD_LIBRARY=OFF ^
    -DGLM_BUILD_TESTS=OFF ^
    -DBUILD_SHARED_LIBS=OFF ^
    -DGLM_BUILD_INSTALL=ON ^
    -DCMAKE_INSTALL_INCLUDEDIR=%GL_INSTALL_DIR%/include ^
    -DCMAKE_INSTALL_DATAROOTDIR=%GL_INSTALL_DIR%/lib/cmake ^
    -B build_glm

cmake --build build_glm --config Release
cmake --install build_glm --config Release --prefix %GL_INSTALL_DIR%


rem --- GLEW ---------------------------------------------------------------

rmdir /S/Q build_glew 2>nul
cmake -B build_glew -S %GLEW_SOURCE_DIR%/build/cmake ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d

cmake --build build_glew --config Release
cmake --install build_glew --config Release --prefix %GL_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_glew --config Debug
    cmake --install build_glew --config Debug --prefix %GL_INSTALL_DIR%
) else (
    echo "No Debug build for GLEW"
)

rem --- GLFW ---------------------------------------------------------------

rmdir /S/Q build_glfw 2>nul
cmake -B build_glfw -S %GLFW_SOURCE_DIR% ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILDSHARED% ^
      -DCMAKE_DEBUG_POSTFIX=_d

cmake --build build_glfw --config Release
cmake --install build_glfw --config Release --prefix %GL_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_glfw --config Debug
    cmake --install build_glfw --config Debug --prefix %GL_INSTALL_DIR%
) else (
    echo "No Debug build for GLFW"
)
