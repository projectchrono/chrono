@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building OpenCRG from 1.1.2 sources.
@rem - Requires the Visual Studio `cl` compiler.
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the GL sources OR indicate that these should be downloaded.
@rem - Specify the install directory.
@rem - IMPORTANT: run the script (.\buildOpenCRG.bat) from an x64 VS command prompt.
@rem - As provided, this script generates the OpenCRG libraries for 
@rem   Release, Debug, RelWithDebInfo, and MinSizeRel configurations.
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON
@rem Link to the MSVC static runtime in case you selected the same in the Chrono CMake configuration (e.g. for FMI).
set LINK_MSVC_STATIC_RUNTIME=OFF

set CRG_INSTALL_DIR="C:\Packages\openCRG"

@if %DOWNLOAD% EQU OFF (
    set CRG_SOURCE_DIR="C:\Sources\OpenCRG-1.1.2"
    set REV=1.1.2
)

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set CRG_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading sources from GitHub"

    rmdir /S/Q download_crg 2>nul
    mkdir download_crg

    powershell -Command "Invoke-WebRequest -UserAgent 'Wget' -Uri https://github.com/hlrs-vis/opencrg/archive/refs/tags/v1.1.2.zip  -OutFile download_crg/crg.zip"
    powershell -Command "Expand-Archive -Force download_crg/crg.zip download_crg"
    set CRG_SOURCE_DIR="../download_crg/opencrg-1.1.2"
    set REV=1.1.2
) else (
    echo "Using provided source directories"
)

echo "Sources in " %CRG_SOURCE_DIR%

@if %LINK_MSVC_STATIC_RUNTIME% EQU ON (
    echo "Linking to MSVC static runtime."
    set LINK_MSVC_RUNTIME_FLAG=/MT
) else (
    echo "Linking to MSVC dynamic runtime."
    set LINK_MSVC_RUNTIME_FLAG=/MD
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %CRG_INSTALL_DIR% 2>nul

mkdir %CRG_INSTALL_DIR%
mkdir %CRG_INSTALL_DIR%\include
mkdir %CRG_INSTALL_DIR%\lib

@rem ------------------------------------------------------------------------

rmdir /S/Q build_crg 2>nul
mkdir build_crg

cd build_crg
del /S/Q *.lib *.obj

rem build release
cl /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc %LINK_MSVC_RUNTIME_FLAG% /O2 /Ob2 /DNDEBUG -I%CRG_SOURCE_DIR%\inc %CRG_SOURCE_DIR%\src\*.c
lib/out:OpenCRG.lib *.obj
lib/list OpenCRG.lib
del *.obj

rem build debug
cl /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc %LINK_MSVC_RUNTIME_FLAG%d /Zi /Ob0 /Od /RTC1 -I%CRG_SOURCE_DIR%\inc %CRG_SOURCE_DIR%\src\*.c
lib/out:OpenCRG_d.lib *.obj
lib/list OpenCRG_d.lib
del *.obj

rem build release with debug info
cl /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc %LINK_MSVC_RUNTIME_FLAG% /Zi /O2 /Ob1 /DNDEBUG -I%CRG_SOURCE_DIR%\inc %CRG_SOURCE_DIR%\src\*.c
lib/out:OpenCRG_rd.lib *.obj
lib/list OpenCRG_rd.lib
del *.obj

rem build relase with minimal size
cl /c /DWIN32 /D_WINDOWS /W3 /GR /EHsc %LINK_MSVC_RUNTIME_FLAG% /O1 /Ob1 /DNDEBUG -I%CRG_SOURCE_DIR%\inc %CRG_SOURCE_DIR%\src\*.c
lib/out:OpenCRG_s.lib *.obj
lib/list OpenCRG_s.lib
del *.obj

rem install the files
copy %CRG_SOURCE_DIR%\\inc\*.h %CRG_INSTALL_DIR%\include
copy *.lib %CRG_INSTALL_DIR%\lib

cd ..
