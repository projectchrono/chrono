@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building and installing Blaze
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the Blaze source OR indicate that it should be downloaded.
@rem - Specify the install directory.
@rem - Run the script (.\buildBlaze.bat).
@rem - The install directory will contain subdirectories for all necessary dependencies.
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set BLAZE_INSTALL_DIR="C:/Packages/blaze"

@if %DOWNLOAD% EQU OFF (
    set BLAZE_SOURCE_DIR="C:/Sources/blaze"
)

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set BLAZE_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading source from Bitbucket"

    rmdir /S/Q download_blaze 2>nul
    mkdir download_blaze

    git clone -c advice.detachedHead=false --depth 1 --branch v3.8.2 "https://bitbucket.org/blaze-lib/blaze.git" "download_blaze"
    set BLAZE_SOURCE_DIR="download_blaze"
) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %BLAZE_INSTALL_DIR% 2>nul

rmdir /S/Q build_blaze 2>nul
cmake -B build_blaze -S %BLAZE_SOURCE_DIR%

@rem cmake --build build_blaze --config Release
cmake --install build_blaze --config Release --prefix %BLAZE_INSTALL_DIR%


