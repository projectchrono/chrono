@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building and installing Spectra
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the Spectra source OR indicate that it should be downloaded.
@rem - Specify the install directory.
@rem - Run the script (.\buildSpectra.bat).
@rem - The install directory will contain subdirectories for all necessary dependencies.
@rem
@rem Notes:
@rem - Chrono uses the Krylov-Schur solver from Spectra, available *only* in the Spectra development branch.
@rem - The script accepts 1 optional argument to override the install directory.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set SPECTRA_INSTALL_DIR="C:/Packages/spectra"

@if %DOWNLOAD% EQU OFF (
    set SPECTRA_SOURCE_DIR="C:/Sources/spectra"
)

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set SPECTRA_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading source from GitHub"

    rmdir /S/Q download_spectra 2>nul
    mkdir download_spectra

    git clone -c advice.detachedHead=false --depth 1 --branch develop "https://github.com/yixuan/spectra.git" "download_spectra"
    set SPECTRA_SOURCE_DIR="download_spectra"
) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %SPECTRA_INSTALL_DIR% 2>nul

rmdir /S/Q build_spectra 2>nul
cmake -B build_spectra -S %SPECTRA_SOURCE_DIR%

@rem cmake --build build_spectra --config Release
cmake --install build_spectra --config Release --prefix %SPECTRA_INSTALL_DIR%
