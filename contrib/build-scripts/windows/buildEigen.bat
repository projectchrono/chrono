@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building and installing Eigen3.
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the Eigen source OR indicate that it should be downloaded.
@rem - Specify the install directory.
@rem - Run the script (.\buildEigen.bat).
@rem - The install directory will contain subdirectories for all necessary dependencies.
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set EIGEN_INSTALL_DIR="C:/Packages/eigen"
set EIGEN_VERSION="5.0.0"
@rem set EIGEN_VERSION="3.4.0"


@if %DOWNLOAD% EQU OFF (
    set EIGEN_SOURCE_DIR="C:/Sources/eigen"
)

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set EIGEN_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading source from GitLab"

    rmdir /S/Q download_eigen 2>nul
    mkdir download_eigen

    git clone -c advice.detachedHead=false --depth 1 --branch %EIGEN_VERSION% "https://gitlab.com/libeigen/eigen.git" "download_eigen"
    set EIGEN_SOURCE_DIR="download_eigen"
) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %EIGEN_INSTALL_DIR% 2>nul

rmdir /S/Q build_eigen 2>nul
cmake -B build_eigen -S %EIGEN_SOURCE_DIR%

@rem cmake --build build_eigen --config Release
cmake --install build_eigen --config Release --prefix %EIGEN_INSTALL_DIR%


