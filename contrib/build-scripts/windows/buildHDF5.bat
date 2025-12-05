@rem ---------------------------------------------------------------------------------------------------------
@rem Windows batch script for building HDF5 based on the last official releases.
@rem - Place in an arbitrary temporary directory.
@rem - Specify the locations for the HDF5 sources OR indicate that these should be downloaded.
@rem - Specify the install directory.
@rem - Optionally, disable building the debug libraries.
@rem - Run the script (.\buildHDF5.bat).
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem - This script downloads HDF5 version 2.0.0 from its repository.
@rem ---------------------------------------------------------------------------------------------------------

set DOWNLOAD=ON

set HDF5_INSTALL_DIR="C:/Packages/hdf5"

set BUILDDEBUG=ON

@if %DOWNLOAD% EQU OFF (
    set HDF5_SOURCE_DIR="C:/Sources/hdf5" 
)

@rem ------------------------------------------------------------------------
@rem Allow overriding installation directory through command line argument

if "%~1" NEQ "" (
   set HDF5_INSTALL_DIR=%1
)

@rem ------------------------------------------------------------------------

@if %DOWNLOAD% EQU ON (
    echo "Downloading sources from GitHub"

    rmdir /S/Q download_hdf5 2>nul
    mkdir download_hdf5

    echo "  ... HDF5"
    git clone -c advice.detachedHead=false --depth 1 --branch hdf5_2_0_0 "https://github.com/HDFGroup/hdf5" "download_hdf5/hdf5"
    set HDF5_SOURCE_DIR="download_hdf5/hdf5"

) else (
    echo "Using provided source directories"
)

@rem ------------------------------------------------------------------------

rmdir /S/Q %HDF5_INSTALL_DIR% 2>nul

rem --- HDF5 ----------------------------------------------------------------

rmdir /S/Q build_hdf5 2>nul
cmake -B build_hdf5 -S %HDF5_SOURCE_DIR% ^
      -DHDF5_BUILD_CPP_LIB:BOOL=ON ^
      -DBUILD_SHARED_LIBS:BOOL=ON ^
      -DBUILD_STATIC_LIBS:BOOL=ON ^
      -DBUILD_TESTING:BOOL=OFF ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd ^
      -DHDF_PACKAGE_NAMESPACE:STRING="hdf5::" ^
      -DHDF5_ENABLE_SZIP_SUPPORT:BOOL=OFF ^
      -DHDF5_ENABLE_ZLIB_SUPPORT:BOOL=OFF 

cmake --build build_hdf5 --config Release
cmake --install build_hdf5 --config Release --prefix %HDF5_INSTALL_DIR%
if %BUILDDEBUG% EQU ON (
    cmake --build build_hdf5 --config Debug
    cmake --install build_hdf5 --config Debug --prefix %HDF5_INSTALL_DIR%
) else (
    echo "No Debug build of HDF5"
)
