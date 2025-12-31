
@rem -------------------------------------------------------------------------------------------------------
@rem Windows shell script for building MUMPS using the scivision/mumps CMake project.
@rem - Requires git, cmake, and the Intel openAPI HPC toolkit (for a Fortran compiler and MKL).
@rem - Place in an arbitrary temporary directory.
@rem - Specify the install directory.
@rem - Run the script (.\buildMUMPS.bat).
@rem   IMPORTANT: to access the Intel Fortran compiler and Intel MKL, run from an Intel oneAPI command prompt!
@rem - The install directory will contain (under subdirectories of MUMPS_INSTALL_DIR/cmake) the MUMPS CMake
@rem   project configuration script required to configure Chrono with the Chrono::MUMPS module enabled.
@rem
@rem Notes:
@rem - The script accepts 1 optional argument to override the install directory.
@rem - This script builds MUMPS with settings appropriate for use in Chrono:
@rem   - single and double precision support, no complex support
@rem   - build static MUMPS libraries
@rem   - OpenMP support enabled
@rem   - no MPI support
@rem   Modify the CMake configuration below to enable other options (e.g., alternative ordering algorithms).
@rem -------------------------------------------------------------------------------------------------------

set MUMPS_INSTALL_DIR="C:/Packages/mumps"

@rem Specify Fortran compiler
set FORTRAN_COMPILER=ifx

@rem Build MUMPS debug libraries
set BUILD_DEBUG=ON

@rem Allow overriding installation directory through command line argument
if "%~1" NEQ "" (
   set MUMPS_INSTALL_DIR=%1
)
rmdir /S/Q %MUMPS_INSTALL_DIR% 2>nul

echo "----------------------------- Download sources from GitHub"

rmdir /S/Q download_mumps 2>nul
mkdir download_mumps

@rem Note: use custom fork of scivision/mumps
git clone "https://github.com/projectchrono/mumps.git" download_mumps

echo "----------------------------- Configure mumps"

echo %FORTRAN_COMPILER%

rmdir /S/Q build_mumps 2>nul

cmake -B build_mumps -S download_mumps ^
      -T fortran=%FORTRAN_COMPILER% ^
      -DBUILD_SINGLE=on ^
      -DBUILD_DOUBLE=on ^
      -DBUILD_SHARED_LIBS=off ^
      -DMUMPS_openmp=on ^
      -DMUMPS_parallel=off ^
      -DCMAKE_DEBUG_POSTFIX=_d ^
      --install-prefix %MUMPS_INSTALL_DIR%

echo "------------------------ Build and install mumps"
 
cmake --build build_mumps --config Release
cmake --install build_mumps --config Release --prefix %MUMPS_INSTALL_DIR%
if %BUILD_DEBUG% EQU ON (
     cmake --build build_mumps --config Debug
     cmake --install build_mumps --config Debug --prefix %MUMPS_INSTALL_DIR%
) else (
     echo "No Debug build of mumps"
)
 