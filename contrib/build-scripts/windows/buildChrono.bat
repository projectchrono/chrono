@rem -----------------------------------------------------------------------------------------
@rem Windows batch script for configuring and building Chrono.
@rem -----------------------------------------------------------------------------------------
@rem The structure of the install directories for VSG and GL libraries is assumed to be as
@rem created by running the buildVSG and buildGL scripts. If using alternative installations
@rem of these libraries, modify as appropriate the CMake variables with paths to the various
@rem project configuration scripts.
@rem -----------------------------------------------------------------------------------------
@rem 1. As needed, use the provided scripts to build and install various dependencies.
@rem 2. Copy this script in an arbitrary location, outside the Chrono source tree.
@rem 3. Edit the script to specify the installation directories for the various dependencies.
@rem    Dependencies for disabled Chrono modules are ignored.
@rem 4. As needed, modify the CMake generator (BUILDSYSTEM).
@rem 5. Edit the CMake command to enable/disable Chrono modules.
@rem 6. Run the script (./builChrono.bat).
@rem -----------------------------------------------------------------------------------------

set SOURCE_DIR="C:/Source/chrono"
set BUILD_DIR="C:/Build/chrono"
set INSTALL_DIR="C:/Install/chrono"

@rem -------------------------------------------

set EIGEN3_INSTALL_DIR="C:/Packages/eigen"

set BLAZE_ROOT="C:/Packages/blaze"
set SPECTRA_INCLUDE_DIR="C:/Packages/spectra/include"
set CRG_INSTALL_DIR="C:/Packages/openCRG"

set SWIG_INSTALL_DIR="C:/Packages/swigwin"

set IRRLICHT_ROOT="C:/Packages/irrlicht"
set VSG_INSTALL_DIR="C:/Packages/vsg"
set GL_INSTALL_DIR="C:/Packages/gl"

set URDF_INSTALL_DIR="C:/Packages/urdf"

set MATLAB_INSTALL_DIR="C:/Program Files/MATLAB/R2019a"
set CASCADE_INSTALL_DIR="C:/OpenCASCADE-7.4.0-vc14-64/opencascade-7.4.0"
set OPTIX_INSTALL_DIR="C:/Program Files/NVIDIA Corporation/OptiX SDK 7.5.0"
set FASTDDS_INSTALL_DIR="C:/Program Files/eProsima/fastrtps 2.4.0"
set PYTHON_EXECUTABLE_DIR="C:/Python39/python.exe"

@rem -------------------------------------------

set BUILDSYSTEM="Visual Studio 17 2022"

rem ------------------------------------------------------------------------

cmake -G %BUILDSYSTEM% -B %BUILD_DIR% -S %SOURCE_DIR% ^
      -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_DIR% ^
      -DCH_ENABLE_MODULE_PARSERS:BOOL=ON ^
      -DCH_ENABLE_MODULE_IRRLICHT:BOOL=ON ^
      -DCH_ENABLE_MODULE_VSG:BOOL=ON ^
      -DCH_ENABLE_MODULE_OPENGL:BOOL=ON ^
      -DCH_ENABLE_MODULE_VEHICLE:BOOL=ON ^
      -DCH_ENABLE_MODULE_POSTPROCESS:BOOL=ON ^
      -DCH_ENABLE_MODULE_MULTICORE:BOOL=ON ^
      -DCH_ENABLE_MODULE_FSI:BOOL=ON ^
      -DCH_ENABLE_MODULE_DEM:BOOL=ON ^
      -DCH_ENABLE_MODULE_PARDISO_MKL:BOOL=ON ^
      -DCH_ENABLE_MODULE_CASCADE:BOOL=ON ^
      -DCH_ENABLE_MODULE_COSIMULATION:BOOL=ON ^
      -DCH_ENABLE_MODULE_SENSOR:BOOL=ON ^
      -DCH_ENABLE_MODULE_MODAL:BOOL=ON ^
      -DCH_ENABLE_MODULE_MATLAB:BOOL=ON ^
      -DCH_ENABLE_MODULE_CSHARP:BOOL=ON ^
      -DCH_ENABLE_MODULE_PYTHON:BOOL=ON ^
      -DCH_ENABLE_MODULE_SYNCHRONO:BOOL=ON ^
      -DBUILD_BENCHMARKING:BOOL=ON ^
      -DBUILD_TESTING:BOOL=ON ^
      -DCH_ENABLE_OPENCRG:BOOL=ON ^
      -DCH_USE_CUDA_NVRTC:BOOL=OFF ^
      -DCH_USE_FAST_DDS:BOOL=ON ^
      -DEIGEN3_INCLUDE_DIR:PATH=%EIGEN3_INSTALL_DIR% ^
      -DIrrlicht_ROOT:PATH=%IRRLICHT_ROOT% ^
      -DBlaze_ROOT:PATH=%BLAZE_ROOT% ^
      -DOptiX_INSTALL_DIR:PATH=%OPTIX_INSTALL_DIR% ^
      -DFastDDS_ROOT:PATH=%FASTDDS_INSTALL_DIR% ^
      -DGLEW_DIR=%GL_INSTALL_DIR%/lib/cmake/glew ^
      -Dglfw3_DIR=%GL_INSTALL_DIR%/lib/cmake/glfw3 ^
      -DGLM_INCLUDE_DIR:PATH=%GL_INSTALL_DIR%/include ^
      -DOpenCRG_INCLUDE_DIR:PATH=%CRG_INSTALL_DIR%%/include ^
      -DOpenCRG_LIBRARY:FILEPATH=%CRG_INSTALL_DIR%%/lib/OpenCRG.lib ^
      -DOpenCASCADE_DIR:PATH=%CASCADE_INSTALL_DIR%/cmake ^
      -DSpectra_INCLUDE_DIR:PATH=%SPECTRA_INCLUDE_DIR%/include ^
      -DMATLAB_SDK_ROOT:PATH=%MATLAB_INSTALL_DIR%/extern ^
      -Dvsg_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsg ^
      -DvsgImGui_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsgImGui ^
      -DvsgXchange_DIR:PATH=%VSG_INSTALL_DIR%%/lib/cmake/vsgXchange ^
      -Durdfdom_DIR:PATH=%URDF_INSTALL_DIR%/CMake ^
      -Durdfdom_headers_DIR:PATH=%URDF_INSTALL_DIR%/CMake ^
      -Dconsole_bridge_DIR:PATH=%URDF_INSTALL_DIR%/CMake ^
      -Dtinyxml2_DIR:PATH=%URDF_INSTALL_DIR%/CMake ^
      -DSWIG_EXECUTABLE:FILEPATH=%SWIG_INSTALL_DIR%/swig.exe ^
      -DPYTHON_EXECUTABLE:PATH=%PYTHON_EXECUTABLE_DIR%

rem ------------------------------------------------------------------------

rem cmake --build %BUILD_DIR% --config Release
rem cmake --build %BUILD_DIR% --config Debug
