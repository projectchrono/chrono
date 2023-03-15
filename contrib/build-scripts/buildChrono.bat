@rem -----------------------------------------------------------------------------------------
@rem Windows batch script for configuring and building Chrono.
@rem -----------------------------------------------------------------------------------------
@rem The structure of the install directories for VSG and GL libraries is assumed to be as
@rem created by running the buildVSG and buildGL scripts. If using alternative installations
@rem of these libraries, modify as appropriate the CMake variables with paths to the various
@rem project configuration scripts.
@rem -----------------------------------------------------------------------------------------
@rem 1. As needed, use the provided scripts to build and install various dependencies.
@rem 2. Edit this script to specify the installation directories for the various dependencies.
@rem    Dependencies for disabled Chrono modules should not be set (comment them out).
@rem 3. Edit the CMake command to disable selected Chrono modules.
@rem 4. Run the script from a VS development console.
@rem -----------------------------------------------------------------------------------------

set SOURCE_DIR="."
set BUILD_DIR="Build/chrono"
set INSTALL_DIR="Install/chrono"

set EIGEN3_INSTALL_DIR="C:/Users/builder/Documents/Eigen3"
set BLAZE_INSTALL_DIR="C:/Users/builder/Documents/blaze-3.8.1"
set CASCADE_INSTALL_DIR="C:/OpenCASCADE-7.4.0-vc14-64/opencascade-7.4.0"
set SPECTRA_INSTALL_DIR="C:/Users/builder/Documents/spectra"
set MATLAB_INSTALL_DIR="C:/Program Files/MATLAB/R2019a"
set CRG_INSTALL_DIR="C:/Users/builder/Documents/OpenCRG"

set IRRLICHT_INSTALL_DIR="C:/Users/builder/Documents/irrlicht-1.8.5"
set VSG_INSTALL_DIR="C:/Users/builder/Documents/vsg_test"
set GL_INSTALL_DIR="C:/Users/builder/Documents/gl"

set OPTIX_INSTALL_DIR="C:/Program Files/NVIDIA Corporation/OptiX SDK 7.5.0"
set FASTRTPS_INSTALL_DIR="C:/Program Files/eProsima/fastrtps 2.4.0"

set PYTHON_EXECUTABLE_DIR="C:/Users/builder/miniconda3/python.exe"
set SWIG_EXE="C:/Users/builder/Documents/swigwin-4.0.2/swig.exe"

rem ------------------------------------------------------------------------

cmake -B %BUILD_DIR% -S %SOURCE_DIR% ^
      -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_DIR% ^
      -DENABLE_MODULE_IRRLICHT:BOOL=ON ^
      -DENABLE_MODULE_VSG:BOOL=OFF ^
      -DENABLE_MODULE_OPENGL:BOOL=ON ^
      -DENABLE_MODULE_VEHICLE:BOOL=ON ^
      -DENABLE_MODULE_POSTPROCESS:BOOL=ON ^
      -DENABLE_MODULE_MULTICORE:BOOL=ON ^
      -DENABLE_MODULE_FSI:BOOL=ON ^
      -DENABLE_MODULE_GPU:BOOL=ON ^
      -DENABLE_MODULE_DISTRIBUTED:BOOL=ON ^
      -DENABLE_MODULE_PARDISO_MKL:BOOL=ON ^
      -DENABLE_MODULE_CASCADE:BOOL=ON ^
      -DENABLE_MODULE_COSIMULATION:BOOL=ON ^
      -DENABLE_MODULE_SENSOR:BOOL=ON ^
      -DENABLE_MODULE_MODAL:BOOL=ON ^
      -DENABLE_MODULE_MATLAB:BOOL=OFF ^
      -DENABLE_MODULE_CSHARP:BOOL=ON ^
      -DENABLE_MODULE_PYTHON:BOOL=ON ^
      -DENABLE_MODULE_SYNCHRONO:BOOL=ON ^
      -DBUILD_BENCHMARKING:BOOL=ON ^
      -DBUILD_TESTING:BOOL=ON ^
      -DENABLE_OPENCRG:BOOL=ON ^
      -DEIGEN3_INCLUDE_DIR:PATH=%EIGEN3_INSTALL_DIR% ^
      -DIRRLICHT_INSTALL_DIR:PATH=%IRRLICHT_INSTALL_DIR% ^
      -DBLAZE_INSTALL_DIR:PATH=%BLAZE_INSTALL_DIR% ^
      -DOptiX_INSTALL_DIR:PATH=%OPTIX_INSTALL_DIR% ^
      -Dfastrtps_INSTALL_DIR:PATH=%FASTRTPS_INSTALL_DIR% ^
      -DGLEW_DIR=%GL_INSTALL_DIR%/lib/cmake/glew ^
      -DGLFW3_DIR=%GL_INSTALL_DIR%/lib/cmake/glfw3 ^
      -DGLM_INCLUDE_DIR:PATH=%GL_INSTALL_DIR%/include ^
      -DOpenCRG_INCLUDE_DIR:PATH=%CRG_INSTALL_DIR%%/include ^
      -DOpenCRG_LIBRARY:FILEPATH=%CRG_INSTALL_DIR%%/lib/OpenCRG.lib ^
      -DOpenCASCADE_DIR:PATH=%CASCADE_INSTALL_DIR%/cmake ^
      -DSPECTRA_INCLUDE_DIR:PATH=%SPECTRA_INSTALL_DIR%/include ^
      -DMATLAB_SDK_ROOT:PATH=%MATLAB_INSTALL_DIR%/extern ^
      -Dvsg_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsg ^
      -DvsgImGui_DIR:PATH=%VSG_INSTALL_DIR%/lib/cmake/vsgImGui ^
      -DvsgXchange_DIR:PATH=%VSG_INSTALL_DIR%%/lib/cmake/vsgXchange ^
      -DSWIG_EXECUTABLE:FILEPATH=%SWIG_EXE% ^
      -DUSE_CUDA_NVRTC:BOOL=OFF ^
      -DUSE_FAST_DDS:BOOL=ON ^
      -DCMAKE_PREFIX_PATH:PATH=%FASTRTPS_INSTALL_DIR%/cmake ^
      -DPYTHON_EXECUTABLE:PATH=%PYTHON_EXECUTABLE_DIR%

rem ------------------------------------------------------------------------

rem cmake --build %BUILD_DIR% --config Release
rem cmake --build %BUILD_DIR% --config Debug
