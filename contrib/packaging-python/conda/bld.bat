REM To avoid building in work/ alongside the source. Rather build in work/build/
mkdir build
cd build
REM Getting VS variables
mkdir attempt_to_run_vsbat_file
REM call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\LaunchDevCmd.bat"
mkdir ran_vsbat_file
REM Remove dot from PY_VER for use in library name
set MY_PY_VER=%PY_VER:.=%
REM set env variables needed by MKL
set MKL_INTERFACE_LAYER = LP64
set MKL_THREADING_LAYER = INTEL
set CONFIGURATION=Release

REM Renaming numpy conda package
del *.tar.bz2
powershell -ExecutionPolicy Bypass -File "%CI_PROJECT_DIR%\contrib\packaging-python\conda\script.ps1"

REM Configure step

REM THIS STATIC LIBRARY DOESN'T APPEAR ANYMORE, USE THE VERSION FROM THE INTEL oneAPI COMPILER SUITE -DIOMP5_LIBRARY="%PREFIX%"\Library\lib\libiomp5md.lib ^

REM For chrono::sensor, we are using the machine's CUDA and optix installation since anaconda does not have the up-to-date packages we need
REM Keep an eye on this in the future. Ideally, all packages we use to build pyChrono should come from anaconda

mkdir cmake_began
cmake -G "Visual Studio 17 2022" -T "v142" ^
 -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
 -DCMAKE_PREFIX_PATH="%LIBRARY_PREFIX%" ^
 -DCMAKE_SYSTEM_PREFIX_PATH="%LIBRARY_PREFIX%" ^
 -DCH_CONDA_INSTALL=ON ^
 -DCH_INSTALL_PYTHON_PACKAGE="%SP_DIR%" ^
 -DCH_PYCHRONO_DATA_PATH="../../../Library/data" ^
 -DCH_PYCHRONO_SHADER_PATH="../../../Library/lib/sensor_ptx" ^
 -DPYTHON_EXECUTABLE:FILEPATH="%PYTHON%" ^
 -DPYTHON_INCLUDE_DIR:PATH="%PREFIX%"/include ^
 -DPYTHON_LIBRARY:FILEPATH="%PREFIX%"/libs/python%MY_PY_VER%.lib ^
 -DSWIG_EXECUTABLE="%PREFIX%"/Library/bin/swig.exe ^
 -DCMAKE_BUILD_TYPE="%CONFIGURATION%" ^
 -DENABLE_MODULE_IRRLICHT=ON ^
 -DENABLE_MODULE_POSTPROCESS=ON ^
 -DENABLE_MODULE_VEHICLE=ON ^
 -DENABLE_MODULE_PYTHON=ON ^
 -DENABLE_MODULE_SENSOR=ON ^
 -DNUMPY_INCLUDE_DIR="C:/Users/builder/miniconda3/envs/build-env/pkgs/numpy-base/Lib/site-packages/numpy/core/include/" ^
 -DOptiX_INSTALL_DIR="C:/Program Files/NVIDIA Corporation/OptiX SDK 7.2.0" ^
 -DCUDA_TOOLKIT_ROOT_DIR="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.7" ^
 -DGLFW_DLL="C:/Users/builder/Documents/glfw-3.3.5/lib-vc2019/glfw3.dll" ^
 -DGLFW_INCLUDE_DIR="C:/Users/builder/Documents/glfw-3.3.5/include" ^
 -DGLFW_LIBRARY="C:/Users/builder/Documents/glfw-3.3.5/lib-vc2019/glfw3dll.lib" ^
 -DBUILD_DEMOS=OFF ^
 -DIRRLICHT_ROOT="%PREFIX%"/Library/include/irrlicht ^
 -DIRRLICHT_LIBRARY="%PREFIX%"/Library/lib/Irrlicht.lib ^
 -DBUILD_TESTING=OFF ^
 -DBUILD_GMOCK=OFF ^
 -DBUILD_BENCHMARKING=OFF ^
 -DENABLE_MODULE_CASCADE=ON ^
 -DCASCADE_INCLUDE_DIR="%PREFIX%"/Library/include/opencascade ^
 -DCASCADE_LIBDIR="%PREFIX%"/Library/lib ^
 -DENABLE_MODULE_PARDISO_MKL=ON ^
 -DMKL_INCLUDE_DIR="%PREFIX%"/Library/include ^
 -DMKL_RT_LIBRARY="%PREFIX%"/Library/lib/mkl_rt.lib ^
 -DIOMP5_LIBRARY="C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/compiler/lib/intel64_win/libiomp5md.lib" ^
 -DPYCHRONO_DATA_PATH="..\..\..\..\..\Library\data" ^
 ..
if errorlevel 1 exit 1
mkdir cmake_ended

REM Build step 
mkdir build_began
cmake --build . --config Release
if errorlevel 1 exit 1
mkdir build_ended

REM Install step 
mkdir install_began
cmake --build . --config Release --target install
if errorlevel 1 exit 1
mkdir install_ended

REM Install step
REM ninja install
REM if errorlevel 1 exit 1

