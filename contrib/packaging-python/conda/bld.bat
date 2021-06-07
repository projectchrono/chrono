REM To avoid building in work/ alongside the source. Rather build in work/build/
mkdir build
cd build

REM Remove dot from PY_VER for use in library name
set MY_PY_VER=%PY_VER:.=%

REM set env variables needed by MKL
set MKL_INTERFACE_LAYER = LP64
set MKL_THREADING_LAYER = INTEL
set CONFIGURATION=Release
REM Configure step
cmake -G "Visual Studio 16 2019" ^
 -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
 -DCMAKE_PREFIX_PATH="%LIBRARY_PREFIX%" ^
 -DCMAKE_SYSTEM_PREFIX_PATH="%LIBRARY_PREFIX%" ^
 -DCH_INSTALL_PYTHON_PACKAGE="%SP_DIR%" ^
 -DPYTHON_EXECUTABLE:FILEPATH="%PYTHON%" ^
 -DPYTHON_INCLUDE_DIR:PATH="%PREFIX%"/include ^
 -DPYTHON_LIBRARY:FILEPATH="%PREFIX%"/libs/python%MY_PY_VER%.lib ^
 -DSWIG_DIR="%CONDA_INSTALL_LOCN%"/Library/bin/Lib ^
 -DSWIG_EXECUTABLE="%CONDA_INSTALL_LOCN%"/Library/bin/swig.exe ^
 -DCMAKE_BUILD_TYPE="%CONFIGURATION%" ^
 -DENABLE_MODULE_IRRLICHT=ON ^
 -DENABLE_MODULE_POSTPROCESS=ON ^
 -DENABLE_MODULE_VEHICLE=ON ^
 -DENABLE_MODULE_PYTHON=ON ^
 -DBUILD_DEMOS=OFF ^
 -DBUILD_TESTING=OFF ^
 -DBUILD_GMOCK=OFF ^
 -DBUILD_BENCHMARKING=OFF ^
 -DIRRLICHT_ROOT="C:\Users\builder\Documents\irrlicht-1.8.4" ^
 -DIRRLICHT_LIBRARY="C:\Users\builder\Documents\irrlicht-1.8.4\lib\Win64-visualStudio\Irrlicht.lib" ^
 -DEIGEN3_INCLUDE_DIR="C:\Users\builder\Documents\eigen-3.3.9" ^
 -DENABLE_MODULE_CASCADE=ON ^
 -DCASCADE_INCLUDE_DIR="%CONDA_INSTALL_LOCN%"\Library\include\opencascade ^
 -DCASCADE_LIBDIR="%CONDA_INSTALL_LOCN%"\Library\lib ^
 -DENABLE_MODULE_PARDISO_MKL=ON ^
 -DMKL_INCLUDE_DIR="%CONDA_INSTALL_LOCN%"\Library\include ^
 -DMKL_RT_LIBRARY="%CONDA_INSTALL_LOCN%"\Library\lib\mkl_rt.lib ^
 -DIOMP5_LIBRARY="%CONDA_INSTALL_LOCN%"\Library\lib\libiomp5md.lib ^
 -DPYCHRONO_DATA_PATH="..\..\..\..\..\Library\data" ^
 .. >> "%LOG_DIR%"\cmakeconfiglog.txt 2>&1
if errorlevel 1 exit 1
 
REM Build step 
cmake --build . --config "%CONFIGURATION%" >> "%LOG_DIR%"\cmakebuildlog.txt 2>&1
if errorlevel 1 exit 1

REM Install step 
cmake --build . --config "%CONFIGURATION%" --target install
if errorlevel 1 exit 1

REM Install step
REM ninja install
REM if errorlevel 1 exit 1

