REM 
REM Execute this script to build all the chrono::engine SDK 
REM that is:
REM   - the library for the MSVC compiler
REM   - the library for the GNU MingW compiler
REM   - the doxygen documentation
REM   - the SDK installer (full, and simple) to distribute!
REM
REM Remmeber that you must execute this batch after having 
REM executed vcvars32.bat or inside the shell window of VisualC,
REM otherwise the 'cl' compiler command won't be found.


echo 
echo --------------------------------------------------------
echo STEP 1 : DELETING ALL .O  OBJECT FILES....

nmake /f "make-chrono_lib" clean

echo 
echo --------------------------------------------------------
echo STEP 1a : MAKING THE LIBRARY FOR MINGW GCC COMPILER - DEBUG

nmake /f "make-chrono_lib" build=DEBUG compiler=COMPILER_GCC lib



echo 
echo --------------------------------------------------------
echo STEP 1b : DELETING ALL .O  OBJECT FILES....

nmake /f "make-chrono_lib" clean

echo 
echo --------------------------------------------------------
echo STEP 1c : MAKING THE LIBRARY FOR MINGW GCC COMPILER

nmake /f "make-chrono_lib" build=RELEASE compiler=COMPILER_GCC lib




echo
echo --------------------------------------------------------
echo STEP 2 : DELETING ALL .O  OBJECT FILES....

nmake /f "make-chrono_lib" clean

echo 
echo --------------------------------------------------------
echo STEP 2a : MAKING THE LIBRARY FOR MSVC COMPILER - DEBUG

nmake /f "make-chrono_lib" build=DEBUG compiler=COMPILER_MSVC lib

echo
echo --------------------------------------------------------
echo STEP 2 : DELETING ALL .O  OBJECT FILES....

nmake /f "make-chrono_lib" clean

echo 
echo --------------------------------------------------------
echo STEP 2a : MAKING THE LIBRARY FOR MSVC COMPILER

nmake /f "make-chrono_lib" build=RELEASE compiler=COMPILER_MSVC lib




echo 
echo --------------------------------------------------------
echo STEP 3 : MAKING THE EXAMPLES FOR MSVC

cd ../demos
nmake /f "makefile" build=RELEASE compiler=COMPILER_MSVC all
cd ../source





echo 
echo --------------------------------------------------------
echo STEP 4 : MAKING THE DOXYGEN DOCUMENTATION

nmake /f "make-chrono_lib" doxygen



echo 
echo --------------------------------------------------------
echo STEP 5 : MAKING THE SETUP.EXE INSTALLERS

echo ***DISACTIVATED***  cd installer
echo ***DISACTIVATED***  "C:\Programmi\Inno Setup 5\ISCC" "installer_engine.iss"
echo ***DISACTIVATED***  "C:\Programmi\Inno Setup 5\ISCC" "installer_engine_full.iss"
echo ***DISACTIVATED***  cd .. 
 
