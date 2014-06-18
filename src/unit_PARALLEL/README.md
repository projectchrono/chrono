chrono-parallel
===============

------------
Build Instructions for chumps using Windows/MSVC. (Note: these are for 64-bit windows builds, using MSVC 2010.)

First, download the following external dependencies for OpenGL and some math:

1) GLEW: OpenGL Extension Wrangler Library (binaries are acceptable)

2) GLM: OpenGL Mathematics (only headers)

3) GLFW: multi-platform OpenGL library for creating windows and inputs/events (need to compile this one)

Second, Build the GLFW libraries by using the included CMake files. 
(Optional) enable BUILD_SHARED_LIBS when generating your project with CMake.

Third, set-up the chrono-parallel CMake file with the following configuration settings:

1) ENABLE_OPENGL, ENABLE_TESTS, ENABLE_UTILS boxes should all be checked

2) CHRONO_INC should be set to ".../Chrono/src"

3) CHRONO_LIB_PATH should be set to ".../Chrono-Build/lib"

4) GLEW_INCLUDE_DIR set to ".../glew-1.10.0/include"

5) GLEW_LIBRARY set to ".../glew-1.10.0/lib/Release/x64/glew32.lib"

6) GLFW_INCLUDE_DIR set to ".../glfw-3.0.4/glfw-3.0.4/include/GLFW"

7) GLFW_LIBRARY set to ".../glfw-3.0.4/glfw-Build/src/Release/glfw3dll.lib"

8) GLM_INCLUDE_DIR set to ".../glm-0.9.5.3"

Configuring the CMake file should report that Chrono lib has been found and copied, as well as GLFW.

------------
How to run the demos:

Copy the .dll file associated with GLEW and GLFW also need to be copied over.

Ensure you've pulled from the "radu-demos" branch. This will result in the CMake file having an additional ENABLE_DEMOS box you should check.

Copy over the openGL resource files from ".../chrono-parallel/chrono_utils/opengl/resources" to your build directory.

Manually create the folder where the output for your particular demo will be written to. E.g., for test_ball, create the folder "../TEST_BALL_DEM" or "../TEST_BALL_DVI", depending on if you're using the DEM or DVI solver. (make the folder in the same directory as the Debug and Release folders).


------------
Debugging inside of MSVC:

In project properties, under "configuration settings --> Debugging", change working directory from ${ProjectDir} to ${OutDir}.

Copy over glew32.dll and glfw3.dll. Also, copy over the Debug version of ChronoEngine.dll

------------
TODO: copy these over automatically: (*windows builds only have this problem)

glew32.dll

glfw3.dll

TODO: create output folders, for any created demos. (windows builds only) Depends on selection of DEM or DVI solver (should be specified in the CMake config process).

TODO: copy over the opengl resource files automatically.

TODO: copy over "../bin/Debug/ChronoEngine.dll" to the debug directory.
