Building a project that uses Chrono             {#tutorial_install_project}
==========================


When you develop a C++ project and you want to use the Chrono API,
you need to:
- **include** the necessary .h headers at compile time;
- **link** the necessary .lib libraries at link time;
- **dynamically link** the necessary .dll libraries at run time.  

as shown below: <br>

<img src="http://www.projectchrono.org/assets/manual/Pic_build.png" class="img-responsive">

<br>
This process can be made almost automatic if you use CMake for building your program, see below.

<div class="ce-info">
The CMake path is not the only option. For example, you could add Chrono libraries
to your C++ project by manually inserting libraries in the IDE settings. However,
the method described here is the one that we suggest in most cases since it is platform-independent as it draws on the CMake
build tool.
</div>



1) Check prerequisites:
-------------------------------------------------------------------

-   [CMake](http://www.cmake.org) must be already installed on
    your computer.
	
-   Chrono must be **already installed** and **built** 
    on your computer, as [explained here](@ref tutorial_install_chrono).

	
2) Create the project directory
-------------------------------------------------------------------

-   Copy the `template_project` directory to some place and rename it as you like.
    For example copy from `C:\workspace\chrono\template_project` to `C:\workspace\my_project`

This will be the directory where you put your source code.


3) Edit the CMake script
------------------------------------------------------------

- In the template directory there is already a CMakeLists.txt script. It will be used by CMake.
  **Optionally** you might edit it, see below.

<div class="ce-info">
For example, suppose you want to use also the [POSTPROCESS module](group__postprocess__module.html).
You can edit CMakeLists.txt where it reads:
~~~{.c}
find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG)
~~~
and add the required module:
~~~{.c}
find_package(Chrono
             COMPONENTS Irrlicht Postprocess
             CONFIG)
~~~
The same is done for any other Chrono module.  The list of available modules is: `Cascade`, `Cosimulation`, `FSI`, `Granular`, `Irrlicht`, `Matlab`, `MKL`, `MUMPS`, `OpenGL`, `Parallel`, `Postprocess`, `Python`, `Vehicle`. The syntax is case insensitive (in other words, any of `OpenGL`, `OPENGL`, or `opengl` are valid component names).
<br><br>
Note also that Chrono modules can be listed as **required** (after the `COMPONENTS` option) or as **optional** (after `OPTIONAL_COMPONENTS`) as in the example below:
~~~{.c}
find_package(Chrono
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS Postprocess
             CONFIG)
~~~
In the former case, CMake processing is stopped if Chrono was not configured and built with one of the required modules enabled.  In the latter case, CMake processing continues.  When listing Chrono modules as optional components, it is the responsibility of the user to appropriately check in their project code whether or not that Chrono module is in fact available; e.g., by using the macros defined in `ChConfig.h` (`CHRONO_IRRLICHT`, `CHRONO_POSTPROCESS`, etc) which are defined for enabled modules.
<br><br>
For more details, see the CMake documentation for `find_package` and its "Config Mode".
</div>

<div class="ce-info">
If you prefer to change the name of the default *my_example.cpp* to 
something more meaningful, say my_simulator.cpp, just rename that file and then change the line<br>
 `add_executable(myexe my_example.cpp)` <br>
 into <br>
 `add_executable(myexe my_simulator.cpp)`<br>
</div>

<div class="ce-info">
If your program is split in multiple .cpp sources, simply list them in this line:
 `add_executable(myexe my_simulator.cpp  my_foo_source.cpp  my_bar_source.cpp)` 
</div>

<div class="ce-warning">
As of version 2017 15.8, Visual Studio has changed its handling of aligned memory allocation.
To properly deal with aligned memory allocation for classes that have members fixed-size vectorizable Eigen objects, make sure your CMakeLists.txt **retains** lines 104-108:
~~~
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()
~~~
which conditionally defines `_ENABLE_EXTENDED_ALIGNED_STORAGE`.
</div>

4) Start CMake 
--------------------------------------------------

-   Start the CMake GUI

-   Use **Browse source...** by setting the source directory that you
    created, e.g. `C:/workspace/my_project`
	
-   Use **Browse build...** by setting a new *empty* directory for the
    output project, e.g. `C:/workspace/my_project_build`

	
5) Configure
------------------------------------------------

- Press the **Configure** button in CMake

- When prompted to pick a generator, select the same 
  compiler that you used to compile Chrono;

- Important: set the `Chrono_DIR` variable. This is the **cmake** directory that 
  you find in the directory where you built Chrono. In our example it is `C:/workspace/chrono_build/cmake`

- Press the **Configure** button again

<img src="http://www.projectchrono.org/assets/manual/Install_my_project_cmake.png" class="img-responsive">


6) Generate the project
-----------------------------------------------------------

- Press the **Generate** button in CMake. A project will be created in
  the build directory. In our example you will find it in `C:/workspace/my_project_build`


7) Compile the project
----------------------------------------------------------

If you used a Visual Studio generator in CMake, 

-   **Open the solution** in Visual Studio editor (in our example double
    click on `C:\workspace\my_project_build\my_project.sln`)

-   Change from Debug to **Release** mode, using the drop-down list in the 
    toolbar of VisualStudio
	
-   Use the menu **Build** > **Build solution...** to compile and link.

If you used a Unix makefile generator in CMake (ex. in Linux), you can
open a shell, cd into the build directory and call **make**: 
~~~{.c}
cd /home/john/my_project_build
make
~~~ 


8) Run your program
-------------------------------------------------------

By default all binaries (ex. myexe.exe) of your project will go into
your build directory, in our example `C:\workspace\my_project_build\Release` 
(or `C:\workspace\my_project_build\Debug` if you decided to compile in Debug mode).

Double click on the .exe file and you should see the demo running in an interactive 3D view
<img src="http://projectchrono.org/assets/manual/Install_my_project_2.jpg" class="img-responsive">

Important information for Windows users
----------------------

If you are a Windows users, your project executables need to know where to find the Chrono shared libraries (i.e. those with .dll extension), otherwise they will crash as soon as you try to run them.

To make things simple, we added an auxiliary CMake target (namely COPY_DLLS) that makes a copy of the Chrono DLLs (as well as other DLL dependencies such as the Irrlicht shared library, as appropriate) in the project binaries folder (e.g. `C:\workspace\my_project_build\Release`).  There are two different scenarios:
  + `Chrono_DIR` is a _build_ folder; if you followed our [installation guide](@ref tutorial_install_chrono) then `C:/workspace/chrono_build` is of this type; in this case you may have both Release and Debug version of Chrono (under `C:/workspace/chrono_build/bin`); if both are present, please mind that **only Release libraries will be copied**; if you want to run your project in Debug configuration then you have to manually copy the Debug libraries contained in `C:/workspace/chrono_build/bin/Debug` into `C:/workspace/my_project_build/Debug`;
  + `Chrono_DIR` is an _install_ folder; this is the folder type that is created by building the `INSTALL` target; this folder is usually `C:/Program Files/Chrono` and contains either Debug or Release libraries, depending on the configuration that you set when you built the `INSTALL` target.


<div class="ce-warning">
The COPY_DLLS target must be re-run (i.e. rebuilt) any time the Chrono library had changed.
</div>

<div class="ce-warning">
Your project has to be compiled with the same build configuration as Chrono. For example, if Chrono was built in Release mode, your project must also be built in Release.  Note that automatic copying of Chrono DLLs as described above is hard-coded for Release builds only.  For other build configuraitons (Debug, RelWithDebInfo, etc.) it is the user's responsibility to ensure the Chrono DLLs are found (e.g., by manually copying them from the Chrono build directory).
</div>

<img src="http://www.projectchrono.org/assets/manual/Install_project_COPYDLLS.png" class="img-responsive">

<div class="ce-info">
Linux users do not have to worry about copying shared libraries because the .so libraries always go into a directory that is globally visible.
</div>

Important information if using Chrono::Sensor
-----------------------------
**NOTE** if linking to Chrono::Sensor install from an external project, make sure to set the directory of the install location where the shader code (compiled ptx code or shaders/*.cu files) is located. This should be set at the top of any external code that will use chrono::sensor from an install location.
  ```cpp
    //function to set the shader location (include ChOptixUtils.h)
    chrono::sensor::SetSensorShaderDir("path/to/sensor/shaders");

    //if USE_CUDA_NVRTC is enabled, use
    chrono::sensor::SetSensorShaderDir("path/to/install/include/chrono_sensor/optix/shaders/");

    //if USE_CUDA_NVRTC is disabled, use
    chrono::sensor::SetSensorShaderDir("path/to/install/lib/sensor_ptx/");
  ```