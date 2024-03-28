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
  Before customizing it for your own purposes it is highly recommended to first try to build the predefined example so to make sure to have all the other dependencies in place.

- Only two elements should be customized:
  - request the `COMPONENTS` and `OPTIONAL_COMPONENTS` required by your project in the call to `find_package`:  
    ~~~{.c}
    find_package(Chrono
                COMPONENTS <list of required components>
                OPTIONAL_COMPONENTS <list of optional components>
                CONFIG)
    ~~~
    For example,
    ~~~{.c}
    find_package(Chrono
                COMPONENTS Irrlicht
                OPTIONAL_COMPONENTS Postprocess
                CONFIG)
    ~~~
    Please mind that the `OPTIONAL_COMPONENTS` argument can be omitted if none.  
    The list of available module is shown during compilation of the Chrono solution. Surely the most common is the visual interface "Irrlicht".
  - the list of source files and the name of the target in the call to `add_executable`:  
    ~~~{.c}
    add_executable(myRobot robot_grip.cpp  robot_motors.cpp)
    ~~~

It is highly recommended to:
- ask only for those `COMPONENTS` that are truly required;
- do not modify any other line, especially for beginner users;
- start your project by using the source file present in the `template_project` folder, not from a blank page;
    

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

To make things simple, we added an auxiliary CMake target (namely COPY_DLLS) that makes a copy of the Chrono DLLs (as well as other DLL dependencies such as the Irrlicht shared library, as appropriate) in the project binaries folder (e.g. `C:\workspace\my_project_build\Release`). However, additional third-party libraries might require to change the `PATH` variable or copy the DLLs in the same folder as the executable.


<div class="ce-warning">
The COPY_DLLS target must be re-run (i.e. rebuilt) any time the Chrono library had changed.
</div>

<div class="ce-warning">
Your project has to be compiled with the same build configuration as Chrono. For example, if Chrono was built in Release mode, your project must also be built in Release.
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