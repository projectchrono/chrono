Building a project that uses Chrono             {#tutorial_install_project}
==========================


When you develop a C++ project and you want to use the Chrono API,
you need to:
- **include** the necessary .h headers at compile time,
- **link** the necessary .lib libraries at link time,
- **dynamically link** the necessary .dll libraries at run time, 

as shown below: <br>

![](Pic_build.png)

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
    For example copy from `C:\chrono_source\template_project` to `C:\my_project_source`

This will be the directory where you put your source code.


3) Edit the CMake script
------------------------------------------------------------

- In the template directory there is already a CMakeLists.txt script. It will be used by CMake.
  **Optionally** you might edit it, see below.

<div class="ce-info">
For example, suppose you want to use also the [postprocessing module](@ref module_postprocess_install).
You can edit CMakeLists.txt where it reads:
~~~{.c}
find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG)
~~~
and add the required module:
~~~{.c}
find_package(Chrono
             COMPONENTS Irrlicht Postprocessing
             CONFIG)
~~~
The same is done for other modules: `FEA`, `Matlab`, `Vehicle`, `Cosimulation`, etc.
</div>

<div class="ce-info">
If you prefer to change the name of the default my_example.cpp to 
something more meaningful, say my_simulator.cpp, just rename that file and then change the line
 `add_executable(myexe my_example.cpp)` 
 into 
 `add_executable(myexe my_simulator.cpp)`
</div>

<div class="ce-info">
If your program is split in multiple .cpp sources, simply list them in this line:
 `add_executable(myexe my_simulator.cpp  my_foo_source.cpp  my_bar_source.cpp)` 
</div>


4) Start CMake 
--------------------------------------------------

-   Start the CMake GUI

-   Use **Browse source...** by setting the source directory that you
    created, ex: `C:\my_project_source`
	
-   Use **Browse build...** by setting a new empty directory for the
    output project, ex: `C:\my_project_build`

	
5) Configure
------------------------------------------------

- Press the **Configure** button in CMake

- When prompted to pick a generator, select the same 
  compiler that you used to compile Chrono

- Important: set the `Chrono_DIR` variable. This is the **cmake** directory that 
  you find in the directory where you built Chrono. In our example it is `C:/chrono_build/cmake`

- Press the **Configure** button again

![](Install_my_project_1.gif)


6) Generate the project
-----------------------------------------------------------

- Press the **Generate** button in CMake. A project will be created in
  the build directory. In our example you will find it in `C:\my_project_build`


7) Compile the project
----------------------------------------------------------

If you used a VisualStudio generator in CMake, 

-   **Open the solution** in VisualStudio editor (in our example double
    click on `C:\my_project_build\my_project.sln`)

-   Change from Debug to **Release** mode, using the dropbox in the 
    toolbar of VisualStudio
	
-   Use the menu **BUILD / Build solution...** to compile and link.

If you used a Unix makefile generator in CMake (ex. in Linux), you can
open a shell, cd into the build directory and call **make**: 
~~~{.c}
cd /home/john/my_project_build
make
~~~ 


8) Run your program
-------------------------------------------------------

By default all binaries (ex. myexe.exe) of your project will go into
your build directory, in our example `C:\my_project_build\Release\my_project.sln` 
(or `C:\my_project_build\Debug\my_project.sln` if you decided to compile in Debug mode).

- double click on the .exe file and you should see the demo running in an interactive 3D view

![](Install_my_project_2.jpg)

Additional information
----------------------

For Windows users, some notes on copying the Chrono DLL:

- Your executable need to know where to find the needed .dll[s], 
otherwise it will crash as soon as you try to run it.

- To make things simple, at the end of the default CMakeFile.txt used in this example 
there is this instruction: `add_DLL_copy_command macro`. This takes care of **automatically copying** all the needed dlls right after each build. 
It also copies the Irrlicht.dll into your executable directory, if unit_IRRLICHT is used. 
  
<div class="ce-danger">
Currently, the `add_DLL_copy_command macro` feature **is not working properly when in Debug mode**. 
In fact it always copies the Release ddls of Chrono even in the Debug/ build directory of your project, 
and this will **cause the program to crash**. 
<br><br>
As a temporary workaround, if you want to compile your program in Debug/, please copy all the .dll files of
the Chrono build directory (ex. from  `C:\chrono_build\bin\Debug` to `C:\my_project_build\Debug`
<br><br>
This is not necessary for builds in Release mode.
</div>


<div class="ce-info">
Linux users do not have to worry about copying dlls because the .so libraries always go into into a directory that is globally visible.
</div>

