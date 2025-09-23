Building a project that uses Chrono {#tutorial_install_project}
==========================

An external C++ project that uses Chrono requires:
- access to the Chrono headers (at compile time)
- access to the Chrono libraries (at link time)
- access to shared libraries, `.dll` on Windows and `.so` on Linux (at run time)  

<img src="http://www.projectchrono.org/assets/manual/Pic_build.png" class="img-responsive">

<br>
This process is automated through the Chrono CMake configuration script generated during Chrono configuration and available in either the Chrono build tree (named here \<__chrono_build__\>) or a Chrono installation (named here \<__chrono_install__\>).<br>
The `chrono-config.cmake` script (\<chrono_build\>/cmake/chrono-config.cmake or \<chrono_install\>/cmake/chrono-config.cmake) allows finding the Chrono components and all necessary information about a given Chrono build or a Chrono installation, respectively, needed to link to and use Chrono libraries.

`chono-config.cmake` is used in a call to `find_package` in the project's CMakeLists.txt CMake configuration script:
\code{.c}
find_package(Chrono
             [COMPONENTS required_components...]
             [OPTIONAL_COMPONENTS optional_components>
             CONFIG)
\endcode
to find a Chono build or install which provides the `required_components` modules and, optionally, also the `optional_components` modules. Note that `OPTIONAL_COMPONENTS` can be missing if there are no optional components needed.

In the call to find_package(), the following Chrono components can be requested (case insensitive): 
Cascade, CSharp, FMI, FSI, GPU, Irrlicht, VSG, Matlab, Modal, Multicore, PardisoMKL, Parsers, Postprocess, Sensor, Synchrono, Vehicle, VehicleCosim.<br>
__Notes__: 
- The core Chrono module is included automatically as a required component.
- A particular component is available only if the targeted Chrono package was built with the corresponding module enabled.

`chrono-config.cmake` performs a recursive processing of all requested components to also enable any other Chrono modules that were built in as dependencies to one of the requested modules (or their dependencies).

Furthermore, `chrono-config.cmake` defaults to using the same configuration and build settings (e.g., C++ compiler, CUDA SDK, MPI), as well as the same settings for any 3rd-party dependencies (e.g., the VSG run-time visualization libraries) as those used during the configuration and build of the Chrono package. However, the caller has the option to redirect any and all of these.

On return from `find_package`, the following variables will be set:
- Chrono_FOUND
       set to true if Chrono and all required components were found
- CHRONO_TARGETS
       list of exported Chrono targets
- CHRONO_STATIC
       set to ON if Chrono static libraries were built, OFF if shared libraries
- CHRONO_MSVC_RUNTIME_LIBRARY
       MSVC run-time library used in building Chrono
- CHRONO_DLL_NAMES
       (Windows) list of Chrono DLLs (without path)
- CHRONO_CSHARP_SOURCES
       list of all SWIG-generated C# scripts corresponding to the requested components
       (currently, only the core, postprocess, Irrlicht, and Vehicle Chrono are wrapped)
- CHRONO_DATA_DIR
       path to the Chrono data directory
- CHRONO_VEHICLE_DATA_DIR
       path to the Chrono::Vehicle data directory
- CHRONO_FSI_DATA_DIR
       path to the Chrono::FSI data directory
- SYNCHRONO_DATA_DIR
       path to the Chrono::Synchrono data directory

In addition, for each requested component 'COMPONENT', a variable `CHRONO_<COMPONENT_UPPER>_FOUND` is set to `TRUE` or `FALSE` (where 'COMPONENT_UPPER' is the component name in upper case). These variable are meaningful only for Chrono modules requested as "optional" in the call to `find_package`, as a missing required component automatically sets `Chrono_FOUND` to `FALSE`.
<br>

---------------------------------------------------------------------
## Configuring and building an external project

The Chrono distribution includes template projects for different types of Chrono-based external projects. These template projects, available in the top-level directory of the Chrono source tree (or of a Chrono install tree) are:
- [template_project](https://github.com/projectchrono/chrono/tree/main/template_project) - a simple C++ project that build a simple pendulum mechanism visualized with Irrlicht
- [template_project_csharp](https://github.com/projectchrono/chrono/tree/main/template_project_csharp) - a simple C# project that builds a bouncing ball visualized with Irrlicht
- [template_project_fmi](https://github.com/projectchrono/chrono/tree/main/template_project_fmi2) - a simple C++ project that builds an FMU (see the [special instructions](@ref module_fmi_installation) for building Chrono with support for FMU generation)
- [template_project_ros](https://github.com/projectchrono/chrono/tree/main/template_project_ros) - a simple C++ project that ilustrates the use of Chrono::ROS from an external project
- [template_project_vehicle_cosim](https://github.com/projectchrono/chrono/tree/main/template_project_vehicle_cosim) - a simple MPI project that builds a Chrono::Vehicle co-simulation of a single wheel on rigid terrain

#### 1. Check prerequisites

At a minimum, the following packages are necessary:
- A C++ compiler and [CMake](http://www.cmake.org) must be available.
- A Chrono build or install must be **available** (see [Chrono installation instructions](@ref tutorial_install_chrono)).

__Note__: Using additional Chrono modules may require the CUDA SDK, an MPI compiler, or 3rd-party libraries such as Irrlicht, VSG, OptiX, OpenGL, Thrust, Blaze, etc.

#### 2. Create the project directory

- Copy the directory for the desired "template" project from the Chrono source (or installation) to a different location and rename it if desired. In what follows, we use **template_project** as an example and \<__my_project__\> as the name of the external project sources.
- The directory \<my_project\> will contain all source code for the new project.

#### 3. Edit the CMakeLists.txt script

- Use the sample **CMakeLists.txt** in the template project directory as a starting point.
  Before customizing it for your own purposes we suggest to first try to build the predefined example to make sure all necessary Chrono modules and dependencies are in place.

- The following elements of CMakeLists.txt should be customized:
  - specify the name of the project in the call to `project()`.

  - request the required (`COMPONENTS`) and optional (`OPTIONAL_COMPONENTS`) Chrono modules needed by you project in the call to `find_package(Chrono...)`. In our case:
    \code{.c}
    find_package(Chrono
                 COMPONENTS Irrlicht
                 OPTIONAL_COMPONENTS Postprocess
                 CONFIG)
    \endcode
    to request the Chrono::Irrlicht module and, if available, the Chrono::Postprocess module.<br>

  - set the list of source files and the name of the target in the call to `add_executable`:  
    \code{.c}
    set(MY_FILES my_example.cpp)
    add_executable(my_demo ${MY_FILES})
    \endcode

#### 4. Configure the project with CMake 

- Start `cmake-gui`

- Use **Browse source...** to set the location of the source directory (in our case \<my_project\>)
	
- Use **Browse build...** to set the location where binaries will be generated. While not required, it is good practice to use a build directory different from the source directory. Here, we assume that binaries will be generated in \<__my_project_build__\>.

- Press the **Configure** button

- Set the `Chrono_DIR` variable. This is the path to the directory that contains the `chrono-config.cmake` script and can therefore be \<chrono_build\>/cmake or \<chrono_install\>/cmake.

- Press the **Configure** button again

- Upon successful configuration of the project, `chrono-config.cmake` provides a list of the requested Chrono components (required or optional) and details on the Chrono configuration.

- Press the **Generate** button in CMake. Build files (depending on the generator selected) will be created in the project build directory \<my_project_build\>.

#### 5. Compile the project

If you used a Visual Studio generator in CMake
- **Open** the Visual Studio solution file generated by CMake (\<my_project_build\>/my_project.sln in our example)
- Select the desired build mode (e.g., **Release**, using the drop-down list in the Visual Studio toolbar
- **Build** the project (use the Visual Studio menu "Build>Build solution..." or the shortcut Ctrl-Shft-B) 

On Linux, assuming you use the Makefile generator, invoke the make command in the build directory:
\code{.c}
$ cd \<my_project_build\>
$ make
\endcode

__Note__: the project should be compiled with the same build configuration as Chrono (e.g., Release or Debug). When using a multi-configuration generator, different builds can exist at the same time in a Chrono build tree.

#### 6. Run your program

Run the executable `my_demo` in \<my_project_build\> (or a configuration-specific subdirectory, if using a multi-configuration generator such as Visual Studio) to start the simulation of the simple pendulum example.
<img src="http://projectchrono.org/assets/manual/Install_my_project_2.jpg" class="img-responsive">
<br>
---------------------------------------------------------------------

### Important information for Windows users

By default, all Chrono modules are built as shared libraries (**DLLs** on Windows). Furthermore, most 3rd-party dependencies are also available as shared libraries. The OS must be able to find all necessary DLLs at run-time.

To simplify things, `chrono-config.cmake` provides a function that copies the necessary Chrono DLLs from their location (in the Chrono build tree or in a Chrono installation) to the project's build directory (\<my_project_build\> in our example).
This is done by introducing a new build target (`COPY_DLLS`) to the Visual Studio solution which is executed **POST_BUILD**.
To enable the inclusion of this convenience target, the project's CMakeLists.txt script should call `add_CHRONO_DLLS_copy_command()` at the end. Note that this function is a no-op on platforms other than Windows.

### Important information if using Chrono::CSharp

A few of the Chrono modules are SWIG-wrapped for use in C# programs. These can be stand-alone C# programs (see the [C# demos](https://github.com/projectchrono/chrono/tree/main/src/demos/csharp) in the Chrono distribution) or else used in the companion [ChronoUnity](https://github.com/projectchrono/chrono-unity) package.

When configuring your C# project, besides enabling the CSharp language you may want to also enable C++:
```cpp
   project(my_csharp_project CSharp CXX)
```

This is to allow certain Chrono features which require C++:
- Chrono::Multicore which requires OpenMP for C++
- Chrono::Vehicle co-simulation which requires MPI for C++
- Chrono::VSG which calls FindThreads

If C++ is not enabled, `chono-config.cmake` will disable these features and any Chrono modules depending on them.

### Important information if using Chrono::Sensor

If linking to the Chrono::Sensor module from an external project, make sure to set the directory of the install location where the shader code (compiled ptx code or shaders cu files) is located. This should be set at the top of any external code that will use Chrono::Sensor from an install location.
```cpp
  //function to set the shader location (include ChOptixUtils.h)
  chrono::sensor::SetSensorShaderDir("path/to/sensor/shaders");

  //if USE_CUDA_NVRTC is enabled, use
  chrono::sensor::SetSensorShaderDir("path/to/install/include/chrono_sensor/optix/shaders/");

  //if USE_CUDA_NVRTC is disabled, use
  chrono::sensor::SetSensorShaderDir("path/to/install/lib/sensor_ptx/");
```
