Install the OPENGL module {#module_opengl_installation}
==========================

[TOC]

OpenGL support module for Chrono. 


## Features

The **OPENGL module** is used for run-time visualization of Chrono simulations.

Compared to the [VSG](group__vsg__module.html) or [IRRLICHT module](group__irrlicht__module.html) modules, this run-time visualization module provides a lower-level access to real-time 3D visualization and a more limited set of visualization features, but it uses less resources and it can be useful in case of large number of objects (e.g., for visualization of granular dynamics simulations).


## Dependencies

- The Chrono::OpenGL module depends on:
    - the [GLM](http://glm.g-truc.net/0.9.6/index.html) library
    - the [GLFW](http://www.glfw.org/) library
    - the [GLEW](http://glew.sourceforge.net/) library


## Building and installing prerequisistes

The simplest way to build and install all requirements for the Chrono::OpenGL module is to use the utility scripts provided with the Chrono distribution. 
These scripts (`buildGL.bat` and `buildGL.sh`, for Windows and Linux, respectively) are available in the `contrib/build-scripts/vsg` directory of the [Chrono repository](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/opengl). 

1. Copy the appropriate script and place in an arbitrary temporary directory.
2. Edit the script copy to:
   - Force a download of the GL library codes.
   - Specify the install directory (set the variable `GL_INSTALL_DIR`).
   - Decide whether to build shared libraries and whether to also build debug libraries.
3. Run the script (`.\buildGL.bat` or `sh buildGL.sh`, as appropriate) from the location of the script copy. This will create a temporary directory where all source repositories will be cloned and a set of directories where the individual URDF dependencies are built.
4. The install directory will contain (under subdirectories of `GL_INSTALL_DIR/lib/cmake`) all GL CMake project configuration scripts required to configure Chrono with the Chrono::OpenGL module enabled.


## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_OPENGL` as 'on', then press 'Configure' (to refresh the variable list) 
 
3. When prompted, provide the paths to the various GL project configuration scripts (`GLEW_DIR`, `glfw3_DIR`) as well as the path to tyhe `GLM` include headers. 
	 
4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
When using shared libraries for third-party dependencies, you must ensure that these are found at run time.<br>
On Windows, you can either copy the dependency DLLs to the same directory as the executables or else add the path to these shared libraries to the `PATH` environment variable.<br>
On Linux, you may need to append to the `LD_LIBRARY_PATH` environment variable.
</div>

<div class="ce-info">
You can configure and build this module on the Mac, but due to malconfigured shader programs you cannot run it.
</div>

