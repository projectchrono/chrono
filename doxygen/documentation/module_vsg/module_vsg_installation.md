Install the VSG module {#module_opengl_installation}
==========================

[TOC]

Vulkan Scene Graph (VSG) support module for Chrono. 

OpenGL has become very popular with the time. It is supported by a lot of systems. Over the decades it turned out, 
that OpenGL is not flexible enough to get all of the power a modern GPU can offer. Vulkan has been developed as a
successor. The developers of Vulkan state: "Vulkan is not for everybody". When you try to write a program to draw
a triangle on the screen, you have to write around 1300 lines of code and you have an idea what was meant.

Fortunately the makers of OpenSceneGraph (OSG) decided to develop their VulkanSceneGraph (VSG) to ease the development
of 3D graphics software based on Vulkan. It is still under development but constantly growing.

Chrono::VSG is meant as a successor for Chrono::IRRLICHT and Chrono::OPENGL. It is aktually in an embrionic state.


## Features

The **VSG module** is used for showing simulations in a VSG view. It comes with a simple graphical user interface based
on the ImGUI library.


## Requirements

- To **run** applications based on this module you need:
    - the [VULKAN](https://vulkan.lunarg.com) library, it is highly recommended also to install the libraries in debug mode
    - the [vsgExamples](https://github.com/vsg-dev/vsgExamples.git) data folder, which contains a lot of important resources

To find the resources an environment variable has to be set correctly:
Windows:    set VSG_FILE_PATH=c:\your\vsg_install_path\data
Linux/Mac:  export VSG_FILE_PATH=/your/vsg/install_path/data

Use the appropriate paths for your needs. Actually all VSG related libraries are in static mode. This may change in future.

- To **build** this module you need:
    - the [VULKAN](https://vulkan.lunarg.com) library, it is highly recommended also to install the libraries in debug mode
    - the [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph.git) library
    - the [vsgXchange](https://github.com/vsg-dev/vsgXchange.git) library and converter tool 'vsgconv'
    - the [vsgImGui](https://github.com/vsg-dev/vsgImGui.git) library
    - the [vsgExamples](https://github.com/vsg-dev/vsgExamples.git) data folder, which contains a lot of important resources

You may want to be able to debug your code, then you should build these libraries in Release and Debug mode as well. To ease 
the configuration process for cmake, the VSG libraries should be installed in a single directory like c:\VSG on Windows.
The vsgXchange library can have additional dependencies (OSG, assimp,...), you can play with it, but for working with
Chrono::VSG you will actually not need it.


## Building instructions
  
1. Download and install these libraries (depending on the platform, the process can be different)
    - the [VULKAN](https://vulkan.lunarg.com) library, it is highly recommended also to install the libraries in debug mode
    - the [VulkanSceneGraph](https://github.com/vsg-dev/VulkanSceneGraph.git) library
    - the [vsgXchange](https://github.com/vsg-dev/vsgXchange.git) library and converter tool 'vsgconv'
    - the [vsgImGui](https://github.com/vsg-dev/vsgImGui.git) library
    - the [vsgExamples](https://github.com/vsg-dev/vsgExamples.git) data folder, which contains a lot of important resources

The Vulkan SDK comes with an installer, the debug libraries must be copied by hand yet.
The VSG library sources can be loaded in form of zip files or by means of git:
    git clone https://github.com/vsg-dev/VulkanSceneGraph.git
for example.
You can configure every project with cmake, on Windows you will like Visual Studio 2019, on the Mac you can build with Xcode.
A nice commandline alternative is to use 'ninja', it is available for Linux/Mac/Windows. When you use
  cmake -G "Ninja Multi-config" ....
you get the makefiles for Release and Debug mode. In the build directory you write
  ninja -f build-Release.ninja install
  ninja -f build-Debug.ninja install
You can build also the documentation for VulkanSceneGraph by
  ninja -f build-Release.ninja docs
It generates an html directory, which has to be copied to the install directory by hand.


2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
3. Set the `ENABLE_MODULE_VSG` as 'on', then press 'Configure' (to refresh the variable list) 
 
4. Set all the values for `VulkanSceneGraph_...`, `vsgExchange_...`, `vsgImGui_...` and so on to proper directory / file values.
	 
5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-info">
Actually there is only one example, which is going to be changed every day...
</div>

