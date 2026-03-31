Install the VSG module {#module_vsg_installation}
==========================

[TOC]

Chrono::VSG is a run-time visualization system for interactive 3D viewing of Chrono simulations.

## Features

The **VSG module** is used to display Chrono simulations in an interactive 3D view.
Here are the main features:

- Supports most visualization shapes specified as _assets_ on Chrono physics objects (bodies, links, etc)
- The following default mouse and key controls are supported:
	- mouse left button for camera rotation
	- mouse right button for camera x z motion
	- mouse wheel rotation for camera forward/backward
	- press arrows to have x z camera motion, press page up & down for y vertical motion

## Building and installing prerequisistes

There are two prerequisites for building the Chrono VSG module:

- [Vulkan](https://www.vulkan.org/) is a cross-platform API for 3D graphics. The [Vulkan SDK](https://www.lunarg.com/vulkan-sdk/) can be downloaded from [https://vulkan.lunarg.com/](https://vulkan.lunarg.com/). Follow the installation instructions specific to your platform.

- [VulkanScenegraph](https://vsg-dev.github.io/VulkanSceneGraph) is a graphics library built on the Vulkan graphics API. At this time, the VSG library, its dependencies, and additional related utility libraries must be built from sources. 

  The official mechanism for building and installing all VSG dependencies is to use [vsgFramework](https://github.com/vsg-dev/vsgFramework) provided by the VSG developers. Note however that this approach will use the latest VSG development code and as such may be incompatible with the current Chrono::VSG module.

  To address this issue, we provide a set of scripts (for both Windows and Linux) in the `contrib/build-scripts/vsg` directory of the Chrono source tree.  These scripts allow us to target specific releases of the VSG dependencies (i.e., tags in their respective GitHub repositories) and ensure compatibility with the current Chrono::VSG code.

  The current Chrono::VSG module requires the following versions of the VSG and dependent libraries:
  - [vsg](github.com/vsg-dev/VulkanSceneGraph.git) 1.1.11
  - [vsgXchange](github.com/vsg-dev/vsgXchange.git) 1.1.7
  - [vsgExamples](github.com/vsg-dev/vsgExamples.git) 1.1.9
  - [vsgImGui](github.com/vsg-dev/vsgImGui.git) 0.7.0
  - [assimp](github.com/assimp/assimp) 5.4.3
  - [draco](github.com/google/draco) 1.5.7
  - [glslang](github.com/KhronosGroup/glslang.git) 15.4.0

  Because of this, we **strongly recommend** using the provided VSG build scripts, instead of `vsgFramework`. 

## VSG build scripts

With the VSG libraries themselves under active development, their latest versions may be incompatible with the current Chrono::VSG code. To ensure compatibility between the Chrono::VSG and its VSG dependencies, we provide (with the Chrono source code) a set of scripts which download specific code versions of the VSG dependencies, build all necessary libraries, and install them in a user-specified location.

These scripts (`buildVSG.bat` and `buildVSG.sh`, for Windows and Linux, respectively) are available in the `contrib/build-scripts/vsg` directory of the [Chrono repository](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/vsg). 

1. Copy the appropriate script and place in an arbitrary temporary directory.
2. Edit the script copy to:
   - Force a download of the VSG library codes.
   - Specify the install directory (set the variable `VSG_INSTALL_DIR`).
   - Decide whether to build shared or static libraries and whether to also build debug libraries.
3. Run the script (`.\buildVSG.bat` or `sh buildVSG.sh`, as appropriate) from the location of the script copy. This will create a temporary directory where all source repositories will be cloned and a set of directories where the individual VSG dependencies are built.
4. The install directory will contain (under subdirectories of `VSG_INSTALL_DIR/lib/cmake`) all VSG CMake project configuration scripts required to configure Chrono with the Chrono::VSG module enabled.


## Building instructions

Once the necessary dependencies are installed, perform the following steps to configure and build the Chrono::VSG module:

1. Repeat the instructions for the [full Chrono installation](@ref tutorial_install_chrono)
   
2. Set `CH_ENABLE_MODULE_VSG` to 'on'.

3. When prompted, provide the paths to the various VSG project configuration scripts (`vsg_DIR`, `vsgImGui_DIR`, and `vsgXchange_DIR`). For example, assuming you used the provided [build scripts](#vsg_scripts), these should be `<VSG_INSTALL_DIR>/lib/cmake/vsg`, `<VSG_INSTALL_DIR>/lib/cmake/vsgImGui`, and `<VSG_INSTALL_DIR>/lib/cmake/vsgXchange`, respectively.

4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
When using shared libraries for third-party dependencies, you must ensure that these are found at run time.<br>
On Windows, you can either copy the dependency DLLs to the same directory as the executables or else add the path to these shared libraries to the `PATH` environment variable.<br>
On Linux, you may need to append to the `LD_LIBRARY_PATH` environment variable.
</div>

## Usage

- Consult the [API section](group__vsg__module.html) of this module for documentation about classes and functions.

- Consult the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
