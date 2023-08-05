Install the VSG module {#module_vsg_installation}
==========================

[TOC]

This is a run-time visualization system for interactive 3D viewing of Chrono simulations.

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.


## Features

The **VSG module** is used to display Chrono simulations in an interactive 3D view.
Here are the main features:

- Supports most visualization shapes specified as _assets_ on Chrono physics objects (bodies, links, etc)
- The following default mouse and key controls are supported:
	- mouse left button for camera rotation
	- mouse right button for camera x z motion
	- mouse wheel rotation for camera forward/backward
	- press arrows to have x z camera motion, press page up & down for y vertical motion


## Dependencies

- This module you requires the Vulkan SDK and the VSG libraries.


## Building and installing prerequisistes

There are two prerequisites for building the Chrono VSG module:

- [Vulkan](https://www.vulkan.org/) is a cross-platform API for 3D graphics. The [Vulkan SDK](https://www.lunarg.com/vulkan-sdk/) can be downloaded from [https://vulkan.lunarg.com/](https://vulkan.lunarg.com/). Follow the installation instructions specific to your platform.

- [VulkanScenegraph](https://vsg-dev.github.io/VulkanSceneGraph) is a graphics library built on the Vulkan graphics API. At this time, the VSG library, its dependencies, and additional related utility libraries must be built from sources. 

  The official mechanism for building and installing all VSG dependencies is to use [vsgFramework](https://github.com/vsg-dev/vsgFramework) provided by the VSG developers. Note however that this approach will use the latest VSG development code and as such may be incompatible with the current Chrono::VSG module.

  To address this issue, we provide a set of scripts (for both Windows and Linux) in the `contrib/build-scripts/vsg` directory of the Chrono source tree.  These scripts allow us to target specific releases of the VSG dependencies (i.e., tags in their respective GitHub repositories) and ensure compatibility with the current Chrono::VSG code.

The two approaches for building and installing the VSG dependencies are described in the next two sections.
For the reasons detailed above, We **strongly recommend** using the provided VSG [build scripts](#vsg_scripts).

#### 1. VSG Framework {#vsg_framework}

The official way of obtaining all VSG prerequisites for the Chrono::VSG module is to build the [vsgFramework](https://github.com/vsg-dev/vsgFramework) which collects several VSG-related projects and facilitates their build and installation in one single step.  

The VSG libraries are themselves under active development, and so is vsgFramework. While functional, things do occasionally break down and some fixes may be needed to get the necessary dependency libraries. The instructions below reflect the current state of the vsgFramework code.

  1. Clone the vsgFramework [GitHub repository](https://github.com/vsg-dev/vsgFramework).
     Assume the sources are in a local directory [vsgFramework_source].
  2. Create a **buid** directory and an **install** directory for vsgFramework. 
     Assume these are **[vsgFramework_build]** and **[vsgFramework_install]**, respectively.
  3. Use CMake to configure vsgFramework.  Note that the only components that are necessary for Chrono::VSG are *assimp*, *vsgImGui*, and *vsgXchange*. Enable the corresponding `BUILD_***` CMake options and unselect all other.
  4. Set the installation directory (`CMAKE_INSTALL_PREFIX`) to be the **[vsgFramework_install]** directory created above.
  5. Note that vsgFramework can create either *static* or *dynamic* VSG libraries.  Either type will work on Linux or on MacOS. However, only *dynamic* libraries (*DLLs*) work on Windows. Set the CMake variable `BUILD_SHARED_LIBS` accordingly.
  6. Complete CMake configuration and generate the build scripts.
  7. Build and install the vsgFramework libraries (using whatever is appropriate for the generator you selected in CMake; make, ninja, VS, etc.)
  8. The VSG headers, libraries, and DLLs (if appropriate) installed in **[vsgFramework_install]** must be made available and accessible to CMake during configuration of Chrono below.  If needed (e.g., on Windows), add to the system `PATH` environment variable the directory **[vsgFramework_install]/bin**.

  <div class="ce-warning">
  The `assimp` component requires `zlib`. 
  On some Linux systems, installing the default zlib package may not provide a library that is suitable for dynamic linking. 
  You may need to rebuild zlib yourself, making sure you generate position-independent code. 
  In particular, if using GCC, make sure to add the flag `-fPIC`.
  </div>


#### 2. VSG build scripts {#vsg_scripts}

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
   
2. During CMake configuration, set `ENABLE_MODULE_VSG` to 'on', then press 'Configure'

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
