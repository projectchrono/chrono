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


## Requirements

- To **run** applications based on this module you need the Vulkan and VSG libraries 
- To **build** applications applications based on this module you must have the Vulkan SDK and the VSG framework installed.


## Building instructions

There are two prerequisites for building the Chrono VSG module:

- [Vulkan](https://www.vulkan.org/) is a cross-platform API for 3D graphics. The [Vulkan SDK](https://www.lunarg.com/vulkan-sdk/) can be downloaded from [https://vulkan.lunarg.com/](https://vulkan.lunarg.com/). Follow the installation instructions specific to your platform.

- [VulkanScenegraph](https://vsg-dev.github.io/VulkanSceneGraph) is a graphics library built on the Vulkan graphics API. At this time, the VSG library, its dependencies, and additional related utility libraries must be built from sources.  The easiest and most straightforward way of obtaining all VSG prerequisites for the Chrono::VSG module is to build the [vsgFramework](https://github.com/vsg-dev/vsgFramework) which collects several VSG-related projects and facilitates their build and installation in one single step.  

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
  The current version of vsgFramework has a bug where a vsgImGui header is not found.  
  To fix this, after step #3 above, you will need to **manually** copy the `imconfig.h` header from 
  `[vsgFramework_buid]/components/vsgimgui-src/src/imgui/` to 
  `[vsgFramework_build]/components/vsgimgui-src/include/vsgImGui/` 
  before you proceed with the vsgFramework build in step #7.
  </div>

  <div class="ce-warning">
  The `assimp` component requires `zlib`. 
  On some Linux systems, installing the default zlib package may not provide a library that is suitable for dynamic linking. 
  You may need to rebuild zlib yourself, making sure you generate position-independent code. 
  In particular, if using GCC, make sure to add the flag `-fPIC`.
  </div>

Once the necessary dependencies are installed, perform the following steps to configure and build the Chrono::VSG module:

1. Repeat the instructions for the [full Chrono installation](@ref tutorial_install_chrono)
   
2. During CMake configuration, set `ENABLE_MODULE_VSG` to 'on', then press 'Configure'

3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Consult the [API section](group__vsg__module.html) of this module for documentation about classes and functions.

- Consult the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
