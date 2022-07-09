Install the OPENGL module {#module_opengl_installation}
==========================

[TOC]

OpenGL support module for Chrono. 

Compared to the [IRRLICHT module](group__irrlicht__module.html) this provides 
a lower-level access to real-time 3D visualization and a more limited
set of visualization features, but it uses less resources and it can be useful
in case of large amounts of objects.


## Features

The **OPENGL module** is used for showing simulations in a OpenGL view.


## Requirements

- To **run** applications based on this module you need:
    - the [GLM](http://glm.g-truc.net/0.9.6/index.html) library
    - the [GLFW](http://www.glfw.org/) library
    - the [GLEW](http://glew.sourceforge.net/) library

- To **build** this module you need:
    - the [GLM](http://glm.g-truc.net/0.9.6/index.html) library
    - the [GLFW](http://www.glfw.org/) library
    - the [GLEW](http://glew.sourceforge.net/) library



## Building instructions
  
1. Download and install these libraries (depending on the platform, the process can be different)
    - the [GLM](http://glm.g-truc.net/0.9.6/index.html) library
    - the [GLFW](http://www.glfw.org/) library
    - the [GLEW](http://glew.sourceforge.net/) library

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
3. Set the `ENABLE_MODULE_OPENGL` as 'on', then press 'Configure' (to refresh the variable list) 
 
4. Set all the values for `GLM_...`, `GLEW_...`, `GLFW_...` to proper directory / file values.
	 
5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-info">
Most demos and tutorials do **not** use this module for visualization, as they use 
the [IRRLICHT module](group__irrlicht__module.html), which is easier to use, and at a more advanced stage of development.
</div>

## MacOS issues

You can configure and build this module on the Mac, but due to malconfigured shader programs you cannot run it.

