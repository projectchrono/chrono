Install the VEHICLE module   {#module_vehicle_installation}
===============================

[TOC]

This is an optional module that enables template-based ground vehicle 
modeling and simulation within Chrono.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.


## Features

The **VEHICLE module** allows users to model and simulate vehicles. 

For more detail, read the [Chrono::Vehicle](@ref manual_vehicle) section of the reference manual.


## Requirements

- To **run** applications based on this module there are no requirements

- To **build** applications based on this module there are no requirements

For run-time visualization, it is recommended to enable and install the [Chrono::Irrlicht](@ref tutorial_install_chrono) module and/or the [Chrono::OpenGL](@ref module_opengl_installation) module. To use the [CRGTerrain](@ref vehicle_terrain_crg) feature, you must download, install, and enable the [OpenCRG](http://opencrg.org/download.html) library. 

## Building instructions
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_VEHICLE` as 'on', then press 'Configure' (to refresh the variable list) 
	 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

If enabling OpenCRG support (`ENABLE_OPENCRG`), you will be prompted to provide the location of a local installation of OpenCRG, including the location of the headers, library, and (Windows only) the location of the OpenCRG DLL.


## How to use it

- Consult the [reference manual](@ref manual_vehicle).

- Look at the [API section](@ref vehicle) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_vehicle) to learn how to use the functions of this module.
