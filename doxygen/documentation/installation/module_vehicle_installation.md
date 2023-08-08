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

## Dependencies

- This module has no additional dependenies
- Use the [CRGTerrain](@ref vehicle_terrain_crg) feature requires the [OpenCRG](https://www.asam.net/standards/detail/opencrg/) library. 

For run-time visualization, it is recommended to enable and install the [Chrono::VSG](@ref module_vsg_installation) module and/or the [Chrono::Irrlicht](@ref module_irrlicht_installation) module. 

## Building and installing prerequisistes

Chrono::Vehicle includes the option of specifying a terrain using an OpenCRG specification. Support for this optional capability requires the OpenCRG library.

The simplest way to build and install the OpenCRG library is to use the utility scripts provided with the Chrono distribution. 
These scripts (`buildOpenCRG.bat` and `buildOpenCRG.sh`, for Windows and Linux, respectively) are available in the `contrib/build-scripts/opencrg` directory of the [Chrono repository](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/opencrg). 

1. Copy the appropriate script and place in an arbitrary temporary directory.
2. Edit the script following the instructions in the top comments within the script.
3. Run the script (`.\buildOpenCRG.bat` or `sh buildOpenCRG.sh`, as appropriate) from the location of the script copy. This will create a temporary directory where all source repositories will be cloned and a set of directories where the individual URDF dependencies are built.
4. The install directory will contain the necessary header and library files required to configure Chrono::Vehicle with OpenCRG support.

## Building instructions
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_VEHICLE` as 'on', then press 'Configure' (to refresh the variable list) 
	 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

If enabling OpenCRG support (`ENABLE_OPENCRG`), you will be prompted to provide the location of a local installation of OpenCRG, including the location of the headers, library, and (Windows only) the location of the OpenCRG DLL.


## How to use it

- Consult the [reference manual](@ref manual_vehicle).

- Look at the [API section](@ref vehicle) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_vehicle) to learn how to use the functions of this module.
