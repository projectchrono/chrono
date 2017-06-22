Install the CASCADE module {#module_cascade_installation}
==========================

[TOC]

This is an optional module that adds 3D CAD file support (STEP format) for Chrono::Engine
using the [OpenCASCADE](http://www.opencascade.org) library. 

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono::Engine project.


## Features

The **CASCADE module** uses the functions of the [OpenCASCADE](http://www.opencascade.org) 
open-source library in order to allow loading CAD models that were saved in the STEP file format. 

Here are the main features:

- Load object shapes saved in STEP file format
- Compute the center of mass, the mass, the inertia of shapes
- Convert shapes in triangulated meshes for visualization 

## Requirements

- To **run** applications based on this module:
	- you must have the [OpenCASCADE](http://www.opencascade.org) libraries installed.
- To **build** this module:
	- you must have the [OpenCASCADE](http://www.opencascade.org) libraries installed.



## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
  
2. Set the `ENABLE_MODULE_CASCADE` as 'on', then press 'Configure' (to refresh the variable list) 

3. Set the `CH_CASCADEDIR` to the path where you have your OpenCASCADE SDK (the one that has inc/, src/, etc. subdirectories),
   for example it could be `D:/OpenCASCADE6.9.0/opencascade-6.9.0` 
 
4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
Warning! if you execute the demo .exe programs of this module, 
most likely you will get an error message because Windows cannot 
find some OpenCascade DLLs. 
</div>

In order to have the DLLs reachable, on windows, do the following:

Execute the `env.bat` script in the OpenCascade directory before 
launching the demo (but only from the same DOS shell, via command line) or **better**, to have path 
and environment variables already set all times you start windows, go to 
control panel / system / environment variables window, 
and just add this minimal set of data:
- to the PATH variable, add the paths containing the required OpenCASCADE .dll files
  (assuming you installed OpenCASCADE 6.9.0 in D:, otherwise put your paths), 
  separating them with  ;   and no spaces:
	- `D:\OpenCASCADE6.9.0\opencascade-6.9.0\win64\vc10\bin`
	- `D:\OpenCASCADE6.9.0\freeimage-3.16.0-vc10-64\bin`
	- `D:\OpenCASCADE6.9.0\tbb42_20140416oss\bin\intel64\vc10`
- optionally add also these environment variables with these values (again, modify the path 
  if you used another installation directory for OpenCASCADE)
	- `CSF_LANGUAGE=us`
	- `MMGT_CLEAR=1`
	- `CSF_EXCEPTION_PROMPT=1`
	- `CSF_STEPDefaults=D:\OpenCASCADE6.9.0\opencascade-6.9.0\src\XSTEPResource`
In this way, you will be able to start the OpenCASCADE demos by simply clicking on the .exe, 
without the need of calling the env.bat script before.



## How to use it

- Look at the [API section](group__cascade__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
