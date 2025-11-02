Install the CASCADE module {#module_cascade_installation}
==========================

[TOC]

Chrono::Cascade is an optional module that adds 3D CAD file support (STEP format) for Chrono
using the [OpenCASCADE](http://www.opencascade.org) library. 

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

Currently, the Chrono API is compatible with the OpenCASCADE **OCCT v.7.9.2**. Other versions of OpenCASCADE *are not compatible*.


## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
  
2. Set `CH_ENABLE_MODULE_CASCADE` to 'on'.

3. If prompted, set `OpenCASCADE_DIR` to the path to the OpenCASCADE project configuration script (`OpenCASCADEConfig.cmake`) is located.

4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
On **Windows**, to make the OpenCASCADE DLLs visible to the Chrono::Cascade demos:
- Execute the `env.bat` script in the OpenCascade directory before launching the demo (but only from the same DOS shell, via command line), or
- Set the path and environment variables to make these DLLs visible from anywhere.

On **Linux**, you may need to add the path to the OpenCascade shared libraries to the `LD_LIBRARY_PATH` environment variable. For example:
````
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
````

On **MacOS** you have to build your own OpenCascade folder. Homebrew has a cascade module, but it isn't compatible with chrono_cascade.
So, remove it if you had installed it before. 
At configuration with CMake, be sure to apply the correct value for <tt>INSTALL_NAME_DIR:PATH=/opt/OCCT/lib</tt> to set the dylib search paths to fixed values.
In this example OpenCascade has been installed into the install prefix <tt>/opt/OCCT</tt>. If you want to install it into somewhere else, adjust your settings.
With fixed dylib paths no cascade related entries in DYLD_LIBRARY_PATH are necessary.
</div>

## How to use it

- Look at the [API section](group__cascade__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.


## Troubleshooting

- if the executables hang, freeze or crash as soon as they are loaded, you are possibly using an older version of OpenCASCADE. Uninstall it, and upgrade it to the version supported in Chrono (see above), rebuild Chrono, and set the correct paths in environment variables.

- Some users reported errors like "missing symbol in dll" when starting the executables even with the correct version of OpenCASCADE. 
This is most likely due to the fact that you already have an older version of OpenCASCADE installed somewhere, even if you do not know it, where the path to those older DLLs is taking precedence in your PATH environment variable. Solution: move your paths to OpenCASCADE DLLs *before* all other paths in the system PATH variable. This might be caused when you installed
	- some packages for Matlab, that use OpenCASCADE under the hood
	- some packages for Anaconda, that use OpenCASCADE under the hood

