Install the POSTPROCESS module {#module_postprocess_installation}
==========================

[TOC]

This is an optional unit that can be used to export scripts for 
postprocessing simulation data. For example, it can generate files that can be
load in Blender for high quality photorealistic rendering, or in the POVray rendering tool. 
It is also used to output files for GNUplot.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.


## Features

The **POSTPROCESS module** is used to export data for
batch rendering of animations etc.

Here are the main features:

- Blender:
	- exports files that can be load in [Blender](http://www.blender.org)
	  using the chrono_import.py add-in
	- converts the ChVisualization assets into Blender objects
	- using the Blender GUI, one can add more objects, or modify the 
	  imported ones by attaching additional textures etc.
	- the camera can be attached to moving objects
	- advanced functionalities for exporting scalar or vector fields
	  attached to ChTriangleMeshConnected or to finite elements,  
	  rendered via falsecolor maps
	- renders object reference frames, contacts, joint references
	- etc.
	
- POVray:
	- converts the ChVisualization assets that one has 
	  attached to rigid bodies into rendering primitives for POVray. 
	- uses the same ChVisualization assets that one can use 
	  for the real-time visualization with the Irrlicht interface
	- allows the introduction of custom POV-specific statements
	- the camera can be attached to moving objects
	- contacts can be rendered in POVray as colored vectors and points
	- etc.

- GNUplot:
	- create graphs using [GNUPLOT](http://www.gnuplot.info).
	- launches gnuplot automatically from c++
	- creates .gpl scripts automatically from c++ with easy functions
	- save plots on disk as .EPS or .PNG or other formats.

In future we plan to support also other type of rendering software, 
because the architecture of this system is not limited to POVray.


## Requirements

- To **run** applications based on this unit, there are no requirements. 
  Note however that you may want to 
  	- install [Blender](http://www.blender.org) to load and render 
	  the output files if you are interested in the POVray output, 
	- install [POVray](http://www.POVray.org) to load and render 
	  the output files if you are interested in the POVray output, 
	- install [GNUPLOT](http://www.gnuplot.info) to display plots if 
	  you are interested in the GNUPLOT output. 

- To **build** applications based on this unit there are no requirements.


## Building instructions

This unit corresponds to an additional shared library, called ChronoEngine_postprocess, that can be linked to your application if you want to use it.
The file extension will be .dll for Win and .so on Linux.

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_POSTPROCESS` as 'on', then press 'Configure' (to refresh the variable list) 
 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

When you will rebuild the project, you could find demo_POST_xxxx in the 
binary directory, among other default demos. 

## How to use it

Please mind that, in order to use Blender or POVray or GNUPLOT, these have to be installed and their environmental variables set properly.

- Look at the [API section](group__postprocess__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
