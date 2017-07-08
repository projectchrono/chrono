Install the IRRLICHT module {#module_irrlicht_installation}
==========================

[TOC]

This is the default visualization system for interactive 3D viewing of Chrono::Engine simulations.

If you develop your custom application based on Chrono::Engine, you may use other 
visualization systems (ex. Ogre3D, CryEngine, plain OpenGL or others) and you do not 
need this module, however this IRRLICHT module is used here in most demos 
and tutorials because it is lightweight and simple. 

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono::Engine project.


## Features

The **IRRLICHT module** is used to show Chrono::Engine tutorials and 
demos in an interactive 3D OpenGL view.

Here are the main features:

- Irrlicht shapes and textures can be added as _assets_ to ChBody objects.
- A default lightweight application framework is provided (ChIrrApp) 
  that can be used for default light/camera managements.
- The Irrlicht view of the default application supports some 
  mouse and key interaction:
	- mouse left button for camera rotation
	- mouse right button for camera x z motion
	- mouse wheel rotation for camera forward/backward
	- mouse wheel button to pick & drag objects (only if they have a collision shape!)
	- press 'i' to see a setting panel,
	- press arrows to have x z camera motion, press page up & down for y vertical motion
	- press spacebar to stop simulation
	- press 'p' to advance one timestep a time.
	- press 'print screen' to start saving screenshots to disk
	- etc. (see the 'i' panel)
- There are some easy functions to draw grids, lines, springs, plots.
- Contacts can be optionally drawn with vectors in 3D view
- Link coordinate systems can be plotted on the 3D view
- etc.


## Requirements

- To **run** applications based on this module you need the Irrlicht library (i.e. the Irrlicht.dll on Windows) 

- To **build** applications applications based on this module you must have the Irrlicht SDK installed.




## Building instructions

Instructions on how to install the required Irrlicht library are already reported in 
the [Chrono installation page](@ref tutorial_install_chrono). If you followed those instructions,
you do not need to do anything. Otherwise do what follows.
   
1. **download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html) 

2. **unzip** it in whatever directory.  
   For example, here we suppose that you unzipped it in `C:/engine_demos/irrlicht-1.8.2`.

3. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
4. Set the `ENABLE_MODULE_IRRLICHT` as 'on', then press 'Configure' (to refresh the variable list) 
 
5. Set the directory in `CH_IRRLICHTDIR`: it must contain the path to your unzipped Irrlicht directory.  
   In our example, browse to `C:/engine_demos/irrlicht-1.8.2`
   
6. The corresponding `CH_IRRLICHTLIB` should be found automatically: it must contain the file of the Irrlicht.lib.  
   In our example, browse to `C:/engine_demos/irrlicht-1.8.2/lib/Win64-visualStudio/Irrlicht.lib`.
	 
7. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Look at the [API section](group__irrlicht__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
