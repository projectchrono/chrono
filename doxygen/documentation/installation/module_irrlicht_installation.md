Install the IRRLICHT module {#module_irrlicht_installation}
==========================

[TOC]

This is a run-time visualization system for interactive 3D viewing of Chrono simulations.

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.


## Features

The **IRRLICHT module** is used to display Chrono simulations in an interactive 3D view.
Here are the main features:

- Supports all visualization shapes specified as _assets_ on Chrono physics objects (bodies, links, etc)
- The following default mouse and key controls are supported:
	- mouse left button for camera rotation
	- mouse right button for camera x z motion
	- mouse wheel rotation for camera forward/backward
	- press 'i' to see a setting panel,
	- press arrows to have x z camera motion, press page up & down for y vertical motion
	- press 'print screen' to start saving screenshots to disk
- Contacts can be optionally drawn with vectors in 3D view
- Link coordinate systems can be plotted on the 3D view


## Dependencies

- To Chrono::Irrlicht requires the Irrlicht SDK 


## Installing prerequisistes

1. **Download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html); the newest tested version is [1.8.5](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.5.zip)

2. **Unzip** it in a directory of your choice. For example, here we suppose that you unzipped it in <tt>C:/Packages/irrlicht-1.8.5</tt>.

On Linux, Irrlicht may also be installed through a package manager: `irrlicht`, `libirrlicht-dev`, `irrlicht-devel`.

The best way to install irrlicht on the Mac is: <tt>brew install irrlicht</tt> (release v.1.8.5). On MacOS 12 (Monterey) you have to set IRRLICHT_INSTALL_DIR to <tt>/opt/homebrew</tt>.<br>

## Building instructions

Instructions on how to install the Chrono::Irrlicht library are already reported on the [Chrono installation page](@ref tutorial_install_chrono). They are repeated here in more details.
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono)
   
2. Set `CH_ENABLE_MODULE_IRRLICHT` to 'on'.
 
3. Set the directory in `Irrlicht_ROOT`: it must contain the path to your unzipped Irrlicht directory. In our example, `C:/Packages/irrlicht-1.8.5`
   	 
4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Consult the [API section](group__irrlicht__module.html) of this module for documentation about classes and functions.

- Consult the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
