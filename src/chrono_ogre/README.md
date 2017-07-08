Install the OGRE module {#module_ogre_installation}
==========================

<div class="ce-danger">
This module is still at an experimental stage.
</div>

<div class="ce-danger">
Most demos and tutorials do **not** use this module for visualization, as they use 
the[IRRLICHT module](@ref module_irrlicht_installer), which is more tested 
and at a more advanced stage of development.
</div>


The **OGRE module** provides an interface to the [Ogre](http://www.ogre3d.org) library
as a realtime 3D visualization system to show Chrono::Engine simulations.



## Requirements

- To **run** applications based on this module you need:
	- [Ogre](http://www.ogre3d.org)
    - [MyGUI](https://github.com/MyGUI/mygui)
    - [SDL](https://www.libsdl.org/download-2.0.php)


- To **build** this module you need:
	- [Ogre](http://www.ogre3d.org)
    - [MyGUI](https://github.com/MyGUI/mygui)
    - [SDL](https://www.libsdl.org/download-2.0.php)
	

## Building instructions
  
Chrono_Ogre depends on Ogre, MyGUI, and SDL. Download them from the links above.

This usually wouldn't be too tricky to setup, but the code is written in C++11, which causes problems
with Ogre. In order to build Ogre so it can be used with Visual Studio 2013, you need to

1. Clone the Ogredeps repository at https://bitbucket.org/cabalistic/ogredeps with Mercurial
2. Install the DirectX June 2010 SDK at http://www.microsoft.com/en-us/download/details.aspx?id=6812
3. Configure the Ogredeps CMake for Visual Studio 2013
4. Build and install it, and then rename the install folder to "Dependencies"
5. Clone the Ogre repository at https://bitbucket.org/sinbad/ogre with Mercurial
	- The version most tested on is 1.9. Version 2.0 and on should be compatible, but this isn't guaranteed
6. Copy the "Dependencies" folder from before into the build directory you made for Ogre in CMake
7. Download boost at http://www.boost.org/users/download/
8. Build boost
9. Configure Ogre, build it, and then install it. All headers, libraries, and DLLs will be in the "sdk" folder in the build directory

