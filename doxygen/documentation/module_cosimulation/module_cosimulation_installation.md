Install the COSIMULATION module   {#module_cosimulation_installation}
===============================

[TOC]

This is an optional module that enables co-simulation capabilities in Chrono::Engine.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono::Engine project.


## Features

The **COSIMULATION module** allows users to perform cosimulation with other
software that accept data exchange via TCP/IP sockets. 

The most relevant case is the cosimulation with the
[Simulink](http://www.mathworks.com/products/simulink) product, to 
simulate subsystems like controls, pneumatic systems or electric circuits
that interact with Chrono::Engine mechanisms. 

The Simulink technology is well known for its ease of use: 
it is a graphical tool built on top of Matlab that allows the 
building of control systems, pneumatic and hydraulic circuits, 
drivetrains, electrical power systems, etc. 
By adopting a co-simulation approach, you can simulate, 
for example, an excavator bow with Chrono::Engine and 
its hydraulic subsystems with Simulink. 

Here are the main features:

- easy C++ functions to send/receive datagrams using TCP/IP sockets from Chrono::Engine
- a **CEcosimulation.mdl** block is provided, to be inserted in your Simulink models as 
  a ready-to-use interface to Chrono::Engine
- examples are provided.


## Requirements

- To **run** applications based on this module:
    - you must have [Simulink](http://www.mathworks.com/products/simulink) installed.
    - you must have [Instrument Control Toolbox](http://www.mathworks.com/products/instrument) for Simulink installed.
- To **build** this module:
    - you must have a socket library (both Linux and Windows have these by default 
	  and you should not need to install any library)


## Building instructions
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_COSIMULATION` as 'on', then press 'Configure' (to refresh the variable list) 
	 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-info">
 Although this module is meant to interface with Simulink, 
 the Simulink and Matlab APIs are not needed during the compilation, 
 and also the [Matlab unit](group__matlab__module.html) is not needed. 
 In fact the interface to Simulink is based on **TCP/IP sockets** only; actually such interface could be 
 used to do cosimulation with other software that supports TCP socked communication, 
 such as <br>
 [Amesim](http://www.lmsintl.com/amesim-platform)  <br>
 [Dymola](http://www.dymola.com) <br>
 etc. <br>
 Anyway, currently, only examples and files for Simulink are provided.
</div>


## How to use it

- Look at the [API section](group__cosimulation__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
