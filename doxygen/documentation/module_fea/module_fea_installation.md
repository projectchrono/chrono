Install the FEA module {#module_fea_installation}
==========================

[TOC]

<div class="ce-warning">
This unit is under development. 
You should not use it unless you are aware that the code is 
not completely tested and some functions are incomplete. 
</div>

This is an optional unit that introduce Finite Element Analysis (FEA) in Chrono::Engine.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono::Engine project.


## Features

The ```FEM unit``` is used to simulate flexible parts in Chrono::Engine.

Here are the main features:

- 3D tetahedrons
	- linear, 4 nodes
	- quadratic, 10 nodes
- 3D hexahedrons
	- linear, 8 nodes
	- quadratic, 20 nodes
- Euler-Bernoulli beams
- Bars
- Shells
	- ANCF, 4 nodes
- All elements support nonlinear displacements
- Analysis:
	- elasticity: statics, dynamics
	- thermal: steady state, transient
	- electrostatics


## Requirements

- To **run** applications based on this unit, there are no requirements. 

- To **build** applications based on this unit there are no requirements.


## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_FEA` as 'on', then press 'Configure' (to refresh the variable list) 
 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

This unit corresponds to an additional DLL library, 
called **ChronoEngine_FEA.dll**, that can be linked to your application 
if you want to use it. On Linux systems, the .dll suffix is .so.

When you will rebuild the project, you could find some demo_FEAxxxx examples in the 
binary directory, among other default demos. 


## How to use it

- Look at the [manual](manual_fea.html) of this module for the reference.

- Look at the [API section](group__fea__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_fea) to learn how to use the functions of this module.
