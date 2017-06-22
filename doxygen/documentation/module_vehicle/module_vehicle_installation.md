Install the VEHICLE module   {#module_vehicle_installation}
===============================

[TOC]

This is an optional module that enables template-based ground vehicle 
modeling and simulation within Chrono::Engine.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono::Engine project.


## Features

The **VEHICLE module** allows users to model and simulate vehicles. 

Here are the main features:

- template-based definition of vehicles
- both tracked and wheeled vehicles
- different types of tire models
	- Pacejka
	- rigid
	- Fiala
	- finite elements, deformable
	- etc.
- different types of soil models
	- rigid
	- deformable
	- granular
	- etc.
- use JSON for easy definition of models
- different types of suspensions
	- multilink
	- rigid axle
	- McPherson
	- Hendrickson
	- double wishbone
- driveline 1D primitives
	- clutches
	- thermal engines
	- brakes
	- reducers
	- gears
	- planetary gears
	- etc.


## Requirements

- To **run** applications based on this module there are no requirements

- To **build** applications applications based on this module there are no requirements


## Building instructions
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_VEHICLE` as 'on', then press 'Configure' (to refresh the variable list) 
	 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-info">
 This module is under development - more features will appear in future.
</div>


## How to use it

- Look at the [API section](group__vehicle__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_vehicle) to learn how to use the functions of this module.
