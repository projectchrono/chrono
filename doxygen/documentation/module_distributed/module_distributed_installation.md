Install the DISTRIBUTED module   {#module_distributed_installation}
===============================

[TOC]

This is an optional module that enables MPI_based distributed parallel computation for granular dynamics.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.


## Features

## Requirements

- To **build** applications based on this module an MPI distribution is required
- This module requires that the Chrono::Multicore module be also enabled. See the instructions for buiding the [MULTICORE module](@ref module_multicore_installation)


## Building instructions
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
2. Set the `ENABLE_MODULE_DISTRIBUTED` as 'on', then press 'Configure' (to refresh the variable list) 
	 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Look at the [API section](group__distributed__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_distributed) to learn how to use the functions of this module.
