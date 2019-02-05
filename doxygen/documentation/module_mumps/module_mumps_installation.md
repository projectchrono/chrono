Install the MUMPS module {#module_mumps_installation}
==========================

[TOC]

This is an optional module that enables Chrono::Engine to use the MUMPS linear solver.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono::Engine project.

Chrono::Engine usually relies on its [built-in solvers](@ref solvers), whose good perfomance are guaranteed by leveraging the internal data structure. 
In fact, for a wide range of applications, these suffice.<br>
However, for higher accuracy results, a direct solver could still be needed.

This module provides an interface to the third-party MUMPS solver.


## Features

The MKL module allows to plug the MUMPS solver into Chrono::Engine and provides two interface:
- an interface for Chrono - namely @ref chrono::ChSolverMumps<> - that is **not** intended to be used directly by the user.<br>
This is the interface that the user should plug into the Chrono environment.
- an interface for the end-user - namely @ref chrono::ChMumpsEngine - that allows to directly operate with Pardiso using the Chrono data classes (if the user would ever have this need).<br>
The demo_MUMPS_MumpsEngine.cpp shows its usage, but the average usare should not be interested in it.

Look at the [API section](group__mumps__module.html) of this module for a more in-depth discussion.
	
## Remarks


## Requirements
[MUMPS library]: http://mumps.enseeiht.fr/

- To **build** applications based on this unit:
	+ the [MUMPS library] must be installed on your machine

## Building instructions

1. Install the MUMPS library (**TODO**)
	
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window,<br>
    you must set `ENABLE_MODULE_MUMPS` as 'on'.<br>

3. Press 'Generate'

4. Set the environmental variables (**TODO**)


## How to use it

(**TODO**)