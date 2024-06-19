Install the MUMPS module {#module_mumps_installation}
==========================

[TOC]

This is an optional module that enables Chrono to use the MUMPS linear solver.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.

Chrono usually relies on its [built-in solvers](@ref solvers), whose good perfomance are guaranteed by leveraging the internal data structure. 
In fact, for a wide range of applications, these suffice.<br>
However, for higher accuracy results, a direct solver could still be needed.

This module provides an interface to the third-party MUMPS solver.


## Features

The Chrono::Mumps module allows to plug the MUMPS solver into Chrono and provides two interface:
- an interface for Chrono - namely @ref chrono::ChSolverMumps<> - that is **not** intended to be used directly by the user.<br>
This is the interface that the user should plug into the Chrono environment.
- an interface for the end-user - namely @ref chrono::ChMumpsEngine - that allows to directly operate with MUMPS using the Chrono data classes (if the user would ever have this need).<br>
The demo_MUMPS_MumpsEngine.cpp shows its usage, but the average usare should not be interested in it.

Look at the [API section](group__mumps__module.html) of this module for a more in-depth discussion.
	
## Remarks


## Requirements
[MUMPS library]: http://mumps.enseeiht.fr/

- To **build** applications based on this unit:
	+ the [MUMPS library] must be installed on your machine

## Building instructions

1. Install the MUMPS library
   + Linux: TODO<br>
   + Windows: since building the MUMPS DLLs is not straightforward, we provide for an archive ([Mumps-5.1.1.zip](http://assets.projectchrono.org/downloads/Mumps-5.1.1.zip)) which includes precompiled libraries as well as all necessary header files.  Simply unpack in a folder on disk and then provide that location during CMake configuration (see below).<br>
   + MacOS: TODO<br>
	

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must set `ENABLE_MODULE_MUMPS` as 'on'.<br>
   Set the CMake variable `MUMPS_ROOT` to point to the installation directory for MUMPS.
   It is expected that this directory contains the following sub-directories: 'include' (MUMPS headers), 'lib' (lib file), and 'bin' (shared library).

3. Press 'Generate'



## How to use it

- Simply add this snippet anywhere in your code, before running the main simulation loop.<br>
This will inform Chrono to use the interface to the MUMPS solver.
~~~{.cpp}
auto mumps_solver = chrono_types::make_shared<ChSolverMumps>();
my_system.SetSolver(mumps_solver);
~~~


- (Optional) Turn on the sparsity pattern lock (see @ref chrono::ChSolverMumps and @ref chrono::ChDirectSolverLS for further details)
~~~{.cpp}
auto mumps_solver = chrono_types::make_shared<ChSolverMumps>();
mumps_solver->SetSparsityPatternLock(true);
my_system.SetSolver(mumps_solver);
~~~


- By default, this solver uses the sparsity pattern learner (see @ref chrono::ChDirectSolverLS) to infer the sparsity pattern before actually loading the non-zero elements.  To disable the use of the sparsity pattern learner, call 
~~~{.cpp}
mumps_solver->UseSparsityPatternLearner(false);
~~~


- Look at the [API section](group__mumps__module.html) of this module for documentation about classes and functions.