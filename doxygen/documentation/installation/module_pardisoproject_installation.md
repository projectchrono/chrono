Install the Pardiso Project module {#module_pardisoproject_installation}
==========================

[TOC]

This is an optional module that enables Chrono to use the Pardiso solver from [Pardiso Project](https://www.pardiso-project.org/).

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.

Chrono::Engine usually relies on its [built-in solvers](@ref solvers), whose good perfomance are guaranteed by leveraging the internal data structure. 
In fact, for a wide range of applications, these suffice.<br>
However, for higher accuracy results, a direct solver could still be needed.

This module provides access to the third-party Pardiso solver in its PardisoProject flavour.

<div class="ce-warning"> 
This module has been tested only in Windows operating systems. Advanced skills are required to adjust the CMake structure to work under other environments.
</div>

## Features

As for the all the direct solvers, two Chrono-specific features are implemented:
- **sparsity pattern _lock_**<br>
    In many cases, the internal data structures undergo very little changes between different timesteps.
	In these cases it is useful to inform the solver that the sparsity pattern does not change consistently in order to speed up the matrix assembly.
- **sparsity pattern _learner_**<br>
    The sparsity pattern \e learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly. Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting any nonzeros)
Look at the [API section](group__pardisoproject__module.html) of this module for a more details.

## Requirements

A valid PardisoProject licence (free for academia) is required to run this module. The Pardiso solver can be downloaded from the [Pardiso Project](https://www.pardiso-project.org/) website. Pardiso is actually only available for Macs with Intel hardware, you cannot use it on Arm64 hardware. 

## Building instructions

1. Extract the library in an arbitrary folder (avoid path with spaces, special characters or just too long). E.g. `C:/workspace/libraries/pardisoproject`
2. Follow the guide for the [full installation](@ref tutorial_install_chrono) of Chrono, but when running CMake make sure that also the option `ENABLE_MODULE_PARDISO_PROJECT` is set to `ON`.
3. Set the variable `PARDISOPROJECT_LIBRARIES` to point to the Pardiso Project _import_ library path (the one ending with `.lib` for Win OS) e.g. `C:/workspace/libraries/pardisoproject/libpardiso600-WIN-X86-64.lib`. Please always use forward slashes `/` in CMake paths.



## How to use it

### Set up the environment

Before creating executables that can actually leverage the Pardiso library, you have to make sure that:

- the licence file is available in proper directories (see Pardiso Project User Guide)

- the Pardiso _run-time_ libraries are available to the executable itself.<br>
For Win OS, this is achieved by setting the PATH environmental variable in a proper way.<br>
The fastest way is to permanently add the path to the Pardiso _runtime_ library (e.g. `C:/workspace/libraries/pardisoproject`) to your PATH environmental variable.<br>
Many guides can be found online to help setting environmental variables properly.


### Set up the code

- Simply add this snippet anywhere in your code, before running the main simulation loop.<br>
This will inform Chrono to use the Pardiso solver from PardisoProject.
~~~{.cpp}
auto parproj_solver = chrono_types::make_shared<ChSolverPardisoProject>();
my_system.SetSolver(parproj_solver);
~~~


- (Optional) Turn on the sparsity pattern lock (see @ref chrono::ChSolverPardisoProject and @ref chrono::ChDirectSolverLS for further details)
~~~{.cpp}
auto parproj_solver = chrono_types::make_shared<ChSolverPardisoProject>();
parproj_solver->SetSparsityPatternLock(true);
my_system.SetSolver(parproj_solver);
~~~


- By default, this solver uses the sparsity pattern learner (see @ref chrono::ChDirectSolverLS) to infer the sparsity pattern before actually loading the non-zero elements.  To disable the use of the sparsity pattern learner, call 
~~~{.cpp}
parproj_solver->UseSparsityPatternLearner(false);
~~~


- Look at the [API section](group__pardisoproject__module.html) of this module for documentation about classes and functions.