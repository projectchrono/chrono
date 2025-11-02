Install the MUMPS module {#module_mumps_installation}
==========================

[TOC]

Chrono::Mumps is an optional module that enables Chrono to use the MUMPS linear solver.

Chrono usually relies on its [built-in solvers](@ref solvers), whose good performance are guaranteed by leveraging the internal data structure. 
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

	
## Building and installing prerequisistes

The Chrono::Mumps module requires the [MUMPS](http://mumps.enseeiht.fr) library. 
Mumps does not provide a CMake-based installation system. To address this issue, we provide (with the Chrono source code) a set of utility scripts which download the Mumps sources, build all necessary libraries, and install them in a user-specified location.

These scripts (`buildMUMPS.bat`, `buildMUMPS.sh`, and buildMUMPS_Mac.sh, for Windows, Linux, and MacOS, respectively) are available in the `contrib/build-scripts/mumps` directory of the [Chrono repository](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/mumps). 

1. Copy the appropriate script and place in an arbitrary temporary directory.
2. Edit the script copy to:
   - Force a download of the source codes.
   - Specify the install directory (set the variable `MUMPS_INSTALL_DIR`).
   - Decide whether to build shared or static libraries and whether to also build debug libraries.
3. Run the script (`.\buildMUMPS.bat` or `sh buildMUMPS.sh`, as appropriate) from the location of the script copy. This will create a temporary directory where all source repositories will be cloned and a set of directories where the individual VSG dependencies are built.
4. The install directory will contain (under `MUMPS_INSTALL_DIR/cmake/`) all CMake project configuration scripts required to configure Chrono with the Chrono::Mumps module enabled.

## Building instructions

1. Install the MUMPS library (see above)

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).

3. Set `CH_ENABLE_MODULE_MUMPS` to 'on'.

4. Set the CMake variable `MUMPS_DIR` to point to the directory containing the MUMPS CMake project configuration script
   (for example, `C:/Packages/mumps/cmake`).

5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
If using the Intel oneAPI Fortran compiler, `ifort` was superseded by `ifx` in the latest releases of oneAPI. 
<br>
To ensure that CMake uses the correct Fortran compiler, make sure to explicitly specify it when invoking CMake, by passing `-T fortran=ifx` (of course, this assumes that you have properly configured Intel oneAPI and `ifx` is in the search path).
</div>


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