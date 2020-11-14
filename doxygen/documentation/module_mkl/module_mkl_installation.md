Install the Pardiso MKL module {#module_mkl_installation}
==========================

[TOC]

This is an optional module that enables Chrono to use the Intel MKL Pardiso solver.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.

Chrono::Engine usually relies on its [built-in solvers](@ref solvers), whose good perfomance are guaranteed by leveraging the internal data structure. 
In fact, for a wide range of applications, these suffice.<br>
However, for higher accuracy results, a direct solver could still be needed.

This module provides access to the third-party Intel MKL Pardiso solver. The interface to Pardiso is provided by [Eigen](https://eigen.tuxfamily.org/dox/classEigen_1_1PardisoLU.html).

## Features

Two Chrono-specific features are implemented:
- **sparsity pattern _lock_**<br>
    In many cases, the internal data structures undergo very little changes between different timesteps.
	In these cases it is useful to inform the solver that the sparsity pattern does not change consistently in order to speed up the matrix assembly.
- **sparsity pattern _learner_**<br>
    The sparsity pattern \e learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly. Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting any nonzeros)
Look at the [API section](group__mkl__module.html) of this module for a more details.

## Requirements
[Intel MKL Library]: https://software.intel.com/en-us/mkl
[Intel MKL Redistributables]: https://software.intel.com/en-us/articles/intelr-composer-redistributable-libraries-by-version

- To **build** applications based on this unit:
	+ the [Intel MKL Library] must be installed on your machine.

- To **run** applications based on this unit:
	+ the [Intel MKL Redistributables] must be installed on your machine and the environment variables has to be properly set.

<div class="ce-info">
The [Intel MKL Redistributables] are included into [Intel MKL Library]
</div>

<div class="ce-info">
The Intel MKL Library is now [distributed for **free**](https://software.intel.com/en-us/articles/free-mkl)!
</div>

## Building instructions

1. Install the Intel MKL Library following the [Developer Guide](https://software.intel.com/en-us/mkl-windows-developer-guide). No particular skills are needed.
    + if you are *not* installing to the default directory, then you have to set the environment variable `MKLROOT` in order to point to<br>
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/mkl` (Windows + MKL>=2016)<br>
	`<install_folder>/intel/compilers_and_libraries/linux/mkl` (Linux + MKL>=2016)
	
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window,<br>
    you must set `ENABLE_MODULE_MKL` as 'on'.<br>
    The CMake output window, on Windows OS, should return the following: (Please mind that the MATH library is not required.)
~~~~~
Find MKL libraries
MKL include dirs:  C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/include
MKL libraries:     C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/lib/intel64/mkl_rt.lib
IOMP5 library:     C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/compiler/lib/intel64/libiomp5md.lib
MATH library:      C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/compiler/lib/intel64/libmmd.lib
MKL library dirs:  C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/mkl/lib/intel64;C:/Program Files (x86)/IntelSWTools/compilers_and_libraries/windows/compiler/lib/intel64
~~~~~

3. Press 'Generate' and build the project.

Building this unit will produce an additional shared library, called **ChronoEngine_mkl**, that can be linked to your application if you want to use it.<br>
The file extension will be .dll for Win and .so on Linux.

Beware! If you run the demos using Chrono::MKL, you might find that they are not able to find the MKL run time libraries in your PATH. The reason is that you should execute the `mklvars.bat` configuration in your shell, using the appropriate parameters to set the paths to find the Intel run time libraries, as stated in Intel MKL documentation about the installation. For example, in Windows, do 
`cd "C:\Program Files (x86)\IntelSWTools\compilers_and_libraries_2019\windows"`  then `mklvars intel64` then execute the MKL demos from the shell.

However, if you find annoying to call mklvars all times from a shell, or if you do not want to embed it in your system startup, you can manually do the following:

1. Add MKL libraries to your system path:
    The MKL installer sets a wide set of environment variables, but not the one needed by MKL.<br>
	In order to set the variables once for all you have to add these directories in your `PATH` (Windows) or `LD_LIBRARY_PATH` (Linux or MacOS) environment variable:
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/redist/intel64_win/mkl`<br>
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/redist/intel64_win/tbb/vc_mt`<br>
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/redist/intel64_win/compiler`<br>
	or<br>
	`<install_folder>/intel/compilers_and_libraries/linux/redist/intel64/mkl`<br>
	`<install_folder>/intel/compilers_and_libraries/linux/redist/intel64/tbb/vc_mt`<br>
	`<install_folder>/intel/compilers_and_libraries/linux/redist/intel64/compiler`<br>
	(on ia32 platforms replace intel64 with ia32 in the lines above).
	
2. Add these system environment variables<br>
	`MKL_INTERFACE_LAYER` = `LP64`<br>
	`MKL_THREADING_LAYER` = `INTEL`<br>
	or, more in general, you can have [different options](https://software.intel.com/en-us/mkl-linux-developer-guide-dynamically-selecting-the-interface-and-threading-layer), depending on your Architecture and the desired Threading Layer.

By doing this, you will be able to start the MKL-based demos and programs by just double clocking on them, without the need of the `mklvars.bat` script.




## How to use it

- Simply add this snippet anywhere in your code, before running the main simulation loop.<br>
This will inform Chrono to use the Eigen interface to the Intel MKL Pardiso solver.
~~~{.cpp}
auto mkl_solver = std::make_shared<ChSolverPardisoMKL>();
my_system.SetSolver(mkl_solver);
~~~


- (Optional) Turn on the sparsity pattern lock (see @ref chrono::ChSolverPardisoMKL and @ref chrono::ChDirectSolverLS for further details)
~~~{.cpp}
auto mkl_solver = std::make_shared<ChSolverPardisoMKL>();
mkl_solver->SetSparsityPatternLock(true);
my_system.SetSolver(mkl_solver);
~~~


- By default, this solver uses the sparsity pattern learner (see @ref chrono::ChDirectSolverLS) to infer the sparsity pattern before actually loading the non-zero elements.  To disable the use of the sparsity pattern learner, call 
~~~{.cpp}
mkl_solver->UseSparsityPatternLearner(false);
~~~


- Look at the [API section](group__mkl__module.html) of this module for documentation about classes and functions.