Install the MKL module {#module_mkl_installation}
==========================

[TOC]

This is an optional module that enables Chrono to use the Intel MKL Pardiso solver.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.

Chrono::Engine usually relies on its [built-in solvers](@ref solvers), whose good perfomance are guaranteed by leveraging the internal data structure. 
In fact, for a wide range of applications, these suffice.<br>
However, for higher accuracy results, a direct solver could still be needed.

This module provides an interface to the third-party Intel MKL Pardiso solver.


## Features

The MKL module allows to plug the Intel MKL Pardiso solver into Chrono::Engine and provides two interface:
- an interface for Chrono - namely @ref chrono::ChSolverMKL<> - that is **not** intended to be used directly by the user.<br>
This is the interface that the user should plug into the Chrono environment.
- an interface for the end-user - namely @ref chrono::ChMklEngine - that allows to directly operate with Pardiso using the Chrono data classes (if the user would ever have this need).<br>
The demo_MKL_MklEngine.cpp shows its usage, but the average usare should not be interested in it.

Two Chrono-specific features are implemented:
- **sparsity pattern _lock_**<br>
    In many cases, the internal data structures undergo very little changes between different timesteps.
	In these cases it is useful to inform the solver that the sparsity pattern does not change consistently in order to speed up the matrix assembling.
- **sparsity pattern _update_**<br>
    At the very first step, the matrix that has to be assembled does not know the sparsity pattern of the problem. In order to speed up the assembly phase it could be useful to pre-acquire the sparsity pattern.

Look at the [API section](group__mkl__module.html) of this module for a more in-depth discussion.
	
## Remarks
In order to operate, the Pardiso solver requires a matrix in [CSR3 format](https://software.intel.com/en-us/mkl-developer-reference-fortran-sparse-blas-csr-matrix-storage-format).<br>
Unfortunately Chrono does *not* operate natively on this kind of matrix, so a conversion from Chrono matrices to CSR3 matrices is required. <br>
This should not affect the user: this conversion is handled internally, and it is completely transparent to the user.

However, a not negligibile time must be taken into account for this conversion.


## Requirements
[Intel MKL Library]: https://software.intel.com/en-us/mkl
[Intel MKL Redistributables]: https://software.intel.com/en-us/articles/intelr-composer-redistributable-libraries-by-version

- To **build** applications based on this unit:
	+ the [Intel MKL Library] must be installed on your machine.

- To **run** applications based on this unit:
	+ the [Intel MKL Redistributables] must be installed on your machine and the environmental variables has to be properly set.

<div class="ce-info">
The [Intel MKL Redistributables] are included into [Intel MKL Library]
</div>

<div class="ce-info">
The Intel MKL Library is now [distributed for **free**](https://software.intel.com/en-us/articles/free-mkl)!
</div>

## Building instructions

1. Install the Intel MKL Library following the [Developer Guide](https://software.intel.com/en-us/mkl-windows-developer-guide). No particular skills are needed.
    + if you are *not* installing to the default directory, then you have to set the environmental variable `MKLROOT` in order to point to<br>
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

When you build the project, you could find some demo_MKL_xxx examples in the 
binary directory, among other default demos.<br>
Please mind that are demo_MKL_MklEngine provides an example for those who wants to use the MKL interface without running any simulation.
So it is not inteded for the average user.

Beware! If you run the MKL demos, you might find that they are not able to find the MKL run time libraries in your PATH. The reason is that you should execute the `mklvars.bat` configuration in your shell, using the appropriate parameters to set the paths to find the Intel run time libraries, as stated in Intel MKL documentation about the installation. For example, in Windows, do 
`cd "C:\Program Files (x86)\IntelSWTools\compilers_and_libraries_2019\windows"`  then `mklvars intel64` then execute the MKL demos from the shell.

However, if you find annoying to call mklvars all times from a shell, or if you do not want to embed it in your system startup, you can manually do the following:

1. Add MKL libraries to your system path:
    The MKL installer sets a wide set of environment variables, but not the one needed by MKL.<br>
	In order to set the variables once for all you have to add these folder in your `PATH` (Windows) or `LD_LIBRARY_PATH` (Linux or MacOS) environmental variable:
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
This will inform Chrono to use the MKL interface.
~~~{.cpp}
auto mkl_solver = std::make_shared<ChSolverMKL<>>();
application.GetSystem()->SetSolver(mkl_solver); // or my_system->SetSolver(mkl_solver);
~~~

- (Optional) Turn on the sparsity pattern lock and sparsity pattern learner (see @ref chrono::ChSolverMKL<> for further references)
~~~{.cpp}
auto mkl_solver = std::make_shared<ChSolverMKL<>>();
mkl_solver->SetSparsityPatternLock(true);
mkl_solver->ForceSparsityPatternUpdate(true);
application.GetSystem()->SetSolver(mkl_solver); // or my_system->SetSolver(mkl_solver);
~~~

- Look at the [API section](group__mkl__module.html) of this module for documentation about classes and functions.