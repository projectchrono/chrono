Install the Pardiso MKL module {#module_mkl_installation}
==========================

[TOC]

Chrono::PardisoMKL is an optional module that exposes the direct sparse linear solver Pardiso from the Intel MKL library.
The interface to Pardiso is provided by [Eigen](https://eigen.tuxfamily.org/dox/classEigen_1_1PardisoLU.html).

## Features

Two Chrono-specific features are implemented:
- **sparsity pattern _lock_**<br>
	In many cases, internal data structures experience minimal changes between timesteps. When this occurs, informing the solver about the consistent sparsity pattern can significantly accelerate matrix assembly.
- **sparsity pattern _learner_**<br>
    The sparsity pattern \e learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly. Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting any nonzeros)
Look at the [API section](group__mkl__module.html) of this module for more details.

## Requirements

This module requires the [Intel MKL Library](https://software.intel.com/en-us/mkl).
Note that Intel MKL can be installed stand-alone or as part of the Intel oneAPI [Base Toolkit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit.html).

- On Linux, the MKL Library can be installed using the distribution's package manager by following Intel's [installation instructions](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=linux&distributions=aptpackagemanager). 
- On Windows, follow the Intel MKL Library [installation instructions](https://software.intel.com/content/www/us/en/develop/documentation/get-started-with-mkl-for-dpcpp/top.html). 


<div class="ce-warning">
<span style="color:red; font-weight:bold">ATTENTION!</span><br>
Eigen3 was **not** updated to work with oneAPI and MKL 2025.<br>
Intel MKL 2023 is known to work fine with Eigen (and therefore Chrono) on both Linux and Windows.
</div>

## Building instructions
	
1. Follow the guide for the [full installation](@ref tutorial_install_chrono) of Chrono, but when running CMake make sure that also the option `CH_ENABLE_MODULE_PARDISO_MKL` is set to `ON`.

2. If prompted, set MKL_DIR to the directory with a CMake configuration file for MKL. Depending on the OS, this will be something like:
    - `C:/Program Files (x86)/Intel/oneAPI/mkl/2023.0.0/lib/cmake/mkl` (Windows)
	- `/opt/intel/oneapi/mkl/latest/lib/cmake/mkl` (Linux)

3. Press 'Configure' again and then 'Generate'

4. Building this module will produce an additional shared library, called **Chrono_pardisomkl**, which can be linked to your application.

## How to use it

### Set up the environment

Before creating executables that can actually leverage the Intel MKL library, you have to make sure that the Intel _run-time_ libraries are available to the executable itself. This is achieved by setting the PATH environmental variable in a proper way. To do so, two options are available:
+ the [official method proposed by Intel](https://software.intel.com/content/www/us/en/develop/documentation/onemkl-windows-developer-guide/top/getting-started/setting-environment-variables.html); this method _temporarly_ sets the PATH variable only from the current command prompt session; this means that, if you run it from Visual Studio or from a new command prompt, this method is difficult/cumbersome for you;
+ the unofficial method, for which the PATH variable is set manually, but _once and for all_; description follows...

The following unofficial method needs that you set the environmental variable of your system.

1. Add the following MKL directories to your system `PATH` (Windows) or `LD_LIBRARY_PATH` (Linux or MacOS) environment variable, adapting them to your OS and architecture:<br>
	**For Windows:**<br>
	`<install_folder>/Intel/oneAPI/mkl/latest/redist/intel64`<br>
	`<install_folder>/Intel/oneAPI/compiler/latest/windows/redist/intel64_win/compiler`<br>
	or, for older installations:<br>
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/redist/intel64_win/mkl`<br>
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/redist/intel64_win/compiler`.<br>
	**For Linux:**<br>
	`/opt/intel/oneapi/mkl/latest/lib/intel64`<br>
	`/opt/intel/oneapi/compiler/2024.0/lib/`
	
1. Add these system environment variables (both Windows and Linux)<br>
	`MKL_INTERFACE_LAYER` = `LP64`<br>
	`MKL_THREADING_LAYER` = `INTEL`<br>
	or, more in general, you can have [different options](https://software.intel.com/en-us/mkl-linux-developer-guide-dynamically-selecting-the-interface-and-threading-layer), depending on your Architecture and the desired Threading Layer.

2. reboot your IDE, close any open CMake

By doing this, you will be able to start the MKL-based demos and programs directly from your IDE or just by double-clicking on them.

### Set up the code

- Simply add this snippet anywhere in your code, before running the main simulation loop.<br>
This will inform Chrono to use the Eigen interface to the Intel MKL Pardiso solver.
~~~{.cpp}
auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
my_system.SetSolver(mkl_solver);
~~~


- (Optional) Turn on the sparsity pattern lock (see @ref chrono::ChSolverPardisoMKL and @ref chrono::ChDirectSolverLS for further details)
~~~{.cpp}
auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
mkl_solver->SetSparsityPatternLock(true);
my_system.SetSolver(mkl_solver);
~~~


- By default, this solver uses the sparsity pattern learner (see @ref chrono::ChDirectSolverLS) to infer the sparsity pattern before actually loading the non-zero elements.  To disable the use of the sparsity pattern learner, call 
~~~{.cpp}
mkl_solver->UseSparsityPatternLearner(false);
~~~


- Look at the [API section](group__mkl__module.html) of this module for documentation about classes and functions.

## MacOS support

This module cannot be built for Macs with Apple Silicon hardware, Macs with Intel hardware can use it, see [here](https://www.intel.com/content/www/us/en/docs/onemkl/developer-guide-macos/2024-0/overview.html).
