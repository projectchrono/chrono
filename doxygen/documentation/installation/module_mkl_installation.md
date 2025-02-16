Install the Pardiso MKL module {#module_mkl_installation}
==========================

[TOC]

This is an optional module that enables Chrono to use the Intel MKL Pardiso solver.

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.

This module provides access to the third-party Intel MKL Pardiso sparse direct linear solver.
The interface to Pardiso is provided by [Eigen](https://eigen.tuxfamily.org/dox/classEigen_1_1PardisoLU.html).

## Features

Two Chrono-specific features are implemented:
- **sparsity pattern _lock_**<br>
	In many cases, internal data structures experience minimal changes between timesteps. When this occurs, informing the solver about the consistent sparsity pattern can significantly accelerate matrix assembly.
- **sparsity pattern _learner_**<br>
    The sparsity pattern \e learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly. Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting any nonzeros)
Look at the [API section](group__mkl__module.html) of this module for more details.

## Requirements
[Intel MKL Library]: https://software.intel.com/en-us/mkl

## Building instructions

### Linux
1. The MKL Library can be installed using the APT Package Manager by following the installation instructions spelled out [here](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=linux&distributions=aptpackagemanager). You can also choose various different Installers from the drop down menu on the webpage if you prefer not to use APT.  

2. Follow the guide for the [full installation](@ref tutorial_install_chrono) of Chrono, but when running CMake make sure to set `CH_ENABLE_MODULE_PARDISO_MKL` to `ON`.<br>
    The CMake output window on Linux systems should return the following (note that the MATH library is not required.):
~~~~~
 Find MKL libraries
    MKL include dirs:   /opt/intel/oneapi/mkl/latest/include
    MKL libraries:      /opt/intel/oneapi/mkl/latest/lib/intel64/libmkl_rt.so
    IOMP5 library:      /opt/intel/oneapi/compiler/2024.0/lib/libiomp5.so
    MATH library:       MATH_LIBRARY-NOTFOUND
    MKL library dirs:   /opt/intel/oneapi/mkl/latest/lib/intel64;/opt/intel/oneapi/compiler/2024.0/lib
~~~~~  

3. Press 'Generate' and build the project.

Building this module will produce an additional shared library, called **ChronoEngine_pardisomkl**, which can be linked to your application.<br>
On Linux, the file extension will be .so.

### Windows
1. Install the Intel MKL Library following the [Get Started Guide](https://software.intel.com/content/www/us/en/develop/documentation/get-started-with-mkl-for-dpcpp/top.html). 

   If you are *not* installing to the default directory, then you have to set the environment variable `MKLROOT` to point to the MKL folder, for example:<br>
    `<install_folder>/Intel/oneAPI/mkl` (Windows + MKL>=2020)<br>
	`<install_folder>/IntelSWTools/compilers_and_libraries/windows/mkl` (Windows + MKL=2016..2019)
	
2. Follow the guide for the [full installation](@ref tutorial_install_chrono) of Chrono, but when running CMake make sure that also the option `CH_ENABLE_MODULE_PARDISO_MKL` is set to `ON`.<br>
    The CMake output window, on Windows OS, should return the following:<br>(Please mind that the MATH library is not required.)
~~~~~
   MKL include dirs:   C:/Program Files (x86)/Intel/oneAPI/mkl/latest/include
   MKL libraries:      C:/Program Files (x86)/Intel/oneAPI/mkl/latest/lib/intel64/mkl_rt.lib
   IOMP5 library:      C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/compiler/lib/intel64_win/libiomp5md.lib
   MATH library:       C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/compiler/lib/intel64_win/libmmd.lib
   MKL library dirs:   C:/Program Files (x86)/Intel/oneAPI/mkl/latest/lib/intel64;C:/Program Files (x86)/Intel/oneAPI/compiler/latest/windows/compiler/lib/intel64_win
~~~~~

3. Press 'Generate' and build the project.

Building this module will produce an additional shared library, called **ChronoEngine_pardisomkl**, which can be linked to your application.<br>
On Windows, the file extension will be .dll.


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
