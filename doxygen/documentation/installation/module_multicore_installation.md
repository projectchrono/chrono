Install the MULTICORE module {#module_multicore_installation}
==========================

[TOC]

Multicore solver module for Chrono.


## Features

The **MULTICORE module** provides features for performing multibody simulations
using shared-memory parallel computing within Chrono

- introduces a custom ChSystemMulticore class
- implements a high-performance multicore collision detection algorithm
- uses efficient APIs for parallelism (OpenMP, Thrust, etc.)


## Requirements

- To **build** this module you need:
    - the [Blaze](https://bitbucket.org/blaze-lib/blaze) library, version 3.8.
    - the [Thrust](https://github.com/thrust/thrust) (also included in CUDA SDK).

<div class="ce-warning">
The easiest way to obtain the Thrust library is by installing the CUDA SDK. 
Alternatively, you can download or clone Thrust from its [GitHub repository](https://github.com/thrust/thrust). In that case, you will need to manually specify the path to the Thrust CMake configuration script (set the variable `Thrust_DIR`).
</div>


## Building instructions
  
1. Download the following libraries (depending on the platform, the process can be different)
    - [Blaze](https://bitbucket.org/blaze-lib/blaze) library, version 3.8
    - [Thrust](https://github.com/thrust/thrust) (also included in CUDA SDK)

    For use with the Chrono::Multicore module, no installation is required for the above headers-only libraries. However, if so desired, they can be installed in the appropriate system directories (on platforms that support them).

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
   
3. Set `CH_ENABLE_MODULE_MULTICORE` to 'on'.
 
4. If prompted, set the path for `Blaze_ROOT_DIR`.
	 
5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
Not all the features of the standard _serial_ version of Chrono are supported.
</div>


## How to use it

- Look at the [API section](group__multicore__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_multicore) to learn how to use the functions of this module.
