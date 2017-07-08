Install the PARALLEL module {#module_parallel_installation}
==========================

[TOC]

Parallel solver module for Chrono.


## Features

The **PARALLEL module** provides features for performing multibody simulations
using parallel computing within Chrono::Engine

- introduces a custom ChSystemParallel class
- implements a high-performance parallel collision detection algorithm
- uses efficient APIs for parallelism (OpenMP, Thrust, etc.)
- planned: support of MPI parallelism


## Requirements

- To **run** applications based on this module you need:
    - the [Blaze](https://code.google.com/p/blaze-lib/) library
    - the [Boost](http://www.boost.org) library
    - the [Thrust](https://github.com/thrust/thrust) (also included in CUDA SDK)

- To **build** this module you need:
    - the [Blaze](https://code.google.com/p/blaze-lib/) library
    - the [Boost](http://www.boost.org) library
    - the [Thrust](https://github.com/thrust/thrust) (also included in CUDA SDK)


## Building instructions
  
1. Download and install these libraries (depending on the platform, the process can be different)
    - the [Blaze](https://code.google.com/p/blaze-lib/) library
    - the [Boost](http://www.boost.org) library
    - the [Thrust](https://github.com/thrust/thrust) (also included in CUDA SDK)

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
3. Set the `ENABLE_MODULE_PARALLEL` as 'on', then press 'Configure' (to refresh the variable list) 
 
4. Set the path for `BLAZE_DIR`

5. Set the path for `BOOST_DIR`
	 
5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-info">
This unit is under development. 
</div>

<div class="ce-warning">
Not all the features of the standard _serial_ version of Chrono::Engine are supported.
</div>


## How to use it

- Look at the [API section](group__parallel__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
