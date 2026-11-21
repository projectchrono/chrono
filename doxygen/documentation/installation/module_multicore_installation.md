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
    - OpenMP support (`CH_ENABLE_OPENMP`, enabled by default).
    - the [Thrust](https://github.com/NVIDIA/thrust) headers. Chrono::Multicore uses Thrust with its OpenMP backend, so no GPU is required.

<div class="ce-warning">
The easiest way to obtain the Thrust library is by installing the CUDA toolkit, which includes it; Chrono finds it there automatically.<br>
On systems without CUDA (for example macOS), download a stand-alone Thrust release from its [GitHub repository](https://github.com/NVIDIA/thrust) (see `contrib/build-scripts/macos/buildThrust.sh`, which uses Thrust 1.17.2). In that case, set `Thrust_ROOT` to the directory that contains the `thrust/` subdirectory (alternatively, set `THRUST_INCLUDE_DIR` directly). Thrust's own CMake configuration scripts are not used.<br>
The same applies to builds with the HIP backend for AMD GPUs: the Thrust headers bundled with ROCm are ignored for host-side Chrono libraries, so a stand-alone Thrust is needed for Chrono::Multicore.
</div>


## Building instructions
  
1. Obtain the Thrust headers as described above (from the CUDA toolkit or as a stand-alone download).

    For use with the Chrono::Multicore module, no installation is required for this headers-only library. However, if so desired, it can be installed in the appropriate system directories (on platforms that support them).

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
   
3. Set `CH_ENABLE_MODULE_MULTICORE` to 'on'.
 
4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
Not all the features of the standard _serial_ version of Chrono are supported.
</div>


## How to use it

- Look at the [API section](group__multicore__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_multicore) to learn how to use the functions of this module.
