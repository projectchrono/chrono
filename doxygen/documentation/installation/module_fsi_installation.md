Install the FSI module {#module_fsi_installation}
==========================

[TOC]

Chrono::FSI is a module for modeling and simulation of fluid-solid interaction problem. 

## Features

The **FSI module** allows users to:

- Use the module as a CFD solver for fluid mechanics problems via the following Lagrangian methods:
   - Implicit incompressible SPH (ISPH)
   - Explicit weakly compressible SPH (WCSPH)
- Use the module as a solver for granular material dynamics via the following Lagrangian method:
   - Explicit weakly compressible SPH (WCSPH)
- Use the module for solving fluid-solid interaction problems that feature:
   - Rigid bodies
   - 1D and 2D flexible bodies simulated via ANCF cable and ANCF shell elements, respectively.
- Use the module for solving rigid multi-body dynamics and its interaction with deformable terrain
   - Rover/vehicle mobility simulation on granular material terrain
- Use GPU-based sparse linear solvers such as BICGSTAB
- Use JSON input files for easy specification of simulation parameters

## Requirements

- To **run** applications based on this module an NVIDIA GPU card is required.
- To **build** this module and applications based on it, a CUDA installation and appropriate compiler are required
- This module has been build/tested on the following:
   - Windows, MS Visual Studio 2022, CUDA 12.8.93 (Ampere GPU architecture)
   - Ubuntu 22.04 Linux, GCC 11.4, CUDA 12.8.93 (Turing GPU architecture)
   - Ubuntu 22.04 Linux, GCC 11.3, CUDA 12.3.0 (Hopper GPU architecture)
   - Ubuntu 24.04 Linux, GCC 13.3, CUDA 13.0.0 (Blackwell GPU architecture)

## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).

2. Set `CH_ENABLE_MODULE_FSI` to 'on'.

3. Optionally, set `CH_USE_FSI_DOUBLE` to 'on', otherwise a single precision FSI solver will be built. The single precision FSI solver has been tested to have similar level of accuracy as the double precision solver with close to a 2X performance improvement.

4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

## How to use it

- Look at the [API section](group__fsi.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_fsi) to learn how to use the functions of this module.

## MacOS support

This module cannot be built for MacOS, Nvidia GPU hardware and CUDA are unsupported.
