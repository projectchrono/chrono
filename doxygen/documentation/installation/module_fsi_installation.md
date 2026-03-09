Install the FSI module {#module_fsi_installation}
==========================

[TOC]

Chrono::FSI is a module for modeling and simulation of fluid-solid interaction problem. 
It provides the framework for coupling a Chrono multibody system (for rigid and/or flexible dynamics) with an arbitrary FSI-aware fluid solver.
Two related Chrono submodules are available: 
- Chrono::FSI-SPH which implements a fluid solver based on the Smoothed Particle Hydrodynamics method
- Chrono::FSI-TDPF which implements a fluid solver based on the Time Domain Potential Flow method

## Features

The **FSI-SPH module** allows users to:

- Use the module as a CFD solver for fluid mechanics problems via the following Lagrangian methods:
   - Implicit incompressible SPH (ISPH)
   - Explicit weakly compressible SPH (WCSPH)
- Use the module as a solver for granular material dynamics (CRM) via the following Lagrangian method:
   - Explicit weakly compressible SPH (WCSPH)
- Use the module for solving fluid-solid interaction problems that feature:
   - Rigid bodies
   - Flexible bodies (FEA models)
- Use the module for solving rigid multi-body dynamics and its interaction with CRM deformable terrain
   - Rover/vehicle mobility simulation on granular material terrain

The **FSI-TDPF module** allows users to:

- Use the module for solving fluid-solid interaction problems that feature rigid multibody systems

## Requirements

- To **build** the FSI-SPH module and related applications, a CUDA installation and appropriate compiler are required.<br>
  The FSI-SPH module requires CUDA version 12.9 (**NOTE**: CUDA 13 is not yet supported).
- To **build** the FSI-TDPF module and related applications, HDF5 support is required and must be enabled (`CH_ENABLE_HDF5`).
- To **run** applications based on the FSI-SPH module an NVIDIA GPU card is required.


## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).

2. Set `CH_ENABLE_MODULE_FSI` to 'on'.

3. If all dependencies are satisfied, both the FSI-SPH and FSI-TDPF modules are enabled. 
   Optionally disable the FSI-SPH module (via `CH_ENABLE_MODULE_FSI_SPH`) or the FSI-TDPF module (via `CH_ENABLE_MODULE_FSI_TDPF`)

4. If the FSI-SPH module is enabled, optionally set `CH_USE_SPH_DOUBLE` to `ON` to use double precision in the SPH solver.<br>
   The single precision FSI solver has been tested to have similar level of accuracy as the double precision solver with close to a 2X performance improvement.

5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

## How to use it

- Look at the [API section](group__fsi__base.html) of the Chrono::FSI module for documentation about the base classes and functions.
- Look at the [API section](group__fsisph.html) for documentation of the classes and functions for the FSI-SPH solver.
- Look at the [API section](group__fsitdpf.html) for documentation of the classes and functions for the FSI-TDPF solver.
- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_fsi) to learn how to use the functions of these modules.

## MacOS support

This module cannot be built for MacOS, Nvidia GPU hardware and CUDA are unsupported.
