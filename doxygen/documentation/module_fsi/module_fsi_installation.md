Install the FSI module {#module_fsi_installation}
==========================

[TOC]

This optional module enables the modeling and simulation of fluid-solid interaction problem. 

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.

## Features

The **FSI module** allows users to:

- Use the module as a CFD solver for fluid mechanics problems via the following Lagrangian methods:
   - Implicit Incompressible SPH
   - Explicit SPH
- Use the module for solving fluid-solid interaction problems that feature:
   - Rigid bodies
   - 1D and 2D flexible bodies simulated via ANCF cable and ANCF shell elements, respectively.
- Use GPU-based sparse linear solvers such as BICGSTAB
- Use JSON input files for easy specification of simulation parameters

## Requirements

- To **run** applications based on this module an NVIDIA GPU card is required.
- To **build** this module and applications based on it, a CUDA installation and appropriate compiler are required
- This module has been build/tested on the following:
   - Linux, CUDA 10, GCC 7.1.0 (Pascal, Volta, and Turing GPU architectures)
   - Linux, CUDA 9.0, GCC/6.1.0 (Pascal, Volta, and Turing GPU architectures)
   - Linux, CUDA 7.5, GCC/4.9.2 (Kepler GPU architecture)
   - Windows, CUDA 10.0, MS Visual Studio 2015 and 2017 (Pascal GPU architecture)

<div class="ce-warning">
Running Chrono::FSI programs on Windows may require adjusting the Timeout Detection and Recovery (TDR) settings. You may need to either increase the timeout delay or even disable TDR. This can be done through the [Windows registry keys](https://docs.microsoft.com/en-us/windows-hardware/drivers/display/tdr-registry-keys) or using the NIDIA [Nsight Monitor](http://developer.download.nvidia.com/NsightVisualStudio/2.2/Documentation/UserGuide/HTML/Content/Timeout_Detection_Recovery.htm)
</div>

## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:

2. Set the `ENABLE_MODULE_FSI` as 'on', then press 'Configure' (to refresh the variable list)

3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

## How to use it

- Look at the [API section](group__fsi.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_fsi) to learn how to use the functions of this module.
