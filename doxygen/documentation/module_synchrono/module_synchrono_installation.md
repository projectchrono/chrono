Install the SYNCHRONO module   {#module_synchrono_installation}
===============================

[TOC]

This is an optional module that enables the paralellization of Chrono::Vehicle simulation across MPI computing ranks.

Read [the introduction to modules](modularity.html) for a technical
background on the modularity of the Chrono project.


## Features

The **SynChrono module** allows users to paralellize the dynamics computations for Chrono::Vehicles across MPI ranks


## Required Dependencies

- To build applications based on this module, the following are required:
  * MPI - Tested with [OpenMPI](https://www.open-mpi.org/) and [MPICH](https://www.mpich.org/) on Linux and with [Intel MPI](https://software.intel.com/en-us/mpi-library/choose-download/windows) and [MS-MPI](https://docs.microsoft.com/en-us/message-passing-interface/microsoft-mpi) on Windows. NOTE: On Windows please ensure that the MPI dll (either `impi.dll` or `msmpi.dll`) is on your system path, for example by running a post-installation script ([Intel MPI example](https://software.intel.com/content/www/us/en/develop/documentation/mpi-developer-guide-windows/top/installation-and-prerequisites/prerequisite-steps.html)).
  * [FlatBuffers](http://google.github.io/flatbuffers/) - A version is included in chrono_thirdparty as a submodule but a separately-installed version (for example through a package manager) will work as well.

## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono). The Chrono::Vehicle is required and at least one of Chrono::Irrlicht or Chrono::Sensor is recommended for visualization. When you see the CMake window, you must add the following steps:

2. Set the `ENABLE_MODULE_SYNCHRONO` as 'on', then press 'Configure' (to refresh the variable list)

3. If Flatbuffers is on your system path (for example package-manager installed versions), then CMake may pick it up automatically. Otherwise set `Flatbuffers_DIR` to the `CMake` folder within your Flatbuffers installation (for example `../src/chrono_thirdparty/flatbuffers/CMake`) and set `Flatbuffers_INCLUDE_DIR` to the include directory within your Flatbuffers installation (for example `../src/chrono_thirdparty/flatbuffers/include`).

4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Consult the [SynChrono API](@ref synchrono).
- Read the [overview](@ref module_synchrono_overview).
