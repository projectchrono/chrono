Install the SYNCHRONO module   {#module_synchrono_installation}
===============================

This is an optional module that enables the parallelization of Chrono::Vehicle simulation across different computing entities.

Read [the introduction to modules](modularity.html) for a technical
background on the modularity of the Chrono project.


## Features

The **SynChrono module** allows users to parallelize the dynamics computations for Chrono::Vehicles across MPI or DDS entities.


## Required Dependencies

- To build applications based on this module, the following are required:
  * MPI
    - Linux: Tested with [OpenMPI](https://www.open-mpi.org/) and [MPICH](https://www.mpich.org/).
    - Windows: Tested with [Intel MPI](https://software.intel.com/en-us/mpi-library/choose-download/windows) and [MS-MPI](https://docs.microsoft.com/en-us/message-passing-interface/microsoft-mpi). You must ensure that the MPI dll (either `impi.dll` or `msmpi.dll`) is on your system path, for example by running a post-installation script ([Intel MPI example](https://software.intel.com/content/www/us/en/develop/documentation/mpi-developer-guide-windows/top/installation-and-prerequisites/prerequisite-steps.html)).
  * [FlatBuffers](http://google.github.io/flatbuffers/) 
    - A version is included in chrono_thirdparty as a submodule, for general development either that version or an externally built one can be used.
- The following are optional:
  * [FastDDS](https://fast-dds.docs.eprosima.com/en/latest/)
    - Please use the [latest binaries](https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-dds) for installation.
    - Versions provided by package managers will likely work as well if they are reasonably up to date.

## Building instructions

1. Initialize the FlatBuffers submodule in `chrono_thirdparty` with `git submodule init` and `git submodule update`. 
    - For running programs with SynChrono you just need the FlatBuffers header from initializing the submodule.
    - If you make modifications to the FlatBuffers message schemas you will need either an external version of the FlatBuffers binaries (for example from a package manager) or to [build the submodule version](https://google.github.io/flatbuffers/flatbuffers_guide_building.html) included in `chrono_thirdparty`.

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono). Chrono::Vehicle is required and at least one of Chrono::Irrlicht or Chrono::Sensor is recommended for visualization. When you see the CMake window, you must make these additional changes:

3. Set the `ENABLE_MODULE_SYNCHRONO` as 'on', then press 'Configure' (to refresh the variable list).

4. (Optional) If using Fast_DDS set `USE_FAST_DDS` to 'on', then press 'Configure'. 
    - If you used the installed binaries they should be picked up automatically by CMake
    - If you built from source or installed in a non-standard location, you'll have to manually specify `FAST_DDS_INSTALL_DIR`.

5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Consult the [SynChrono API](@ref synchrono).
- Read the [overview](@ref module_synchrono_overview).
