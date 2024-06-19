Install the PARSERS module {#module_parsers_installation}
==========================

[TOC]

This module provides various import utilities.

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.


## Features

The **PARSERS module** provides utilities to import Chrono models from various file-based specifications. Parsers are provided for the following formats:
- URDF
- OpenSim
- Adams
- Python

Refer to the [user manual](@ref manual_parsers) for additional information.

## Dependencies

- The URDF parser depends on the urdfdom, urdfdom_headers, console_bridge, and tinyxml2 libraries.
- The Python parser requires Python 3 and the PythonInterp and PythonLibs libraries.


## Building and installing prerequisistes

The dependencies for the Python parser are satisfied by a Python 3 installation.

The URDF parser relies on features of `urdfdom` that are not yet available in the main branch of its official repository. As such, we currently rely on a fork of `urdfdom` which implements features that will eventually be incorporated in the upstream repository.  

The simplest way to build and install all requirements for the Chrono URDF parser is to use the utility scripts provided with the Chrono distribution. 
These scripts (`buildURDF.bat` and `buildURDF.sh`, for Windows and Linux, respectively) are available in the `contrib/build-scripts/urdf` directory of the [Chrono repository](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/urdf). 

1. Copy the appropriate script and place in an arbitrary temporary directory.
2. Edit the script copy to:
   - Force a download of the URDF library codes.
   - Specify the install directory (set the variable `URDF_INSTALL_DIR`).
   - Decide whether to also build debug libraries.
3. Run the script (`.\buildURDF.bat` or `sh buildURDF.sh`, as appropriate) from the location of the script copy. This will create a temporary directory where all source repositories will be cloned and a set of directories where the individual URDF dependencies are built.
4. The install directory will contain, under `URDF_INSTALL_DIR/CMake` (on Windows) and under subdirectories `URDF_INSTALL_DIR/***/cmake` (on Linux)) all URDF CMake project configuration scripts required to configure Chrono with the Chrono::Parser module enabled.


## Building instructions

Once the necessary dependencies are installed, perform the following steps to configure and build the Chrono::Parsers module:

1. Repeat the instructions for the [full Chrono installation](@ref tutorial_install_chrono)
   
2. During CMake configuration, set `ENABLE_MODULE_PARSERS` to 'on', then press 'Configure'

3. When prompted, provide the paths to the various URDF project configuration scripts (`urdfdom_DIR`, `urdfdom_headers_DIR`, and `console_bridge_DIR`). Assuming the dependencies were installed as described above, all these CMake variables should be set to `<URDF_INSATALL_DIR>/CMake` on Windows, while on Linux they should be `<URDF_INSTALL_DIR>/lib/urdfdom/cmake`, `<URDF_INSTALL_DIR>/lib/urdfdom_headers/cmake`, and `<URDF_INSTALL_DIR>/lib/console_bridge/cmake`, respectively.

4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


<div class="ce-warning">
When using shared libraries for third-party dependencies, you must ensure that these are found at run time.<br>
On Windows, you can either copy the dependency DLLs to the same directory as the executables or else add the path to these shared libraries to the `PATH` environment variable.<br>
On Linux, you may need to append to the `LD_LIBRARY_PATH` environment variable.
</div>


## Usage

- Consult the [API section](group__parsers__module.html) of this module for documentation about classes and functions.

- Consult the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
