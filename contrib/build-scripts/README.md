Utility scripts for configuring, building, and installing Chrono 3rd-party dependencies
==============

The scripts in this directory provide a simple mechanism for installing dependencies for the various Chrono modules.  The scripts are organized iun sub-directories by OS: Bash shell scripts in `linux/` and Windows batch scripts in `windows/`. The directory `macOS/` contains Bash shell scripts tailored for the specifics of MacOS. If a script is not present in that directory, simply use the generic one in `linux/`.

Notes
- the scripts assume that git, cmake, and a C++ compiler are available
- additional requirements (if any) are indicated in the comments at the top of each script
- usage instructions are listed in the comments at the top of each script

Currently, scripts are provided for the following packages:
- Eigen3 -- the only external dependency of the core Chrono module
- Blaze -- a linear algebra package required by Chrono::Multicore
- GL and GLEW -- OpenGL packages used in Chrono::OpenGL and (optionally) in Chrono::Sensor
- MUMPS -- a direct sparse linear solver required by Chrono::Mumps
- OpenCRG -- a file format for road description used (optionally) in Chrono::Vehicle
- Spectra -- an algebra package required by Chrono::Modal
- URDF -- URDF file parser utilities optionally used by Chrono::Parsers
- VDB -- OpenVDB packages optionally used in Chrono::Sensor
- VSG -- VulkanSceneGraph packages required by Chrono::VSG

In addition, each sub-directory includes a sample script for configuring and building Chrono with various modules enabled and satisfying the corresponding dependencies.

## Usage

Consult the instructions in the comments at the top of each script.
The general usage is as follows:
- copy the desired script(s) in a workspace directory (each script will create temporary directories to download the necessary sources and to configure and build the various packages)
- edit the script to tailor to your particular machine (at a minimum, set the desired installation directory)
- run the script
- verify that the package files were properly installed in the specified install directory

With the exception of OpenCRG, all packages built and installed by these scripts provide CMake configuration scripts which are used during CMake Chrono configuration to find the respective dependencies. To simplify the Chrono CMake configuration process and avoid manually specifying package-specific information every time, we recommend setting the `CMNAKE_PREFIX_PATH` environment variable to add the directory containing the CMake project configuration scripts for each installed package. 
An example of `CMAKE_PREFIX_PATH` (on a Windows machine) is:
`E:\Packages\eigen\share\eigen3\cmake;E:\Packages\blaze\share\blaze\cmake;E:\Packages\vsg\lib\cmake;E:\Packages\urdf\lib\urdfdom\cmake;E:\Packages\urdf\CMake;E:\Packages\spectra\share\spectra\cmake;E:\Packages\gl\lib\cmake;E:\Packages\mumps\cmake;C:\OpenCASCADE-7.4.0-vc14-64\opencascade-7.4.0\cmake;C:\Program Files (x86)\Intel\oneAPI\mkl\2023.0.0\lib\cmake\mkl;`

Since many of these packages create and install shared libraries, the following steps may be required:
- on Linux, run `ldconfig` (you will likely need root permissions) to cache the necessary link to the newly created shared libraries or set the `LD_LIBRARY_PATH` environment variable
- on Windows, add to the `PATH` environment variable the directories containing the package DLLs (otherwise, these DLLs have to be manually copied next to the binaries so that they can be found at run-time)
