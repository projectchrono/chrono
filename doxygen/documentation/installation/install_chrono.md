Install Chrono {#tutorial_install_chrono}
==========================

\tableofcontents

The Chrono source code can be obtained from the Chrono GitHub [repository](https://github.com/projectchrono/chrono) as zip files (for the current development branch or one of the official [releases](https://github.com/projectchrono/chrono/releases)). Alternatively, you can use [git](https://git-scm.com/) to clone the Chrono repository.

------------------------------------------------------------
## Prerequisites {#prerequisites}

Building Chrono from sources requires a C++ compiler and the [CMake](https://cmake.org/) build system.

#### Recommended compilers {#compilers}

- Windows: **MSVC** from Visual Studio 2019 or newer.
  ([VS 2022 Community Edition](https://visualstudio.microsoft.com/downloads/) free download)<br>
  Note: the C++ compiler is not installed by default; make sure to install the C++ toolchain during VS setup.
- Linux: **GNU** C++ compiler for Linux-based platforms (version 4.9 or newer)
- Linux: **LLVM Clang** C and C++ compiler (version 1.6 or newer)
- MacOS: **Clang** through Xcode Package.
  ([Xcode](https://apps.apple.com/us/app/xcode/id497799835?mt=12/) free download)

Other compilers were also tested (e.g. Intel C++, PGI) but they are not officially supported and maintained. While it is likely possible to build Chrono with other toolchains, this might require changes to the CMake scripts.

#### Install CMake {#install_cmake}

[CMake](https://cmake.org/) is required to configure the build toolchain before compiling Chrono. 

CMake will configure and create the necessary solution files (Windows) or make files (Linux) necessary to compile and build Chrono from sources.

For Windows users: make sure to put the CMake executable in your `Path` environmental variable (the installer can do this for you).

On Linux, if not already installed, use the package manager to install `cmake`.  Some distributions may need to install additional packages along with `cmake` (e.g., `cmake-gui` for the window-based CMake GUI interface or `cmake-curses-gui`  to use the `ccmake` terminal-based GUI for CMake).

For Xcode users: while the CMake.app bundle also contains command line tools, you must set appropriate links to use it from the terminal. It is better to install a pure command line version via homebrew: <tt>brew install cmake</tt> in the terminal.

#### Install a GUI git client {#install_git}

While `git` can be used through command line we recommend using a [GUI git client](https://git-scm.com/downloads/guis).

On Windows and MacOS, we suggest [SourceTree](http://www.sourcetreeapp.com/), also used for illustration below.

Note that most modern IDEs have git integration (e.g. the free [Visual Studio Code](https://code.visualstudio.com/) available on Windows, Linux, and Mac).

------------------------------------------------------------
## Third-party dependencies {#dependencies}

The Chrono core module has a single dependency on the [Eigen3](http://eigen.tuxfamily.org/) template library for linear algebra. 

Since optional Chrono modules may bring in additional dependencies we recommend to enable only those that are required by your project, at least for the first installation; for their installation please refer to [Installation Guides](@ref install_guides).

#### Install Eigen {#install_eigen}

Eigen is a headers-only library and as such it can be used immmediately as downloaded. However, it can also be configured and installed with `cmake` (see below).

On Linux, Eigen is also available through the system package manager (e.g. <tt>sudo apt install eigen3-dev</tt>).

On the Mac, Eigen can be installed via homebrew: <tt>brew install eigen</tt>. Beginning with MacOS 12 Monterey, homebrew installs in `/opt/homebrew`.

We strongly **recommend** using the latest Eigen3 version 3.4.0.

#### Utility scripts for installing 3rd-party Chrono dependencies {#scripts}

The Chrono distribution includes, in the directory `contrib/build-scripts` a set of scripts that provide a simple mechanism for installing dependencies for the various Chrono modules.  The scripts are organized iun sub-directories by OS: Bash shell scripts in `contrib/build-scripts/linux/` and Windows batch scripts in `contrib/build-scripts/windows/`. The directory `contrib/build-scripts/macOS/` contains Bash shell scripts tailored for the specifics of MacOS; if a script is not present in that directory, simply use the Linux script.

**Notes**<br>
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

**Usage**<br>
Consult the instructions in the comments at the top of each script.
The general usage is as follows:
- copy the desired script(s) in a workspace directory (each script will create temporary directories to download the necessary sources and to configure and build the various packages)
- edit the script to tailor to your particular machine (at a minimum, set the desired installation directory)
- run the script
- verify that the package files were properly installed in the specified install directory

With the exception of OpenCRG, all packages built and installed by these scripts provide CMake configuration scripts which are used during CMake Chrono configuration to find the respective dependencies. To simplify the Chrono CMake configuration process and avoid manually specifying package-specific information every time, we recommend setting the `CMAKE_PREFIX_PATH` environment variable to add the directory containing the CMake project configuration scripts for each installed package. 
An example of `CMAKE_PREFIX_PATH` (on a Windows machine) is:
```cpp
E:\Packages\eigen\share\eigen3\cmake;
E:\Packages\blaze\share\blaze\cmake;
E:\Packages\vsg\lib\cmake;
E:\Packages\urdf\lib\urdfdom\cmake;
E:\Packages\urdf\CMake;
E:\Packages\spectra\share\spectra\cmake;
E:\Packages\gl\lib\cmake;
E:\Packages\mumps\cmake;
C:\OpenCASCADE-7.4.0-vc14-64\opencascade-7.4.0\cmake;
C:\Program Files (x86)\Intel\oneAPI\mkl\2023.0.0\lib\cmake\mkl;
```

#### Notes on 3rd-party Chrono dependencies {#notes}

- Never mix 64-bit and 32-bit binaries and libraries. <br>
For example, you must link the 64-bit Irrlicht library.
- Since many of the 3rd-party dependency packages create and install shared libraries, the following steps may be required:
  - on Linux, run `ldconfig` (you will likely need root permissions) to cache the necessary link to the newly created shared libraries or set the `LD_LIBRARY_PATH` environment variable
  - on Windows, add to the `PATH` environment variable the directories containing the package DLLs (otherwise, these DLLs have to be manually copied next to the binaries so that they can be found at run-time)

------------------------------------------------------------
## Configuring Chrono with CMake {#configure_chrono}

CMake can be used through a GUI interface, `cmake-gui` (note that on Linux, this requires installing a separate CMake package), from a Curses interface (`ccmake`), or else directly from the command prompt (`cmake`). Note that the latter requires passing all necessary CMake variables as arguments to `cmake`, or else write a script (example scripts are provided with the Chrono distribution, under the `contrib/build-scripts/` directory).

Below, we describe in more details configuration of Chrono with `cmake-gui` and `ccmake`.

#### Using CMake through the GUI interface {#configure_chrono_cmake_gui}

Start `cmake-gui` to configure the build.
-  In the field "Where is the source code" set the path to your Chrono directory. <br>
   This is the directory where you created your Git repository, in our example is <tt>C:/workspace/chrono</tt>. 

-  In the field "Where to build the binaries" set the path to a directory **different** from the Chrono source location, as Chrono prohibits "in-source" builds. This build directory can be manually created beforehand; otherwise, CMake will ask to create it. For our example, let's use <tt>C:/workspace/chrono_build</tt>  
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_destinations.png" class="img-responsive">

-  Press the **Configure** button.
  
-  Set the appropriate generator (e.g. Visual Studio, Makefile, etc.) and the appropriate platform (Win32, x64, etc.) 
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_platform.png" class="img-responsive">

-  Specify the location of the Eigen installation.
   If Eigen was itself configured and installed with CMake, set the CMake variable `Eigen3_DIR` to point to the Eigen installation directory that contains a project configuration script (e.g., `C:/Packages/eigen/share/eigen3/cmake/`). Otherwise, set the CMake variable `EIGEN3_INCLUDE_DIR` to point to the directory containing the subdirectory `Eigen` with header files (e.g., `C:/Packages/eigen-3.4.0/`.

-  Enable any additional module. Refer to their [Installation Guides](@ref install_guides) before proceeding any further.

-  Remember that you might need to press **Configure** after you change some setting, even multiple times,
   until all variable dependencies are resolved. 

-  Finally, press **Generate**.

<div class="ce-warning">
If using a multi-config generator (e.g., Visual Studio or ninja multi-config), CMake sets the variable `CMAKE_CONFIGURATION_TYPES` to include all available configurations (Debug, Release, MinSizeRel, RelWithDebInfo). Leave that unchanged.
<br><br>
If using a single-configuration generator (e.g., makefiles or ninja), set `CMAKE_CONFIGURATION_TYPES` to the desired build type (e.g., Release or Debug). Do not leave it blank (default) as this will result in a build with **no** optimization and **no** debug information.
<br><br>
CMake uses the slash `/` character for paths. Unix users are already used to this convention.<br>
Windows users should take care to convert the default separator `\` to `/`!
</div>

At this point you just created a project that will be later used to build Chrono. You can close CMake.

#### Using CMake through the Curses interface {#configure_chrono_ccmake}

- Create a build directory **different** from the Chrono source directory (as Chrono prohibits "in-source" builds) and change your current directory to it.
  
- Run `ccmake <path-to-chrono-sources>` from the build directory. A Curses-based GUI will appear in the terminal.

<img src="http://www.projectchrono.org/assets/manual/install_ccmake_1.png" class="img-responsive" width="600">

- Enter `c` to **Configure** and continue. The interface will reload to a new screen with more options.
  
<img src="http://www.projectchrono.org/assets/manual/install_ccmake_2.png" class="img-responsive" width="600">

- Specify the build type (unless using a multi-config generator, see above).

- Specify the location of the Eigen installation (see above).

<img src="http://www.projectchrono.org/assets/manual/install_ccmake_3.png" class="img-responsive" width="600">

- Enter `c` to **Configure** and continue until you reach the final screen. At which point enter `g` to **Generate**, CCMake will close on completion. 

<img src="http://www.projectchrono.org/assets/manual/install_ccmake_4.png" class="img-responsive" width="600">

Build files are now available in the build directory (in this example, `Makefile`).

------------------------------------------------------------
## Building Chrono {#build_chrono}

#### Visual Studio {#build_chrono_windows}

1. Double-click the `Chrono.sln` file in the build directory to open the Visual Studio solution file. Alternatively, if using `cmake-gui`, click the _Open Project_ button.

2. In the VS toolbar, from the _Solution Configurations_ drop-down menu select the desired build mode (e.g., **Release*).  
  <img src="http://www.projectchrono.org/assets/manual/Install_vs_buildtype.png" class="img-responsive">

3. In the toolbar, click on **Build > Build solution** or **Build > Build ALL_BUILD**.
   This will build all Chrono projects (one per Chrono module), as well as demos and tests (if enabled during configuration).
   By default, shared libraries (DLLs) are generated for the Chrono modules, unless `BUILD_SHARED` was set to `off` during configuration. All DLLs and executables will be placed in a directory `bin\Release` of the build tree.

4. Optionally, repeat step 2 and 3, chosing **Debug** as configuration type, to build debug binaries which include debugging symbols. All DLLs and executables will be placed in a directory `bin\Debug` of the build tree.

#### Linux/make {#build_chrono_linux}

Depending on the generator used during CMake configuration, invoke the appropriate build command. For example:

- `make -j 10`<br>
  to build with Make using 10 parallel build threads.
- `ninja -j 10`<br>
  to build with ninja using 10 parallel build threads.
- `ninja -f build.Release.ninja -j 10`<br>
  for a Release build with `ninja` multi-config using 10 parallel build threads.


#### MacOS/clang {#build_chrono_mac}

- CMake generates a hierarchy of makefiles in the directory specified in "Where to build the binaries".

- To build the Chrono libraries and demo executables, simply invoke <tt>make</tt> from the command line in that directory.

- Optionally, type <tt>make install</tt> to install the Chrono libraries, data files, and demo executables in the directory specified during CMake configuration.

- CMake can be configured to generate Xcode (<tt>cmake -G Xcode ....</tt>) configurations. You would normally use it with the Xcode IDE. The advantage is the possibilty to debug the code. Like in MS Visual Studio, you choose the build type from the IDE.

<div class="ce-warning"> 
**MacOS issues:** clang++ does not come with OpenMP support out of the box.
You will not be able to build <tt>libChrono_multicore</tt> successfully.<br> 
However, OpenMP support can be added using homebrew: <tt>brew install libomp</tt>. 
Having done so, you can then configure Chrono with OpenMP support. For this, you must define the right compiler flags:
<tt>-Xpreprocessor -fopenmp</tt> for the compiler and <tt>-lomp</tt> for the linker. Please give the OpenMP options for both, the C compiler
and the C++ compiler, otherwise the OpenMP configuration will fail.
</div> 

------------------------------------------------------------
## Testing Chrono build {#test_chrono}

The Chrono distribution includes a large number of demos, unit tests, and benchmark test. These are included in the build if the corresponding CMake variables are set to `on` during CMake configuration: `BUILD_DEMOS`, `BUILD_TESTING`, and `BUILD_BENCHAMRKING`, respectively. By default, only generation of demo executabnles is enabled.

Each Chrono module adds its own set of demos and tests and these are built only if the corresponding Chrono module is enabled. Note that some demo programs depend on more than one Chrono module being available. For example, most MBD and FEA demos require a run-time visualization module (VSG or Irrlicht). Similarly, Chrono::Vehicle demos require a run-time visualization module with some other vehicle demos also requiring additional modules (e.g., Chrono::FSI, Chrono::Multicore, etc.).

Executables are available under a subdirectory `bin/<config>/` (e.g., `bin/Release/` or `bin/Debug/`) for a multi-config generator or directly under `bin/` otherwise. 

Unit tests do not use run-time visualization and are based on googletest. Running any of the Chrono unit tests will generate a standard report, indicating whether the test succeeded or failed. You can run all unit tests at once using `ctest`. For example, o run all unit tests for a Release build (mult-config generator), execute the following command from the top-level build directory:<br>
`ctest -C Release`

-----------------------------------------------------------
## Installing Chrono {#install_chrono}

Upon a successful build, Chrono can be installed (to the directory specified during CMake configuration through the `CMAKE_INSTALL_PREFIX`). This will copy Chrono libraries, headers, data files, and demo executable to the install directory.

In Visual Studio, the install process is done by building the "INSTALL" project of the Chrono.sln solution.

If using make or ninja, installation is performed with:
- `make install`
- `ninja install`
