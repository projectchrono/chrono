Install Chrono {#tutorial_install_chrono_linux}
==========================
<div class="ce-info">
The instruction process detailed here are universal to most major Linux distributions.<br>
Windows and Mac instructions may be read [here](@ref tutorial_install_chrono).
</div>

## 1) Check/Install a C++ compiler

Recommended compilers:
- GNU C++ compiler for Linux-based platforms (version 4.9 or newer).<br>
Install the latest version of GCC through your package manger: `gcc`.
- LLVM Clang C and C++ compiler (version 1.6 or newer).<br>
Install the latest version of Clang through your package manger: `clang`.

Other compilers were also tested (e.g. Intel C++, PGI) but they are not officially supported and maintained.
While it is likely possible to build Chrono with other tool chains, this might require changes to the CMake scripts.

## 2) Download and install the Eigen library

Chrono now uses [Eigen3](http://eigen.tuxfamily.org/) for all of its internal dense linear algebra needs. Chrono requires Eigen version 3.3.0 or newer, but we strongly encourage using the **latest stable release, Eigen 3.4.0**. Eigen is available through your system package manager as: `eigen`, `eigen3-dev`, `eigen3-devel`. 

<div class="ce-warning"> 
Chrono has been tested most extensively with Eigen 3.3.4 and Eigen 3.3.7.<br>
Most Chrono modules will build and work with Eigen 3.3.0 or newer.<br>
However, if you are building the Chrono::FSI or Chrono::Granular modules, note that CUDA 9.1 removed a header file (math_functions.hpp) which was referenced in older versions of Eigen; this issue was addressed as of Eigen 3.3.6. 
</div>

## 3) Download the Irrlicht library

While Chrono::Irrlicht is an optional module and not required to begin modeling with Chrono, it is suggested you enable this module to get access to many Chrono demos which rely on Irrlicht for their run-time visualization.

- **download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html)<br>
May also be installed through a package manager: `irrlicht`, `libirrlicht-dev`, `irrlicht-devel`.

<div class="ce-info">
If you encounter any problem with version 1.8.4 please try to roll back to [release 1.8.2](http://irrlicht.sourceforge.net/downloads.html).<br>
Previous versions must be built from source.<br>
Release v.1.8.3 does not contain the precompiled 64bit DLL.<br>
Release v.1.8.0 has issues with soft shadows.<br>
</div>

## 4) Download and Install CMake

[CMake](https://cmake.org/) is required to configure the build before compiling Chrono. It is also widely used among the Linux community to build many other software units. If it isn't already installed, use the a package manager to install it: `cmake`.

<div class="ce-warning">
Debian distributions may need to install the package `cmake-curses-gui` along with `cmake` to use terminal based GUI for CMake.
</div>

## 5) Download the project by cloning the Git repository

Download the Chrono SDK by performing a **clone** of the Git repository on your machine. `git clone -b master git@github.com:projectchrono/chrono.git` will create a copy of the Github repository in the current directory.

<div class="ce-info">
The `release` branches contain the various Chrono releases and contains the most stable code. If you are interested in using the latest features as they are developed and before the next official release, you can checkout the `main` branch at any time after the initial cloning: <code>git checkout main</code>.
</div>

## 6) Run CMake

Create a new directory **outside** of the Chrono SDK source directory. This will be used to build Chrono: `mkdir build_chrono`. Next, change your current directory to it: `cd build_chrono`.
-  We recommend using the terminal based GUI for CMake to configure the Chrono build, however the following process may be replicated through a single command line.

-  Run `ccmake /path/to/chrono` from the build directory. A text based GUI will appear in the terminal.

-  Enter `c` to **Configure** and continue. The interface will reload to a new screen with more options.
  
-  Specify the location of the Eigen installation.
   If this is not detected automatically, you may need to manually set the CMake variable `EIGEN3_INCLUDE_DIR`.<br>

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_1.png" class="img-responsive" width="400">

<div class="ce-warning">
The Eigen directory field should be correctly filled in already, assuming the Eigen library was installed through a package manager. If the library was built from the source verify the path to the library is correct before continuing.
</div>

   - The following instructions are optional but highly **recommended**.

      - Enter `c` to **Configure** and continue to the next screen.

      - Enable the recommended optional modules: at least tick `ENABLE_MODULE_IRRLICHT` and `ENABLE_MODULE_POSTPROCESS`.<br>
     Other modules might require additional settings and dependencies.
      -  Enter `c` to **Configure** and continue to the next screen.
      - Verify the path to the Irrlicht include directory and the Irrlicht library are automatically filled in the `IRRLICHT_INCLUDE_DIR` and `IRRLICHT_LIBRARY` fields respectively. If this is not the case, update to match the example below.

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_2.png" class="img-responsive" width="400">

<div class="ce-warning">
Similar to the Eigen directory field set earlier, we assume the Irrlicht library was installed through a package manager. If the library was built from, source verify the location of the library is correct before continuing.
</div>
 
- Enter `c` to **Configure** and continue until you reach the final screen. At which point enter `g` to **Generate**, CCMake will close on completion.

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_3.png" class="img-responsive" width="400">

## 7) Compile the project

Run command `make` followed by `make install` while in the same build directory as the newly created Makefile. Be prepared to wait 15 - 25 minutes for make to compile. This is the final step of the Chrono installation process. Congratulations!

<div class="ce-info">
<code>make install</code> will copy the Chrono libraries, data files, and demo executable to the install directory specified during CMake configuration.
</div>

## 8) Test the demos

Navigate to the directory that you used to build Chrono earlier. Change the current directory to the subdirectory, `bin`. Demo example files are stored here, they are great resource to **demo**nstrate the capacities of Project Chrono.

<div class="ce-danger">
IMPORTANT: never mix 64-bit and 32-bit binaries and libraries! 
For example, you must link the 64-bit Irrlicht library. Beginning with MacOS 10.15 Catalina there is no 32-bit support anymore.
</div>
