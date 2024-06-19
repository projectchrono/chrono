Install Chrono {#tutorial_install_chrono}
==========================

## Prerequisites

In order to download, configure, and build the Chrono libraries from source, the following packages and libraries are required:
- **git**: to clone the GitHub repository; we recommend using [Sourcetree](https://www.sourcetreeapp.com/) for Win and Mac users that comes with Git already embedded;
- **CMake**: utility that generates all the required solution/make files for your specific toolchain;
- a **C++ compiler**: see requirements below
- the **Eigen** library: a linear algebra library

These are the minimal requirements, sufficient for building the core Chrono module. However, we strongly recommend to enable at least the [IRRLICHT module](@ref irrlicht_visualization) to enable the visualization of your models. This will ask for an additional dependency:
- the [Irrlicht](https://irrlicht.sourceforge.io/?page_id=10) library  
We will suggest to refer to [IRRLICHT install guide](@ref module_irrlicht_installation) for in-detailed instructions, but we will provide some basic instructions also here.

Since additional modules may bring in additional dependencies we recommend to enable only those that are required by your project, at least for the first installation; for their installation please refer to [Installation Guides](@ref install_guides).


## 1) Check/Install a C++ compiler

Recommended compilers:
- **Microsoft Visual C++** from Visual Studio 2019 or newer. The *community edition* of the latest [Visual Studio](https://visualstudio.microsoft.com/downloads/) is available for free. 
- **GNU** C++ compiler for Linux-based platforms (version 4.9 or newer).<br>
Install the latest version of GCC through your package manger: `gcc`.
- **LLVM Clang** C and C++ compiler (version 1.6 or newer).<br>
Install the latest version of Clang through your package manger: `clang`.
- **Xcode Package** for MacOS: Download via App Store for free - it contains the clang++ compiler.

Other compilers were also tested (e.g. Intel C++, PGI) but they are not officially supported and maintained.
While it is likely possible to build Chrono with other toolchains, this might require changes to the CMake scripts.

<div class="ce-warning"> 
The Microsoft Visual C++ compiler is included in the Visual Studio package, but it is **not** installed by default.<br>
Make sure to install the C++ toolchain during the setup!<br>
<br>
Visual Studio 2017 has problems with the heavy use of inlining in recent version of Eigen. 
For the latest version of Chrono (specifically due to the reimplemented ANCF elements), this can result in very long compilation times or even hang ups.
We recommend using VS 2019 or newer.
</div>

## 2) Download and install the Eigen library

Chrono now uses [Eigen3](http://eigen.tuxfamily.org/) for all of its internal dense linear algebra needs. Chrono requires Eigen version 3.3.0 or newer, but we strongly encourage using the **latest stable release, Eigen 3.4.0**. 

On Linux, Eigen is available through your system package manager as: `eigen`, `eigen3-dev`, `eigen3-devel`.

On the Mac you should install it via homebrew: <tt>brew install eigen</tt>. Homebrew installs into /opt/homebrew since MacOS 12 Monterey and the new Apple Silicon (arm46, M1, M2...) hardware. If Eigen is not found automatically, you can search its folder with:<br>
<tt>find /opt/homebrew -name Eigen</tt><br>
The response is actually:<br>
<tt>/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/unsupported/Eigen</tt><br>
<tt>/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/Eigen</tt><br>
<tt>/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/Eigen/Eigen</tt><br>
The include path is then **/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3**.

<div class="ce-warning"> 
Chrono has been tested most extensively with Eigen 3.3.9 and Eigen 3.4.0.<br>
Most Chrono modules will build and work with Eigen 3.3.0 or newer.<br>
However, if you are building the Chrono::FSI or Chrono::Granular modules, note that CUDA 9.1 removed a header file (math_functions.hpp) which was referenced in older versions of Eigen; this issue was addressed as of Eigen 3.3.6. 
</div>

## 3) Install CMake

[CMake](https://cmake.org/) is required to configure the build toolchain before compiling Chrono. 

CMake will configure and create the necessary solution files (Windows) or make files (Linux) necessary to compile and build Chrono from sources.

For Windows users: make sure to put the CMake executable in your `Path` environmental variable (the installer can do this for you). <br>

On Linux, if not already installed, use the package manager to install `cmake`.  Some distributions may need to install the package `cmake-curses-gui` along with `cmake` to use terminal based GUI for CMake.

For Xcode users: the CMake.app bundle also contains command line tools, you must set appropriate links to use it from the terminal. It is better to install a pure command line version via homebrew (https://brew.sh). After installing the home brew package manager type: <tt>brew install cmake</tt> in the terminal.

## 4) Install a GIT client

While **git** can be used through command line we recommend using a git client. We suggest:

On Windows and MacOS, we suggest [SourceTree](http://www.sourcetreeapp.com/).
On MacOS you will find an Application Bundle under /Applications.<br>

Various git clients are available for Linux as well. Consult the documentation for your distribution.

## 5) Download the project by cloning the Git repository

Download the Chrono source by performing a **clone** of the Git repository in a directory on your machine. 

Assuming you are using [SourceTree](http://www.sourcetreeapp.com/):

-  in the menu bar press **File** and then **Clone / New** button in SourceTree  
   <img src="http://www.projectchrono.org/assets/manual/Install_ST1.gif" class="img-responsive">

-  enter <tt>https://github.com/projectchrono/chrono.git</tt> in the field "Source Path / URL"
  
-  in the field "Destination Path" enter a path to an empty directory, say <tt>C:/workspace/chrono</tt>  
   <img src="http://www.projectchrono.org/assets/manual/Install_ST2.png" class="img-responsive">

-  leave the **Local Folder** field as it is
  
-  under *Advanced Options* set *Checkout branch* to `main`
  
-  press **Clone** and the source code will be downloaded into the folder you specified

If using `git` from the command line, `git clone -b main git@github.com:projectchrono/chrono.git` will create a copy of the Github repository in the current directory and checkout the `main` (development) branch.

<div class="ce-info">
The `release` branches contain the various Chrono releases and contain the most stable code. If you are interested in using the latest features as they are developed and before the next official release, you can checkout the `main` branch at any time after the initial cloning: <code>git checkout main</code>.
</div>

## 6) Run CMake

The average user is suggested to use the CMake GUI interface. For those that have no graphical interface available on their systems, please refer to the command line instructions below.

#### Using CMake through the GUI interface

Start `cmake-gui` to configure the build.
-  In the field "Where is the source code" set the path to your Chrono directory. <br>
   This is the directory where you created your Git repository, in our example is <tt>C:/workspace/chrono</tt>. 
-  In the field "Where to build the binaries" set the path to *another* directory on your system, 
   that must be empty. This is where the Visual C++ project will be created. <br>
   For our example, let's use <tt>C:/workspace/chrono_build</tt>  
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_destinations.png" class="img-responsive">

-  Press the **Configure** button.
  
-  Set the appropriate generator (e.g. Visual Studio, Makefile, etc...) and the appropriate platform (Win32, x64, etc...) i.e. if you have a 64-bit processor, as usually is, you should configure a 64-bit project.<br>
   In older CMake, there is a single list of generators in which you may choose between  
   e.g. 'Visual Studio 15 2017' and 'Visual Studio 15 2017 **Win64**' (then choose the latter).<br>
   In the latest CMake, there are separated fields, one for the generator (e.g. 'Visual Studio 15 2017') and another one for the platform (e.g. x64).  
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_platform.png" class="img-responsive">

-  Specify the location of the Eigen installation.
   If this is not detected automatically, you may need to manually set the CMake variable `EIGEN3_INCLUDE_DIR`.<br>
   For example, <tt>C:/workspace/libraries/eigen-3.3.7</tt>.<br>

-  Enable any additional module. Refer to their [Installation Guides](@ref install_guides) before proceeding any further.  
   If you decided to install the Irrlicht module:
   - Tick `ENABLE_MODULE_IRRLICHT`.<br>
     Other modules might require additional settings and dependencies.
   - Press **Configure**.
   - Set the `IRRLICHT_INSTALL_DIR` variable: it must contain the path to the directory where you unzipped Irrlicht.<br>
     In our example, browse to <tt>C:/workspace/libraries/irrlicht-1.8.4</tt>
   - Press **Configure** again.
   - The `IRRLICHT_LIBRARY` variable should be filled automatically.
     If not, select (for Win users): <tt>C:/workspace/libraries/irrlicht-1.8.4/lib/Win64-visualStudio/Irrlicht.lib</tt>.<br>
	 Then press **Configure** again.  
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_parameters.png" class="img-responsive">

-  Remember that you might need to press **Configure** after you change some setting, even multiple times,
   until all the variables get a white background. 

-  Finally, press **Generate**.

<div class="ce-warning"> 
Visual Studio users should leave the `CMAKE_CONFIGURATION_TYPES` variable untouched (it should report *Debug;Release;MinSizeRel;RelWithDebInfo*; if not, please do *File*>*Delete Cache* and start again the CMake configuration).<br>
The build configuration will be chosen directly from Visual Studio.
For Makefile-based solutions, on the contrary, you should set `CMAKE_CONFIGURATION_TYPES` to either *Debug* or *Release* (or *MinSizeRel* or *RelWithDebInfo*). Makefile does not support multiple configuration types.
</div>

<div class="ce-warning"> 
CMake uses the slash `/` character for paths. Unix users are already used to this convention.<br>
Windows users should take care to convert the default separator `\` to `/`!
</div>

At this point you just created a project that will be later used to build Chrono. You can close CMake.

#### Using CMake through the command prompt


- Create a build directory **outside** of the Chrono source directory (`mkdir chrono_build`) and change your current directory to it: `cd chrono_build`.
  
- Run `ccmake /path/to/chrono` from the build directory. A text based GUI will appear in the terminal.

- Enter `c` to **Configure** and continue. The interface will reload to a new screen with more options.
  
- Specify the location of the Eigen installation.
   If this is not detected automatically, you may need to manually set the CMake variable `EIGEN3_INCLUDE_DIR`.<br>

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_1.png" class="img-responsive" width="400">

<div class="ce-warning">
The Eigen directory field should be correctly filled in already, assuming the Eigen library was installed through a package manager. If the library was built from the source verify the path to the library is correct before continuing.
</div>

   - The following instructions are optional but highly **recommended**.

      - Enter `c` to **Configure** and continue to the next screen.

      - Enable the recommended optional modules: at least tick `ENABLE_MODULE_IRRLICHT`.<br>
     Other modules might require additional settings and dependencies.

      -  Enter `c` to **Configure** and continue to the next screen.
     
      - Verify the path to the Irrlicht include directory and the Irrlicht library are automatically filled in the `IRRLICHT_INCLUDE_DIR` and `IRRLICHT_LIBRARY` fields respectively. If this is not the case, update to match the example below.

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_2.png" class="img-responsive" width="400">
 
- Enter `c` to **Configure** and continue until you reach the final screen. At which point enter `g` to **Generate**, CCMake will close on completion.

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_3.png" class="img-responsive" width="400">


## 7) Compile the project

#### Visual Studio

1. Go to the directory that you set in "Where to build the binaries". You will find the file **Chrono.sln**.

2. **Double-click** on that file: your Visual Studio solution will open.

3. In the toolbar, from the _Solution Configurations_ drop-down menu **choose 'Release' mode**  
  <img src="http://www.projectchrono.org/assets/manual/Install_vs_buildtype.png" class="img-responsive">

4. In the toolbar, click on **Build** > **Build solution...** .  
  The entire Chrono project and its demos will be compiled, creating many .exe and .dll files, in the bin/Release subdirectory.   
  This will take a few minutes.

5. Repeat step 3 and 4, but choosing **Debug** as configuration type. This will generate the binaries with debugging symbols: they will be placed under the bin/Debug subfolder.

#### Linux/make

Run command `make`, optionally followed by `make install` while in the same build directory as the newly created Makefile. 

<div class="ce-info">
<code>make install</code> will copy the Chrono libraries, data files, and demo executable to the install directory specified during CMake configuration.
</div>

#### MacOS/clang

- CMake generates a hierarchy of makefiles in the directory specified in "Where to build the binaries".

- To build the Chrono libraries and demo executables, simply invoke <tt>make</tt> from the command line in that directory.

- Optionally, type <tt>make install</tt> to install the Chrono libraries, data files, and demo executables in the directory specified during CMake configuration.

- CMake can be configured to generate Xcode (<tt>cmake -G Xcode ....</tt>) configurations. You would normally use it with the Xcode IDE. The advantage is the possibilty to debug the code. Like in MS Visual Studio, you choose the build type from the IDE.

<div class="ce-warning"> 
**MacOS issues:** clang++ does not come with OpenMP support out of the box.
You will not be able to build <tt>libChronoEngine_multicore</tt> successfully.<br> 
However, OpenMP support can be added using homebrew: <tt>brew install libomp</tt>. 
Having done so, you can then configure Chrono with OpenMP support. For this, you must define the right compiler flags:<br>
<tt>-Xpreprocessor -fopenmp</tt> for the compiler and <tt>-lomp</tt> for the linker. Please give the OpenMP options for both, the C compiler
and the C++ compiler, otherwise the OpenMP configuration will fail.
</div> 


## 8) Test the demos

For Windows users, executables for the various demos distributed with Chrono will be available in the `bin/Release` or `bin/Debug` subdirectory, depending on the configuration selected to build the Chrono libraries and executables.

For Linux users, these will be available in the subdirectory `bin`.

## Notes

- IMPORTANT: never mix 64-bit and 32-bit binaries and libraries! <br>
For example, you must link the 64-bit Irrlicht library. Beginning with MacOS 10.15 Catalina there is no 32-bit support anymore.

- Currently, the only dependency of the core Chrono module is the Eigen library. Since Eigen is a headers-only library, no additional steps must be taken to be able to run demos and programs that only link to the core Chrono library. <br>
Other Chrono module (e.g., the run-time visualization Chrono modules) have additional dependencies. If using shared libraries for these dependencies, you must ensure that these are found at run time.
  - On Windows, you can either copy the corresponding DLLs (e.g., Irrlicht.dll) to the same directory as the executables or else add the path to these shared libraries to the `PATH` environment variable.
  - On Linux, you may need to append to the `LD_LIBRARY_PATH` environment variable.

