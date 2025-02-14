Install Chrono {#tutorial_install_chrono}
==========================

\tableofcontents

The Chrono source code can be obtained from the Chrono GitHub [repository](https://github.com/projectchrono/chrono) as zip files (for the current development branch or one of the official [releases](https://github.com/projectchrono/chrono/releases)). Alternatively, you can use [git](https://git-scm.com/) to clone the Chrono repository.

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

On Linux, if not already installed, use the package manager to install `cmake`.  Some distributions may need to install the package `cmake-curses-gui` along with `cmake` to use terminal based GUI for CMake.

For Xcode users: the CMake.app bundle also contains command line tools, you must set appropriate links to use it from the terminal. It is better to install a pure command line version via homebrew: <tt>brew install cmake</tt> in the terminal.

#### Install a GUI git client {#install_git}

While `git` can be used through command line we recommend using a [GUI git client](https://git-scm.com/downloads/guis).

On Windows and MacOS, we suggest [SourceTree](http://www.sourcetreeapp.com/), also used for illustration below.

Note that most modern IDEs have git integration (e.g. the free [Visual Studio Code](https://code.visualstudio.com/) available on Windows, Linux, and Mac).

------------------------------------------------------------
## Prerequisites {#prerequisites}

The Chrono core module has a single dependency on the [Eigen3](http://eigen.tuxfamily.org/) template library for linear algebra. 

Since optional Chrono modules may bring in additional dependencies we recommend to enable only those that are required by your project, at least for the first installation; for their installation please refer to [Installation Guides](@ref install_guides).

#### Install Eigen {#install_eigen}

Eigen is a headers-only library and as such it can be used immmediately as downloaded. However, it can also be configured and installed with `cmake`.

On Linux, Eigen is also available through the system package manager (e.g. <tt>sudo apt install eigen3-dev</tt>).

On the Mac, Eigen can be installed via homebrew: <tt>brew install eigen</tt>. Beginning with MacOS 12 Monterey, homebrew installs in `/opt/homebrew`.

We strongly **recommend** using the latest Eigen3 version 3.4.0.


#### Notes on 3rd-party Chrono dependencies {#dependencies}

- IMPORTANT: never mix 64-bit and 32-bit binaries and libraries! <br>
For example, you must link the 64-bit Irrlicht library. Beginning with MacOS 10.15 Catalina there is no 32-bit support anymore.

- Currently, the only dependency of the core Chrono module is the Eigen library. Since Eigen is a headers-only library, no additional steps must be taken to be able to run demos and programs that only link to the core Chrono library. <br>
Other Chrono module (e.g., the run-time visualization Chrono modules) have additional dependencies. If using shared libraries for these dependencies, you must ensure that these are found at run time.
  - On Windows, you can either copy the corresponding DLLs (e.g., Irrlicht.dll) to the same directory as the executables or else add the path to these shared libraries to the `PATH` environment variable.
  - On Linux, you may need to append to the `LD_LIBRARY_PATH` environment variable.


------------------------------------------------------------
## Configuring Chrono with CMake {#configure_chrono}

The average user is suggested to use the CMake GUI interface. For those that have no graphical interface available on their systems, please refer to the command line instructions below.

#### Using CMake through the GUI interface {#configure_chrono_cmake_gui}

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

#### Using CMake through the command prompt {#configure_chrono_cmake}

- Create a build directory **outside** of the Chrono source directory (`mkdir chrono_build`) and change your current directory to it: `cd chrono_build`.
  
- Run `ccmake /path/to/chrono` from the build directory. A text based GUI will appear in the terminal.

- Enter `c` to **Configure** and continue. The interface will reload to a new screen with more options.
  
- Specify the location of the Eigen installation.
   If this is not detected automatically, you may need to manually set the CMake variable `EIGEN3_INCLUDE_DIR`.<br>

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_1.png" class="img-responsive" width="400">

<div class="ce-warning">
The Eigen directory field should be correctly filled in already, assuming the Eigen library was installed through a package manager. If the library was built from the source verify the path to the library is correct before continuing.
</div>

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_2.png" class="img-responsive" width="400">
 
- Enter `c` to **Configure** and continue until you reach the final screen. At which point enter `g` to **Generate**, CCMake will close on completion.

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_3.png" class="img-responsive" width="400">

------------------------------------------------------------
## Building Chrono {#build_chrono}

#### Visual Studio {#build_chrono_windows}

1. Go to the directory that you set in "Where to build the binaries". You will find the file **Chrono.sln**.

2. **Double-click** on that file: your Visual Studio solution will open.

3. In the toolbar, from the _Solution Configurations_ drop-down menu **choose 'Release' mode**  
  <img src="http://www.projectchrono.org/assets/manual/Install_vs_buildtype.png" class="img-responsive">

4. In the toolbar, click on **Build** > **Build solution...** .  
  The entire Chrono project and its demos will be compiled, creating many .exe and .dll files, in the bin/Release subdirectory.   
  This will take a few minutes.

5. Repeat step 3 and 4, but choosing **Debug** as configuration type. This will generate the binaries with debugging symbols: they will be placed under the bin/Debug subfolder.

#### Linux/make {#build_chrono_linux}

Run command `make`, optionally followed by `make install` while in the same build directory as the newly created Makefile. 

<div class="ce-info">
<code>make install</code> will copy the Chrono libraries, data files, and demo executable to the install directory specified during CMake configuration.
</div>

#### MacOS/clang {#build_chrono_mac}

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

------------------------------------------------------------
## Testing Chrono build {#test_chrono}

For Windows users, executables for the various demos distributed with Chrono will be available in the `bin/Release` or `bin/Debug` subdirectory, depending on the configuration selected to build the Chrono libraries and executables.

For Linux users, these will be available in the subdirectory `bin`.

------------------------------------------------------------
## Installing Chrono {#install_chrono}

