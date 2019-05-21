Install Chrono {#tutorial_install_chrono}
==========================


A summary of the process required to **build** Chrono is provided in the picture below. 

<img src="http://www.projectchrono.org/assets/manual/Pic_build_ce.png" class="img-responsive">
<br>

## 1) Check/Install a C++ compiler

Recommended compilers:
- Microsoft Visual C++: Visual Studio 2015 or newer. The [community edition of the latest Visual Studio](https://visualstudio.microsoft.com/downloads/) is available for free. 
- GNU C++ compiler for Linux-based platforms (version 4.9 or newer).
- Xcode Package for MacOS: Download via App Store for free - it contains the clang++ compiler.

Other compilers were also tested (e.g. Intel C++, PGI) but they are not officially supported and maintained.
While it is likely possible to build Chrono with other toolchains, this might require changes to the CMake scripts.

<div class="ce-warning"> 
The Microsoft Visual C++ compiler is included in the Visual Studio package, but it is **not** installed by default.<br>
So, make sure to install the C++ toolchain during the setup!<br>
Moreover, since Visual Studio 2017 make sure to also select, under the *Single Components* tab, the `Windows Universal CRT SDK` and the `Windows 8.1 SDK`.
</div> 

<div class="ce-danger">
The initial release of Visual Studio 2015 gives an 
error compiling Chrono. [Upgrade](https://www.visualstudio.com/en-us/news/vs2015-update2-vs.aspx) to 
**update 2** to fix this problem.
</div>


## 2) Install [CMake](http://www.cmake.org/cmake/resources/software.html)

The free CMake utility is used to manage the building process. It creates a project/solution for your specific compiler/IDE that will allow the user to compile the Chrono source code with minimal effort.<br>
For Visual Studio users: make sure to put the CMake executable in your *Path* environmental variable (the installer can do this for you). <br>
For Xcode users: the CMake.app bundle also contains command line tools, you must set appropriate links to use it from the terminal. It is
better to install a pure command line version via homebrew (https://brew.sh). After installing the home brew package manager type: <tt>brew install cmake</tt> in the terminal.


## 3) Install a GIT client

On Windows and MacOS, you might want to **download and install** [SourceTree](http://www.sourcetreeapp.com/). <br>
On MacOS you will find an Application Bundle under /Applications<br>
On Linux, there are several good [options](https://git-scm.com/download/gui/linux).


## 4) Download the project by cloning the Git repository

Download the Chrono SDK by performing a **clone** of the Git repository in a directory on your machine. 
Assuming you are using [SourceTree](http://www.sourcetreeapp.com/):

-  in the menu bar press **File** and then **Clone / New** button in SourceTree  
   <img src="http://www.projectchrono.org/assets/manual/Install_ST1.gif" class="img-responsive">

-  enter <tt>https://github.com/projectchrono/chrono.git</tt> in the field "Source Path / URL"
  
-  in the field "Destination Path" enter a path to an empty directory, say <tt>C:/workspace/chrono</tt>  
   <img src="http://www.projectchrono.org/assets/manual/Install_ST2.png" class="img-responsive">

-  leave the **Local Folder** field as it is
  
-  under *Advanced Options* set *Checkout branch* to `master`
  
-  press **Clone** and the source code will be downloaded into the folder you specified

<div class="ce-info">
The `master` branch contains the various Chrono releases and contains the most stable code. If you are interested in using the latest features as they are developed and before the next official release, you can checkout the `develop` branch at any time after the initial cloning. <br>
</div>

## 5) Download the Irrlicht library

While Chrono::Irrlicht is an optional module and not required to begin modeling with Chrono, it is suggested you enable this module to get access to many Chrono demos which rely on Irrlicht for their run-time visualization.

- **download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html); the newest tested version is [1.8.4](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.4.zip)
- **unzip** it in a directory of your choice. For example, here we suppose that you unzipped it in <tt>C:/workspace/libraries/irrlicht-1.8.4</tt>.

<div class="ce-info">
Release v.1.8.4 should work perfectly. MacOS users should use this, since previous releases showed some issues.<br>
If you encounter any problem with version 1.8.4 please try to roll back to [release 1.8.2](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.2.zip).<br>
Release v.1.8.3 does not contain the precompiled 64bit DLL.<br>
Release v.1.8.0 has issues with soft shadows.<br>
</div>

<div class="ce-warning"> 
**MacOS issues:** irrlicht-1.8.4 is fairly outdated compared to XCode 9.3.1.
<br>
Before any building, you must correct a bug in the file `irrlicht-1.8.4/source/Irrlicht/MacOSX/CIrrDeviceMacOSX.mm`. 
Open it with CotEdit or BBEdit. Search for the string `NSFileManagerDelegate` and replace it by `NSApplicationDelegate`.
In the terminal go to the directory containing the `MacOSX.xcodeproj` bundle:

    % cd irrlicht-1.8.4/source/Irrlicht/MacOSX
To build the library, type:

    % xcodebuild
The `libIrrlicht.a` library should be found in `irrlicht-1.8.4/source/Irrlicht/MacOSX/build/Release`.
It can be used from here, but it is better to copy it to `irrlicht-1.8.4/lib/MacOS`. After copying type:
    
    % cd irrlicht-1.8.4/lib/MacOSX
    % ranlib libIrrlicht.a
If you use Mac OS >= 10.13 (High Sierra) and Xcode 10, please apply the patch from the contribution directory before building.
Unlike the Windows version, we get a static library that will be part of <tt>libChrono_irrlicht.dylib</tt>, so we don't have to copy it around 
anymore after building chrono.
<br>
For chrono_opengl users:
Beginning with Mac OS 10.14 GLFW 3.2.1 doesn't work any more, use the latest version from GitHub.
</div> 


## 6) Run CMake

Start CMake to configure the build. We assume that you are using the graphical interface.
-  In the field "Where is the source code" set the path to your Chrono directory. <br>
   This is the directory where you created your Git repository, in our example is <tt>C:/workspace/chrono</tt>. 
-  In the field "Where to build the binaries" set the path to *another* directory on your system, 
   that must be empty. This is where the Visual C++ project will be created (or the Makefile, if on Linux). <br>
   For our example, let's use <tt>C:/workspace/chrono_build</tt>  
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_destinations.png" class="img-responsive">

-  Press the **Configure** button.
  
-  Set the appropriate generator (e.g. Visual Studio, Makefile, etc...) and the appropriate platform (Win32, x64, etc...) i.e. if you have a 64-bit processor, as usually is, you should configure a 64-bit project.<br>
   In older CMake, there is a single list of generators in which you may choose between  
   e.g. 'Visual Studio 15 2017' and 'Visual Studio 15 2017 **Win64**' (then choose the latter).<br>
   In the latest CMake, there are separated fields, one for the generator (e.g. 'Visual Studio 15 2017') and another one for the platform (e.g. x64).  
   <img src="http://www.projectchrono.org/assets/manual/Install_cmake_platform.png" class="img-responsive">

-  Change the settings in the user interface of CMake.
   Some of these settings are automatically detected, but some other must be changed. 
   - Enable the required modules: at least tick `ENABLE_MODULE_IRRLICHT` and `ENABLE_MODULE_POSTPROCESS`.<br>
     Other modules might require additional settings and dependencies.
   - Press **Configure**.
   - Set the `IRRLICHT_ROOT` variable: it must contain the path to the directory where you unzipped Irrlicht.<br>
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
Windows users should take care to convert the backslash `\` (the default in Win OS) to slash `/`!
</div>

At this point you just created a project that will be later used to build Chrono. You can close CMake.


## 7) Compile the project

For Visual Studio:

1. Go to the directory that you set in "Where to build the binaries". You will find the file **Chrono.sln**.

2. **Double-click** on that file: your Visual Studio solution will open.

3. In the toolbar, from the _Solution Configurations_ drop-down menu **choose 'Release' mode**  
  <img src="http://www.projectchrono.org/assets/manual/Install_vs_buildtype.png" class="img-responsive">

4. In the toolbar, click on **Build** > **Build solution...** .  
  The entire Chrono project and its demos will be compiled, creating many .exe and .dll files, in the bin/Release subdirectory.   
  This will take a few minutes.

5. Repeat step 3 and 4, but choosing **Debug** as configuration type. This will generate the binaries with debugging symbols: they will be placed under the bin/Debug subfolder.

For Linux/GCC and for MacOS/clang:

- CMake generates a hierarchy of makefiles in the directory specified in "Where to build the binaries".

- To build the Chrono libraries and demo executables, simply invoke <tt>make</tt> from the command line in that directory.

- Optionally, type <tt>make install</tt> to install the Chrono libraries, data files, and demo executables in the directory specified during CMake configuration.

<div class="ce-warning"> 
**MacOS issues:** clang++ does not come with OpenMP support out of the box.
You will not be able to build <tt>libChrono_parallel</tt> successfully.<br> 
However, OpenMP support can be added using the OpenMP sources from the <tt>llvm.org</tt> project. 
Download the source code from there, then configure the omp library with CMake, build it, and install it to /usr/local.<br>
Having done so, you can then configure Chrono with OpenMP support. For this, you must define the right compiler flags:<br>
<tt>-Xpreprocessor -fopenmp</tt> for the compiler and <tt>-lomp</tt> for the linker.
</div> 


## 8) Test the demos

Go to the directory that you set in "Where to build the binaries", 
in our case <tt>C:/workspace/chrono_build</tt>, then go to <tt>bin/Release</tt> or <tt>bin/Debug</tt> (Windows), or to <tt>bin</tt> (Linux).

<div class="ce-info">
**Windows**: If demos using 3D visualization do not start, this may indicate that the **Irrlicht.dll** is not found.  
You have to manually copy the Irrlicht.dll from your Irrlicht `/bin/Win64-visualStudio` directory into your `chrono_build/bin/Debug` and/or `chrono_build/bin/Release` directory.
</div>

<div class="ce-danger">
IMPORTANT: never mix 64-bit and 32-bit binaries and libraries! 
For example, you must link the 64-bit Irrlicht library.  
</div>