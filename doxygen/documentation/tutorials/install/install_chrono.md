Install Chrono {#tutorial_install_chrono}
==========================


A summary of the process required to **build** Chrono is provided in the picture below. 

<img src="http://www.projectchrono.org/assets/manual/Pic_build_ce.png" class="img-responsive">
<br>

## 1) Check/Install a C++ compiler

Recommended compilers:
- Microsoft Visual C++: Visual Studio from 2013 (free Community edition is fine, both 32 or 64 bit are ok)
- MingW GNU C++ compiler for Windows
- GNU C++ compiler for Linux-based platforms.
- Xcode Package for MacOS: Download via App Store for free - it contains the clang++ compiler

<small>Other compilers could work as well, but they might require changes to the CMake scripts.</small>

<div class="ce-danger">
The initial release of Visual Studio 2015 gives an 
error compiling Chrono. [Upgrade](https://www.visualstudio.com/en-us/news/vs2015-update2-vs.aspx) to 
**update 2** to fix this problem.
</div>

<div class="ce-warning"> 
The Microsoft Visual C++ compiler is included in the Visual Studio package, but it is **not** installed by default.<br>
So, make sure to install the C++ toolchain during the setup!<br>
Moreover, in the Visual Studio 2017 (and later) installer make sure to install also, under the *Single Components* tab, the `Windows Universal CRT SDK` and the`Windows 8.1 SDK`.
</div> 


## 2) Install [CMake](http://www.cmake.org/cmake/resources/software.html)

The free CMake utility is used to manage the building process.<br>
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

-  in the field "Destination Path" enter a path to an empty directory, say <tt>C:/chrono_source</tt>
   <img src="http://www.projectchrono.org/assets/manual/Install_ST2.gif" class="img-responsive">

-  leave the **Local Folder** field as it is

-  under "Advanced Options" make sure that `Checkout branch` is set on `master`
   
-  press **Clone**, and the source code will be downloaded into the folder you specified.

<div class="ce-info">
The `master` branch is the most stable and tested. We will refer to this branch for tutorials, demos and reference manuals. <br>
</div>

## 5) Download the Irrlicht library

- **download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html) 
- **unzip** it in a directory of your choice. For example, here we suppose that you unzipped it in <tt>C:/engine_demos/irrlicht-1.8.2</tt>.

<div class="ce-info"> 
Click here for the direct download of the 
[release v.1.8.2 of Irrlicht](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.2.zip)<br>
This release is tested to be stable and working well with Chrono. This is the recommended release for Windows and Linux.<br>
Release v.1.8.4 should work perfectly as well. On MacOS only use this one!<br>
Release v.1.8.3 does not contain the precompiled 64bit dlls.<br>
Release v.1.8.0 has issues with soft shadows.<br>
</div>

<div class="ce-warning"> 
**MacOS issues:** irrlicht-1.8.4 is fairly outdated compared to XCode 9.3.1.<br>
Before any building, you must correct a bug in the file:
<tt>irrlicht-1.8.4/source/Irrlicht/MacOSX/CIrrDeviceMacOSX.mm</tt>. Open it with CotEdit or BBEdit. Search for the string <tt>NSFileManagerDelegate</tt> 
and replace it by <tt>NSApplicationDelegate</tt>, don't forget to save your changes. In the terminal go to the directory containing the
<tt>MacOSX.xcodeproj</tt> bundle:<br>
<tt>cd irrlicht-1.8.4/source/Irrlicht/MacOSX</tt><br>
To build the library, type:<br>
<tt>xcodebuild</tt><br>
The library <tt>libIrrlicht.a</tt> should be found in <tt>irrlicht-1.8.4/source/Irrlicht/MacOSX/build/Release</tt>. It can be used from here, but it is better
to copy it to <tt>irrlicht-1.8.4/lib/MacOS</tt>. After copying type:<br>
<tt>cd irrlicht-1.8.4/lib/MacOSX</tt><br>
<tt>ranlib libIrrlicht.a</tt><br>
Unlike the Windows version we get a static library, that will be part of <tt>libChrono_irrlicht.dylib</tt>, so we don't have to copy it around 
anymore after building chrono.
</div> 



## 6) Run CMake

Start CMake to configure the build. We assume that you are using the graphical interface.
-  In the field "Where is the source code" set the path to your Chrono directory. <br>
   This is the directory where you created your Git repository, in our example is <tt>C:/chrono_source</tt>. 
-  In the field "Where to build the binaries" set the path to *another* directory on your system, 
   that must be empty. This is where the Visual C++ project will be created (or the makefiles, if on Linux). <br>
   For our example, let's use <tt>C:/chrono_build</tt>, 
   ![](http://www.projectchrono.org/assets/manual/Install_5.gif)
-  Press the **Configure** button.
-  Set the compiler among the generators in the window that opens, and press **Ok**.<br> 
   If possible, choose a **64bit** compiler.
-  Change the settings in the user interface of CMake.
   Some of these settings are automatically detected, but some other must be changed. 
   ![](http://www.projectchrono.org/assets/manual/Install_7.gif)
   - Enable the required modules: at least tick `ENABLE_MODULE_IRRLICHT` and `ENABLE_MODULE_POSTPROCESS`.<br>
     Other modules might require additional settings and dependencies.
   - Press **Configure**.
   - Set the `CH_IRRLICHTDIR` variable: it must contain the path to the directory where you unzipped Irrlicht.<br>
     In our example, browse to <tt>C:/engine_demos/irrlicht-1.8.2</tt>
   - Press **Configure** again.
   - The `CH_IRRLICHTLIB` variable should be filled automatically.
     If not, select <tt>C:/engine_demos/irrlicht-1.8.2/lib/Win64-visualStudio/Irrlicht.lib</tt>.<br>
	 Then press **Configure** again.
-  Remember that you might need to press **Configure** after you change some setting, even multiple times,
   until all the variables get a white background. 
-  Finally, press **Generate**.

<div class="ce-warning"> 
Visual Studio users should leave the `CMAKE_CONFIGURATION_TYPES` variable untouched (it should report *Debug;Release;MinSizeRel;RelWithDebInfo*;
if not please do *File*>*Delete Cache* and start again the CMake configuration).<br>
The build configuration will be chosen directly from Visual Studio.
</div>

<div class="ce-warning"> 
CMake uses the slash `/` character for paths. Unix users are already used to this convention.<br>
Windows users, on the opposite, should take particular care to convert the backslash `\` (the default in Win OS) to slash `/`!
</div>

At this point you just created a project that will be later used to build Chrono. You can close CMake.


## 7) Compile the project

For Visual Studio:

- Go to the directory that you set in "Where to build the binaries". You will find the file **Chrono.sln**.

- **Double-click** on that file: your Visual Studio solution will open.

- **Choose 'Debug' mode** using the drop-down list in the toolbar.

- Use the menu **Build** > **Build solution...** in the Visual Studio editor. The entire Chrono project
  and its demos will be compiled, creating many .exe and .dll files in the bin/Debug subdirectory.  
  This will take a few minutes.

- **Choose 'Release' mode** using the drop-down list in the toolbar.

- Use again the menu **Build** > **Build solution...** in the Visual Studio editor. The entire Chrono project
  and its demos will be compiled, creating many .exe and .dll files, this time in the bin/Release subdirectory.   
  This will take a few minutes.

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


## 8) Take the demos for a ride!

Go to the directory that you set in "Where to build the binaries", 
in our case <tt>C:/chrono_build</tt>, then go to <tt>bin/Release</tt> or <tt>bin/Debug</tt> (Windows), or to <tt>bin</tt> (Linux).

<div class="ce-info">
**Windows**: If demos using 3D visualization do not start, this may indicate that the **Irrlicht.dll** is not found.  
You have to manually copy the Irrlicht.dll from your Irrlicht `/bin/Win64-visualStudio` directory into your `chrono_build/bin/Debug` and/or `chrono_build/bin/Release` directory.
</div>

<div class="ce-danger">
IMPORTANT: never mix 64bit and 32bit binaries and libraries! 
For example, if you built Chrono in 64 bit, using a 64bit compiler, you must link the 64bit Irrlicht library.  
</div>

