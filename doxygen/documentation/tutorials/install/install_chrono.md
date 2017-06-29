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

The free CMake utility is used to manage the building process. <br>
For Visual Studio users: make sure to put the CMake executable in your *Path* environmental variable (the installer can do this for you).


## 3) Install a GIT client

On Windows, you might want to **download and install** [SourceTree](http://www.sourcetreeapp.com/).<br>
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

<div class="ce-warning"> 
Click here for the direct download of the 
[release v.1.8.2 of Irrlicht](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.2.zip)<br>
This release is tested to be stable and working well with Chrono. This is the recommended release.<br>
Release v.1.8.4 should work perfectly as well.<br>
Release v.1.8.3 does not contain the precompiled 64bit dlls.<br>
Release v.1.8.0 has issues with soft shadows.
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

## 8) Take the demos for a ride!

Go to the directory that you set in "Where to build the binaries", 
in our case <tt>C:/chrono_build</tt>, then go to <tt>bin/Release</tt> or <tt>bin/Debug</tt> directory.
There you will find the **demo_xxxxx.exe** files that you can launch by **double-clicking** on them.

<div class="ce-info">
It could happen that demos using 3D visualization might not start. This could happen if the **Irrlicht.dll** is missing.  
You have to manually copy the Irrlicht.dll from your Irrlicht `/bin/Win64-visualStudio` directory (or /Win32-.. if you are on a 32 bit platform) into your `chrono_build/bin/Debug` and/or `chrono_build/bin/Release` directory.
</div>

<div class="ce-danger">
IMPORTANT: never mix 64bit and 32bit binaries and libraries! 
For example, if you built Chrono in 64 bit, using a 64bit compiler, you must link the 64bit Irrlicht library.  
</div>

