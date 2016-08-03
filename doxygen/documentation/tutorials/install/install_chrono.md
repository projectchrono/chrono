Install Chrono {#tutorial_install_chrono}
==========================


A summary of the process required to **build** Chrono is provided in the picture below. 

![](http://www.projectchrono.org/assets/manual/Pic_build_ce.png)

<br>

## 1) Check/Install a C++ compiler

Recommended compilers:
- Microsoft C++: Visual Studio from 2013 (free Community edition is fine, both 32 or 64 bit are ok)
- MingW GNU C++ compiler for Windows
- GNU C++ compiler for Linux-based platforms.

<small>Other compilers could work as well, but they might require changes to the CMake scripts.</small>

<div class="ce-danger">
The initial release of Visual Studio 2015 gives an 
error compiling Chrono. [Upgrade](https://www.visualstudio.com/en-us/news/vs2015-update2-vs.aspx) to 
**update 2** to fix this problem.
</div>


## 2) Install [CMake](http://www.cmake.org/cmake/resources/software.html)

The free CMake utility is used to manage the building process.


## 3) Install a GIT client

On Windows, you might want to **download and install** [SourceTree](http://www.sourcetreeapp.com/). 
On Linux, there are several good [options](https://git-scm.com/download/gui/linux).


## 4) Download the project by cloning the Git repository

Download the Chrono SDK by performing a **clone** of the Git repository in a directory on your machine. 
Assuming you are using [SourceTree](http://www.sourcetreeapp.com/):

-  press the **Clone / New** button in SourceTree 
   ![](http://www.projectchrono.org/assets/manual/Install_ST1.gif)
   
-  enter <tt>https://github.com/projectchrono/chrono.git</tt> in the field "Source Path / URL"

-  enter a path to an empty directory, say <tt>C:/chrono_source</tt>, in the field "Destination path" 
   ![](http://www.projectchrono.org/assets/manual/Install_ST2.gif)
   
-  press **Clone**, and the source code will be downloaded to your directory.

## 5) Download the Irrlicht library

- **download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html) 
- **unzip** it in a directory of your choice. For example, here we suppose that you unzipped it in <tt>C:/engine_demos/irrlicht-1.8.2</tt>.

<div class="ce-warning"> 
Click here for the direct download of the 
[release v.1.8.2 of Irrlicht](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.2.zip)
This release is tested to be stable and working well with Chrono. This is the recommended release.   
Release v.1.8.3 does not contain the precompiled 64bit dlls.  
Release v.1.8.0 has issues with soft shadows.
</div> 



## 6) Run CMake

Start the CMake tool to configure the build. 

-  In the field "Where is the source code" set the path to your Chrono directory 
   This is the directory where you created your Git repository, in our example is <tt>C:/chrono_source</tt>. 
-  In the field "Where to build the binaries" set the path to another directory on your system, 
   that must be empty. This is where the Visual C++ project will be created (or the makefiles, if on Linux). 
   For our example, let's use <tt>C:/chrono_build</tt>, 
   ![](http://www.projectchrono.org/assets/manual/Install_5.gif)
-  Press the **Configure** button.
-  Set the compiler among the generators in the window that opens, and press **Ok**. 
   If possible, choose a **64bit** compiler.
-  Change the settings in the user interface of CMake.
   Some of these settings are automatically detected, but some other must be changed. 
   ![](http://www.projectchrono.org/assets/manual/Install_7.gif)
   - Activate the needed units with check-marks: at least click on ENABLE_MODULE_IRRLICHT, ENABLE_MODULE_POSTPROCESS. 
     Other modules might require additional settings and dependencies. 
   - Press **Configure**.
   - Set the directory in CH_IRRLICHTDIR: it must contain the path to the directory where you unzipped Irrlicht.  
     In our example, browse to <tt>C:/engine_demos/irrlicht-1.8.2</tt>
   - Set the library in CH_IRRLICHTLIB: it must contain the file of the Irrlicht.lib.  
     In our example, browse to <tt>C:/engine_demos/irrlicht-1.8.2/lib/Win64-visualStudio/Irrlicht.lib</tt>.
   - Press **Configure**.
-  Remember that you might need to press **Configure** after you change some setting, even multiple times,
   until all the labels do not have a red background any longer. 
-  Finally, press **Generate**.
At this point you just finished creating a project to build Chrono. You can close CMake.

## 7) Compile the project

-  Go to the directory that you set in "Where to build the binaries". You will find a file **ChronoEngine.sln**

-  **Double-click** on that file: your Visual Studio project will open.

-  **Choose 'Debug' mode** using the dropbox in the toolbar.
 
-  Use the menu **BUILD / Build solution...** in the Visual Studio editor. The entire Chrono project
   and its demos will be compiled, creating many .exe and .dll files in the bin/Debug directory.  
   This will take a few minutes.

-  **Choose 'Release' mode** using the dropbox in the toolbar.
 
6. Use again the menu **BUILD / Build solution...** in the Visual Studio editor. The entire Chrono project
   and its demos will be compiled, creating many .exe and .dll files, this time in the bin/Release directory.   
   This will take a few minutes.
   
   
## 8) Take the demos for a ride!

Go to the directory that you set in "Where to build the binaries", 
in our case  <tt>C:/chrono_build</tt>, then 
go to <tt>bin/Release</tt> or <tt>bin/Debug</tt> directory. There you will find the **demo_xxxxx.exe** files that you can launch by **double-clicking** on them.


<div class="ce-info">
It could happen that demos using 3D visualization might not start. This could happen if the **Irrlicht.dll** is missing.  
You have to manually copy the Irrlicht.dll from your Irrlicht \bin\Win64-VisualStudio directory (or \Win32-.. if you are on a 32 bit platform) into your ChronoEngine\bin\Win64_VisualStudio directory (or \Win32-.. if you are on a 32 bit platform).
</div>

<div class="ce-danger">
IMPORTANT: never mix 64bit and 32bit binaries and libraries! 
For example, if you built Chrono in 64 bit, using a 64bit compiler, you must link the 64bit Irrlicht library.  
</div>

