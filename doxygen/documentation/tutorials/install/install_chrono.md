Install Chrono {#tutorial_install_chrono}
==========================


This page explains how to **install** the Chrono::Engine sources and to **build** 
them into binaries that later you can use to develop your project.

![](Pic_build_ce.png)

Note that some steps are mandatory, other are only suggested.

## 1) Check/Install a C++ compiler

Supported and tested compilers are:
- Microsoft C++: Visual Studio from 2013 (free Community edition or others, both 32 or 64 bit)
- MingW GNU C++ compiler for Windows
- GNU C++ compiler for Linux-based platforms.

In case you do not have a C++ compiler, you can install it. There are some links to free C++ compilers in the [Download](http://www.projectchrono.org/download) page. 

<small>Note that other non listed compilers might work as well, but they might require some changes to the CMake scripts.</small>

<div class="ce-danger">
Warning! The initial release of Visual Studio 2015 gives an 
error compiling Chrono::Engine! If you use it, you must upgrade it to 
the **update 2** that fixed the problem. Download it from 
[this page](https://www.visualstudio.com/en-us/news/vs2015-update2-vs.aspx). 
</div>


## 2) Install CMake

The free CMake utility is used to setup the project building process.
Please **download and install** [CMake](http://www.cmake.org/cmake/resources/software.html).


## 3) Install a GIT client

If you do not have a GIT client in your computer, you must install one.
For example in Windows **download and install** [SourceTree](http://www.sourcetreeapp.com/). 


## 4) Download the project by cloning the Git repository

Download the Chrono::Engine SDK by performing a **clone** of the Git repository in a directory of your workstation. 
Assuming you are using [SourceTree](http://www.sourcetreeapp.com/):

-  press the **Clone / New** button in SourceTree 
   ![](Install_ST1.gif)
   
-  enter <tt>https://github.com/projectchrono/chrono.git</tt> in the field "Source Path / URL"

-  enter a path to an empty directory, say <tt>C:/chrono_source</tt>, in the field "Destination path" 
   ![](Install_ST2.gif)
   
-  press **Clone** and wait few minutes: the source code will be downloaded to your directory.

## 5) Download the Irrlicht library

- **download** [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html) 
- **unzip** it in whatever directory.  
  For example, here we suppose that you unzipped it in <tt>C:/engine_demos/irrlicht-1.8.2</tt>.

<div class="ce-warning"> 
Click here for the direct download of the 
[release v.1.8.2 of Irrlicht](http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.2.zip)
This release is tested to be stable and working well with Chrono::Engine. We suggest you to pick it.   
Release v.1.8.3 does not contain the precompiled 64bit dlls.  
Release v.1.8.0 has some issues with soft shadows.
</div> 



## 6) Run CMake

Start the CMake tool and configure the build. In detail:

-  In the field "Where is the source code" set the path to your Chrono directory 
   This is the directory where you created your Git repository, in our example is <tt>C:/chrono_source</tt>. 
-  In the field "Where to build the binaries" set the path to another directory in your system, 
   that must be empty. Here the Visual C++ project will be created (or the makefiles if in Linux). 
   For our example, let's use <tt>C:/chrono_build</tt>, 
   ![](Install_5.gif)
-  Press the **Configure** button.
-  Set the compiler among the generators in the window that opens, and press **Ok**. 
   If possible, choose a **64bit** compiler.
-  Change the settings in the user interface of CMake.
   Some of these settings are automatically detected, but some other must be changed. 
   ![](Install_7.gif)
   - Activate the needed units with checkmarks: at least click on ENABLE_MODULE_IRRLICHT, ENABLE_MODULE_POSTPROCESS. 
     Other modules might require additional settings and dependencies. More info on advanced modules in a separate section. [Units]().
   - Press **Configure**.
   - Set the directory in CH_IRRLICHTDIR: it must contain the path to your unzipped Irrlicht directory.  
     In our example, browse to <tt>C:/engine_demos/irrlicht-1.8.2</tt>
   - Set the library in CH_IRRLICHTLIB: it must contain the file of the Irrlicht.lib.  
     In our example, browse to <tt>C:/engine_demos/irrlicht-1.8.2/lib/Win64-visualStudio/Irrlicht.lib</tt>.
   - Press **Configure**.
-  Remember that you might need to press **Configure** after you change some setting, even multiple times,
   until all the labels do not have a red background anymore. 
-  Finally, press **Generate**.
Now you just created a project to build Chrono::Engine. You can also close CMake.

## 7) Compile the project

-  Go to the directory that you set in "Where to build the binaries". You will find a file **ChronoEngine.sln**

-  **Double-click** on that file: your Visual Studio project will open.

-  **Choose 'Debug' mode** using the dropbox in the toolbar.
 
-  Use the menu **BUILD / Build solution...** in the Visual Studio editor: the entire Chrono::Engine project
   and its demos will be compiled, creating many .exe and .dll files in the bin/Debug directory.  
   This will take few minutes.

-  **Choose 'Release' mode** using the dropbox in the toolbar.
 
6. Use again the menu **BUILD / Build solution...** in the Visual Studio editor: the entire Chrono::Engine project
   and its demos will be compiled, creating many .exe and .dll files, this time in the bin/Release directory.   
   This will take few minutes.
   
   
## 8) Play with the demos

Where are the binaries that you compiled? Go to the directory that you set in "Where to build the binaries", 
that is  <tt>C:/chrono_build</tt> in our case, than 
go to <tt>bin/Release</tt> or <tt>bin/Debug</tt> directory, 
here you will find the **demo_xxxxx.exe** files that you can launch by **double-clicking** on them.


<div class="ce-info">
It could happen that EXE demos with 3D visualization might not start, 
because Windows will tell you that the **Irrlicht.dll** is missing.  
If so, you have to manually copy the Irrlicht.dll from your Irrlicht \bin\Win64-VisualStudio directory (or \Win32-.. if you are on a 32 bit platform) into your ChronoEngine\bin\Win64_VisualStudio directory (or \Win32-.. if you are on a 32 bit platform).
</div>

<div class="ce-danger">
Remember: never mix 64bit and 32bit binares and libraries! 
For example, if you built Chrono in 64 bit, using a 64bit compiler, you must link the 64bit Irrlicht library.  
</div>

Now you are ready: you can proceed and [develop your programs](@ref tutorial_install_project) based on Chrono::Engine.