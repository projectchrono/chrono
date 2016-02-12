Install Chrono {#tutorial_install_chrono}
==========================

##Simple Install Guide

<span class="label label-warning"><span class="glyphicon glyphicon-warning-sign"></span></span> The new version 2.0.0 will use GIT instead than SVN, and it will require a different installation and build process. Instructions will be rewritten as soon as the migration to GIT will be complete. 

The simplified installation uses an executable installer for deploying the Chrono::Engine SDK on your system. Such installer also adds a new wizard to your Microsoft Visual Studio C++ IDE, so that you can easily start and compile a new project.

This approach is over simplified compared to the full installation, but it also has some limitations:

* requirements are:
  *  Windows [XP/7/Vista] - there's no installer for Linux
  *  Microsoft Visual C++ compiler [Express/Visual Studio, 9.0/10.0] 
* the installer does not deploy .cpp source files (only .h headers are installed)
* using the SVN to get the code is more complex than using this installer, but it is more powerful.
* using the CMake build tool to compile your programs is more complex than using the wizard, but it is more powerful. 

Follow these steps. 

###Check/Install a C++ compiler
* Supported and tested compilers for the simplified installation are:
  * Microsoft Visual C++ 9.0/10.0 (professional and Express releases, both 64 and 32 bit) 

In case you do not have a C++ compiler, you can install the free Microsoft Visual C++ Express, see the Download page. 

###Download the Irrlicht library

Please download the free [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html) and unzip it in whatever directory. 

###Install the precompiled Chrono::Engine SDK

*Go to Download, download the installer for the precompiled SDK.
*Run the installer. When requested, set the path of the SDK to same directory, such as "C:\ChronoEngine". Note: do not install in some directory where you do not have permissions, otherwise the build process might fail, later.
*If all was fine, you can find the ChronoEngine entry in the Start menu of Windows. You can read the docs, run the demos, etc. 

###Run the wizard

To develop your C++ program using Chrono::Engine, read the instructions in [this page](). 

##Full Install Guide

This page explains how to install the Chrono::Engine sources and to compile them into binaries that later you can use to develop your project.

![](/images/Pic_build_ce.png "fig:pic_build_ce.png")

Note that some steps are mandatory, other are only suggested.

###Check/Install a C++ compiler

Supported and tested compilers are:
* Microsoft Visual C++ (professional and Express releases, both 64 and 32 bit)
* MingW GNU C++ compiler for Windows
* GNU C++ compiler for Linux-based platforms.

In case you do not have a C++ compiler, you can install it. There are some links to free C++ compilers in the [Download]() page. 

<small>Note that other non listed compilers might work as well, but they might require some changes to the CMake scripts.</small>

###Install CMake

The free CMake utility is used to setup the project building process.
Please **download and install** [CMake](http://www.cmake.org/cmake/resources/software.html).


###Install Git

If you do not have a Git client in your computer, you must install one.
For example in Windows **download and install** [SourceTree](http://www.sourcetreeapp.com/). 

For more informations, see the [Git repository]() page.


###Download the project by cloning the Git repository

Download the Chrono::Engine SDK by performing a **clone** of the Git repository in a directory of your workstation. 
Assuming you are using [http://http://www.sourcetreeapp.com/ SourceTree]:

1. press the **Clone / New** button in SourceTree ![](/images/Install_ST1.gif)
2. enter **<tt>https://github.com/projectchrono/chrono.git</tt>** in the field "Source Path / URL"
3. enter a path to an empty directory, say <tt>C:/chrono_source</tt>, in the field "Destination path" ![](/images/Install_ST2.gif)
4. press **Clone** and wait few minutes: the source code will be downloaded to your directory.

###Download the Irrlicht librar

Please **download** the free [Irrlicht Engine](http://irrlicht.sourceforge.net/downloads.html) and **unzip** it in whatever directory.


<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span>  **Note**. Some default Irrlicht archive contains precompiled libraries for Windows **32 bit** only. If you want to compile in **64 bit** mode, follow these steps:

**Building Irrlicht x64** (tested for Irrlicht 1.7.2)
Instructions are for VS2008 Professional

{% highlight bash %}
Open Irrlicht9.0.sln (in: \source\Irrlicht )
Create a new x64 Solution Platform 
Change build mode to release-FastFPU 

Open up the Irrlicht property page 
under C/C++ -> Preprocessor change WIN32 to WIN64 
under Linker -> General change ..\..\bin\Win32-visualstudio\Irrlicht.dll to ..\..\bin\Win64-visualstudio\Irrlicht.dll 
under Linker -> Advanced change ..\..\lib\Win32-visualstudio\Irrlicht.lib to ..\..\lib\Win64-visualstudio\Irrlicht.lib 
under Linker -> Command Line remove /MACHINE:I386 
{% endhighlight %}

**Disable DirextX** ''(If only using OpenGL, DirectX SDK is needed Otherwise)'':
{% highlight bash %}
Open IrrCompileConfig.h
On line 121 comment out #define _IRR_COMPILE_WITH_DIRECT3D_9_
{% endhighlight %}
The project should now build properly

###Run CMake

Start the CMake tool and configure the build. In detail:

1. In the field "Where is the source code" set the path to the directory <tt>/src</tt> (look in the directory where you created your Git repository). 
2. In the field "Where to build the binaries" set the path to another directory in your system, that must be empty. Here the Visual C++ project will be created (or the makefiles if in Linux). ![](/images/Install_5.gif)
3. Press the **Configure** button.
4. Set the generator in the window that opens , and press **Ok**. (in this case select the Visual Studio 9, because in this example we will compile with Microsoft Visual C++) ![](/images/Install_6.gif)
5. Change the settings in the user interface of CMake. Some of these settings are automatically detected, but some other must be changed. ![](/images/Install_7.gif)
  * Deactivate the advanced units by removing the checkmarks from ENABLE_UNIT_JS, ENABLE_UNIT_MATLAB, ENABLE_UNIT_PYTHON, ENABLE_UNIT_TESTS, ENABLE_UNIT_CASCADE, because they might require additional settings and dependencies. More info on advanced units [Units]().
  * Press **Configure**.
  * Set the directory in CH_IRRLICHTDIR, it must contain the path to your unzipped Irrlicht directory. 
  * Press **Configure**.
6. Remember that you might need to press **Configure** after you change some setting, even multiple times. 
7. Finally, press **Generate**.
Now you just created a project to build Chrono::Engine. You can also close CMake.

###Compile the project

Go to the directory that you set in "Where to build the binaries". You will find a file **ChronoEngine.sln**

**Double-click** on that file: your Visual Studio project will open.

**Choose 'Release' or 'Debug' mode** using the toggle in the toolbar.
 
**Press F7** in the Visual Studio editor: the entire Chrono::Engine project and its demos will be compiled, creating many .exe and .dll files. 
This will take few minutes.

###Play with the demos

Where are the binaries that you compiled? Go to the directory that you set in "Where to build the binaries", go to bin/Release or bin/Debug directory, here you will find the **demo_xxxxx.exe** files that you can launch by **double-clicking** on them.


<span class="label label-warning"><span class="glyphicon glyphicon-warning-sign"></span></span>
Note that the EXE demos with 3D visualization might not start, because Windows will tell you that the **Irrlicht.dll** is missing. If so, you have to manually copy the Irrlicht.dll from your Irrlicht \bin\Win32-VisualStudio directory (or \Win64-.. if you are on a 64 bit platform) into your ChronoEngine\bin\Win32_VisualStudio directory (or \Win64-.. if you are on a 64 bit platform).


Now you are ready: you can proceed and [develop your programs]() based on Chrono::Engine.