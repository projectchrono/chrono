Use the CMake build tool {#tutorial_cmake}
==========================

There are different methods to create a C++ program that uses
Chrono::Engine (see [this page](/documentation/tutorials/develop/) to see the
others).

The method described here is the one that we suggest in most cases,
because it is platform-independent and it is based on the powerful CMake
build tool.

Prerequisites:

-   [CMake](http://www.cmake.org) must be already installed on
    your computer.
-   Chrono::Engine must be already [installed and
    compiled](ChronoEngine:Installation "wikilink") on your computer.

![](Checkmark.png "fig:Checkmark.png") Create the project directory
-------------------------------------------------------------------

-   Copy the ChronoEngine/template_project directory to some place,
-   Rename the directory, for example: **C:/foo_project**
-   Rename the **my_example.cpp** as you want, for example:
    **foo_example.cpp**.

This will be the directory where you put your source code.

![](Checkmark.png "fig:Checkmark.png") Edit the CMake script
------------------------------------------------------------

To make things easier for you, in the template directory there is
already a CMake script: just edit the **CMakeLists.txt** following the
instructions contained inside that file.

![](Checkmark.png "fig:Checkmark.png") Start CMake
--------------------------------------------------

-   Start CMake GUI,
-   Use **Browse source...** by setting the source directory that you
    created, ex: **C:/foo_project**
-   Use **Browse build...** by setting a different directory for the
    output project, ex: **D:/build/foo_project**

![](Checkmark.png "fig:Checkmark.png") Configure
------------------------------------------------

Press the **Configure** button in CMake, and set the proper values in
the CMake user interface.

If prompted to pick a generator, select the same compiler that you used
to compile Chrono::Engine.

The most important variables to set are:

-   CH_CHRONO_SDKDIR , here set where you have the src/ directory of
    your Chrono::Engine API.
-   CH_LIBDIR_DEBUG and CH_LIBDIR_RELEASE, here set the directories
    where you compiled, respectively, the debug and release versions
    of ChronoEngine.lib.

For other variables, just hoover on the CMake variable name and you'll
see a tool tip help.

You may need to press **Configure** two or more times, the first time
might show some red warnings.

![](/images/tutorials/cmake_fooproject2.gif "cmake_fooproject2.gif")

![](Checkmark.png "fig:Checkmark.png") Generate the project
-----------------------------------------------------------

Press the **Generate** button in CMake, and a project will be created in
the build directory, ex: **D:/build/foo_project**

![](Checkmark.png "fig:Checkmark.png") Compile the project
----------------------------------------------------------

If you used a VisualStudio generator in CMake, now you

-   **open the solution** in VisualStudio editor (in this case double
    click on D:/build/foo_project/foo_project.sln)
-   use the menu **BUILD / Build solution...** to compile and link.

If you used a Unix makefile generator in CMake (ex. in Linux), you can
open a shell, cd into the build directory, ex: **cd
/home/john/foo_project** and type **make**, and the project will be
built.

![](Checkmark.png "fig:Checkmark.png") Run your program
-------------------------------------------------------

By default all binaries (ex. myexe.exe) of your project will go into
your build directory, in this example D:/build/foo_project/Release or
D:/build/foo_project/Debug.

Go there, double click on the .exe file and you should see the demo
running:

![](/images/tutorials/fooproject.jpg "fooproject.jpg")

Additional information
----------------------

<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span> Copying the DLL of Chrono::Engine.

For Windows users. Your executable need to know where to find the .dll (dynamic-link libraries), otherwise it will give an error as soon as you try to run it.

These DLLs are in the bin/Debug/ and bin/Release/ build directory where you build Chrono::Engine.

You have three options:

* you copy ChronoEngine.dll, ChronoEngine_IRRLICHT.dll, and other .dll files into the same directory where you have your .exe
* you add the build directories of Chrono::Engine in a globally-visible PATH
* you just use the **CHRONOENGINE_ENABLE_CHRONODLLCOPY()** macro in your CMakeFile.txt as in this example. This enables an automatic copy of the needed dll just after each build. This is the suggested option. It also copies the Irrlicht.dll into your executable directory, if unit_IRRLICHT is used. 

Note that Linux users do not have this issue because the .so libraries always go into into a directory that is globally visible.

<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span> Copying the data (textures, etc).

Your executable might need to load some textures, fonts, etc. For this example, the textures in the data/ directory are used. The executable will try to load them from the relative path ../data/. However, for various reason (ex. have them versioned in a GIT) it may be nice to have the data folder in the same directory of the .cpp files, and copy them to the build directory later.

You have two options:

* you copy the C:/foo_project/data directory into your D:/build/foo_project/data by hand,
* you just use the **CHRONOENGINE_COPY_PROJECT_DATA_DIR()** macro in your CMakeFile.txt as in this example. This enables an automatic copy of the needed data directory just after each CMake run. 


