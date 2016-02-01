---
layout: default
title: Develop a new program with manual configuration
permalink: /documentation/tutorials/develop/new_demo/
---

There are different methods to create a C++ program that uses
Chrono::Engine (see [this page](/documentation/tutorials/develop/) to see the
others).

The method described here is the method that we suggest to do quick
tests, in fact you simply add a new demo to the SDK of ChronoEngine.

-   it uses on CMake, so it is platform independent,
-   it requires fery simple steps.
-   you will find your program integrated the same solution/project of
    the SDK.

The drawbacks, however, are that:

-   you need CMake to be installed on your system
-   it pollutes your ChronoEngine/source/demos directory (you might
    consider this method for developing simple stuff, than you remove it
    from the demos directory).

Follow these instructions.

![](Checkmark.png "fig:Checkmark.png") Create a .cpp file
---------------------------------------------------------

**Create a .cpp file** by duplicating one of the files in the demos
directory. For example, in the directory
ChronoEngine/source/demos/irrlicht, copy demo\_collision.cpp to
demo\_foo.cpp.

![](Checkmark.png "fig:Checkmark.png") Edit the CMake script
------------------------------------------------------------

**Edit the CMakeList.txt** file, adding your new demo. In detail, for
our example ChronoEngine/source/demos/irrlicht/demo\_foo.cpp, open the
file ChronoEngine/source/demos/irrlicht/CMakeList.txt, copy & duplicate
one of the blocks of test (few lines) going from
`ADD_EXECUTABLE(demo_collision ...` to
`ADD_DEPENDENCIES(demo_collision ...`, then replace the string
`demo_collision` with `demo_foo` in that new block of text.

![](Checkmark.png "fig:Checkmark.png") Compile
----------------------------------------------

To compile, just **rebuild** the SDK: your program demo\_foo.exe will be
generated just like all other demos.

<small>By the way, users from the developer team might consider commit a
new demo to the repository if they consider it useful to other users </small>
