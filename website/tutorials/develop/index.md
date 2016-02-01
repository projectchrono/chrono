---
layout: default
title: Develop a program
permalink: /documentation/tutorials/develop/
---

When you want to develop a program based on Chrono::Engine, there are
different alternatives:

-   [Use the CMake build tool](/documentation/tutorials/develop/cmake/)

    This is approach that we suggest when you want to develop
    serious projects

<!-- -->

-   [Add a new source in the ChronoEngine/source/demos/ directory](/documentation/tutorials/develop/new_demo/)
    
    This is the *fastest* method to do quick tests, but it is not
    good for large projects

<!-- -->

-   [Manually configure your IDE or build tools](/documentation/tutorials/develop/advanced/)

    This section is for people that need to integrate
    Chrono::Engine in a project where they are already using other
    build systems. It contains low-level details on build flags, library
    paths etc.

See the pages for details.

In all cases, the build process requires that you

-   include the .h headers of Chrono::Engine,
-   link the .lib libraries of Chrono::Engine,
-   build your .exe

as shown in the following scheme:

![](/images/pic_build.png "pic_build.png")
