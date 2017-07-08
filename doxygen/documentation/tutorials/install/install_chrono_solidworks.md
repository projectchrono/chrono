Install Chrono Solidworks {#tutorial_install_chrono_solidworks}
==========================

Chrono::SolidWorks is an add-in tool that allows to model complex
mechanisms using the powerful [SolidWorks](http://www.solidworks.com) 3D
CAD software. The user can create mechanisms with mouse and 3D
interface, then a description file can be output from SolidWorks and
load in a C++ or Python program.

![](http://projectchrono.org/assets/manual/SWaddin.jpg)

The generated description file is actually a Python .py file that
contains all the statements to create Chrono::Engine scenes, with
masses, positions, collision shapes, assets for visualization, etc., so
it basically saves you hours of blind programming. You only need to load
the .py file in a simple Python program (or C++ program, using the
unit\_PYTHON for parsing), add items to a ChSystem object, and run a
loop for computing the simulation.

Installation
------------

-   A copy of [SolidWorks](http://www.solidworks.com) must be installed
    on your computer. Supported from version 2011, 64bit, or later.
-   Install the
    [Chrono::SolidWorks](http://projectchrono.org/download/#chronosolidworks) add-in,
    using the installer in our download section.
-   install the [Chrono::PyEngine](@ref chrono_pyengine_installation)
    module, using the installer in our download section.

How to use it
-------------

You can find examples of use in [these
tutorials](Tutorials#Chrono::SolidWorks "wikilink").
