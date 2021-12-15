Chrono::Solidworks {#introduction_chrono_solidworks}
==========================

![](http://projectchrono.org/assets/manual/carousel_chronosolidworks.jpg)

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
**Python module** for parsing), add items to a ChSystem object, and run a
loop for computing the simulation.

* @subpage chrono_solidworks_installation
* [Tutorials](@ref tutorial_table_of_content_chrono_solidworks).

