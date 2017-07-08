Chrono::PyEngine tutorials  {#tutorial_table_of_content_chrono_pyengine}
==========================


Tutorials for users that installed the [Chrono::PyEngine](@ref introduction_chrono_pyengine) module.

<div class="ce-info">
These examples show how to use Chrono API **from the Python side**.
If you want to learn how to parse and execute Python programs
**from the C++ side** see also 
[the tutorials for Chrono::PyEngine](@ref tutorial_table_of_content_chrono_python)
</div>

-   @subpage tutorial_chrono_pyengine_demo_python1

    Learn the basics of Python interoperation with Chrono::Engine.

    - import the PyChrono::Engine module
    - use basic classes: vectors, matrices, etc.
    - inherit classes 

-   @subpage tutorial_chrono_pyengine_demo_python2

    Basic creation of a physical system and rigid bodies.

    - create a ChSystem
    - create and add rigid bodies
    - iterate on created contacts
    - iterate on added rigid bodies using the Python syntax 

-   @subpage tutorial_chrono_pyengine_demo_python3

    Create a postprocessing system based on POVray.

    - create a basic system with two bodies
    - create a postprocessor object
    - add asset objects to rigid bodies, for visualization
    - generate POVray scripts for rendering a 3D animation  

-   @subpage tutorial_chrono_pyengine_demo_masonry

    Create a small stack of bricks, move the floor like an earthquake, and see the bricks falling. Learn how:

    - use Irrlicht 3D realtime visualization in Chrono::PyEngine
    - impose a motion law to an object (the shaking platform)
    - add soft shadows to the realtime simulation  

-   @subpage tutorial_chrono_pyengine_demo_solidworks

    Import a SolidWorks scene into your Chrono::PyEngine program, and simulate it.

    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in Chrono::PyEngine and simulate it
    - generate POVray scripts for rendering a 3D animation 
	

-   @subpage tutorial_chrono_pyengine_demo_spider_robot

    Import a SolidWorks model of a crawling robot into your Chrono::PyEngine program, and simulate it.

    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in Chrono::PyEngine 
	- add actuators and additional items not modeled in CAD
    - show the simulation in an Irrlicht 3D view
	