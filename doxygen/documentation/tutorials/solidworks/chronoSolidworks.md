Chrono::SolidWorks&copy; tutorials {#tutorial_table_of_content_chrono_solidworks}
==========================


Tutorials for users that installed the [Chrono::Solidworks](@ref manual_chrono_solidworks) add-in for SolidWorks.

Ohter examples can be found directly in the Chrono::SolidWorks repository under [examples](https://github.com/projectchrono/chrono-solidworks/tree/master/to_put_in_app_dir/examples).

<br>
-   @subpage tutorial_chrono_solidworks_demo_engine

    Learn the basics of Chrono::SolidWorks add-in for SolidWorks:

    - model a 3D assembly using SolidWorks,
    - export it as a .py file containing the Chrono::Engine system description.
    - run a Python simulation of the exported system
    - render the simulation using POVray 

<br>
-   @subpage tutorial_chrono_solidworks_demo_engine_advanced

    Tips for advanced use of Chrono::SolidWorks add-in, with some Python tricks and tips for:

    - modify the exported scene by creating custom constraints
    - assigning POVray materials
    - creating custom POVray objects
    - attaching a camera to a moving part 

<br>
-   @subpage tutorial_chrono_solidworks_demo_shapes
 
    Learn how to define collision shapes when using the Chrono::SolidWorks add-in for SolidWorks&copy;.

    - assign collision shapes to parts in SolidWorks
    - create and assign surface materials to parts using Python
    - customize advanced POVray pigments and textures
    - move the ground to simulate an earthquake 

<br>
-   @subpage tutorial_chrono_solidworks_demo_spider_robot

    Import a SolidWorks model of a crawling robot into your PyChrono program, and simulate it.

    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono
    - add actuators and additional items not modeled in CAD
    - show the simulation in an Irrlicht 3D view
	

## Tutorials using pre-exported models

These tutorials come with pre-exported models to allow the user to test the import functionalities without having to install the add-in straight away.
	
- [demo_PARSER_Python_SolidWorks.py](https://github.com/projectchrono/chrono/blob/main/src/demos/python/parsers/demo_PARSER_Python_SolidWorks.py)

    - simulation by PyChrono
    - uses either Irrlicht or POVRAy rendering

<br>

- [demo_PARSER_Python_SolidWorks.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/parsers/demo_PARSER_Python_SolidWorks.cpp)

    - simulation with Chrono (C++) by loading a Python exported model (requires PARSERS module)



