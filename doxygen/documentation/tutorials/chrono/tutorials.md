Chrono core tutorials      {#tutorial_table_of_content_chrono}
==========================

The tutorials below focus on how to set up physical systems in Chrono. 
No graphical user interface comes into play. See the [Irrlicht tutorials](@ref tutorial_table_of_content_chrono_irrlicht) 
for examples with a 3D graphical output. 

-   @subpage tutorial_demo_buildsystem
	
    Basics of building and simulating mechanical systems. Learn how to:

    - create a physical system (a slider-crank)
    - add/remove rigid bodies
    - create mechanical joints between bodies
    - perform a simulation 

-   @subpage tutorial_demo_powertrain

    Building systems that embed powertrains modeled with one-degree-of-freedom elements (rotating shafts). Learn how to:

    - connect 1D shafts with transmission ratios
    - connect 1D shafts with 1D clutches, brakes, etc.
    - connect a 1D shaft to a 3D body 

-   @subpage tutorial_demo_chfunctions

    Using ChFunction inherited classes to build math functions of \f$ y=f(x) \f$ type. The ChFunction objects are 'building blocks' used to describe motion laws such as, for instance, trajectories in automation and robotics. Learn how to:

    - create and use ChFunction objects
    - define a custom function by inheriting from the ChFunction class

-   @subpage tutorial_demo_math

    Main mathematical entities used in Chrono such as vectors and matrices. See also the [math support](@ref mathematical_objects) page in the reference manual.
	
-   @subpage tutorial_demo_coords

    Coordinate transformations related issues. See also the [manual page](@ref coordinate_transformations) discussing this topic.
	
-   @subpage tutorial_demo_stream

    Using ChStream classes to write/load from file streams.

-   @subpage tutorial_demo_archive

    Perform serialization/deserialization of C++ classes