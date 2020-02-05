PyChrono tutorials  {#tutorial_table_of_content_pychrono}
==========================


This is a collection of tutorials for users that installed the [PyChrono](@ref pychrono_introduction) module.

We suggest you to **study them in the presented order** of increasing difficulty.

<div class="ce-info">
These examples show how to use Chrono API **from the Python side**.
If you want to learn how to parse and execute Python programs
**from the C++ side** see also 
[the tutorials for the C++ Chrono Python module](@ref tutorial_table_of_content_chrono_python)
</div>

## Introductory tutorials

-   @subpage tutorial_pychrono_demo_python1

    Learn the basics of Python interoperation with Chrono.

    - import the PyChrono module
    - use basic classes: vectors, matrices, etc.
    - inherit classes 

<br>
-   @subpage tutorial_pychrono_demo_python2

    Basic creation of a physical system and rigid bodies.

    - create a ChSystem
    - create and add rigid bodies
    - iterate on created contacts
    - iterate on added rigid bodies using the Python syntax 

<br>
-   @subpage tutorial_pychrono_demo_python3

    Create a postprocessing system based on POVray. *(optional reading)*

    - create a basic system with two bodies
    - create a postprocessor object
    - add asset objects to rigid bodies, for visualization
    - generate POVray scripts for rendering 3D animation as post-processing

<br>
-   @subpage tutorial_pychrono_demo_irrlicht

    Create a simple pendulum and display it in an interactive 3D view

    - use pychrono.irrlicht, the Irrlicht 3D realtime visualization of PyChrono
    - create bodies and constraints

<br>
-   @subpage tutorial_pychrono_demo_masonry

    Create a small stack of bricks, move the floor like an earthquake, and see the bricks falling. Learn how:

    - impose a motion law to an object (the shaking platform)
    - add soft shadows to the Irrlicht realtime simulation  

<br>
-   @subpage tutorial_pychrono_demo_vehicle

    Simulate vehicle dynamics in Python. In the 2 related demos we show 2 ways of modelling the same HMMWV vehicle.
	
<br>
-   @subpage tutorial_pychrono_demo_crank_plot

    Create a slider-crank. Learn how:

    - add a motor
    - plot results using python's matplotlib library 

<br>
-   @subpage tutorial_pychrono_demo_paths

    Create two pendulums following parametric lines. Learn how:

    - create piecewise lines built from sub-lines, and visualize them
	- add a constraint of 'curvilinear glyph' type
    - add a constraint of 'inposed trajectory' type

<br>
-   @subpage tutorial_pychrono_demo_mesh

    Create complex rigid body shapes based on meshes. Learn how:

    - load a .obj mesh file and use it for visualization of the shape
    - load a .obj mesh file and use it for collision
	- adjust position of center of mass respect to reference in ChBodyAuxRef
	- change inertia properties.
	
## FEA tutorials

<br>
-   @subpage tutorial_pychrono_demo_fea_beams

    Use the pychrono.fea module to simulate flexible beams

    - use the python.fea module
    - create beams with constraints

<br>
-   @subpage tutorial_pychrono_demo_fea_beams_dynamics

    Use the pychrono.fea module to simulate the Jeffcott rotor

    - use the python.fea module
    - create a flexible rotor passing through instability
	- tweak the integrator and solver settings for higher precision
	- create an ad-hoc motion function by python-side inheritance from ChFunction

## OpenCascade tutorials
	
<br>
-   @subpage tutorial_pychrono_demo_cascade

    Use the pychrono.cascade module to create a shape with the OpenCascade kernel, then let it fall on the ground.

    - use the python.cascade module
    - create collisions with concave meshes
	- control collision tolerances (envelope, margin)

<br>
-   @subpage tutorial_pychrono_demo_cascade_step

    Use the pychrono.cascade module to load a STEP file saved from a CAD.

    - load a STEP file, saved from a 3D CAD.
	- fetch parts from the STEP document and conver into Chrono bodies.

<br>
	-   @subpage tutorial_pychrono_demo_cascade_step_robot

    Use pychrono.cascade to load a STEP file and create constraints between the bodies.

    - load a STEP file, saved from a 3D CAD.
	- fetch parts and recerences from the STEP document, and create joints between them.
	- assign a ChLinkTrajectory to a part

	
## Tutorials using the SolidWorks add-in
	
<br>
-   @subpage tutorial_pychrono_demo_solidworks_irrlicht

    Import a SolidWorks scene into your PyChrono program, and simulate it.

    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono and simulate it
    - visualize it in Irrlicht

<br>
-   @subpage tutorial_pychrono_demo_solidworks_pov

    Import a SolidWorks scene into your PyChrono program, and simulate it.

    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono and simulate it
    - visualize it with POVray scripts for rendering a 3D animation 
	

<br>
-   @subpage tutorial_pychrono_demo_spider_robot

    Import a SolidWorks model of a crawling robot into your PyChrono program, and simulate it.

    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono
        * add actuators and additional items not modeled in CAD
    - show the simulation in an Irrlicht 3D view

	
## Tutorials about AI

<br>
-   @subpage tutorial_pychrono_demo_tensorflow

    Use PyChrono and TensorFlow to train a NN in simulation to control actuators.
    - Build a learning model with Tensorflow
    - Build a training environment with Pychrono
    - Use simulation to train the Neural Network 
