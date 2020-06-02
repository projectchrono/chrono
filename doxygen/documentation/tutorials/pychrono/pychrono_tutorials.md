PyChrono tutorials  {#tutorial_table_of_content_pychrono}
==========================

\tableofcontents

This is a collection of tutorials for users of the [PyChrono](@ref pychrono_introduction) module.
We suggest you to study them in the presented order of increasing difficulty.

The examples below show how to use the Chrono API **from the Python side**.
For examples of using Chrono::Python to parse and execute Python programs **from the C++ side** see the 
[tutorials for the C++ Chrono Python module](@ref tutorial_table_of_content_chrono_python).

<span style="color:red;font-weight:bold">ATTENTION!</span> Sources for these demos, **compatible** with the version of the Chrono libraries you have installed, can be found in the following locations:
- if you have a clone of the Chrono sources and you have built PyChrono from scratch, the code for the Python demos mentioned below can be found in the Chrono source tree under `src/demos/python/`;
- if you installed a conda PyChrono package, the Python demos mentioned below can be found in your local anaconda installation under `Lib\pychrono\demos\`.

## Introductory tutorials

- demo_python_1.py

    Learn the basics of Python interoperation with Chrono.
    - import the PyChrono module
    - use basic classes: vectors, matrices, etc.
    - inherit classes 

<br>

- demo_python_2.py

    Basic creation of a physical system and rigid bodies.
    - create a ChSystem
    - create and add rigid bodies
    - iterate on created contacts
    - iterate on added rigid bodies using the Python syntax 

<br>

- demo_python_3.py

    Create a postprocessing system based on POVray.
    - create a basic system with two bodies
    - create a postprocessor object
    - add asset objects to rigid bodies, for visualization
    - generate POVray scripts for rendering 3D animation as post-processing

<br>

- demo_irrlicht.py

    Create a simple pendulum and display it in an interactive 3D view
    - use pychrono.irrlicht, the Irrlicht 3D realtime visualization of PyChrono
    - attach textures as visualization assets
    - create bodies and constraints

<br>

- demo_masonry.py

    Create a small stack of bricks, move the floor like an earthquake, and see the bricks falling.
    - impose a motion law to an object (the shaking platform)
    - add soft shadows to the Irrlicht realtime simulation  

<br>

- demo_crank_plot.py

    Create a slider-crank.
    - add a motor with imposed angular speed
    - plot results using python's matplotlib library 

<br>

- demo_paths.py

    Create two pendulums following parametric lines. Learn how:
    - create piecewise paths built from sub-lines, and visualize them
    - add a constraint of 'curvilinear glyph' type (the body can freely move with one of its points being constrained to slide along a path)
    - add a constraint of 'imposed trajectory' type (the body must with one of its points being constrained to walk along a path as a parametric line with motion function)

<br>

- demo_mesh.py

    Create complex rigid body shapes based on meshes. Learn how:
    - load a .obj mesh file and use it for visualization of the shape
    - load a .obj mesh file and use it for collision
	- adjust position of center of mass respect to reference in ChBodyAuxRef
	- change inertia properties.

    Notes:
    - Collision detection with generic concave meshes is slower and less robust than any other options for collision shapes, so use it only if defining  the collision shape via primitives like spheres boxes cylinders or their clusters is too complex. (In fact, a good trade-off often is the following: use a detailed mesh for visualization, and few simple primitives for collision).
    - The mesh shape is a .obj file in Wavefront file format, you can generate it from 3D modelers such as Blender, Maya, etc., as well as from some CAD.
    - For collision purposes, the .obj mesh must be "watertight", i.e. having no gaps in edges, no repeated vertexes, etc. If you are not sure about this, the free tool MeshLab, for example, has tools to check the topological correctness of the mesh.
    - For visualization purposes only, i.e. if you do not use the mesh also for  collision, the mesh does not need to be watertight. (btw. If the visualization does not look good, check if the normals are correct in your .obj file.)
	
## FEA tutorials

- demo_fea_beams.py

    Use the pychrono.fea module to simulate flexible beams
    - use the python.fea module
    - create beams with constraints

<br>

- demo_fea_beams_dynamics.py

    Use the pychrono.fea module to simulate the Jeffcott rotor
    - use the python.fea module
    - create a flexible rotor passing through instability
	- tweak the integrator and solver settings for higher precision
	- create an ad-hoc motion function by python-side inheritance from ChFunction

<br>

## Chrono::Vehicle tutorials

Simulate vehicle dynamics in Python, using complete pre-built wheeled vehicle models.

- demo_vehicle_HMMWV.py

    HMMWV vehicle demo using pre-built model

<br>

- demo_vehicle_HMMWV_JSON.py

    Simulation of a vehicle completely specified through JSON files (in this case the same HMMWV vehicle as in demo_vehicle_HMMWV.py)

<br>

- demo_vehicle_HMMWV9_YUP.py

    Demonstration of setting a world frame with a vertical Y axis, using a reduced-order model of the HMMWV vehicle

<br>

- demo_vehicle_citybus.py

    Bus vehicle demo showing a vehicle with double tires on the rear axle

<br>

- demo_vehicle_UAZ.py

    Van vehicle demo 

<br>

- demo_vehicle_MAN.py: 

    Truck vehicle demo showing a vehicle with two steerable axles

<br>


## OpenCascade tutorials
	
- demo_cascade.py

    Use the pychrono.cascade module to create a shape with the OpenCascade kernel, then let it fall on the ground.
    - use the python.cascade module
    - create collisions with concave meshes
	- control collision tolerances (envelope, margin)

<br>

- demo_cascade_step.py

    Use the pychrono.cascade module to load a STEP file saved from a CAD.
    - load a STEP file, saved from a 3D CAD.
	- fetch parts from the STEP document and conver into Chrono bodies.

<br>

- demo_cascade_step_robot.py

    Use pychrono.cascade to load a STEP file and create constraints between the bodies.
    - load a STEP file, saved from a 3D CAD.
	- fetch parts and recerences from the STEP document, and create joints between them.
	- assign a ChLinkTrajectory to a part

	
## Tutorials using the SolidWorks add-in
	
- demo_solidworks_irrlicht.py

    Import a SolidWorks scene into your PyChrono program, and simulate it. The Chrono::Solidworks add-in can be used in SolidWorks to export an assembly into a .py file that can be later load and simulated.
    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono and simulate it
    - visualize it in Irrlicht

<br>

- demo_solidworks_pov.py

    Import a SolidWorks scene into your PyChrono program, and simulate it.
    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono and simulate it
    - visualize it with POVray scripts for rendering a 3D animation 

<br>

- Spider robot demo

    Import a SolidWorks model of a crawling robot into your PyChrono program, and simulate it.
    - use the Chrono::SolidWorks Add-In for exporting a mechanical system
    - load the system in PyChrono
    - add actuators and additional items not modeled in CAD
    - show the simulation in an Irrlicht 3D view
    - [Additional details](@ref tutorial_pychrono_demo_spider_robot)

	
## Tutorials about AI

- Using PyChrono with TensorFlow

    Ttrain a Neural Network in simulation to control actuators.
    - Build a learning model with Tensorflow
    - Build a training environment with Pychrono
    - Use simulation to train the Neural Network 
    - [Additional details](@ref tutorial_pychrono_demo_tensorflow)

<br>
