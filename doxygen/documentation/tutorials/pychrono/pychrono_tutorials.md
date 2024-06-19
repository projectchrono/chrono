PyChrono tutorials  {#tutorial_table_of_content_pychrono}
==========================

\tableofcontents

This is a collection of tutorials for users of the [PyChrono](@ref pychrono_introduction) module.
We suggest you study them in the presented order of increasing difficulty.

The examples below show how to use the Chrono API **from the Python side**.
For examples of using Chrono::Python to parse and execute Python programs **from the C++ side** see the 
[tutorials for the C++ Chrono Python module](@ref tutorial_table_of_content_chrono_python).

<span style="color:red;font-weight:bold">ATTENTION!</span> Sources for these demos, **compatible** with the version of the Chrono libraries you have installed, can be found in the following locations:
- if you have a clone of the Chrono sources and you have built PyChrono from scratch, the code for the Python demos mentioned below can be found in the Chrono source tree under `src/demos/python/`;
- if you installed a conda PyChrono package, the Python demos mentioned below can be found in your local anaconda site-packages under `pychrono/demos/`.

## Introductory tutorials

- **core/demo_CH_coords.py**

    Learn the basics of Python interoperation with Chrono.
    - import the PyChrono module
    - use basic classes: vectors, matrices, etc.
    - inherit classes 

<br>

- **core/demo_CH_buildsystem.py**

    Basic creation of a physical system and rigid bodies.
    - create a ChSystem
    - create and add rigid bodies
    - iterate on created contacts
    - iterate on added rigid bodies using the Python syntax 

<br>

- **postprocess/demo_POST_povray1.py**

    Create a postprocessing system based on POVray.
    - create a basic system with two bodies
    - create a postprocessor object
    - add asset objects to rigid bodies, for visualization
    - generate POVray scripts for rendering 3D animation as post-processing

<br>

- **irrlicht/demo_IRR_revolute.py**

    Create a simple pendulum and display it in an interactive 3D view
    - use pychrono.irrlicht, the Irrlicht 3D realtime visualization of PyChrono
    - attach textures as visualization assets
    - create bodies and constraints

<br>

- **irrlicht/demo_IRR_earthquake.py**

    Create a small stack of bricks, move the floor like an earthquake, and see the bricks falling.
    - impose a motion law to an object (the shaking platform)
    - add soft shadows to the Irrlicht realtime simulation  

<br>

- **irrlicht/demo_IRR_crank_plot.py**

    Create a slider-crank.
    - add a motor with imposed angular speed
    - plot results using python's matplotlib library 

<br>

- **irrlicht/demo_IRR_paths.py**

    Create two pendulums following parametric lines. Learn how:
    - create piecewise paths built from sub-lines, and visualize them
    - add a constraint of 'curvilinear glyph' type (the body can freely move with one of its points being constrained to slide along a path)
    - add a constraint of 'imposed trajectory' type (the body must with one of its points being constrained to walk along a path as a parametric line with motion function)

<br>

- **irrlicht/demo_IRR_collision_trimesh.py**

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

- **fea/demo_FEA_beams.py**

    Use the pychrono.fea module to simulate flexible beams
    - use the python.fea module
    - create beams with constraints

<br>

- **fea/demo_FEA_beamsIGA.py**

    Use the pychrono.fea module to simulate the Jeffcott rotor
    - use the python.fea module
    - create a flexible rotor passing through instability
	- tweak the integrator and solver settings for higher precision
	- create an ad-hoc motion function by python-side inheritance from ChFunction

<br>

## Chrono::Vehicle tutorials

Simulate vehicle dynamics in Python, using complete pre-built wheeled vehicle models.

- **vehicle/demo_VEH_HMMWV.py**

    HMMWV vehicle demo using pre-built model

<br>

- **vehicle/demo_VEH_WheeledJSON.py**

    Simulation of a vehicle completely specified through JSON files (in this case the same HMMWV vehicle as in demo_vehicle_HMMWV.py)

<br>

- **vehicle/demo_VEH_HMMWV9_YUP.py**

    Demonstration of setting a world frame with a vertical Y axis, using a reduced-order model of the HMMWV vehicle

<br>

- **vehicle/demo_VEH_CityBus.py**

    Bus vehicle demo showing a vehicle with double tires on the rear axle

<br>

- **vehicle/demo_VEH_UAZBUS.py**

    Demo of a van (UAZ) vehicle simulation  

<br>

- **vehicle/demo_VEH_MAN_10t.py** 

    Truck vehicle demo showing a vehicle with two steerable axles

<br>


## OpenCascade tutorials
	
- **cascade/demo_CAS_cascade.py**

    Use the pychrono.cascade module to create a shape with the OpenCascade kernel, then let it fall on the ground.
    - use the python.cascade module
    - create collisions with concave meshes
	- control collision tolerances (envelope, margin)

<br>

- **cascade/demo_CAS_stepfile.py**

    Use the pychrono.cascade module to load a STEP file saved from a CAD.
    - load a STEP file, saved from a 3D CAD.
	- fetch parts from the STEP document and conver into Chrono bodies.

<br>

- **cascade/demo_CAS_robot.py**

    Use pychrono.cascade to load a STEP file and create constraints between the bodies.
    - load a STEP file, saved from a 3D CAD.
	- fetch parts and recerences from the STEP document, and create joints between them.
	- assign a ChLinkTrajectory to a part



	
## Machine Learning Tutorials

- Using PyChrono with TensorFlow

    Ttrain a Neural Network in simulation to control actuators.
    - Build a learning model with Tensorflow
    - Build a training environment with Pychrono
    - Use simulation to train the Neural Network 
    - [Additional details](@ref tutorial_pychrono_demo_tensorflow)

<br>
