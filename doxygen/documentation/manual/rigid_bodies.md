
Rigid bodies      {#rigid_bodies}
============

Rigid bodies play an important role in Chrono as they represent parts of mechanisms.
For instance, an engine is made 
of rigid bodies such as rods, pistons, crankshaft, valves, etc. 

In some cases the rigid body assumption 
is an approximation of what happens in the real world. For instance the crankshaft of a car engine might have 
torsional and bending vibrations, but in many cases 
this can be neglected. If this flexibility attribute of the time evolution cannot be neglected, 
one should use the [FEA module](group__fea__module.html), which introduces flexible bodies at the cost of a more complex model definition/set up and longer run times.

# ChBody   {#manual_ChBody}

The most used type of rigid bodies is the ChBody.
See @ref chrono::ChBody for API details.

![](http://www.projectchrono.org/assets/manual/pic_ChBody.png)

- Rigid bodies inherit (in the C++ sense) from the ChFrameMoving classes and as such they have a position, rotation, velocity, and acceleration

- The position, speed, acceleration are that of the center of mass (COG) 

- They have a mass and an inertia tensor

- They can be connected by ChLink constraints

- They can participate in collisions


Creating/Setting up a ChBody object typically involves the following steps:

1. Create the ChBody and set its position and possibly its velocity along with its mass and inertia tensor properties
2. Add the body to a @ref chrono::ChSystem
3. Optional: add [collision shapes](@ref collision_shapes)
4. Optional: add [visualization assets](@ref visualization_assets)

The following example illustrates Steps 1 and 2:

~~~{.cpp}
// Create a body – use shared pointer
auto body_b = std::make_shared<ChBody>();

// Set initial position and velocity of the COG of body,
// using the same syntax used for ChFrameMoving
body_b->SetPos( ChVector<>(0.2,0.4,2) );
body_b->SetPos_dt( ChVector<>(0.1,0,0) );

// Set mass and inertia tensor attributes
body_b->SetMass(10);
body_b->SetInertiaXX( ChVector<>(4,4,4) );

// Here's how one can indicate that the body is fixed to ground:
body_b->SetBodyFixed(true);

// Finally, add the newly created to the system being simulated
my_system.Add(body_b);
~~~

# ChBodyAuxRef   {#manual_ChBodyAuxRef}

This is a special type of rigid body that has an auxiliary 
frame that is not necessarily coincident with the COG frame.

See @ref chrono::ChBodyAuxRef for API details.
 
![](http://www.projectchrono.org/assets/manual/pic_ChBodyAuxRef.png)

Remarks:
- Inherits (in the C++ sense) from ChBody
- Handy when using a COG reference frame is cumbersome and instead another reference is preferred, for instance, coming from CAD
- Calls such as mybody->GetPos(), mybody->GetRot(), mybody->GetPos_dt(), mybody->GetWvel(), etc., will report the kinematic quantities ''for the COG frame''. If you need those of the REF, do mybody->GetFrame_REF_to_abs().GetPos(), etc.
- The REF frame is used for
  - [collision shapes](@ref collision_shapes)
  - [visualization shapes](@ref visualization_assets)

The following is a short example on how to set the position 
of the body using the REF frame:

~~~{.cpp}
// Create a body with an auxiliary reference frame
auto body_b = std::make_shared<ChBodyAuxRef>();

// Set position of COG with respect to the reference frame
body_b->SetFrame_COG_to_REF(X_bcogref);
// Set position of the reference frame in absolute space
body_b->SetFrame_REF_to_abs(X_bref);
// Position of COG in absolute space is simply body_b
// ex. body_b->GetPos(),  body_b->GetRot(),  etc.
pos_vec = body_b->GetPos();
~~~


# Other bodies   {#manual_otherbodies}

There are other classes that inherit from ChBody. They are specializations 
that introduce additional features. The most relevant classes in this context are:


## Conveyor belt   

The ChConveyor is a body that has a rectangular collision surface 
used in the simulation of a conveyor belt.

See @ref chrono::ChConveyor for API details.


## Basic shapes

- @ref chrono::ChBodyEasySphere,
- @ref chrono::ChBodyEasyCylinder,
- @ref chrono::ChBodyEasyBox,
- @ref chrono::ChBodyEasyConvexHull,
- @ref chrono::ChBodyEasyClusterOfSpheres,

These define ready/easy-to-use bodies that simplify the definition of its attributes when the body is a simple shape such as a box, a sphere, etc.
When creating one of these objects, one automatically gets:

- The mass and inertia tensor are computed from the geometry, given the density
- Optional: a visualization asset showing the shape is added automatically
- Optional: a collision shape is added automatically



# Examples
See:
- [demo_crank](@ref tutorial_demo_crank)
- [demo_buildsystem](@ref tutorial_demo_buildsystem)
- [demo_conveyor](@ref tutorial_demo_conveyor)
