
Rigid bodies      {#rigid_bodies}
============

Rigid bodies are the most important objects in Chrono::Engine. 
They represent parts of mechanisms, for example an engine is made 
of rigid bodies such as rods, pistons, crankshaft, valves, etc. 

Note that in some cases the rigid body assumption 
is an approximation of what happens in a real world, 
for instance the crankshaft of a car engine might have 
torsional and flexional vibrations, but in many cases 
this can be neglected (if flexibility cannot be neglected, 
one could use the [FEA module](@ref FEA_module) 
that introduces flexible parts, but at the cost of more 
complication and slower performance.)

# ChBody   {#manual_ChBody}

The most used type of rigid bodies is the ChBody.

See @ref chrono::ChBody for API details.

![](pic_ChBody.png)

- Rigid bodies inherit ChFrameMoving features (position, rotation, velocity, acceleration, etc.)

- The position, speed, acceleration are those of the center of mass (COG) 

- They contain a mass and a tensor of inertia

- They can be connected by ChLink constraints

- They can participate to collisions


When you create a ChBody, basically you perform those steps:

- Create the ChBody and set position/mass properties
- Add the body to a @ref chrono::ChSystem
- Optional: add [collision shapes](@ref collision_shapes)
- Optional: add [visualization assets](@ref visualization_assets)

The following example shows steps 1 and 2:

~~~{.cpp}
// Create a body – use shared pointer!
ChSharedPtr<ChBody> body_b(new ChBody);

// Set initial position & speed of the COG of body,
// using the same syntax used for ChFrameMoving
body_b->SetPos( ChVector<>(0.2,0.4,2) );
body_b->SetPos_dt( ChVector<>(0.1,0,0) );

// Set mass and inertia tensor
body_b->SetMass(10);
body_b->SetInertiaXX( ChVector<>(4,4,4) );

// If body is fixed to ground, use this:
body_b->SetBodyFixed(true);

// Finally do not forget this
my_system.Add(body_b);
~~~

# ChBodyAuxRef   {#manual_ChBodyAuxRef}

This is a special type of rigid body that has an auxiliary 
frame that does not match necessarily with the COG frame.

See @ref chrono::ChBodyAuxRef for API details.
 
![](pic_ChBodyAuxRef.png)

- Inherited from ChBody
- Used when the COG is not practical as the main reference for the body, and another reference is preferred, ex. from a CAD, so it adds an auxiliary REF frame
- Note that the straight mybody->GetPos(), mybody->GetRot(), mybody->GetPos_dt(), mybody->GetWvel() etc. will give you the kinematic quantities ''for the GOG frame''. If you need those of the REF, do mybody->GetFrame_REF_to_abs().GetPos(), etc.
- The REF frame is used for
  - [collision shapes](@ref collision_shapes)
  - [visualization shapes](@ref visualization_assets)

The following is a short example on how to set the position 
of the body using the REF frame:

~~~{.cpp}
// Create a body with aux.reference
ChSharedPtr<ChBodyAuxRef> body_b(new ChBodyAuxRef);

// Set position of COG respect to reference
body_b->SetFrame_COG_to_REF(X_bcogref);
// Set position of reference in absolute space
body_b->SetFrame_REF_to_abs(X_bref);
// Position of COG in absolute space is simply body_b
// ex. body_b->GetPos()  body_b->GetRot()  etc.
~~~


# Other bodies   {#manual_otherbodies}

There are other classes inherited from ChBody. Those are specializations 
that introduce additional features. In the following we list the most relevant.


## Conveyor belt   

The ChConveyor is a body that has a rectangular collision surface 
that simulates a conveyor belt.

See @ref chrono::ChConveyor for API details.


## Basic shapes

- @ref chrono::ChBodyEasySphere,
- @ref chrono::ChBodyEasyCylinder,
- @ref chrono::ChBodyEasyBox,
- @ref chrono::ChBodyEasyConvexHull,
- @ref chrono::ChBodyEasyClusterOfSpheres,

Those are ready-to-use bodies that simplify the definition of 
body properties in case the body is a simple shape such as a box, a sphere, etc.
In fact when you create one of these, you automatically get the following:

- the mass and inertia tensor are computed from the geometry, given the density
- optional: a visualization asset showing the shape is added automatically
- optional: a collision shape is added automatically



# Examples

Among the many examples, look at:
- demo_crank.cpp
- demo_buildsystem.cpp
- demo_conveyor.cpp






