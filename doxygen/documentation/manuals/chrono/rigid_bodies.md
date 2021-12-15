
Rigid bodies      {#rigid_bodies}
============

Rigid bodies play an important role in Chrono as they represent parts of mechanisms.  
You can add a rigid body using different methods:
 - [using chrono::ChBody directly](@ref manual_ChBody); in this case, the body will not come with any visualization asset nor collision shape: you will need to add them in a second step;
 - [using 'Easy' bodies](@ref manual_easybodies); if you need basic shapes (sphere, cylinder, box, convex hull, cluster of spheres) this is the way to go; the visualization and collision shapes are (optionally) added automatically;

Rigid bodies are not the only option. Chrono can simulate also flexible finite-elements bodies. Please refer to [FEA manual](@ref manual_fea) for a description of the FEA capabilities.

# ChBody   {#manual_ChBody}


The most used type of rigid bodies is the ChBody.
See @ref chrono::ChBody for API details.

![](http://www.projectchrono.org/assets/manual/pic_ChBody.png)

- Rigid bodies inherit (in the C++ sense) from the @ref chrono::ChFrameMoving classes and as such they have a position, rotation, velocity, and acceleration

- The position, speed, acceleration are that of the center of mass (COG) 

- They have a mass and an inertia tensor

- They can be connected by @ref chrono::ChLink constraints

- They can participate in collisions


Creating/Setting up a ChBody object typically involves the following steps:

1. Create the ChBody; 
   ~~~{.cpp}
   auto body_b = std::make_shared<ChBody>();
   ~~~
2. Set its mass and inertia tensor properties
   ~~~{.cpp}
   body_b->SetMass(10);
   body_b->SetInertiaXX( ChVector<>(4,4,4) );
   ~~~
3. Set its position and its velocity, if needed
   ~~~{.cpp}
   body_b->SetPos( ChVector<>(0.2,0.4,2) );
   body_b->SetPos_dt( ChVector<>(0.1,0,0) );
   ~~~
4. Add the body to a @ref chrono::ChSystem
   ~~~{.cpp}
   my_system.Add(body_b);
   ~~~
5. Optional: add [collision shapes](@ref collision_shapes)
6. Optional: add [visualization assets](@ref visualization_assets)

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

# 'Easy' bodies   {#manual_easybodies}
Chrono provides some classes that greatly facilitate the creation of bodies with basic shapes:

- @ref chrono::ChBodyEasySphere,
- @ref chrono::ChBodyEasyCylinder,
- @ref chrono::ChBodyEasyBox,
- @ref chrono::ChBodyEasyConvexHull,
- @ref chrono::ChBodyEasyClusterOfSpheres,

The syntax differs among the different classes, but basically is of the type

~~~{.cpp}
auto mySphere = std::make_shared<ChBodyEasySphere>(4,      // radius
                                                   8000,   // density
                                                   true,   // collision enabled
                                                   true);  // visualization enabled
system.Add(mySphere);
~~~

For these objects:

- The mass and inertia tensor are computed from the geometry, given the density;
- Optional: a visualization asset showing the shape is added automatically;
- Optional: a collision shape is added automatically;
  
Since they inherit from @ref chrono::ChBody, setting the position, velocity and any other operation can be done as in ChBody.


# Other bodies   {#manual_otherbodies}

There are other classes that inherit from ChBody. They are specializations 
that introduce additional features. The most relevant classes in this context are:


## Conveyor belt   

The ChConveyor is a body that has a rectangular collision surface 
used in the simulation of a conveyor belt.

See @ref chrono::ChConveyor for API details.

# Examples
See:
- [demo_crank](@ref tutorial_demo_crank)
- [demo_buildsystem](@ref tutorial_demo_buildsystem)
- [demo_conveyor](@ref tutorial_demo_conveyor)
