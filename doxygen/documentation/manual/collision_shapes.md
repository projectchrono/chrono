
Collision shapes       {#collision_shapes}
================


Collision shapes can be added optionally to ChBody objects. Having collision shapes attached to bodies enable them to interact during simulation with other bodies through friction and contact.


# Collision models     {#collision_models}

Each body contains a ChCollisionModel.
This can be used to define the collision shapes.

![](http://www.projectchrono.org/assets/manual/pic_ChCollisionModel.png)

- Collision shapes are defined with respect to the REF frame of the [body](@ref rigid_bodies)

- Examples of shapes: spheres, boxes, cylinders, convex hulls, ellipsoids, compounds, etc.

- One can add multiple collision shapes to a rigid body. However, compound shapes will slow down the collision detection phase of the simulation. In this context, there is no geometry simpler to handle than a sphere that is located at the center of mass of the rigid body. 

- Handling of concave shapes falls back on a decomposition of the geometry in a union of convex shapes

- For simple ready-to-use bodies that already contain 
  collision shapes, use  ChBodyEasySphere, ChBodyEasyBox, etc. 
  (see [body manual pages](@ref manual_otherbodies)

- Collision shapes and visualization assets do not need to match; 
  e. g. one may have a detailed visualization shape for rendering purposes, 
  yet the collision shape is much simpler to avoid a slowdown of the simulation.

- Avoid shapes that are too thin, too flat or in general that 
  lead to extreme size ratios 


Steps to set up collision after you create a ChBody:

~~~{.cpp}
body_b->GetCollisionModel()->ClearModel();
body_b->GetCollisionModel()->AddSphere(myradius);
... 
body_b->GetCollisionModel()->BuildModel();
body_b->SetCollide(true);
~~~


# Collision families   {#collision_families}

You can define _collision families_ for selective collisions. 
For example you may not want the object of family=2 to 
collide with any objects of family=4:

~~~{.cpp}
// default collision family is 0. Change it:
body_b->GetCollisionModel()->SetFamily(2); 
body_b->SetFamilyMaskNoCollisionWithFamily(4);
~~~

Currently, Chrono allows up to 15 different collision families that can be involved in a simulation.


# Collision materials   {#collision_materials}

Adding collision shapes to a body typically requires the user to define the friction coefficient of the surface. This observation also holds for other collision-specific properties such as rolling friction, coefficient of restitution, etc. This can be done in two ways.

**Easy** (Memory consuming):

~~~{.cpp}
body_b->SetFriction(0.4f);
body_b->SetRollingFriction(0.001f);
~~~

**Advanced** (Shared material). 
A ChSharedMaterial is created and subsequently used for one or more bodies:

~~~{.cpp}
// Create a surface material and change properties:
ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
mat->SetFriction(0.4f);
mat->SetRollingFriction(0.001f);

// Assign surface material to body/bodies:
body_b->SetSurfaceMaterial(mat); 
body_c->SetSurfaceMaterial(mat);
body_d->SetSurfaceMaterial(mat);
~~~


# Collision tolerances     {#collision_tolerances}

Each collision model has two tolerance used by collision engine to create and delete contacts:

- An outward safe **envelope**. 
  This represents the volume where potential contacts are searched. In other words, for the sake of collision detection the body becomes puffy, or inflated, by a small amount that is user controlled. Rationale: although at the beginning of a time step two bodies might not be in contact, during the duration of one integration time step they might come in contact. This envelope provisions for this possibility. More specifically, this envelope is needed by the numerical schemes since they must anticipate contacts ahead of time. If these contacts were sent to the solver when objects are already interpenetrating, the motion could be shaky and less accurate. Having a too large envelope will produce a lot of false positives (in terms of contacts), which will slow down both the collision detection and the solution stage. The latter is due to the fact that a lot of contacts will be solved only to produce a ``zero contact force'' answer.

- An inward **margin**. 
  Two collision shapes belonging to different bodies might interpenetrate
  even if considered rigid owing to numerical integration errors, 
  collision detection approximations, ill specified initial conditions, etc. 
  The 'go-to' collision detection algorithm can support interpenetration up to this _margin_. 
  When this margin is crossed, the solver falls back on a slower collision detection algorithm.

The fast collision algorithm must run between 'shrunk' versions of the 
shapes, and the original shape is recovered later by offsetting the collision 
points outward. Hence the rounding effect. In the picture below, the two blue 
shapes were added as sharp boxes, but the effect from the point of view of 
collision is like having boxes with rounded corners whose rounding radius 
is the collision margin.

![](http://www.projectchrono.org/assets/manual/pic_margins.png)


Drawbacks to poor envelope and/or margin choices:

- Collision envelope too large: 
  - Many potential contacts
  - High CPU time
  - Waste of memory

- Collision envelope too small: 
  - Risk of tunnelling effects
  - Unstable, shaky simulation, particularly when dealing with stacked objects

- Collision margin too large: 
  - Shapes appear too ''rounded'' 

- Collision margin too small: 
  - When interpenetration occurs beyond this value, 
    an inefficient algorithm is used

Setting the value of the envelope/margin:

~~~{.cpp}
ChCollisionModel::SetDefaultSuggestedEnvelope(0.001); 
ChCollisionModel::SetDefaultSuggestedMargin  (0.0005); 
~~~

<div class="ce-info">
These settings will affect collision shapes that are created 
**after** these function calls. 

SetDefaultSuggestedEnvelope and SetDefaultSuggestedMargin can be called 
multiple times, but once the collision model is populated with collision shapes
the envelopes and margins cannot be changed.
</div>


Finally, there is also a **contact breaking threshold**, which is a global tolerance for all models. It can be set as in this example:

~~~{.cpp}
ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);
~~~

The contact breaking threshold represents the maximum distance between two collision shapes where were in contact before the contact is considered as non-existent. This is due to the fact that Chrono relies in many instances on the Bullet collision algorithms and Bullet keeps 
contact points persistent from one simulation frame to the next. This threshold value instructs Bullet when to severe the contact between the two collision shapes.


# Examples
For further guidance, see:
- [demo_bricks.cpp](@ref tutorial_demo_bricks)
- [demo_collision.cpp](@ref tutorial_demo_collision)
- [demo_friction.cpp](@ref tutorial_demo_friction)