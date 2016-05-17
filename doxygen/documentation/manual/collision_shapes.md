
Collision shapes       {#collision_shapes}
================


Collision shapes can be added optionally to ChBody objects. 
This allows you to have rigid bodies that touch each other and collide in the simulation.


# Collision models     {#collision_models}

Each body contains a ChCollisionModel.
This can be used to define the collision shapes.

![](pic_ChCollisionModel.png)

- Collision shapes are defined respect to the REF frame of the [body](@ref rigid_bodies)

- Spheres, boxes, cylinders, convex hulls, ellipsoids, compounds,...

- shapes are unlimited in number. However, compound shapes simulate slower 
  than single shapes, and the fastest is the COG-centered sphere.

- Concave shapes: decompose in compounds of convex shapes

- Hint: for simple ready-to-use bodies that already contain 
  collision shapes, use  ChBodyEasySphere, ChBodyEasyBox, etc. 
  (see [body manual pages](@ref manual_otherbodies)

- Collision shapes and visualization assets does not need to match; 
  ex. you may have a detailed visualization shape for rendering purposes, 
  but the collision shape is much simpler to speedup simulation.

- avoid shapes that are too thin, too flat or in general that 
  lead to extreme size ratios 


These are the typical steps to setup collision, after you created a ChBody:

~~~{.cpp}
body_b->GetCollisionModel()->ClearModel();
body_b->GetCollisionModel()->AddSphere(myradius);
... 
body_b->GetCollisionModel()->BuildModel();
body_b->SetCollide(true);
~~~


# Collision families   {#collision_families}

You can define _collision families_ for selective collisions. 
For example you may want that one object of family=2, 
does not collide with all objects of family=4: 

~~~{.cpp}
// default collision family is 0. Change it:
body_b->GetCollisionModel()->SetFamily(2); 
body_b->SetFamilyMaskNoCollisionWithFamily(4);
~~~

There is a max.number of 15 collision families that you can use.


# Collision materials   {#collision_materials}

When you add collision shapes to a body, you may need to 
define the friction coefficient of the surface. 
The same for other collision-specific properties 
such as rolling friction, coeffcient of restitution etc. 

This can be done in two ways.

**Easy** but memory-consuming approach:

~~~{.cpp}
body_b->SetFriction(0.4f);
body_b->SetRollingFriction(0.001f);
~~~

**Advanced** approach with a shared material. 
This means that you create a ChSharedMaterial and you use it for one or more bodies:

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

Each collision model has two tolerances, that are used by the 
collision engine to create and delete constact:

- an outward safe **envelope**. This represents the volume where potential 
  contacts are searched. This means that contacts are feed into the solver even _before_ 
  the objects come into true contact, but in fact this is needed by the numerical 
  schemes because they must know the existence of contacts a bit in advance 
  (then, if not touching, their contact forces will be computed as zero by the solver). 
  If this contacts were sent to the solver when objects are already interpenetrating, 
  the motion could be shaky and unprecise.

- an inward **margin**. It might happen that qbject might interpenetrate 
  even if considered rigid, because of numerical integration errors, 
  approximations, or because of badly designed initial conditions. 
  If this happens, it is not a problem because the collision detection 
  algorithm can support interpenetration up to this _margin_. 
  When this margin is crossed, a different type of collision detection 
  is used, with a much slower algorithm. 

In fact the fast collision algorithm must run between 'shrunk' versions of the 
shapes, and the original shape is recovered later by offsetting the collision 
points outward. Hence the rounding effect. In the picture below, the two blue 
shapes were added as sharp boxes, but the effect from the point of view of 
collision is like having boxes with rounded corners, whose rounding radius 
is the collision margin.

![](pic_margins.png)

This said, one might think that it would be optimal to have large outward 
envelope and large inward margin. However there are drawbacks:

- Too large collision envelope: 
  - too many potential contacts, 
  - high CPU time, 
  - high waste of RAM

- Too small collision envelope: 
  - risk of tunnelling effects, 
  - unstable, shaky simulation of stacked objects

- Too large collision margin: 
  - shapes are way too ''rounded''. 

- Too small collision margin: 
  - when interpenetration occurs beyond this value, 
    an inefficient algorithm is used

To change these margins, see this example:

~~~{.cpp}
ChCollisionModel::SetDefaultSuggestedEnvelope(0.001); 
ChCollisionModel::SetDefaultSuggestedMargin  (0.0005); 
~~~

Note that these settings will affect collision models that are created 
**after** these changes. 

You can call SetDefaultSuggestedEnvelope and SetDefaultSuggestedMargin 
multiple times, maybe at the beginning of your program, 
or maybe once per collision model creation; however, 
once you populated the collision model with collision shapes, 
there is no way to change the margins afterward.

Finally, there is also a **contact breaking threshold**, 
that is a global tolerance for all models, and it can be set as in this example:

~~~{.cpp}
ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);
~~~

This represents the maximum distance of two persistent contact 
points that is accepted before they are removed by the collision 
detection engine. This because the Bullet collision algorithm keeps 
contact point persistent from one simulation frame to the other 
if two bodies are sliding, and it must know when to cut the 
contact when they separate too much.


# Examples

Among the many examples, look at:
- demo_bricks.cpp
- demo_collision.cpp
- demo_friction.cpp





