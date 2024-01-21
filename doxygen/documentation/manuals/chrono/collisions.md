
Collisions {#collisions}
================

Chrono can compute and simulate collisions between collidable objects - being them bodies or finite element meshes - allowing them to interact through contacts. 

The contact problem includes two different aspects:
- **collision detection**: finding the point pairs that are, or may come, in contact in the next future;
- **contact formulation**: defining the reaction forces between shapes in contact.

<h4> Contact Formulation </h4>
As described in the [ChSystem manual](@ref manual_ChSystem), Chrono can setup two different kind of systems, depending on the contact method:

- \ref chrono::ChSystemNSC "ChSystemNSC" implements **Non Smooth Contacts** (NSC):
  + contacts are treated has hard/stiff constraints;
  + since no artificial stiffening terms are introduced longer timesteps are allowed;
  + specific [VI solvers](simulation_system.html#solvers) are required;
  + only the more recent \ref chrono::ChSolverADMM "ChSolverADMM" can handle non-smooth contacts and finite elements together;

- \ref chrono::ChSystemSMC "ChSystemSMC" implements **SMooth Contacts** (SMC):
  + contacts reaction forces are computed based on the interpenetration of the bodies, multiplied by a compliance/stiffness term (penalty approach);
  + the contact stiffness is an artificial term, usually empirically related to the material stiffness;
  + shorter timsteps are required, especially for harder contact stiffness;
  + no VI solvers are required

Users should carefully consider which system fits better their needs, since timesteps might differ up to two or three orders of magnitude. On the contrary, the two system types are completely equivalente if no collisions are involved.

<h4> Collision Detection </h4>

In Chrono, two different collision systems are available:
+ \ref chrono::ChCollisionSystemBullet "ChCollisionSystemBullet": a customized version of [Bullet](https://github.com/bulletphysics/bullet3);
+ \ref chrono::ChCollisionSystemMulticore "ChCollisionSystemMulticore": an in-house multicore collision engine (enabled only if Thrust is available on the machine).

The choice between these two is made by calling \ref chrono::ChSystem::SetCollisionSystemType "ChSystem::SetCollisionSystemType" on \ref chrono::ChSystem "ChSystem". For example:

~~~{.cpp}
sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
~~~

Each collidable object carries a set of collision shapes and materials:
+  \ref chrono::ChBody "ChBody" holds its collision shapes through a \ref chrono::ChCollisionModel "ChCollisionModel" object;
+  \ref chrono::fea::ChMesh "ChMesh" holds its collision shapes through a \ref chrono::fea::ChContactSurface "ChContactSurface" object;

For finite elements the collision objects and properties are defined through the \ref chrono::fea::ChContactSurface "ChContactSurface" and inherited classes (e.g. \ref chrono::fea::ChContactSurfaceMesh "ChContactSurfaceMesh" and \ref chrono::fea::ChContactSurfaceNodeCloud "ChContactSurfaceNodeCloud") that can be set in this way:

~~~{.cpp}
auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceMesh>(mysurfmaterial);
my_mesh->AddContactSurface(mcontactsurf);
mcontactsurf->AddFacesFromBoundary(sphere_swept_thickness);  // do this after my_mesh->AddContactSurface
~~~

Collision objects for the \ref chrono::ChBody "ChBody" classes are discussed more in detail in the following sections.


# Collision Models and Shapes {#collision_models_shapes}

Each \ref chrono::ChBody "ChBody" (as well as any other object derived from \ref chrono::ChContactable "ChContactable") carries information about its collision properties through a \ref chrono::ChCollisionModel "ChCollisionModel" object. The _ChCollisionModel_ object may contain multiple \ref chrono::ChCollisionShape "ChCollisionShape"s, each of which carrying a geometric shape object together with its [Collision Surface Material](#collision_materials).

Each \ref chrono::ChBody "ChBody" may contain a:
+ \ref chrono::ChCollisionModel "ChCollisionModel", that contains (multiple):
 + \ref chrono::ChCollisionShape "ChCollisionShape", each of which containing:
   + \ref chrono::geometry::ChGeometry "ChGeometry"
   + \ref chrono::ChMaterialSurface "ChMaterialSurface"

A similar structure can be found also for the [Visualization System](@ref visualization_system).

Actually, as many other objects in Chrono, items within the *ChCollisionModel* are stored through pointers, so to allow to easily share them across different bodies. This is very useful especially for *ChMaterialSurface* objects.

In order to provide a collision shape to a _ChBody_ object:
~~~{.cpp}
auto body = chrono_types::make_shared<ChBody>();

auto collmat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

auto collshape = chrono_types::make_shared<ChCollisionShapeBox>(collmat, 0.1, 0.2, 0.3);

body->AddCollisionShape(collshape);
body->SetCollide(true);
~~~

Always remember to set a given collision system type in \ref chrono::ChSystem "ChSystem" e.g.
~~~{.cpp}
ChSystemNSC sys;
sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
~~~
additional settings can be tuned on the collision system (see [Collision Tolerances](#collision_tolerances)).


For the most common primitive objects, some auxiliary classes named _ChBodyEasyXXXX_ are offered: \ref chrono::ChBodyEasyBox "ChBodyEasyBox", \ref chrono::ChBodyEasyCylinder "ChBodyEasyCylinder", \ref chrono::ChBodyEasySphere "ChBodyEasySphere", ...  
These classes allow to create bodies together with their visualization and collision objects in a single command. However, please mind that, while the visualization is already added by default, the collision shape is not. To enable it the proper argument should be passed to the constructor e.g.
~~~{.cpp}
auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(0.05,      // radius size
                                                              1000,      // density
                                                              true,      // visualization?
                                                              true,      // collision?
                                                              collmat);  // contact material
~~~


![](http://www.projectchrono.org/assets/manual/pic_ChCollisionModel.png)


Please note that:

- collision shapes are defined with respect to the REF frame of the [body](@ref rigid_bodies)

- collision shapes can be chosen among the many classes inherited from \ref chrono::ChCollisionShape "ChCollisionShape":
  - \ref chrono::ChCollisionShapeBox "ChCollisionShapeBox"
  - \ref chrono::ChCollisionShapeSphere "ChCollisionShapeSphere"
  - ...

- multiple collision shapes can be added to a rigid body. However, compound shapes will slow down the collision detection phase of the simulation. In this context, there is no geometry simpler to handle than a sphere that is located at the center of mass of the rigid body. 

- handling of concave shapes falls back on a decomposition of the geometry in a union of convex shapes

- collision shapes and visualization assets do not need to match; e.g. one may have a detailed visualization shape for rendering purposes, yet the collision shape being much simpler to avoid a slowdown of the simulation.

- avoid shapes that are too thin, too flat or in general that lead to extreme size ratios
- 
- to retrieve the contact forces acting on a body: \ref chrono::ChBody::GetContactForce() "ChBody::GetContactForce()" and similarly for torques;


# Collision Families {#collision_families}

You can define _collision families_ for selective collisions. 
For example you may not want the object of family=2 to 
collide with any objects of family=4:

~~~{.cpp}
// default collision family is 0. Change it:
body_b->GetCollisionModel()->SetFamily(2);
body_b->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
~~~

Currently, Chrono allows up to 15 different collision families that can be involved in a simulation.


# Collision Surface Materials {#collision_materials}

The contact between shapes is influenced by various properties of the material (friction, damping, cohesion, restitution, ...). These properties are specified for each shape, by providing either a \ref chrono::ChMaterialSurfaceNSC "ChMaterialSurfaceNSC" or \ref chrono::ChMaterialSurfaceSMC "ChMaterialSurfaceSMC" object to the *ChCollisionShape*.

Chrono can handle two different contact formulations - Non Smooth Contacts (**NSC**) and SMooth Contacts (**SMC**) - each of which requires different _surface material_ types as well as different \ref chrono::ChSystem "ChSystem" types (see [ChSystem manual](@ref manual_ChSystem)).

<div class="ce-warning"> 
When a body is copied (copy-constructed) or Cloned, what gets copied of the ChMaterialSurface is the pointer, not the object itself. This means that two copied bodies will share the same instance of ChMaterialSurface, meaning that any change on the material of an object is reflected also to all the copied ones.
</div> 

# Collision Tolerances {#collision_tolerances}

Collision shapes have two (global) tolerances used by collision engine to create and delete contacts, namely an **envelope** and a **margin**:


- the outward **envelope** (whose thickness is modified through \ref chrono::ChCollisionModel::SetDefaultSuggestedEnvelope "ChCollisionModel::SetDefaultSuggestedEnvelope"): when two collisions shapes are *close* to contact a constraint equation should be immediately added to *ChSystem*. The *envelope* parameter tells *how close* these shapes (or contact pairs) should be to trigger the creation of the constraint equation. Please notice that adding a collision constraint equation **does not** mean that a reaction force is *already* present, but only that a **potential** collision might happen in the early future. This allows the system to catch the potential contact with due advance, thus reducing the risk of tunneling or unstable contacts.
  + if **too large**: too many contacts might be added too ahead of time, thus increasing the problem size (waste of memory and computational time);
  + if **too small**: the risk of interpenetration increases and objects might even pass through others if speeds are high or objects thickness is small (tunneling effect).

- an inward **margin** (whose thickness is modified through \ref chrono::ChCollisionModel::SetDefaultSuggestedMargin "ChCollisionModel::SetDefaultSuggestedMargin"): shape interpenetrations are usually source of numerical issues (integration errors, approximations, ill-conditioned initial conditions, etc). While a small amount of interpenetration is usually allowed, the collision detection algorithm needs to fall back to a slower collision detection algorithm when this interpenetration gets too high. This margin represents this threshold.
  + if **too large**: shapes appear too ''rounded'';
  + if **too small**: the slower algorithm kicks in too soon, thus leading to poor performance.

An additional setting is available for the Bullet engine only: 
- the **contact breaking threshold** (whose value is modified through \ref chrono::ChCollisionSystemBullet::SetContactBreakingThreshold "ChCollisionSystemBullet::SetContactBreakingThreshold"): Bullet keeps track of collision happened in the previous simulation frames. In order to purge the list a threshold is set: when the shapes move further away than this threshold they get removed from the list.

In order to improve the stability and performance of the collision detection, collision shapes are initially shrunk by an amount equal to the margin; they then get expanded back by the same amount thus introducing a sort of "rounding effect". In the picture below, the two blue shapes were added as sharp boxes, but after the treatment they end up having rounded corners whose rounding radius is the collision margin.

![](http://www.projectchrono.org/assets/manual/pic_margins.png)


An example of usage:

~~~{.cpp}
ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
ChCollisionModel::SetDefaultSuggestedMargin(0.0005);
ChCollisionSystemBullet::SetContactBreakingThreshold(0.001); // only for Bullet collision systems
~~~

<div class="ce-info">
Envelope and margin settings will affect collision shapes that are created **after** the call to the relative functions. 

*SetDefaultSuggestedEnvelope* and *SetDefaultSuggestedMargin* can be called 
multiple times, but once the collision model is populated with collision shapes
the envelopes and margins cannot be changed.
</div>

# Collision Callbacks {#collision_callbacks}
While the contact simulation is handled by Chrono in full autonomy, the user can still interact with the contact simulation through appropriate callbacks that gets automatically called by Chrono whenever a contact is found. The user is asked to create its own specific implementation of such callbacks by extending from \ref chrono::ChCollisionSystem::BroadphaseCallback "BroadphaseCallback" or \ref chrono::ChCollisionSystem::NarrowphaseCallback "NarrowphaseCallback" classes, and then register an instance of this new class through the \ref chrono::ChCollisionSystem::RegisterBroadphaseCallback "RegisterBroadphaseCallback" and \ref chrono::ChCollisionSystem::RegisterNarrowphaseCallback "RegisterNarrowphaseCallback" methods.

Please refer to [demo_MBS_callbackNSC.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_callbackNSC.cpp) and [demo_MBS_callbackSMC.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_callbackSMC.cpp) for further details.

# Examples
For further guidance, see:
- [demo_bricks.cpp](@ref tutorial_demo_bricks)
- [demo_collision.cpp](@ref tutorial_demo_collision)
- [demo_friction.cpp](@ref tutorial_demo_friction)