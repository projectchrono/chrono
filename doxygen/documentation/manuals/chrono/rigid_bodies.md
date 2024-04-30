
Rigid Bodies      {#rigid_bodies}
============

@ref chrono::ChBody "ChBody" is the base class for all the rigid bodies in Chrono and, as such, it carries a mass and a rotational inertia and has the capability to move in the 3D space.

![](http://www.projectchrono.org/assets/manual/pic_ChBody.png)

Its location and rotation, as well as their derivatives, **always refer to the Center of Mass (COM)** while visual and collision shapes, on the contrary, are attached to a **secondary reference frame** (`Ref`).  
Please mind that in @ref chrono::ChBody "ChBody" base class the COM and `Ref` always coincide, while only in @ref chrono::ChBodyAuxRef "ChBodyAuxRef" and derived classes the `Ref` frame might be placed elsewhere.

Methods like @ref chrono::ChBody::GetPos() "GetPos()" / @ref chrono::ChBody::SetPos() "SetPos()", @ref chrono::ChBody::GetRot() "GetRot()" / @ref chrono::ChBody::SetRot() "SetRot()" and their derivatives always refer to COM frame.  
The auxiliary frame is handled through dedicated methods of the kind `[Get|Set]FrameRefTo____` | `[Get|Set]Frame____ToRef` wherever it makes sense.

![](http://www.projectchrono.org/assets/manual/pic_ChBodyAuxRef.png)

Rigid bodies can also:
- be involved in collisions, if a collision model is provided (see [collisions](@ref collisions)) and the collision is enabled on the body;
- be visualized, if a visual model is provided and a proper visualization system is available (see [visualization](@ref visualization_system));
- be constrained by means of ChLink objects (see [links](@ref links));
- be loaded by ChLoad objects (see [loads](@ref loads));
- be used in coordinate transformation, being themselves inherited from ChFrameMoving;

@ref chrono::ChBody "ChBody" and @ref chrono::ChBodyAuxRef "ChBodyAuxRef" bodies **do not come with any visual or collision model**, thus requiring the user to specify them as well as providing valid mass and inertia parameters.  
However, in the case the rigid body could be described **through a primitive shape**, a set classes of the type `ChBodyEasy` can simplify this task, by calculating mass, inertia and by optionally creating a visual and collision shape:
- @ref chrono::ChBodyEasySphere "ChBodyEasySphere"
- @ref chrono::ChBodyEasyCylinder "ChBodyEasyCylinder"
- @ref chrono::ChBodyEasyBox "ChBodyEasyBox"
- @ref chrono::ChBodyEasyConvexHull "ChBodyEasyConvexHull"
- @ref chrono::ChBodyEasyClusterOfSpheres "ChBodyEasyClusterOfSpheres"

Rigid bodies are not the only option. Chrono can simulate also flexible finite-elements bodies. Please refer to [FEA manual](@ref manual_fea) for a description of the FEA capabilities.

## Usage

Creating/Setting up a ChBody object typically involves the following steps:

1. Create the rigid body; 
   ~~~{.cpp}
      auto mybody = chrono_types::make_shared<ChBody>();
      mybody->SetMass(10);
      mybody->SetInertiaXX( ChVector3d(4,4,4) );
      mybody->SetPos( ChVector3d(0.2,0.4,2) );
      mybody->SetPosDt( ChVector3d(0.1,0,0) );
      my_system.Add(mybody);
   ~~~
2. Optional: add [visual shapes](@ref visualization_system)
   ~~~{.cpp}
      auto visshape = chrono_types::make_shared<ChVisualShapeBox>(20, 1, 20);
      visshape->SetColor(ChColor(0.2f, 0.3f, 1.0f));
      mybody->AddVisualShape(visshape, ChFramed(ChVector3d(0, -1, 0), QUNIT));
   ~~~
4. Optional: add [collision shapes and material](@ref collisions)
   ~~~{.cpp}
      auto collmat = chrono_types::make_shared<ChContactMaterialNSC>();
      auto collshape = chrono_types::make_shared<ChCollisionShapeBox>(collmat, 0.1, 0.2, 0.3);
      mybody->AddCollisionShape(collshape);
      mybody->EnableCollision(true);
   ~~~

Please refer to the dedicate pages for [collision](@ref collisions) and [visualization](@ref visualization_system) to complete the configuration of the system.


For bodies of the class `ChBodyEasy` the constructor is richer:
~~~{.cpp}
auto mySphere = chrono_types::make_shared<ChBodyEasySphere>(4,      // radius
                                                            8000,   // density
                                                            true,   // visualization enabled
                                                            true,   // collision enabled
                                                            collmat // collision material
                                                         );  
my_system.Add(mySphere);
~~~

# Demos
See:
- [demo_CH_buildsystem](https://github.com/projectchrono/chrono/blob/main/src/demos/core/demo_CH_buildsystem.cpp)
- [demo_MBS_crank](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_crank.cpp)
- [demo_MBS_conveyor](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_conveyor.cpp)
