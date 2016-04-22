
Links      {#links}
========


Rigid bodies can be constrained: think about the case of a rod 
and a piston, which are connected by a revolute joint, 
as well as the rod is connected to the crankshaft, and so on. 

This is achieved by using the ChLink classes. 
Look at the API documentation for ChLink and its many sub-classes.

![](pic_ChLink.png)

- Links are used to connect two ChBody

- There are many sub-classes of ChLink:
  - ChLinkLockSpherical
  - ChLinkLockRevolute
  - ChLinkLockLock
  - ChLinkLockPrismatic
  - ChLinkGears
  - ChLinkDistance
  - ...

- Most links use two ChMarker as references

- The marker m2 (in body n.2) is the master marker

- Reactions and joint rotations/speeds etc. are computed respect to the master marker

- Motion is constrained respect to the x,y,z axes of the frame of the master marker, ex:
  - ChLinkLockRevolute: allowed DOF on z axis rotation
  - ChLinkLockPrismatic: allowed DOF on x axis translation
  - etc. 

In most cases, one does not need to create explicitly the two markers, 
because you just have to create a ChLink and use the ChLink::Initialize() 
function, passing the two bodies and the position of the constraint; 
such function will automatically create the two markers and it will add them to the bodies.

Alternatively you can create the two markers by yourself and add them to
the two bodies, then call ```Initialize()``` by passing the two markers.

In general the process is based on these steps:

- Create the link from the desired ChLinkXxxyyy class
- Use ```mylink->Initialize(…)``` to connect two bodies
- Add the link to a ChSystem
- Optional: set link properties

See the following example:

~~~{.cpp}
// 1- Create a constraint of ‘engine’ type, that constrains
// all x,y,z,Rx,Ry,Rz relative motions of marker 1 respect 
// to 2, and Rz will follow a prescribed rotation.
ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);

// 2- Initialization: define the position of m2 in absolute space:
my_motor->Initialize( rotatingBody, // <- body 1
                      floorBody,    // <- body 2                       
                      ChCoordsys<>(ChVector<>(2,3,0), Q_from_AngAxis(CH_C_PI_2, VECT_X))  
                      );

// 3- Add the link to the system!
mphysicalSystem.AddLink(my_motor);

// 4- Set some properties:
my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
if (ChSharedPtr<ChFunction_Const> mfun = my_motor->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
   mfun->Set_yconst(CH_C_PI/2.0); // speed w=90°/s
~~~


# Examples

Among the many examples, look at:

- demo_fourbar.cpp
- demo_suspension.cpp


