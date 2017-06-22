
Links      {#links}
========


A Chrono body can be constrained in its relative motion with respect to a different body or ground.
For instance, consider the case of a connecting rod and a piston, which are connected by a revolute joint. Or the rod and the crankshaft, which are also connected through a joint.
 
Connecting the relative motion of bodies is achieved in Chrono using ChLink classes.

![](http://www.projectchrono.org/assets/manual/pic_ChLink.png)

- Links are used to connect two ChBody

- There are many sub-classes of ChLink:
  - ChLinkLockSpherical
  - ChLinkLockRevolute
  - ChLinkLockLock
  - ChLinkLockPrismatic
  - ChLinkGears
  - ChLinkDistance
  - ...

- Most links use two ChMarker objects 

- Reactions and joint rotations/velocities are computed with respect to the master marker

- Motion is constrained with respect to the x,y,z axes of the frame of the master marker
  Example:
  - ChLinkLockRevolute: allows one DOF, a rotation about the z axis
  - ChLinkLockPrismatic: allows one DOF, a translation along the x axis
  - etc. 

In many cases there is no need to explicitly create two markers. Use in the case the ChLink::Initialize() 
function, passing the two bodies and the position of the constraint. This function will automatically create the two markers and it will add them to the bodies.

Alternatively, one can create the two markers by explicitly, add them to
the two bodies, and then call ```Initialize()``` by passing the two markers.

In general, the process involves the following steps:

- Create the link from the desired ChLinkXxxyyy class
- Use ```mylink->Initialize(…)``` to connect two bodies
- Add the link to a ChSystem
- Optional: set link properties

Example:

~~~{.cpp}
// 1- Create a constraint of ‘engine’ type, that constrains
// all x,y,z,Rx,Ry,Rz relative motions of marker 1 with respect 
// to 2; Rz will follow a prescribed rotation.
auto my_motor = std::make_shared<ChLinkEngine>();

// 2- Initialization: define the position of m2 in absolute space:
my_motor->Initialize( rotatingBody, // <- body 1
                      floorBody,    // <- body 2                       
                      ChCoordsys<>(ChVector<>(2,3,0), Q_from_AngAxis(CH_C_PI_2, VECT_X))  
                      );

// 3- Add the link to the system!
mphysicalSystem.AddLink(my_motor);

// 4- Set some properties:
my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>())
   mfun->Set_yconst(CH_C_PI_2); // speed w=90 deg/s
~~~


# Examples

See also:

- [demo_fourbar](@ref tutorial_demo_fourbar)
- [demo_suspension](@ref tutorial_demo_suspension)


