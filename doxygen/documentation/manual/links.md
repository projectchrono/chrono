
Links      {#links}
========


A Chrono body can be constrained in its relative motion with respect to a different body or ground. This is achieved by using ChLink classes.  
From the @ref chrono::ChLink class various sets of links are derived. The most noticeable are those derived from:
- @ref chrono::ChLinkMate : more efficient, however they do not implement limits and only few of them can have an imposed motion;
- @ref chrono::ChLinkLock : more general, the relative motion can be limited within boundaries, forces and relative displacements can easily be retrieved; it is possible to specify a constraint over points that are _moving_ respect to the body reference frame;
- @ref chrono::ChLinkMotor : ChLinkMate derived joints with included actuation.

Thus, some of the ChLinkMate and ChLinkLock derived classes may overlap. The latter being more flexible, the first more efficient. Because of this, the ChLinkMate version should be preferred, in general.

![](http://www.projectchrono.org/assets/manual/pic_ChLink.png)

Some general information are fundamental to effectively use these classes:
- links often refer to a pair @ref chrono:ChMarker, but such markers can be automatically added to bodies during link initialization; see below;
- each link has a reference/master frame; reaction forces and axis direction are computed with respect to this reference marker;
- it's worth to set the initial position of the markers/bodies to a feasible position;
- it's generally useful to set the solver parameters to finer values when constraints are present; see [Solvers](@ref solvers);

# ChLink Quick Reference
| ConDOF | Task | Description | Class |
| :-: | :--- | :-- | :-- |
| 6 | Fix | Fix position and rotation | @ref chrono::ChLinkMateFix <br> @ref chrono::ChLinkLockLock |
| 6\|3 | Bushing | Linear compliance + optional spherical joint | @ref chrono::ChLinkBushing |
| 5 | Revolute | Allows rotation along Z axis | @ref chrono::ChLinkLockRevolute <br> @ref chrono::ChLinkRevolute |
| 5 | Prismatic | Allows translation along Z axis  | @ref chrono::ChLinkLockPrismatic |
| 4 | Universal | Universal joint (along X and Y axes) | @ref chrono::ChLinkUniversal |
| 4 | Revolute+Prismatic | Allow translation along X axis and rotation along Z axis | @ref chrono::ChLinkLockRevolutePrismatic |
| 4 | Oldham | Oldham joint; does not fix axes position | @ref chrono::ChLinkLockOldham |
| 4 | Cylindrical | Z axes are collinear | @ref chrono::ChLinkMateCoaxial <br> @ref chrono::ChLinkLockCylindrical |
| 3 | Spherical | Fix translations | @ref chrono::ChLinkMateSpherical <br> @ref chrono::ChLinkLockSpherical |
| 3 | Planar | YZ planes are coplanar | @ref chrono::ChLinkLockPlanePlane <br> @ref chrono::ChLinkMatePlane |
| 3 | Aligned | Fix rotations | @ref chrono::ChLinkLockAlign |
| 2 | Revolute+Spherical | Fix distance to a X axis; free rotations | @ref chrono::ChLinkRevoluteSpherical <br> @ref chrono::ChLinkMateXdistance|
| 2 | Revolute+Align | Allow translation respect to a rotating frame | @ref chrono::ChLinkRevoluteTranslational |
| 2 | Point on a Plane | Z translations are blocked | @ref chrono::ChLinkLockPointPlane |
| 2 | Point on a Line | Point belongs to a given line; free to rotate | @ref chrono::ChLinkLockPointLine <br> @ref chrono::ChLinkPointSpline |
| 2 | Parallel | X axes are parallel | @ref chrono::ChLinkMateParallel <br> @ref chrono::ChLinkLockParallel |
| 2 | Orthogonal | X axes are orthogonal | @ref chrono::ChLinkMateOrthogonal <br> @ref chrono::ChLinkLockPerpend |
| 1 | Distance | Polar distance is fixed | @ref chrono::ChLinkDistance |
| 1 | Rack-Pinion | Couple rotation of the pinion Z axis with rack X axis | @ref chrono::ChLinkRackpinion |
| 1 | Pulley | Couple rotation over Z axes; pulley-specific features | @ref chrono::ChLinkPulley |
| 1 | Gear | Couple rotation over Z axes; gear-specific features | @ref chrono::ChLinkGear |
| 0 | Free | No constraints | @ref chrono::ChLinkLockFree |

Additionally, if a body has to be fixed to the global frame and reaction forces are of no interest, it is possible to use the @ref chrono::ChBody::SetFixed() method; this will also eliminate the state from the system.

## Actuators
| ConDOF | MotDOF | Task | Description | Class |
| :-: | :-: | :--- | :-- | :-- |
| 0\|3\|5 | 1 | Linear Actuator | Applies linear force\|speed\|position between frames; <br> optionally adds none\|prismatic\|spherical joints to its ends <br> can be paired with 1D @ref chrono::ChShaft | @ref chrono::ChLinkMotorLinear and derived |
| 0\|3\|5 | 1 | Rotating Actuator | Applies torque\|speed\|position between frames; <br> optionally adds none\|revolute\|cylindrical\|Oldham joints to its ends <br> can be paired with 1D @ref chrono::ChShaft | @ref chrono::ChLinkMotorRotation and derived |
| 0 | 1 | Linear Spring+Damper | Spring+Damper depending to frame distance; also with custom force | @ref chrono::ChLinkSpring <br> @ref chrono::ChLinkSpringCB |
| 0 | 1 | Rotational Spring+Damper | Spring+Damper depending to frame rotation along Z axis; with custom force | @ref chrono::ChLinkRotSpringCB |

Also @ref chrono::ChLinkLockLock can be used to impose a motion between bodies.

## Usage
In many cases there is no need to explicitly create two markers. Use in the case the ChLink::Initialize() 
function, passing the two bodies and the position of the constraint. This function will automatically create the two markers and it will add them to the bodies.

Alternatively, one can create the two markers by explicitly, add them to
the two bodies, and then call ```Initialize()``` by passing the two markers.

In general, the process involves the following steps:

1. Create one of the links of the @ref chrono::ChLink derived class (e.g. @ref chrono:ChLinkLockSpherical)
   ~~~{.cpp}
   auto mylink =  std::make_shared<ChLinkLockSpherical>();
   ~~~
2. Use ```mylink->Initialize(…)``` to connect two bodies; different links may accept different arguments. Refer to the documentation of the specific link for further information.
   ~~~{.cpp}
   mylink->Initialize(pendulumBody,        // the 1st body to connect
                      floorBody,      // the 2nd body to connect
                      ChCoordsys<>(ChVector<>(1, 0, 0),
                                   Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0))
                                   )   // master reference
                      );
   ~~~
3. Add the link to a ChSystem
   ~~~{.cpp}
   mphysicalSystem.Add(mylink);
   ~~~
4. Optional: set link properties

# Examples

See also:

- [demo_fourbar](@ref tutorial_demo_fourbar)
- [demo_suspension](@ref tutorial_demo_suspension)


