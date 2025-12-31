Steering mechanism models {#wheeled_steering}
=============================================

\tableofcontents

The base class [ChSteering](@ref chrono::vehicle::ChSteering) imposes that any derived steering mechanism class (a steering mechanism template) provide a steering link body to which a steerable suspension can be sonnected (usually through the suspension's tierods).

A derived steering mechanism type defines the bodies, joints, force elements, and topology of a particular type of steering mechanism. All locations are assumed to be provided with respect to a mechanism reference frame (a derived steering mechanism type is free to pick the location and orientation of this frame).

A steering mechanism assembly is attached to a vehicle's chassis by specifying the location and orientation of the mechanism assembly reference frame with respect to the chassis reference frame (see the definition of the [ISO reference frame](@ref vehicle_ISO_frame)).

A wheeled vehicle may have multiple steering mechanisms, each associated with a different steerable vehicle axle. Similalry, a single steering mechanism may be connected to multiple steerable vehicle axles.

## Pitman arm {#wheeled_steering_pitman}

This steering mechanism is a four-bar linkage with the steering link body connected to the chassis via the Pitman arm and an idler arm. The Pitman arm body is connected through an universal joint to the steering link and a revolute joint to the chassis. The driver steering input is used to control the angle of the revolute joint.  In the Chrono::Vehicle Pitman arm template, the idler arm is modeled using a composite revolute-spherical joint.

See [ChPitmanArm](@ref chrono::vehicle::ChPitmanArm) and [PitmanArm](@ref chrono::vehicle::PitmanArm).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/PitmanArm_bodies.png" width="600" />

The topology of this steering mechanism template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/PitmanArm_topology.png" width="800" />

The hardpoints are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/PitmanArm_points.png" width="600" />

A sample JSON file with the specification of a PitmanArm steering mechanism is:
\include "data/vehicle/hmmwv/steering/HMMWV_PitmanArm.json"

## Rack-pinion {#wheeled_steering_rack_pinion}

The Chrono::Vehicle rack-pinion steering template is a kinematic model of a rack and pinion steering mechanism.  The steering link body is connected through a prismatic joint to the chassis. The rack displacement is calculated as:
\f[
d = r (\alpha_{max} s)
\f]
where \f$ r \f$ is the pinion radius, \f$ \alpha_{max} \f$ is the maximum pinion angle, and \f$ s \in [-1,1] \f$ is the driver steering input. This displacement is used to control the translation of the steering link.

See [ChRackPinion](@ref chrono::vehicle::ChRackPinion) and [RackPinion](@ref chrono::vehicle::RackPinion).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RackPinion_bodies.png" width="600" />

The topology of this steering mechanism template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RackPinion_topology.png" width="800" />

The hardpoints are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RackPinion_points.png" width="600" />

A sample JSON file with the specification of a RackPinion steering mechanism is:
\include "data/vehicle/hmmwv/steering/HMMWV_RackPinion.json"


## Rotary arm {#wheeled_steering_rotary_arm}

The rotary arm steering is a simple lever arm that rotates around an axis. It works with solid bellcrank axles and solid toebar axles. It is often used as steering system for trucks, farm tractors and combine harvesters.

See [ChRotaryArm](@ref chrono::vehicle::ChRotaryArm) and [RotaryArm](@ref chrono::vehicle::RotaryArm).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RotaryArm_bodies.png" width="600" />

The topology of this steering mechanism template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RotaryArm_topology.png" width="800" />

The hardpoints are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RotaryArm_points.png" width="600" />

A sample JSON file with the specification of a RotaryArm steering mechanism is:
\include "data/vehicle/uaz/steering/UAZBUS_RotaryArm.json"
