Suspension models {#wheeled_suspension}
=======================================


\tableofcontents

A suspension subsystem is a model of one axle of a wheeld vehicle. The base class [ChSuspension](@ref chrono::vehicle::ChSuspension) imposes that any derived suspension class (a suspension template) provide two wheel spindles (left and right) each connected through a revolute joint to some part of that type of suspension, and two spindle axles (elements of [ChShaft](@ref chrono::ChShaft) type) which may be connected to a vehicle driveline if that axle is driven.

A derived suspension type defines the bodies, joints, force elements, and topology of a particular type of suspension. All locations are assumed to be provided with respect to a suspension reference frame (a derived suspension type is free to pick the location of this frame but not its orientation, which is assumed to be parallel to the chassis ISO reference frame).

A suspension assembly is attached to a vehicle's chassis by specifying the location of the suspension assembly reference frame with respect to the chassis reference frame (see the definition of the [ISO reference frame](@ref vehicle_ISO_frame)).

## Double wishbone {#wheeled_suspension_ChDoubleWishbone}

Independent steerable suspension using two wishbone control arms (also know as A-arms) to connect the knuckle and chassis.  Used as both front and rear suspension on the [HMMWV](@ref chrono::vehicle::hmmwv::HMMWV) vehicle model.

See [ChDoubleWishbone](@ref chrono::vehicle::ChDoubleWishbone) and [DoubleWishbone](@ref chrono::vehicle::DoubleWishbone).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/DoubleWishbone_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/DoubleWishbone_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/DoubleWishbone_points.png" width="600" />



## Double wishbone (simplified) {#wheeled_suspension_ChDoubleWishboneReduced}

This simplified double-wishbone suspension template models the lower and upper control arms using two distance constraints for each.  This suspension type is suitable when the mass and inertia of the control arms are small relative to the other bodies in the system and can therefore be neglected.

See [ChDoubleWishboneReduced](@ref chrono::vehicle::ChDoubleWishboneReduced) and [DoubleWishboneReduced](@ref chrono::vehicle::DoubleWishboneReduced).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/DoubleWishboneReduced_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/DoubleWishboneReduced_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/DoubleWishboneReduced_points.png" width="600" />



## MacPherson strut {#wheeled_suspension_ChMacPhersonStrut}

Steerable independent suspension system which is preferred for small to mid-sized passenger cars with front wheel drive and traverse mounted engine/gearbox.

See [ChMacPhersonStrut](@ref chrono::vehicle::ChMacPhersonStrut) and [MacPhersonStrut](@ref chrono::vehicle::MacPhersonStrut).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/MacPhersonStrut_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/MacPhersonStrut_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/MacPhersonStrut_points.png" width="600" />



## Multi-link {#wheeled_suspension_ChMultiLink}

This suspension system is similar to a double wishbone axle. The trailing arm can bear high longitudinal forces.

See [ChMultiLink](@ref chrono::vehicle::ChMultiLink) and [MultiLink](@ref chrono::vehicle::MultiLink).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/MultiLink_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/MultiLink_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/MultiLink_points.png" width="600" />



## Semi-trailing arm {#wheeled_suspension_ChSemiTrailingArm}

Simple independent axle system used in smaller passenger cars as rear suspension.

See [ChSemiTrailingArm](@ref chrono::vehicle::ChSemiTrailingArm) and [SemiTrailingArm](@ref chrono::vehicle::SemiTrailingArm).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SemiTrailingArm_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SemiTrailingArm_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SemiTrailingArm_points.png" width="600" />



## Solid axle {#wheeled_suspension_ChSolidAxle}

A solid axle system guided by four links. It normally uses coilsprings or airsprings and could be found in older passenger cars.

See [ChSolidAxle](@ref chrono::vehicle::ChSolidAxle) and [SolidAxle](@ref chrono::vehicle::SolidAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidAxle_points.png" width="600" />



## Solid three-link axle {#wheeled_suspension_ChSolidThreeLinkAxle}

Used as rear suspensions on the [MAN 5t](@ref chrono::vehicle::man::MAN_5t), [MAN 7t](@ref chrono::vehicle::man::MAN_7t), and [MAN 10t](@ref chrono::vehicle::man::MAN_10t) truck models. This suspension allows very high wheel travel, which could not be realized with leafsprings. It is also in on-road trucks with airsprings. Airsprings and coilsprings need a suspension guided by links.

See [ChSolidThreeLinkAxle](@ref chrono::vehicle::ChSolidThreeLinkAxle) and [SolidThreeLinkAxle](@ref chrono::vehicle::SolidThreeLinkAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidThreeLinkAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidThreeLinkAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidThreeLinkAxle_points.png" width="600" />



## Solid three-link axle with bellcrank {#wheeled_suspension_ChSolidBellcrankThreeLinkAxle}

Used as front suspensions on the [MAN 5t](@ref chrono::vehicle::man::MAN_5t), [MAN 7t](@ref chrono::vehicle::man::MAN_7t), and [MAN 10t](@ref chrono::vehicle::man::MAN_10t) truck models.

See [ChSolidBellcrankThreeLinkAxle](@ref chrono::vehicle::ChSolidBellcrankThreeLinkAxle) and [SolidBellcrankThreeLinkAxle](@ref chrono::vehicle::SolidBellcrankThreeLinkAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidBellcrankThreeLinkAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidBellcrankThreeLinkAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SolidBellcrankThreeLinkAxle_points.png" width="600" />



## Leaf-spring solid axle {#wheeled_suspension_ChLeafspringAxle}

Used as rear suspension on the [UAZ](@ref chrono::vehicle::uaz::UAZBUS) vehicle models. Leafspring axles have complex guiding behavior. This is a work-a-like solution, where the guiding effect of the leafsprings is simulated by a special joint in the center of the axle tube. The suspension effect is modeled by coil springs. The rolling behavior is close to a real leafspring axle.

See [ChLeafspringAxle](@ref chrono::vehicle::ChLeafspringAxle) and [LeafspringAxle](@ref chrono::vehicle::LeafspringAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/LeafspringAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/LeafspringAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/LeafspringAxle_points.png" width="600" />



## Leaf-spring solid axle with toebar {#wheeled_suspension_ChToeBarLeafspringAxle}

Used as front suspension on the [UAZ](@ref chrono::vehicle::uaz::UAZBUS) vehicle models.

See [ChToeBarLeafspringAxle](@ref chrono::vehicle::ChToeBarLeafspringAxle) and [ToeBarLeafspringAxle](@ref chrono::vehicle::ToeBarLeafspringAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/ToeBarLeafspringAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/ToeBarLeafspringAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/ToeBarLeafspringAxle_points.png" width="600" />



## SAE Leaf-spring solid axle {#wheeled_suspension_ChSAELeafspringAxle}

The SAE Spring Design Handbook shows a way to model a leaf spring with realistic deformation behavior under load conditions. The kinematics of a leaf spring can be defined by 5 points. These points can be used to define a leaf spring consisting of 6 rigid bodies (front leaf, front clamp, rear clamp, rear leaf and shackle). The bodies are connected by joints. The rotational springs of the front and rear leaf as well as the front clamp and rear clamp have a rotational stiffness that can be set by the user to define the correct behavior. This suspension is used as a rear suspension on the [UAZ](@ref chrono::vehicle::uaz::UAZBUS) vehicle SAE type models. The movement of the axle body due to wheel travel and the tie-up effect due to longitudinal forces can be simulated correctly with this leaf spring model. 

See [ChSAELeafspringAxle](@ref chrono::vehicle::ChLeafspringAxle) and [SAELeafspringAxle](@ref chrono::vehicle::LeafspringAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SAELeafspringAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SAELeafspringAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SAELeafspringAxle_points.png" width="600" />



## SAE Leaf-spring solid axle with toebar {#wheeled_suspension_ChSAEToeBarLeafspringAxle}

Used as front suspension on the [UAZ](@ref chrono::vehicle::uaz::UAZBUS) SAE type vehicle models. The leaf spring definition is the same as in the SAE leaf spring axle.

See [ChSAEToeBarLeafspringAxle](@ref chrono::vehicle::ChToeBarLeafspringAxle) and [SAEToeBarLeafspringAxle](@ref chrono::vehicle::SAEToeBarLeafspringAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SAEToeBarLeafspringAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SAEToeBarLeafspringAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/SAEToeBarLeafspringAxle_points.png" width="600" />



## Three-link Independent Rear Suspension {#wheeled_suspension_ChThreeLinkIRS}

Three-link Independent Rear Suspension (IRS), as seen on the Polaris RZR vehicle.

See [ChThreeLinkIRS](@ref chrono::vehicle::ChThreeLinkIRS) and [ThreeLinkIRS](@ref chrono::vehicle::ThreeLinkIRS).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/ThreeLinkIRS_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/ThreeLinkIRS_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/ThreeLinkIRS_points.png" width="600" />




## Rigid suspension {#wheeled_suspension_ChRigidSuspension}

Trivial assembly with spindles directly attached to an axle tube pinned to the chassis. It is typical for farm tractors and combine harvesters.

See [ChRigidSuspension](@ref chrono::vehicle::ChRigidSuspension) and [RigidSuspension](@ref chrono::vehicle::RigidSuspension).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RigidSuspension_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RigidSuspension_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RigidSuspension_points.png" width="600" />



## Rigid pinned-axle {#wheeled_suspension_ChRigidPinnedAxle}

Trivial assembly with spindles directly attached to the axle tube that can swing around a pivot point against the chassis. This can be used, if an unsprung axle system is needed but has to run on ondulated terrain to avoid wheel lift off.

See [ChRigidPinnedAxle](@ref chrono::vehicle::ChRigidPinnedAxle) and [RigidPinnedAxle](@ref chrono::vehicle::RigidPinnedAxle).

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RigidPinnedAxle_bodies.png" width="600" />

The topology of this suspension template is:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RigidPinnedAxle_topology.png" width="800" />

The hardpoints (defined for the left side only and mirrored to construct the right side) are:

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/RigidPinnedAxle_points.png" width="600" />


## Generic wheeled vehicle suspension template {#wheeled_suspension_ChGenericWheeledSuspension}

This template allows specification of a user-defined, custom suspension subsystem with arbitrary topology.
A concrete suspension is constructed by adding bodies, joints, distance constraints, and TSDAs. Each modeling component can be marked as "mirrored". The user is responsible for defining all non-mirrored components and the left-side mirrored components (the corresponding right-side components are generated automatically).


See [ChGenericWheeledSuspension](@ref chrono::vehicle::ChGenericWheeledSuspension) and [GenericWheeledSuspension](@ref chrono::vehicle::GenericWheeledSuspension).
