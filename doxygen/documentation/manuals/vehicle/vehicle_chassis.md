Vehicle chassis {#vehicle_chassis}
==================================

Any vehicle system must include a chassis subsystem.  The vehicle position, orientation, velocity and acceleration are defined to be those of the chassis reference frame.  All other vehicle subsystems are initialized relative to the vehicle chassis.

Currently, only one chassis template is provided: [ChRigidChassis](@ref chrono::vehicle::ChRigidChassis) which consists of a single rigid body.

For modeling vehicles with different topologies (e.g., articulated chassis), chassis with torsional compliance, as well as trailer systems, Chrono::Vehicle also includes a so-called "rear chassis" (base class [ChChassisRear](@ref chrono::vehicle::ChChassisRear), rigid-body template [ChRigidChassisRear](@ref chrono::vehicle::ChRigidChassisRear)) which can be attached to a front chassis using a specialized "chassis connector".

The following three types of chassis connector templates are available:
* [ChChassisConnectorArticulated](@ref chrono::vehicle::ChChassisConnectorArticulated) allows connecting the front and rear chassis with an actuated revolute joint which is controlled through the steering driver input
* [ChChassisConnectorTorsion](@ref chrono::vehicle::ChChassisConnectorTorsion) allows for connecting the front and rear chassis with a longitudinal revolute joint with specified torsional stiffness
* [ChChassisConnectorHitch](@ref chrono::vehicle::ChChassisConnectorHitch) models a trailer hitch connection (using a spherical joint)
