Wheeled vehicles {#wheeled_vehicle}
===================================

A wheeled vehicle in Chrono::Vehicle is a specialization of the abstract generic vehicle system ([ChVehicle](@ref chrono::vehicle::ChVehicle)) and is defined as a collection of subsystems (as illustrated below). 

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled_subsystems.png" width="800" />

A wheeled vehicle contains a chassis subsystem, a driveline subsystem, and an arbitrary number of axles which, by convention, are numbered starting at the front of the vehicle.  Multi-steer vehicles are supported by allowing either an arbitrary number of steering mechanisms (which are connected to different axles) or by allowing multiple steerable axles to be connected to the same steering mechanism. Each axle can accept one or two tires per side.

See [ChWheeledVehicle](@ref chrono::vehicle::ChWheeledVehicle).

The various subsystem templates are described in the following sections:
* @subpage wheeled_suspension
* @subpage wheeled_steering
* @subpage wheeled_driveline
* @subpage wheeled_tire

