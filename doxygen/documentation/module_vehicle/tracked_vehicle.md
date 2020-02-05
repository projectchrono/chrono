Tracked vehicles {#tracked_vehicle}
===================================

Similar to the wheeled vehicle, a track vehicle in Chrono::Vehicle is a specialization of the generic vehicle system and is defined as a hierarchy of subsystems, as illustrated below.

<img src="http://www.projectchrono.org/assets/manual/vehicle/tracked_subsystems.png" width="800" />

Currently, a single topology of tracked vehicles is supported which includes, at the top-level a chassis subsystem, the vehicle driveline, a steering mechanism, and two track assembly subsystems.  The latter are containers of further subsystems and each includes a sprocket mechanism, an idler tensioner subsystem, an arbitrary number of suspension components, and an arbitrary number of track-shoe assemblies.

See [ChTrackedVehicle](@ref chrono::vehicle::ChTrackedVehicle).

The various subsystem templates are described in the following sections:
* @subpage tracked_assembly
* @subpage tracked_suspension
* @subpage tracked_sprocket
* @subpage tracked_idler
* @subpage tracked_shoe
