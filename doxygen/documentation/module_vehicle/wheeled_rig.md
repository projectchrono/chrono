Test rigs {#wheeled_rig}
=========================

\tableofcontents

## Suspension test rig {#wheeled_rig_suspension}

[ChSuspensionTestRig](@ref chrono::vehicle::ChSuspensionTestRig) is a mechanism for testing an entire vehicle suspension assembly.  The tested suspension can be specified:
- through a stand-alone JSON file (may or may not include a steering subsystem)
- as a specified axle in a vehicle JSON specification file
- as a specified axle in an existing vehicle (which must have been initialized)

Two variants of the suspension test rig are provided:
- [ChSuspensionTestRigPlatform](@ref chrono::vehicle::ChSuspensionTestRigPlatform) uses two independent rigid platforms to actuate the suspension mechanism through interaction with the tires. See demo_VEH_SuspensionTestRigPlatform.
- [ChSuspensionTestRigPushrod](@ref chrono::vehicle::ChSuspensionTestRigPushrod) applies direct actuation to the wheel spindles. See demo_VEH_SuspensionTestRigPushrod.

## Tire test rig {#wheeled_rig_tire}

[ChTireTestRig](@ref chrono::vehicle::ChTireTestRig) is a mechanism for testing a single tire interacting with [rigid](@ref vehicle_terrain_rigid), [SCM deformable](@ref vehicle_terrain_scm), or [granular](@ref vehicle_terrain_granular) terrain.  All available Chrono::Vehicle [tire models](@ref wheeled_tire) can be used with this test rig, but only in conjunction with a consistent terrain model.

The tire test rig allows  variation of longitudinal speed, wheel angular speed, and wheel slip angle as functions of time, provides support for automatic selection of longitudinal and angular speeds in order to enforce a specified longitudinal slip value, and allows specifying a camber angle (kept fixed during the simulation). 

See demo_VEH_TireTestRig and demo_VEH_TireTestRig_Parallel.
