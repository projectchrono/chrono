Chrono VEHICLE module tutorials {#tutorial_table_of_content_chrono_vehicle}
===============================

The Chrono distribution contains several demos for modeling and simulating ground vehicle systems with the [VEHICLE module](@ref vehicle).

Chrono::Vehicle provides support for both wheeled and tracked vehicles, in a template-based framework. These templates are parameterized models of various vehicles and vehicle subsystems.

In addition to the main library, the VEHICLE module also creates a library of pre-defined [VEHICLE models](@ref vehicle_models) which currently contains:

- Wheeled vehicle models
  - [HMMWV](@ref vehicle_models_hmmwv): off-road 4-wheel vehicle
  - [Sedan](@ref vehicle_models_sedan): generic passenger car
  - [Citybus](@ref vehicle_models_citybus): passenger bus
  - [UAZ](@ref vehicle_models_uaz): minibus model of the UAZ-452 vehicle
  - [M-role](@ref vehicle_models_mrole): multi-purpose wheeled vehicle model
  - [MAN](@ref vehicle_models_man): truck models (3 different variants: 5t, 7t, and 10t)
  - [Kraz](@ref vehicle_models_kraz): semi-trailer truck model
  - [FMTV](@ref vehicle_models_fmtv): models of trucks from the Family of Medium Tactical Vehicles 
  - [Gator](@ref vehicle_models_gator): small utility vehicle model
  - [RC](@ref vehicle_models_rccar): remote-controlled vehicle model
- Tracked vehicle models
  - [M113](@ref vehicle_models_m113): M113 tracked vehicle
  - [Marder](@ref vehicle_models_marder): Marder tracked vehicle

Selected vehicle demos:

- demo_VEH_HMMWV - off-road 4WD wheeled vehicle (from the Vehicle models library)
- demo_VEH_HMMWV_DefSoil - wheeled vehicle on [SCM deformable terrain](@ref vehicle_terrain_scm) (Bekker-Wong-type)
- demo_VEH_HMMWV_Multicore - wheeled vehicle on [granular terrain](@ref vehicle_terrain_granular) simulation with the [MULTICORE module](@ref multicore_module)
- demo_VEH_M113 - tracked vehicle (from the Vehicle models library)
- demo_VEH_M113_DefSoil - tracked vehicle on [SCM deformable terrain](@ref vehicle_terrain_scm) (Bekker-Wong-type)
- demo_VEH_M113_Multicore - tracked vehicle simulation with the [MULTICORE module](@ref multicore_module)
- demo_VEH_SteeringController - using a lateral steering controller for a double-lane change maneuver
- demo_VEH_CRGTerrain - demonstration of the optional interface to OpenCRG
- demo_VEH_ArticulatedVehicle - demonstration of extending Chrono::Vehicle (articulated chassis)
- demo_VEH_TactorTrailer - demonstration of extending Chrono::Vehicle (tractor-trailer vehicle)
- demo_VEH_TwoCars - demonstration of using multiple vehicles in a single simulation
- demo_VEH_WheeledJSON - wheeled vehicle defined through JSON specification files
- demo_VEH_TrackedJSON - tracked vehicle defined through JSON specification files
- demo_VEH_DeformableSoil - demonstration of location-dependent soil parameters in [SCM deformable terrain](@ref vehicle_terrain_scm)
- demo_VEH_MovingPatch - demonstration of the moving patch feature for [granular terrain](@ref vehicle_terrain_granular)
- demo_VEH_RigidTerrain - demonstration of using multiple patches for a [rigid terrain](@ref vehicle_terrain_rigid) model
- demo_VEH_SuspensionTestRig - demonstration of the wheeled vehicle [suspension test rig](@ref wheeled_rig_suspension) with tire actuation
- demo_VEH_TireTestRig - demonstration of the single [tire test rig](@ref wheeled_rig_tire)
- demo_VEH_TrackTestRig - demonstration of the tracked vehicle track assembly test rig
