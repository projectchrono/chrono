Chrono VEHICLE module tutorials {#tutorial_table_of_content_chrono_vehicle}
===============================

The Chrono distribution contains several demos for modeling and simulating ground vehicle systems with the [VEHICLE module](group__vehicle.html).

Chrono::Vehicle provides support for both wheeled and tracked vehicles, in a template-based framework. These templates are parameterized models of various vehicles and vehicle subsystems.

In addition to the main library, the VEHICLE module also creates a library of pre-defined [VEHICLE models](group__vehicle__models.html) which currently contains:

- HMMWV: off-road 4-wheel vehicle
- Sedan: passenger car
- Generic: generic wheeled vehicle (sandbox for testing various configurations and subsystems)
- M113: tracked vehicle
- UAZ: minibus model of the UAZ-452 vehicle

Selected vehicle demos:

- demo_VEH_HMMWV - wheeld vehicle (from the Vehicle library)
- demo_VEH_HMMWV_DefSoil - wheeled vehicle on SCM deformable terrain (Bekker-Wong-type)
- demo_VEH_HMMWV_Parallel - wheeled vehicle on granular terrain simulation with the [PARALLEL module](group__parallel_module)
- demo_VEH_M113 - tracked vehicle (from the Vehicle library)
- demo_VEH_DefSoil - tracked vehicle on [SCM deformable terrain](@ref chrono::vehicle::SCMDeformableTerrain)
- demo_VEH_Parallel - tracked vehicle simulation with the PARALLEL module
- demo_VEH_SteerinController - using a lateral steering controller for a double-lane change maneuver
- demo_VEH_CRGTerrain - demonstration of the optional interface to OpenCRG
- demo_VEH_ArticulatedVehicle - demonstration of extending Chrono::Vehicle (articulated chassis)
- demo_VEH_TactorTrailer - demonstration of extending Chrono::Vehicle (tractor-trailer vehicle)
- demo_VEH_TwoCars - demonstration of using multiple vehicles in a single simulation
- demo_VEH_WheeledJSON - wheeled vehicle defined through JSON specification files
- demo_VEH_TrackedJSON - tracked vehicle defined through JSON specification files
- demo_VEH_MovingPatch - demonstration of the moving patch feature for [Granular terrain](@ref chrono::vehicle::GranularTerrain)
- demo_VEH_RigidTerrain - demonstration of using multoiple patches for [Rigid terrain](@ref chrono::vehicle::RigidTerrain)
