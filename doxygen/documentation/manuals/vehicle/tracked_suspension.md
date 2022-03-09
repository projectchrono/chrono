Suspension models {#tracked_suspension}
=======================================

\tableofcontents

Different suspension configurations are available, including torsion spring with linear or rotational dampers and a hydropneumatic suspension template.  A track assembly can contain an arbitrary number of suspension subsystems which, for the templates using a torsion spring, may or may not include a damper.  A Chrono::Vehicle suspension subsystem also contains a road-wheel, themselves templatized based on the type of track shoe used (central or lateral guiding pins).

Similar to the case of wheeled vehicle, a tracked vehicle suspension template allows complete freedom in specifying spring and damper forces which can be linear or non-linear, defined through table lookup or implemented in user-provided C++ functions. 


## Road-wheel assembly models {#tracked_suspension_roadwheel_assembly}


### Linear damper suspension {#tracked_suspension_linear_damper}

See [ChLinearDamperRWAssembly](@ref chrono::vehicle::ChLinearDamperRWAssembly) and [LinearDamperRWAssembly](@ref chrono::vehicle::LinearDamperRWAssembly).


### Rotational damper suspension {#tracked_suspension_rotational_damper}

See [ChRotationalDamperRWAssembly](@ref chrono::vehicle::ChRotationalDamperRWAssembly) and [RotationalDamperRWAssembly](@ref chrono::vehicle::RotationalDamperRWAssembly).



## Road-wheel models {#tracked_suspension_roadwheel}

### Central-pin road wheel {#tracked_susepension_roadwheel_central}

See [ChSingleRoadWheel](@ref chrono::vehicle::ChSingleRoadWheel) and [SingleRoadWheel](@ref chrono::vehicle::SingleRoadWheel).

### Lateral-pin road wheel {#tracked_suspension_roadwheel_lateral}

See [ChDoubleRoadWheel](@ref chrono::vehicle::ChDoubleRoadWheel) and [DoubleRoadWheel](@ref chrono::vehicle::DoubleRoadWheel).
