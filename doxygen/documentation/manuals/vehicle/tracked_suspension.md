Suspension models {#tracked_suspension}
=======================================

\tableofcontents

Different suspension configurations are available, including torsion spring with linear or rotational dampers and a hydropneumatic suspension template.  A track assembly can contain an arbitrary number of suspension subsystems which, for the templates using a torsion spring, may or may not include a damper.  A Chrono::Vehicle suspension subsystem also contains a road-wheel, itself templatized based on the type of track shoe used (central or lateral guiding pins).

Similar to the case of wheeled vehicle, a tracked vehicle suspension template allows complete freedom in specifying spring and damper forces which can be linear or non-linear, defined through table lookup or implemented in user-provided C++ functions. 


## Track suspension models {#tracked_suspension}


### Translational damper suspension {#tracked_suspension_translational_damper}

See [ChTranslationalDamperSuspension](@ref chrono::vehicle::ChTranslationalDamperSuspension) and [TranslationalDamperSuspension](@ref chrono::vehicle::TranslationalDamperSuspension).


### Rotational damper suspension {#tracked_suspension_rotational_damper}

See [ChRotationalDamperSuspension](@ref chrono::vehicle::ChRotationalDamperSuspension) and [RotationalDamperSuspension](@ref chrono::vehicle::RotationalDamperSuspension).



## Road-wheel models {#tracked_suspension_roadwheel}

### Central-pin road wheel {#tracked_susepension_roadwheel_central}

See [ChDoubleTrackWheel](@ref chrono::vehicle::ChDoubleTrackWheel) and [DoubleTrackWheel](@ref chrono::vehicle::DoubleTrackWheel).

### Lateral-pin road wheel {#tracked_suspension_roadwheel_lateral}

See [ChSingleTrackWheel](@ref chrono::vehicle::ChSingleTrackWheel) and [SingleTrackWheel](@ref chrono::vehicle::SingleTrackWheel).

