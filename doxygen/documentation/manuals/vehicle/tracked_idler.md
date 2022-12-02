Idler models {#tracked_idler}
=============================

A Chrono::Vehicle idler mechanism consists of the idler wheel and a mechanism attaching the idler to the chassis.
The idler wheel can be either a [ChDoubleTrackWheel](@ref chrono::vehicle::ChDoubleTrackWheel) which assumes a central guiding pin, or [ChSingleTrackWheel](@ref chrono::vehicle::ChSingleTrackWheel) which assumes lateral guiding pins.

An idler subsystem is defined with respect to a frame centered at the origin of the idler wheel, optionally pitched relative to the chassis reference frame. The translational joint is aligned with the X axis of this reference frame, while the axis of rotation of the revolute joint is aligned with its Y axis.

Different templates are provided for different idler mechanism topologies.


## Translational idler {#tracked_idler_translational}

The idler wheel is connected through a revolute joint to the connecting body which in turn is connected to the chassis through a translational joint. A linear actuator acts as a tensioner which is modeled as a general spring-damper with optional preload.

See [ChTranslationalIdler](@ref chrono::vehicle::ChTranslationalIdler) and [TranslationalIdler](@ref chrono::vehicle::TranslationalIdler).


## Distance idler {#tracked_idler_distance}

The idler wheel is connected to an arm pinned to the chassis.  A fixed distance constraint positions the arm relative to the chassis.

See [ChDistanceIdler](@ref chrono::vehicle::ChDistanceIdler) and [DistanceIdler](@ref chrono::vehicle::DistanceIdler).

