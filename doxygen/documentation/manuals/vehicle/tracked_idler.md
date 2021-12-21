Idler models {#tracked_idler}
=============================

A Chrono::Vehicle idler mechanism consists of the idler wheel and a connecting body.  The idler wheel is connected through a revolute joint to the connecting body which in turn is connected to the chassis through a translational joint. A linear actuator acts as a tensioner which is modeled as a general spring-damper with optional preload.

An idler subsystem is defined with respect to a frame centered at the origin of the idler wheel, optionally pitched relative to the chassis reference frame. The translational joint is aligned with the X axis of this reference frame, while the axis of rotation of the revolute joint is aligned with its Y axis.

Different templates are provided for the case of tracks with central or lateral guiding pins.


## Central-pin idler {#tracked_idler_central}

See [ChDoubleIdler](@ref chrono::vehicle::ChDoubleIdler) and [DoubleIdler](@ref chrono::vehicle::DoubleIdler).


## Lateral-pin idler {#tracked_idler_lateral}

See [ChSingleIdler](@ref chrono::vehicle::ChSingleIdler) and [SingleIdler](@ref chrono::vehicle::SingleIdler).

