Track assembly {#tracked_assembly}
==================================

A Chrono::Vehicle track assembly is a container of vehicle subsystems including a sprocket, and idler with tensioner, optionally a set of rollers, a number of suspension mechanisms, and an arbitrary number of track-shoe mechanisms.

To eliminate the burden of consistent initialization of the track shoe bodies, the track assembly subsystem provides algorithmic support for automatic assembly of the track around the sprocket, idler, road-wheels, and any existing rollers.  Specialized track assemblies and corresponding assembly routines are provided for different combinations of sprocket profiles and associated track shoe models.

The various track assembly templates in Chrono::Vehicle differ in the type of track-shoe used:
- [ChTrackAssemblySinglePin](@ref chrono::vehicle::ChTrackAssemblySinglePin)
- [ChTrackAssemblyDoublePin](@ref chrono::vehicle::ChTrackAssemblyDoublePin)
- [ChTrackAssemblyBandBushing](@ref chrono::vehicle::ChTrackAssemblyBandBushing)
- [ChTrackAssemblyBandANCF](@ref chrono::vehicle::ChTrackAssemblyBandANCF)
