Track shoe models {#tracked_shoe}
=================================

\tableofcontents

Chrono::Vehicle offers templates for both single-pin and double-pin track shoes, each of which can have central or lateral guiding pins. The single-pin track shoe consists of a single body with non-trivial contact geometry which is connected to its neighbors through revolute joints. The double-pin track shoe template contains, in addition to the main track pad body, two additional connector bodies which are connected through revolute joints to the adjacent pads and which carry the contact geometry for collision with the track's sprocket gears.

In addition, two different models of continuous-band tracks are available. 

All track shoe templates are fully parameterized in terms of dimensions, masses and inertias of the constituent bodies, as well as their contact geometry.

## Single-pin track shoe {#tracked_shoe_single_pin}

See [ChTrackShoeSinglePin](@ref chrono::vehicle::ChTrackShoeSinglePin) and [TrackShoeSinglePin](@ref chrono::vehicle::TrackShoeSinglePin).


## Double-pin track shoe {#tracked_shoe_double_pin}

See [ChTrackShoeDoublePin](@ref chrono::vehicle::ChTrackShoeDoublePin) and [TrackShoeDoublePin](@ref chrono::vehicle::TrackShoeDoublePin).


## Band-bushing track shoe {#tracked_shoe_band_bushing}

See [ChTrackShoeBandBushing](@ref chrono::vehicle::ChTrackShoeBandBushing) and [TrackShoeBandBushing](@ref chrono::vehicle::TrackShoeBandBushing).  See also the base class [ChTrackShoeBand](@ref chrono::vehicle::ChTrackShoeBand).


## Band-ANCF track shoe {#tracked_shoe_band_ancf}

See [ChTrackShoeBandANCF](@ref chrono::vehicle::ChTrackShoeBandANCF) and [ChTrackShoeBandANCF](@ref chrono::vehicle::TrackShoeBandANCF).  See also the base class [ChTrackShoeBand](@ref chrono::vehicle::ChTrackShoeBand).
