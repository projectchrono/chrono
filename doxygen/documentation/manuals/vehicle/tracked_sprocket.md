Sprocket models {#tracked_sprocket}
===================================

\tableofcontents

The sprocket subsystem connects the tracked vehicle driveline to the track assembly and is responsible for collision detection and contact processing with the track shoe bodies.  A sprocket subsystem template implements the custom collision detection algorithm for a consistent pair of sprocket gear profile and associated track shoe.  Chrono::Vehicle provides two templates for the sprocket subsystem, corresponding to the types of supported track shoes, namely single-pin, double-pin, and band track.  The sprocket gear profile is defined as a 2D path composed of line segments and circular arcs which are parameterized for each type of profile.  Collision detection is performed in 2D, working in the plane of the sprocket gear, but contact forces are calculated in 3D before being applied to the sprocket and interacting track shoe bodies.

In addition to the gear profile, a sprocket template is parameterized by the mass and inertia of the sprocket body, the rotational inertia of the sprocket axle, and the separation distance between the two gears.

## Single-pin sprocket {#tracked_sprocket_single_pin}

This sprocket template uses a gear profile composed of circular arcs. This sprocket type is suitable for interaction with single-pin track shoes.

See [ChSprocketSinglePin](@ref chrono::vehicle::ChSprocketSinglePin) and [SprocketSinglePin](@ref chrono::vehicle::SprocketSinglePin).


## Double-pin sprocket {#tracked_sprocket_double_pin}

This sprocket template uses a gear profile composed of circular arcs and a flat seat. This sprocket type is suitable for interaction with double-pin track shoes.

See [ChSprocketDoublePin](@ref chrono::vehicle::ChSprocketDoublePin) and [SprocketDoublePin](@ref chrono::vehicle::SprocketDoublePin).


## Band-track sprocket {#tracked_sprocket_band}

This sprocket template uses a gear profile composed of circular arcs and a flat seat. This sprocket type is suitable for interaction with a continuous-band track.

See [ChSprocketBand](@ref chrono::vehicle::ChSprocketBand) and [SprocketBand](@ref chrono::vehicle::SprocketBand).

