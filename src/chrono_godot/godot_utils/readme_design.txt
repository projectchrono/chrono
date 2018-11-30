The purpose of godot_utils are to provide a set of functions/classes that
make life easier for other classes.

Examples:
-conversions from Chrono structure to equivalent Godot structures
    (ChVector->Vector, ChQuaternion->Quat, etc)
-handling of godot assets like material, color, geometry that may be used by
    many nodes but we want to prevent creating more than 1 copy of the asset
