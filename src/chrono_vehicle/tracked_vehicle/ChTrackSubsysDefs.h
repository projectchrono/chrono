// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Various utility classes for tracked vehicle subsystems.
//
// TODO: combine with ChSubsysDefs.h
//
// =============================================================================

#ifndef CH_TRACK_SUBSYS_DEFS_H
#define CH_TRACK_SUBSYS_DEFS_H

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Enumerations for track collision flags.
namespace TrackCollide {
enum Enum {
    NONE = 0,
    SPROCKET_LEFT = 1 << 0,
    SPROCKET_RIGHT = 1 << 1,
    IDLER_LEFT = 1 << 2,
    IDLER_RIGHT = 1 << 3,
    WHEELS_LEFT = 1 << 4,
    WHEELS_RIGHT = 1 << 5,
    SHOES_LEFT = 1 << 6,
    SHOES_RIGHT = 1 << 7,
    ALL = 0xFFFF
};
}

/// Enumerations for track collision families.
namespace TrackCollisionFamily {
enum Enum { CHASSIS, IDLERS, WHEELS, SHOES };
}

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
