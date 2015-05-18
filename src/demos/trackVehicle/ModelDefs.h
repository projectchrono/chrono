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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//	Common enums for user input to public TrackedVehicle functions
// =============================================================================

#ifndef MODEL_DEFS_H
#define MODEL_DEFS_H

#include "physics/ChSystem.h"

namespace chrono {

// descriptive namespace for types
namespace VisualizationType {
enum Enum {
    None,
    Primitives,
    CompoundPrimitives,
    Mesh,
};
}

namespace CollisionType {
enum Enum {
    None,
    Primitives,
    Mesh,
    ConvexHull,
    CompoundPrimitives,
    CallbackFunction,
};
}

// Collision family definitions
// GROUND can collide with anything
// GEAR includes the idler, support roller, and other non-moving rolling elements
// WHEELS includes the road wheels, which can collide with their neighbors
// SHOES collide with everything, except for their own family
namespace CollisionFam {
enum Enum {
    Ground,
    Hull,
    Wheel,
    ShoeRight,
    ShoeLeft,
    Gear,
};
}

// to specify if the part is on the right or left side,
// relative to the chassis when facing forward.
enum VehicleSide { RIGHTSIDE, LEFTSIDE };

// specify what type of output 
enum DebugType {
    DBG_BODY = 1 << 0,
    DBG_CONSTRAINTS = 1 << 2,
    DBG_CONTACTS = 1 << 3,
};

// vehicle writes debug info for these objects (e.g., subsystems)
enum DebugInformation {
    DBG_CHASSIS = 1 << 0,
    DBG_FIRSTSHOE = 1 << 1,
    DBG_GEAR = 1 << 2,
    DBG_IDLER = 1 << 3,
    DBG_PTRAIN = 1 << 4,
    DBG_COLLISIONCALLBACK = 1 << 5,
    DBG_SUSPENSION = 1 << 6,
    DBG_ALL_CONTACTS = 1 << 7,
};

static std::ostream& operator<<(std::ostream& out, const ChVector<double>& vect) {
    out << vect.x << "," << vect.y << "," << vect.z;
    return out;
}

static std::ostream& operator<<(std::ostream& out, const ChQuaternion<double>& q) {
    out << q.e0 << "," << q.e1 << "," << q.e2 << "," << q.e3;
    return out;
}

}  // end namespace chrono

#endif
