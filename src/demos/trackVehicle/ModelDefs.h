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
namespace VisualizationType
{
  enum Enum
  {
    None,
    Primitives,
    CompoundPrimitives,
    Mesh,
  };

}

namespace CollisionType
{
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
namespace CollisionFam
{
  enum Enum {
    Ground,
    Hull,
    Wheel,
    Shoe,
    Gear,
  };
}

enum VehicleSide {
    Right = 0,    
    Left = 1     ///< x-forward leads to right side being position z-dir
};

enum DebugInformation {
  DBG_FIRSTSHOE   = 1 << 0,
  DBG_GEAR        = 1 << 1,
  DBG_IDLER       = 1 << 2,
  DBG_PTRAIN      = 1 << 3,
  DBG_CONSTRAINTS = 1 << 4
};

static std::ostream& operator<< (std::ostream &out, const ChVector<double>& vect)
{
  out << vect.x <<","<< vect.y <<","<< vect.z;
  return out;
}

static std::ostream& operator<< (std::ostream &out, const ChQuaternion<double>& q)
{
  out << q.e0 <<","<< q.e1 <<","<< q.e2 <<","<< q.e3;
  return out;
}

} // end namespace chrono

#endif
