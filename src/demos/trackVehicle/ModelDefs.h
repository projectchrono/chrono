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

// #include "ChronoT_config.h"

namespace chrono {

enum struct VisualizationType {
  NONE,
  PRIMITIVES,
  MESH
};

enum struct CollisionType {
  NONE,
  PRIMITIVES,
  MESH,
  CONVEXHULL,
  COMPOUNDPRIMITIVES
};

// Collision family definitions
// GROUND can collide with anything
// GEARS includes the idler, support roller, and other non-moving rolling elements
// WHEELS includes the road wheels, which can collide with their neighbors
// SHOES collide with everything, except for their own family
enum struct CollisionFam {
  GROUND,
  HULL,
  GEARS,
  WHEELS,
  SHOES,
  ALL
};


enum struct VehicleSide {
  RIGHT = 0,    
  LEFT = 1     ///< x-forward leads to right side being position z-dir
};

} // end namespace chrono

#endif
