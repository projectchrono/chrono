// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SHAPE
#define CH_COLLISION_SHAPE

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChMaterialSurface.h"

namespace chrono {
namespace collision {

/// Class defining a collision shape.
class ChApi ChCollisionShape {
  public:
    /// Supported collision shape types.
    enum Type {
        SPHERE,
        ELLIPSOID,
        BOX,
        CYLINDER,
        CONVEXHULL,
        TRIANGLEMESH,
        BARREL,
        POINT,
        TRIANGLE,
        CAPSULE,      // Currently implemented in parallel only
        CONE,         // Currently implemented in parallel only
        ROUNDEDBOX,   // Currently implemented in parallel only
        ROUNDEDCYL,   // Currently implemented in parallel only
        ROUNDEDCONE,  // Currently implemented in parallel only
        CONVEX,       // Currently implemented in parallel only
        TETRAHEDRON,  // Currently implemented in parallel only
        PATH2D,
        COMPOUND,
        UNKNOWN_SHAPE
    };

    ChCollisionShape();
    ChCollisionShape(Type type);
    virtual ~ChCollisionShape() {}

    Type m_type;
};

}  // end namespace collision
}  // end namespace chrono

#endif
