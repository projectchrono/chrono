// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SHAPE_CHRONO
#define CH_COLLISION_SHAPE_CHRONO

#include "chrono/collision/ChCollisionShape.h"

#include "chrono/multicore_math/real3.h"
#include "chrono/multicore_math/real4.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_mc
/// @{

/// Collision shape for the custom multicore Chrono collision system.
class ChCollisionShapeChrono : public ChCollisionShape {
  public:
    ChCollisionShapeChrono(Type t, std::shared_ptr<ChMaterialSurface> material)
        : ChCollisionShape(t, material), convex(nullptr) {}

    real3 A;        ///< location
    real3 B;        ///< dimensions
    real3 C;        ///< extra
    quaternion R;   ///< rotation
    real3* convex;  ///< pointer to convex data;
};

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono

#endif
