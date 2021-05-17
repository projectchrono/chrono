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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Minkowski Portal Refinement Narrowphase.
//
// =============================================================================

#pragma once

#include "chrono/multicore_math/real3.h"

namespace chrono {
namespace collision {

class ConvexBase;

/// @addtogroup collision_mc
/// @{

/// Minkovski Portal Refinement convex-convex collision detection
/// (adapted from Xeno Collide).
ChApi bool MPRCollision(const ConvexBase* ShapeA,
                        const ConvexBase* ShapeB,
                        real envelope,
                        real3& returnNormal,
                        real3& pointA,
                        real3& pointB,
                        real& depth);

/// Analytical sphere-sphere collision detection.
ChApi bool MPRSphereSphere(const ConvexBase* ShapeA,
                           const ConvexBase* ShapeB,
                           real3& N,
                           real& dist,
                           real3& p1,
                           real3& p2);

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
