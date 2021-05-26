// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Minkowski Portal Refinement Narrowphase.
//
// =============================================================================

#pragma once

#include "chrono_multicore/ChApiMulticore.h"

namespace chrono {
namespace collision {

class ConvexBase;

/// @addtogroup multicore_collision
/// @{

CH_MULTICORE_API
bool MPRContact_mc(const ConvexBase* ShapeA,
                const ConvexBase* ShapeB,
                const real& envelope,
                real3& returnNormal,
                real3& point,
                real& depth);

CH_MULTICORE_API
bool MPRCollision_mc(const ConvexBase* ShapeA,
                  const ConvexBase* ShapeB,
                  real envelope,
                  real3& returnNormal,
                  real3& pointA,
                  real3& pointB,
                  real& depth);

CH_MULTICORE_API
void MPRGetPoints_mc(const ConvexBase* ShapeA,
                  const ConvexBase* ShapeB,
                  const real& envelope,
                  real3& N,
                  real3 p0,
                  real3& p1,
                  real3& p2);

CH_MULTICORE_API
bool MPRSphereSphere_mc(const ConvexBase* ShapeA, const ConvexBase* ShapeB, real3& N, real& dist, real3& p1, real3& p2);

/// @} multicore_colision

} // end namespace collision
} // end namespace chrono
