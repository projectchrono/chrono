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

namespace chrono {
namespace collision {

class ConvexBase;

/// @addtogroup parallel_collision
/// @{

CH_PARALLEL_API
bool MPRContact(const ConvexBase* ShapeA,
                const ConvexBase* ShapeB,
                const real& envelope,
                real3& returnNormal,
                real3& point,
                real& depth);

CH_PARALLEL_API
bool MPRCollision(const ConvexBase* ShapeA,
                  const ConvexBase* ShapeB,
                  real envelope,
                  real3& returnNormal,
                  real3& pointA,
                  real3& pointB,
                  real& depth);

CH_PARALLEL_API
void MPRGetPoints(const ConvexBase* ShapeA,
                  const ConvexBase* ShapeB,
                  const real& envelope,
                  real3& N,
                  real3 p0,
                  real3& p1,
                  real3& p2);

CH_PARALLEL_API
bool MPRSphereSphere(const ConvexBase* ShapeA, const ConvexBase* ShapeB, real3& N, real& dist, real3& p1, real3& p2);

/// @} parallel_colision

} // end namespace collision
} // end namespace chrono
