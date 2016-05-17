#pragma once

#include "chrono_parallel/collision/ChCDataStructures.h"

namespace chrono {
namespace collision {

CH_PARALLEL_API
bool MPRContact(const ConvexShape& ShapeA,
                const ConvexShape& ShapeB,
                const real& envelope,
                real3& returnNormal,
                real3& point,
                real& depth);
CH_PARALLEL_API
bool MPRCollision(const ConvexShape& ShapeA,
                  const ConvexShape& ShapeB,
                  real envelope,
                  real3& returnNormal,
                  real3& pointA,
                  real3& pointB,
                  real& depth,
				  real& effectiveRadius,
				  real defaultRadius);
CH_PARALLEL_API
void MPRGetPoints(const ConvexShape& ShapeA,
                  const ConvexShape& ShapeB,
                  const real& envelope,
                  real3& N,
                  real3 p0,
                  real3& p1,
                  real3& p2);

CH_PARALLEL_API
bool MPRSphereSphere(const ConvexShape& ShapeA, const ConvexShape& ShapeB, real3& N, real& dist, real3& p1, real3& p2);

}  // end namespace collision
}  // end namespace chrono
