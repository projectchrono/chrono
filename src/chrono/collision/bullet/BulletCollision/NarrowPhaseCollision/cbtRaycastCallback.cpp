/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#include <stdio.h>

#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "BulletCollision/CollisionShapes/cbtTriangleShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/cbtContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.h"
#include "cbtRaycastCallback.h"

cbtTriangleRaycastCallback::cbtTriangleRaycastCallback(const cbtVector3& from, const cbtVector3& to, unsigned int flags)
	: m_from(from),
	  m_to(to),
	  //@BP Mod
	  m_flags(flags),
	  m_hitFraction(cbtScalar(1.))
{
}

void cbtTriangleRaycastCallback::processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
{
	const cbtVector3& vert0 = triangle[0];
	const cbtVector3& vert1 = triangle[1];
	const cbtVector3& vert2 = triangle[2];

	cbtVector3 v10;
	v10 = vert1 - vert0;
	cbtVector3 v20;
	v20 = vert2 - vert0;

	cbtVector3 triangleNormal;
	triangleNormal = v10.cross(v20);

	const cbtScalar dist = vert0.dot(triangleNormal);
	cbtScalar dist_a = triangleNormal.dot(m_from);
	dist_a -= dist;
	cbtScalar dist_b = triangleNormal.dot(m_to);
	dist_b -= dist;

	if (dist_a * dist_b >= cbtScalar(0.0))
	{
		return;  // same sign
	}

	if (((m_flags & kF_FilterBackfaces) != 0) && (dist_a <= cbtScalar(0.0)))
	{
		// Backface, skip check
		return;
	}

	const cbtScalar proj_length = dist_a - dist_b;
	const cbtScalar distance = (dist_a) / (proj_length);
	// Now we have the intersection point on the plane, we'll see if it's inside the triangle
	// Add an epsilon as a tolerance for the raycast,
	// in case the ray hits exacly on the edge of the triangle.
	// It must be scaled for the triangle size.

	if (distance < m_hitFraction)
	{
		cbtScalar edge_tolerance = triangleNormal.length2();
		edge_tolerance *= cbtScalar(-0.0001);
		cbtVector3 point;
		point.setInterpolate3(m_from, m_to, distance);
		{
			cbtVector3 v0p;
			v0p = vert0 - point;
			cbtVector3 v1p;
			v1p = vert1 - point;
			cbtVector3 cp0;
			cp0 = v0p.cross(v1p);

			if ((cbtScalar)(cp0.dot(triangleNormal)) >= edge_tolerance)
			{
				cbtVector3 v2p;
				v2p = vert2 - point;
				cbtVector3 cp1;
				cp1 = v1p.cross(v2p);
				if ((cbtScalar)(cp1.dot(triangleNormal)) >= edge_tolerance)
				{
					cbtVector3 cp2;
					cp2 = v2p.cross(v0p);

					if ((cbtScalar)(cp2.dot(triangleNormal)) >= edge_tolerance)
					{
						//@BP Mod
						// Triangle normal isn't normalized
						triangleNormal.normalize();

						//@BP Mod - Allow for unflipped normal when raycasting against backfaces
						if (((m_flags & kF_KeepUnflippedNormal) == 0) && (dist_a <= cbtScalar(0.0)))
						{
							m_hitFraction = reportHit(-triangleNormal, distance, partId, triangleIndex);
						}
						else
						{
							m_hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex);
						}
					}
				}
			}
		}
	}
}

cbtTriangleConvexcastCallback::cbtTriangleConvexcastCallback(const cbtConvexShape* convexShape, const cbtTransform& convexShapeFrom, const cbtTransform& convexShapeTo, const cbtTransform& triangleToWorld, const cbtScalar triangleCollisionMargin)
{
	m_convexShape = convexShape;
	m_convexShapeFrom = convexShapeFrom;
	m_convexShapeTo = convexShapeTo;
	m_triangleToWorld = triangleToWorld;
	m_hitFraction = 1.0f;
	m_triangleCollisionMargin = triangleCollisionMargin;
	m_allowedPenetration = 0.f;
}

void cbtTriangleConvexcastCallback::processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
{
	cbtTriangleShape triangleShape(triangle[0], triangle[1], triangle[2]);
	triangleShape.setMargin(m_triangleCollisionMargin);

	cbtVoronoiSimplexSolver simplexSolver;
	cbtGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver;

//#define  USE_SUBSIMPLEX_CONVEX_CAST 1
//if you reenable USE_SUBSIMPLEX_CONVEX_CAST see commented out code below
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
	cbtSubsimplexConvexCast convexCaster(m_convexShape, &triangleShape, &simplexSolver);
#else
	//cbtGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
	cbtContinuousConvexCollision convexCaster(m_convexShape, &triangleShape, &simplexSolver, &gjkEpaPenetrationSolver);
#endif  //#USE_SUBSIMPLEX_CONVEX_CAST

	cbtConvexCast::CastResult castResult;
	castResult.m_fraction = cbtScalar(1.);
	castResult.m_allowedPenetration = m_allowedPenetration;
	if (convexCaster.calcTimeOfImpact(m_convexShapeFrom, m_convexShapeTo, m_triangleToWorld, m_triangleToWorld, castResult))
	{
		//add hit
		if (castResult.m_normal.length2() > cbtScalar(0.0001))
		{
			if (castResult.m_fraction < m_hitFraction)
			{
				/* cbtContinuousConvexCast's normal is already in world space */
				/*
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
				//rotate normal into worldspace
				castResult.m_normal = m_convexShapeFrom.getBasis() * castResult.m_normal;
#endif //USE_SUBSIMPLEX_CONVEX_CAST
*/
				castResult.m_normal.normalize();

				reportHit(castResult.m_normal,
						  castResult.m_hitPoint,
						  castResult.m_fraction,
						  partId,
						  triangleIndex);
			}
		}
	}
}
