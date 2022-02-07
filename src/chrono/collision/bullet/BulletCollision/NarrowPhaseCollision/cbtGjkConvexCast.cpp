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

#include "cbtGjkConvexCast.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "cbtGjkPairDetector.h"
#include "cbtPointCollector.h"
#include "LinearMath/cbtTransformUtil.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define MAX_ITERATIONS 64
#else
#define MAX_ITERATIONS 32
#endif

cbtGjkConvexCast::cbtGjkConvexCast(const cbtConvexShape* convexA, const cbtConvexShape* convexB, cbtSimplexSolverInterface* simplexSolver)
	: m_simplexSolver(simplexSolver),
	  m_convexA(convexA),
	  m_convexB(convexB)
{
}

bool cbtGjkConvexCast::calcTimeOfImpact(
	const cbtTransform& fromA,
	const cbtTransform& toA,
	const cbtTransform& fromB,
	const cbtTransform& toB,
	CastResult& result)
{
	m_simplexSolver->reset();

	/// compute linear velocity for this interval, to interpolate
	//assume no rotation/angular velocity, assert here?
	cbtVector3 linVelA, linVelB;
	linVelA = toA.getOrigin() - fromA.getOrigin();
	linVelB = toB.getOrigin() - fromB.getOrigin();

	cbtScalar radius = cbtScalar(0.001);
	cbtScalar lambda = cbtScalar(0.);
	cbtVector3 v(1, 0, 0);

	int maxIter = MAX_ITERATIONS;

	cbtVector3 n;
	n.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	bool hasResult = false;
	cbtVector3 c;
	cbtVector3 r = (linVelA - linVelB);

	cbtScalar lastLambda = lambda;
	//cbtScalar epsilon = cbtScalar(0.001);

	int numIter = 0;
	//first solution, using GJK

	cbtTransform identityTrans;
	identityTrans.setIdentity();

	//	result.drawCoordSystem(sphereTr);

	cbtPointCollector pointCollector;

	cbtGjkPairDetector gjk(m_convexA, m_convexB, m_simplexSolver, 0);  //m_penetrationDepthSolver);
	cbtGjkPairDetector::ClosestPointInput input;

	//we don't use margins during CCD
	//	gjk.setIgnoreMargin(true);

	input.m_transformA = fromA;
	input.m_transformB = fromB;
	gjk.getClosestPoints(input, pointCollector, 0);

	hasResult = pointCollector.m_hasResult;
	c = pointCollector.m_pointInWorld;

	if (hasResult)
	{
		cbtScalar dist;
		dist = pointCollector.m_distance;
		n = pointCollector.m_normalOnBInWorld;

		//not close enough
		while (dist > radius)
		{
			numIter++;
			if (numIter > maxIter)
			{
				return false;  //todo: report a failure
			}
			cbtScalar dLambda = cbtScalar(0.);

			cbtScalar projectedLinearVelocity = r.dot(n);

			dLambda = dist / (projectedLinearVelocity);

			lambda = lambda - dLambda;

			if (lambda > cbtScalar(1.))
				return false;

			if (lambda < cbtScalar(0.))
				return false;

			//todo: next check with relative epsilon
			if (lambda <= lastLambda)
			{
				return false;
				//n.setValue(0,0,0);
				break;
			}
			lastLambda = lambda;

			//interpolate to next lambda
			result.DebugDraw(lambda);
			input.m_transformA.getOrigin().setInterpolate3(fromA.getOrigin(), toA.getOrigin(), lambda);
			input.m_transformB.getOrigin().setInterpolate3(fromB.getOrigin(), toB.getOrigin(), lambda);

			gjk.getClosestPoints(input, pointCollector, 0);
			if (pointCollector.m_hasResult)
			{
				if (pointCollector.m_distance < cbtScalar(0.))
				{
					result.m_fraction = lastLambda;
					n = pointCollector.m_normalOnBInWorld;
					result.m_normal = n;
					result.m_hitPoint = pointCollector.m_pointInWorld;
					return true;
				}
				c = pointCollector.m_pointInWorld;
				n = pointCollector.m_normalOnBInWorld;
				dist = pointCollector.m_distance;
			}
			else
			{
				//??
				return false;
			}
		}

		//is n normalized?
		//don't report time of impact for motion away from the contact normal (or causes minor penetration)
		if (n.dot(r) >= -result.m_allowedPenetration)
			return false;

		result.m_fraction = lambda;
		result.m_normal = n;
		result.m_hitPoint = c;
		return true;
	}

	return false;
}
