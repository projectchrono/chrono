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

#include "cbtContinuousConvexCollision.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSimplexSolverInterface.h"
#include "LinearMath/cbtTransformUtil.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"

#include "cbtGjkPairDetector.h"
#include "cbtPointCollector.h"
#include "BulletCollision/CollisionShapes/cbtStaticPlaneShape.h"

cbtContinuousConvexCollision::cbtContinuousConvexCollision(const cbtConvexShape* convexA, const cbtConvexShape* convexB, cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* penetrationDepthSolver)
	: m_simplexSolver(simplexSolver),
	  m_penetrationDepthSolver(penetrationDepthSolver),
	  m_convexA(convexA),
	  m_convexB1(convexB),
	  m_planeShape(0)
{
}

cbtContinuousConvexCollision::cbtContinuousConvexCollision(const cbtConvexShape* convexA, const cbtStaticPlaneShape* plane)
	: m_simplexSolver(0),
	  m_penetrationDepthSolver(0),
	  m_convexA(convexA),
	  m_convexB1(0),
	  m_planeShape(plane)
{
}

/// This maximum should not be necessary. It allows for untested/degenerate cases in production code.
/// You don't want your game ever to lock-up.
#define MAX_ITERATIONS 64

void cbtContinuousConvexCollision::computeClosestPoints(const cbtTransform& transA, const cbtTransform& transB, cbtPointCollector& pointCollector)
{
	if (m_convexB1)
	{
		m_simplexSolver->reset();
		cbtGjkPairDetector gjk(m_convexA, m_convexB1, m_convexA->getShapeType(), m_convexB1->getShapeType(), m_convexA->getMargin(), m_convexB1->getMargin(), m_simplexSolver, m_penetrationDepthSolver);
		cbtGjkPairDetector::ClosestPointInput input;
		input.m_transformA = transA;
		input.m_transformB = transB;
		gjk.getClosestPoints(input, pointCollector, 0);
	}
	else
	{
		//convex versus plane
		const cbtConvexShape* convexShape = m_convexA;
		const cbtStaticPlaneShape* planeShape = m_planeShape;

		const cbtVector3& planeNormal = planeShape->getPlaneNormal();
		const cbtScalar& planeConstant = planeShape->getPlaneConstant();

		cbtTransform convexWorldTransform = transA;
		cbtTransform convexInPlaneTrans;
		convexInPlaneTrans = transB.inverse() * convexWorldTransform;
		cbtTransform planeInConvex;
		planeInConvex = convexWorldTransform.inverse() * transB;

		cbtVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis() * -planeNormal);

		cbtVector3 vtxInPlane = convexInPlaneTrans(vtx);
		cbtScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

		cbtVector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
		cbtVector3 vtxInPlaneWorld = transB * vtxInPlaneProjected;
		cbtVector3 normalOnSurfaceB = transB.getBasis() * planeNormal;

		pointCollector.addContactPoint(
			normalOnSurfaceB,
			vtxInPlaneWorld,
			distance);
	}
}

bool cbtContinuousConvexCollision::calcTimeOfImpact(
	const cbtTransform& fromA,
	const cbtTransform& toA,
	const cbtTransform& fromB,
	const cbtTransform& toB,
	CastResult& result)
{
	/// compute linear and angular velocity for this interval, to interpolate
	cbtVector3 linVelA, angVelA, linVelB, angVelB;
	cbtTransformUtil::calculateVelocity(fromA, toA, cbtScalar(1.), linVelA, angVelA);
	cbtTransformUtil::calculateVelocity(fromB, toB, cbtScalar(1.), linVelB, angVelB);

	cbtScalar boundingRadiusA = m_convexA->getAngularMotionDisc();
	cbtScalar boundingRadiusB = m_convexB1 ? m_convexB1->getAngularMotionDisc() : 0.f;

	cbtScalar maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA + angVelB.length() * boundingRadiusB;
	cbtVector3 relLinVel = (linVelB - linVelA);

	cbtScalar relLinVelocLength = (linVelB - linVelA).length();

	if ((relLinVelocLength + maxAngularProjectedVelocity) == 0.f)
		return false;

	cbtScalar lambda = cbtScalar(0.);

	cbtVector3 n;
	n.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	bool hasResult = false;
	cbtVector3 c;

	cbtScalar lastLambda = lambda;
	//cbtScalar epsilon = cbtScalar(0.001);

	int numIter = 0;
	//first solution, using GJK

	cbtScalar radius = 0.001f;
	//	result.drawCoordSystem(sphereTr);

	cbtPointCollector pointCollector1;

	{
		computeClosestPoints(fromA, fromB, pointCollector1);

		hasResult = pointCollector1.m_hasResult;
		c = pointCollector1.m_pointInWorld;
	}

	if (hasResult)
	{
		cbtScalar dist;
		dist = pointCollector1.m_distance + result.m_allowedPenetration;
		n = pointCollector1.m_normalOnBInWorld;
		cbtScalar projectedLinearVelocity = relLinVel.dot(n);
		if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= SIMD_EPSILON)
			return false;

		//not close enough
		while (dist > radius)
		{
			if (result.m_debugDrawer)
			{
				result.m_debugDrawer->drawSphere(c, 0.2f, cbtVector3(1, 1, 1));
			}
			cbtScalar dLambda = cbtScalar(0.);

			projectedLinearVelocity = relLinVel.dot(n);

			//don't report time of impact for motion away from the contact normal (or causes minor penetration)
			if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= SIMD_EPSILON)
				return false;

			dLambda = dist / (projectedLinearVelocity + maxAngularProjectedVelocity);

			lambda += dLambda;

			if (lambda > cbtScalar(1.) || lambda < cbtScalar(0.))
				return false;

			//todo: next check with relative epsilon
			if (lambda <= lastLambda)
			{
				return false;
				//n.setValue(0,0,0);
				//break;
			}
			lastLambda = lambda;

			//interpolate to next lambda
			cbtTransform interpolatedTransA, interpolatedTransB, relativeTrans;

			cbtTransformUtil::integrateTransform(fromA, linVelA, angVelA, lambda, interpolatedTransA);
			cbtTransformUtil::integrateTransform(fromB, linVelB, angVelB, lambda, interpolatedTransB);
			relativeTrans = interpolatedTransB.inverseTimes(interpolatedTransA);

			if (result.m_debugDrawer)
			{
				result.m_debugDrawer->drawSphere(interpolatedTransA.getOrigin(), 0.2f, cbtVector3(1, 0, 0));
			}

			result.DebugDraw(lambda);

			cbtPointCollector pointCollector;
			computeClosestPoints(interpolatedTransA, interpolatedTransB, pointCollector);

			if (pointCollector.m_hasResult)
			{
				dist = pointCollector.m_distance + result.m_allowedPenetration;
				c = pointCollector.m_pointInWorld;
				n = pointCollector.m_normalOnBInWorld;
			}
			else
			{
				result.reportFailure(-1, numIter);
				return false;
			}

			numIter++;
			if (numIter > MAX_ITERATIONS)
			{
				result.reportFailure(-2, numIter);
				return false;
			}
		}

		result.m_fraction = lambda;
		result.m_normal = n;
		result.m_hitPoint = c;
		return true;
	}

	return false;
}
