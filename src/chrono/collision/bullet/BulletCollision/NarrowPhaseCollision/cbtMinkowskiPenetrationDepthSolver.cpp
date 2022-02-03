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

#include "cbtMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"

#define NUM_UNITSPHERE_POINTS 42

bool cbtMinkowskiPenetrationDepthSolver::calcPenDepth(cbtSimplexSolverInterface& simplexSolver,
													 const cbtConvexShape* convexA, const cbtConvexShape* convexB,
													 const cbtTransform& transA, const cbtTransform& transB,
													 cbtVector3& v, cbtVector3& pa, cbtVector3& pb,
													 class cbtIDebugDraw* debugDraw)
{
	(void)v;

	bool check2d = convexA->isConvex2d() && convexB->isConvex2d();

	struct cbtIntermediateResult : public cbtDiscreteCollisionDetectorInterface::Result
	{
		cbtIntermediateResult() : m_hasResult(false)
		{
		}

		cbtVector3 m_normalOnBInWorld;
		cbtVector3 m_pointInWorld;
		cbtScalar m_depth;
		bool m_hasResult;

		virtual void setShapeIdentifiersA(int partId0, int index0)
		{
			(void)partId0;
			(void)index0;
		}
		virtual void setShapeIdentifiersB(int partId1, int index1)
		{
			(void)partId1;
			(void)index1;
		}
		void addContactPoint(const cbtVector3& normalOnBInWorld, const cbtVector3& pointInWorld, cbtScalar depth)
		{
			m_normalOnBInWorld = normalOnBInWorld;
			m_pointInWorld = pointInWorld;
			m_depth = depth;
			m_hasResult = true;
		}
	};

	//just take fixed number of orientation, and sample the penetration depth in that direction
	cbtScalar minProj = cbtScalar(BT_LARGE_FLOAT);
	cbtVector3 minNorm(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	cbtVector3 minA, minB;
	cbtVector3 seperatingAxisInA, seperatingAxisInB;
	cbtVector3 pInA, qInB, pWorld, qWorld, w;

#ifndef __SPU__
#define USE_BATCHED_SUPPORT 1
#endif
#ifdef USE_BATCHED_SUPPORT

	cbtVector3 supportVerticesABatch[NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
	cbtVector3 supportVerticesBBatch[NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
	cbtVector3 seperatingAxisInABatch[NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
	cbtVector3 seperatingAxisInBBatch[NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
	int i;

	int numSampleDirections = NUM_UNITSPHERE_POINTS;

	for (i = 0; i < numSampleDirections; i++)
	{
		cbtVector3 norm = getPenetrationDirections()[i];
		seperatingAxisInABatch[i] = (-norm) * transA.getBasis();
		seperatingAxisInBBatch[i] = norm * transB.getBasis();
	}

	{
		int numPDA = convexA->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i = 0; i < numPDA; i++)
			{
				cbtVector3 norm;
				convexA->getPreferredPenetrationDirection(i, norm);
				norm = transA.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.getBasis();
				seperatingAxisInBBatch[numSampleDirections] = norm * transB.getBasis();
				numSampleDirections++;
			}
		}
	}

	{
		int numPDB = convexB->getNumPreferredPenetrationDirections();
		if (numPDB)
		{
			for (int i = 0; i < numPDB; i++)
			{
				cbtVector3 norm;
				convexB->getPreferredPenetrationDirection(i, norm);
				norm = transB.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.getBasis();
				seperatingAxisInBBatch[numSampleDirections] = norm * transB.getBasis();
				numSampleDirections++;
			}
		}
	}

	convexA->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch, supportVerticesABatch, numSampleDirections);
	convexB->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch, supportVerticesBBatch, numSampleDirections);

	for (i = 0; i < numSampleDirections; i++)
	{
		cbtVector3 norm = getPenetrationDirections()[i];
		if (check2d)
		{
			norm[2] = 0.f;
		}
		if (norm.length2() > 0.01)
		{
			seperatingAxisInA = seperatingAxisInABatch[i];
			seperatingAxisInB = seperatingAxisInBBatch[i];

			pInA = supportVerticesABatch[i];
			qInB = supportVerticesBBatch[i];

			pWorld = transA(pInA);
			qWorld = transB(qInB);
			if (check2d)
			{
				pWorld[2] = 0.f;
				qWorld[2] = 0.f;
			}

			w = qWorld - pWorld;
			cbtScalar delta = norm.dot(w);
			//find smallest delta
			if (delta < minProj)
			{
				minProj = delta;
				minNorm = norm;
				minA = pWorld;
				minB = qWorld;
			}
		}
	}
#else

	int numSampleDirections = NUM_UNITSPHERE_POINTS;

#ifndef __SPU__
	{
		int numPDA = convexA->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i = 0; i < numPDA; i++)
			{
				cbtVector3 norm;
				convexA->getPreferredPenetrationDirection(i, norm);
				norm = transA.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}

	{
		int numPDB = convexB->getNumPreferredPenetrationDirections();
		if (numPDB)
		{
			for (int i = 0; i < numPDB; i++)
			{
				cbtVector3 norm;
				convexB->getPreferredPenetrationDirection(i, norm);
				norm = transB.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}
#endif  // __SPU__

	for (int i = 0; i < numSampleDirections; i++)
	{
		const cbtVector3& norm = getPenetrationDirections()[i];
		seperatingAxisInA = (-norm) * transA.getBasis();
		seperatingAxisInB = norm * transB.getBasis();
		pInA = convexA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
		qInB = convexB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);
		pWorld = transA(pInA);
		qWorld = transB(qInB);
		w = qWorld - pWorld;
		cbtScalar delta = norm.dot(w);
		//find smallest delta
		if (delta < minProj)
		{
			minProj = delta;
			minNorm = norm;
			minA = pWorld;
			minB = qWorld;
		}
	}
#endif  //USE_BATCHED_SUPPORT

	//add the margins

	minA += minNorm * convexA->getMarginNonVirtual();
	minB -= minNorm * convexB->getMarginNonVirtual();
	//no penetration
	if (minProj < cbtScalar(0.))
		return false;

	cbtScalar extraSeparation = 0.5f;  ///scale dependent
	minProj += extraSeparation + (convexA->getMarginNonVirtual() + convexB->getMarginNonVirtual());

//#define DEBUG_DRAW 1
#ifdef DEBUG_DRAW
	if (debugDraw)
	{
		cbtVector3 color(0, 1, 0);
		debugDraw->drawLine(minA, minB, color);
		color = cbtVector3(1, 1, 1);
		cbtVector3 vec = minB - minA;
		cbtScalar prj2 = minNorm.dot(vec);
		debugDraw->drawLine(minA, minA + (minNorm * minProj), color);
	}
#endif  //DEBUG_DRAW

	cbtGjkPairDetector gjkdet(convexA, convexB, &simplexSolver, 0);

	cbtScalar offsetDist = minProj;
	cbtVector3 offset = minNorm * offsetDist;

	cbtGjkPairDetector::ClosestPointInput input;

	cbtVector3 newOrg = transA.getOrigin() + offset;

	cbtTransform displacedTrans = transA;
	displacedTrans.setOrigin(newOrg);

	input.m_transformA = displacedTrans;
	input.m_transformB = transB;
	input.m_maximumDistanceSquared = cbtScalar(BT_LARGE_FLOAT);  //minProj;

	cbtIntermediateResult res;
	gjkdet.setCachedSeperatingAxis(-minNorm);
	gjkdet.getClosestPoints(input, res, debugDraw);

	cbtScalar correctedMinNorm = minProj - res.m_depth;

	//the penetration depth is over-estimated, relax it
	cbtScalar penetration_relaxation = cbtScalar(1.);
	minNorm *= penetration_relaxation;

	if (res.m_hasResult)
	{
		pa = res.m_pointInWorld - minNorm * correctedMinNorm;
		pb = res.m_pointInWorld;
		v = minNorm;

#ifdef DEBUG_DRAW
		if (debugDraw)
		{
			cbtVector3 color(1, 0, 0);
			debugDraw->drawLine(pa, pb, color);
		}
#endif  //DEBUG_DRAW
	}
	return res.m_hasResult;
}

cbtVector3* cbtMinkowskiPenetrationDepthSolver::getPenetrationDirections()
{
	static cbtVector3 sPenetrationDirections[NUM_UNITSPHERE_POINTS + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2] =
		{
			cbtVector3(cbtScalar(0.000000), cbtScalar(-0.000000), cbtScalar(-1.000000)),
			cbtVector3(cbtScalar(0.723608), cbtScalar(-0.525725), cbtScalar(-0.447219)),
			cbtVector3(cbtScalar(-0.276388), cbtScalar(-0.850649), cbtScalar(-0.447219)),
			cbtVector3(cbtScalar(-0.894426), cbtScalar(-0.000000), cbtScalar(-0.447216)),
			cbtVector3(cbtScalar(-0.276388), cbtScalar(0.850649), cbtScalar(-0.447220)),
			cbtVector3(cbtScalar(0.723608), cbtScalar(0.525725), cbtScalar(-0.447219)),
			cbtVector3(cbtScalar(0.276388), cbtScalar(-0.850649), cbtScalar(0.447220)),
			cbtVector3(cbtScalar(-0.723608), cbtScalar(-0.525725), cbtScalar(0.447219)),
			cbtVector3(cbtScalar(-0.723608), cbtScalar(0.525725), cbtScalar(0.447219)),
			cbtVector3(cbtScalar(0.276388), cbtScalar(0.850649), cbtScalar(0.447219)),
			cbtVector3(cbtScalar(0.894426), cbtScalar(0.000000), cbtScalar(0.447216)),
			cbtVector3(cbtScalar(-0.000000), cbtScalar(0.000000), cbtScalar(1.000000)),
			cbtVector3(cbtScalar(0.425323), cbtScalar(-0.309011), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(-0.162456), cbtScalar(-0.499995), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(0.262869), cbtScalar(-0.809012), cbtScalar(-0.525738)),
			cbtVector3(cbtScalar(0.425323), cbtScalar(0.309011), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(0.850648), cbtScalar(-0.000000), cbtScalar(-0.525736)),
			cbtVector3(cbtScalar(-0.525730), cbtScalar(-0.000000), cbtScalar(-0.850652)),
			cbtVector3(cbtScalar(-0.688190), cbtScalar(-0.499997), cbtScalar(-0.525736)),
			cbtVector3(cbtScalar(-0.162456), cbtScalar(0.499995), cbtScalar(-0.850654)),
			cbtVector3(cbtScalar(-0.688190), cbtScalar(0.499997), cbtScalar(-0.525736)),
			cbtVector3(cbtScalar(0.262869), cbtScalar(0.809012), cbtScalar(-0.525738)),
			cbtVector3(cbtScalar(0.951058), cbtScalar(0.309013), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(0.951058), cbtScalar(-0.309013), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(0.587786), cbtScalar(-0.809017), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(0.000000), cbtScalar(-1.000000), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(-0.587786), cbtScalar(-0.809017), cbtScalar(0.000000)),
			cbtVector3(cbtScalar(-0.951058), cbtScalar(-0.309013), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(-0.951058), cbtScalar(0.309013), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(-0.587786), cbtScalar(0.809017), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(-0.000000), cbtScalar(1.000000), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(0.587786), cbtScalar(0.809017), cbtScalar(-0.000000)),
			cbtVector3(cbtScalar(0.688190), cbtScalar(-0.499997), cbtScalar(0.525736)),
			cbtVector3(cbtScalar(-0.262869), cbtScalar(-0.809012), cbtScalar(0.525738)),
			cbtVector3(cbtScalar(-0.850648), cbtScalar(0.000000), cbtScalar(0.525736)),
			cbtVector3(cbtScalar(-0.262869), cbtScalar(0.809012), cbtScalar(0.525738)),
			cbtVector3(cbtScalar(0.688190), cbtScalar(0.499997), cbtScalar(0.525736)),
			cbtVector3(cbtScalar(0.525730), cbtScalar(0.000000), cbtScalar(0.850652)),
			cbtVector3(cbtScalar(0.162456), cbtScalar(-0.499995), cbtScalar(0.850654)),
			cbtVector3(cbtScalar(-0.425323), cbtScalar(-0.309011), cbtScalar(0.850654)),
			cbtVector3(cbtScalar(-0.425323), cbtScalar(0.309011), cbtScalar(0.850654)),
			cbtVector3(cbtScalar(0.162456), cbtScalar(0.499995), cbtScalar(0.850654))};

	return sPenetrationDirections;
}
