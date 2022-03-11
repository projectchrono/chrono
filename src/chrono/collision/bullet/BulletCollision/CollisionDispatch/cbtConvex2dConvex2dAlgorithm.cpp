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

#include "cbtConvex2dConvex2dAlgorithm.h"

//#include <stdio.h>
#include "BulletCollision/NarrowPhaseCollision/cbtDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "BulletCollision/CollisionShapes/cbtCapsuleShape.h"

#include "BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "BulletCollision/CollisionDispatch/cbtManifoldResult.h"

#include "BulletCollision/NarrowPhaseCollision/cbtConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/cbtContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkConvexCast.h"

#include "BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.h"

#include "BulletCollision/NarrowPhaseCollision/cbtGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

cbtConvex2dConvex2dAlgorithm::CreateFunc::CreateFunc(cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* pdSolver)
{
	m_simplexSolver = simplexSolver;
	m_pdSolver = pdSolver;
}

cbtConvex2dConvex2dAlgorithm::CreateFunc::~CreateFunc()
{
}

cbtConvex2dConvex2dAlgorithm::cbtConvex2dConvex2dAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* pdSolver, int /* numPerturbationIterations */, int /* minimumPointsPerturbationThreshold */)
	: cbtActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_simplexSolver(simplexSolver),
	  m_pdSolver(pdSolver),
	  m_ownManifold(false),
	  m_manifoldPtr(mf),
	  m_lowLevelOfDetail(false)
{
	(void)body0Wrap;
	(void)body1Wrap;
}

cbtConvex2dConvex2dAlgorithm::~cbtConvex2dConvex2dAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void cbtConvex2dConvex2dAlgorithm ::setLowLevelOfDetail(bool useLowLevel)
{
	m_lowLevelOfDetail = useLowLevel;
}

extern cbtScalar gContactBreakingThreshold;

//
// Convex-Convex collision algorithm
//
void cbtConvex2dConvex2dAlgorithm ::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
	{
		//swapped?
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
	resultOut->setPersistentManifold(m_manifoldPtr);

	//comment-out next line to test multi-contact generation
	//resultOut->getPersistentManifold()->clearManifold();

	const cbtConvexShape* min0 = static_cast<const cbtConvexShape*>(body0Wrap->getCollisionShape());
	const cbtConvexShape* min1 = static_cast<const cbtConvexShape*>(body1Wrap->getCollisionShape());

	cbtVector3 normalOnB;
	cbtVector3 pointOnBWorld;

	{
		cbtGjkPairDetector::ClosestPointInput input;

		cbtGjkPairDetector gjkPairDetector(min0, min1, m_simplexSolver, m_pdSolver);
		//TODO: if (dispatchInfo.m_useContinuous)
		gjkPairDetector.setMinkowskiA(min0);
		gjkPairDetector.setMinkowskiB(min1);

		{
			input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold();
			input.m_maximumDistanceSquared *= input.m_maximumDistanceSquared;
		}

		input.m_transformA = body0Wrap->getWorldTransform();
		input.m_transformB = body1Wrap->getWorldTransform();

		gjkPairDetector.getClosestPoints(input, *resultOut, dispatchInfo.m_debugDraw);

		cbtVector3 v0, v1;
		cbtVector3 sepNormalWorldSpace;
	}

	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
}

cbtScalar cbtConvex2dConvex2dAlgorithm::calculateTimeOfImpact(cbtCollisionObject* col0, cbtCollisionObject* col1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

	///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
	///col0->m_worldTransform,
	cbtScalar resultFraction = cbtScalar(1.);

	cbtScalar squareMot0 = (col0->getInterpolationWorldTransform().getOrigin() - col0->getWorldTransform().getOrigin()).length2();
	cbtScalar squareMot1 = (col1->getInterpolationWorldTransform().getOrigin() - col1->getWorldTransform().getOrigin()).length2();

	if (squareMot0 < col0->getCcdSquareMotionThreshold() &&
		squareMot1 < col1->getCcdSquareMotionThreshold())
		return resultFraction;

	//An adhoc way of testing the Continuous Collision Detection algorithms
	//One object is approximated as a sphere, to simplify things
	//Starting in penetration should report no time of impact
	//For proper CCD, better accuracy and handling of 'allowed' penetration should be added
	//also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)

	/// Convex0 against sphere for Convex1
	{
		cbtConvexShape* convex0 = static_cast<cbtConvexShape*>(col0->getCollisionShape());

		cbtSphereShape sphere1(col1->getCcdSweptSphereRadius());  //todo: allow non-zero sphere sizes, for better approximation
		cbtConvexCast::CastResult result;
		cbtVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object is simplified as a sphere
		cbtGjkConvexCast ccd1(convex0, &sphere1, &voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->getWorldTransform(), col0->getInterpolationWorldTransform(),
								  col1->getWorldTransform(), col1->getInterpolationWorldTransform(), result))
		{
			//store result.m_fraction in both bodies

			if (col0->getHitFraction() > result.m_fraction)
				col0->setHitFraction(result.m_fraction);

			if (col1->getHitFraction() > result.m_fraction)
				col1->setHitFraction(result.m_fraction);

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;
		}
	}

	/// Sphere (for convex0) against Convex1
	{
		cbtConvexShape* convex1 = static_cast<cbtConvexShape*>(col1->getCollisionShape());

		cbtSphereShape sphere0(col0->getCcdSweptSphereRadius());  //todo: allow non-zero sphere sizes, for better approximation
		cbtConvexCast::CastResult result;
		cbtVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object is simplified as a sphere
		cbtGjkConvexCast ccd1(&sphere0, convex1, &voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->getWorldTransform(), col0->getInterpolationWorldTransform(),
								  col1->getWorldTransform(), col1->getInterpolationWorldTransform(), result))
		{
			//store result.m_fraction in both bodies

			if (col0->getHitFraction() > result.m_fraction)
				col0->setHitFraction(result.m_fraction);

			if (col1->getHitFraction() > result.m_fraction)
				col1->setHitFraction(result.m_fraction);

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;
		}
	}

	return resultFraction;
}
