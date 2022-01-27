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

#ifndef BT_CONVEX_CONVEX_ALGORITHM_H
#define BT_CONVEX_CONVEX_ALGORITHM_H

#include "cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.h"
#include "cbtCollisionCreateFunc.h"
#include "cbtCollisionDispatcher.h"
#include "LinearMath/cbtTransformUtil.h"  //for cbtConvexSeparatingDistanceUtil
#include "BulletCollision/NarrowPhaseCollision/cbtPolyhedralContactClipping.h"

class cbtConvexPenetrationDepthSolver;

///Enabling USE_SEPDISTANCE_UTIL2 requires 100% reliable distance computation. However, when using large size ratios GJK can be imprecise
///so the distance is not conservative. In that case, enabling this USE_SEPDISTANCE_UTIL2 would result in failing/missing collisions.
///Either improve GJK for large size ratios (testing a 100 units versus a 0.1 unit object) or only enable the util
///for certain pairs that have a small size ratio

//#define USE_SEPDISTANCE_UTIL2 1

///The convexConvexAlgorithm collision algorithm implements time of impact, convex closest points and penetration depth calculations between two convex objects.
///Multiple contact points are calculated by perturbing the orientation of the smallest object orthogonal to the separating normal.
///This idea was described by Gino van den Bergen in this forum topic http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=4&t=288&p=888#p888
class cbtConvexConvexAlgorithm : public cbtActivatingCollisionAlgorithm
{
#ifdef USE_SEPDISTANCE_UTIL2
	cbtConvexSeparatingDistanceUtil m_sepDistance;
#endif
	cbtConvexPenetrationDepthSolver* m_pdSolver;

	cbtVertexArray worldVertsB1;
	cbtVertexArray worldVertsB2;

	bool m_ownManifold;
	cbtPersistentManifold* m_manifoldPtr;
	bool m_lowLevelOfDetail;

	int m_numPerturbationIterations;
	int m_minimumPointsPerturbationThreshold;

	///cache separating vector to speedup collision detection

public:
	cbtConvexConvexAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, cbtConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);

	virtual ~cbtConvexConvexAlgorithm();

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		///should we use m_ownManifold to avoid adding duplicates?
		if (m_manifoldPtr && m_ownManifold)
			manifoldArray.push_back(m_manifoldPtr);
	}

	void setLowLevelOfDetail(bool useLowLevel);

	const cbtPersistentManifold* getManifold()
	{
		return m_manifoldPtr;
	}

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		cbtConvexPenetrationDepthSolver* m_pdSolver;
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		CreateFunc(cbtConvexPenetrationDepthSolver* pdSolver);

		virtual ~CreateFunc();

		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtConvexConvexAlgorithm));
			return new (mem) cbtConvexConvexAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
		}
	};
};

#endif  //BT_CONVEX_CONVEX_ALGORITHM_H
