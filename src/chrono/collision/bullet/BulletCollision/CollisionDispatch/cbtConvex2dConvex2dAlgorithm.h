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

#ifndef BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H
#define BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "LinearMath/cbtTransformUtil.h"  //for cbtConvexSeparatingDistanceUtil

class cbtConvexPenetrationDepthSolver;

///The convex2dConvex2dAlgorithm collision algorithm support 2d collision detection for cbtConvex2dShape
///Currently it requires the cbtMinkowskiPenetrationDepthSolver, it has support for 2d penetration depth computation
class cbtConvex2dConvex2dAlgorithm : public cbtActivatingCollisionAlgorithm
{
	cbtSimplexSolverInterface* m_simplexSolver;
	cbtConvexPenetrationDepthSolver* m_pdSolver;

	bool m_ownManifold;
	cbtPersistentManifold* m_manifoldPtr;
	bool m_lowLevelOfDetail;

public:
	cbtConvex2dConvex2dAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);

	virtual ~cbtConvex2dConvex2dAlgorithm();

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
		cbtSimplexSolverInterface* m_simplexSolver;
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		CreateFunc(cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* pdSolver);

		virtual ~CreateFunc();

		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtConvex2dConvex2dAlgorithm));
			return new (mem) cbtConvex2dConvex2dAlgorithm(ci.m_manifold, ci, body0Wrap, body1Wrap, m_simplexSolver, m_pdSolver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
		}
	};
};

#endif  //BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H
