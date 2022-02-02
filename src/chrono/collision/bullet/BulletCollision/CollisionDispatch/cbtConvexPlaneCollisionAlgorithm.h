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

#ifndef BT_CONVEX_PLANE_COLLISION_ALGORITHM_H
#define BT_CONVEX_PLANE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"
class cbtPersistentManifold;
#include "cbtCollisionDispatcher.h"

#include "LinearMath/cbtVector3.h"

/// cbtSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class cbtConvexPlaneCollisionAlgorithm : public cbtCollisionAlgorithm
{
	bool m_ownManifold;
	cbtPersistentManifold* m_manifoldPtr;
	bool m_isSwapped;
	int m_numPerturbationIterations;
	int m_minimumPointsPerturbationThreshold;

public:
	cbtConvexPlaneCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold);

	virtual ~cbtConvexPlaneCollisionAlgorithm();

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	void collideSingleContact(const cbtQuaternion& perturbeRot, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		CreateFunc()
			: m_numPerturbationIterations(1),
			  m_minimumPointsPerturbationThreshold(0)
		{
		}

		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtConvexPlaneCollisionAlgorithm));
			if (!m_swapped)
			{
				return new (mem) cbtConvexPlaneCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
			}
			else
			{
				return new (mem) cbtConvexPlaneCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
			}
		}
	};
};

#endif  //BT_CONVEX_PLANE_COLLISION_ALGORITHM_H
