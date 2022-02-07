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

#ifndef BT_SPHERE_BOX_COLLISION_ALGORITHM_H
#define BT_SPHERE_BOX_COLLISION_ALGORITHM_H

#include "cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"
class cbtPersistentManifold;
#include "cbtCollisionDispatcher.h"

#include "LinearMath/cbtVector3.h"

/// cbtSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class cbtSphereBoxCollisionAlgorithm : public cbtActivatingCollisionAlgorithm
{
	bool m_ownManifold;
	cbtPersistentManifold* m_manifoldPtr;
	bool m_isSwapped;

public:
	cbtSphereBoxCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped);

	virtual ~cbtSphereBoxCollisionAlgorithm();

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	bool getSphereDistance(const cbtCollisionObjectWrapper* boxObjWrap, cbtVector3& v3PointOnBox, cbtVector3& normal, cbtScalar& penetrationDepth, const cbtVector3& v3SphereCenter, cbtScalar fRadius, cbtScalar maxContactDistance);

	cbtScalar getSpherePenetration(cbtVector3 const& boxHalfExtent, cbtVector3 const& sphereRelPos, cbtVector3& closestPoint, cbtVector3& normal);

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtSphereBoxCollisionAlgorithm));
			if (!m_swapped)
			{
				return new (mem) cbtSphereBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
			}
			else
			{
				return new (mem) cbtSphereBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
			}
		}
	};
};

#endif  //BT_SPHERE_BOX_COLLISION_ALGORITHM_H
