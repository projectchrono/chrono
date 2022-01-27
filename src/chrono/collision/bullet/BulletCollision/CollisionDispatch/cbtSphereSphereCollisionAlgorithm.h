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

#ifndef BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H
#define BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H

#include "cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"
#include "cbtCollisionDispatcher.h"

class cbtPersistentManifold;

/// cbtSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
/// Also provides the most basic sample for custom/user cbtCollisionAlgorithm
class cbtSphereSphereCollisionAlgorithm : public cbtActivatingCollisionAlgorithm
{
	bool m_ownManifold;
	cbtPersistentManifold* m_manifoldPtr;

public:
	cbtSphereSphereCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap);

	cbtSphereSphereCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci)
		: cbtActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	virtual ~cbtSphereSphereCollisionAlgorithm();

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtSphereSphereCollisionAlgorithm));
			return new (mem) cbtSphereSphereCollisionAlgorithm(0, ci, col0Wrap, col1Wrap);
		}
	};
};

#endif  //BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H
