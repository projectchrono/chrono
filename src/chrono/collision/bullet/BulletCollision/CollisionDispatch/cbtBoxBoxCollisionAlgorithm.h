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

#ifndef BT_BOX_BOX__COLLISION_ALGORITHM_H
#define BT_BOX_BOX__COLLISION_ALGORITHM_H

#include "cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"

class cbtPersistentManifold;

///box-box collision detection
class cbtBoxBoxCollisionAlgorithm : public cbtActivatingCollisionAlgorithm
{
	bool m_ownManifold;
	cbtPersistentManifold* m_manifoldPtr;

public:
	cbtBoxBoxCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci)
		: cbtActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	cbtBoxBoxCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap);

	virtual ~cbtBoxBoxCollisionAlgorithm();

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			int bbsize = sizeof(cbtBoxBoxCollisionAlgorithm);
			void* ptr = ci.m_dispatcher1->allocateCollisionAlgorithm(bbsize);
			return new (ptr) cbtBoxBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap);
		}
	};
};

#endif  //BT_BOX_BOX__COLLISION_ALGORITHM_H
