/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#ifndef BT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H
#define BT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H

#include "cbtCompoundCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseInterface.h"

#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
class cbtDispatcher;
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"
#include "LinearMath/cbtAlignedObjectArray.h"
#include "BulletCollision/CollisionDispatch/cbtHashedSimplePairCache.h"
class cbtDispatcher;
class cbtCollisionObject;

class cbtCollisionShape;

extern cbtShapePairCallback gCompoundCompoundChildShapePairCallback;

/// cbtCompoundCompoundCollisionAlgorithm  supports collision between two cbtCompoundCollisionShape shapes
class cbtCompoundCompoundCollisionAlgorithm : public cbtCompoundCollisionAlgorithm
{
	class cbtHashedSimplePairCache* m_childCollisionAlgorithmCache;
	cbtSimplePairArray m_removePairs;

	int m_compoundShapeRevision0;  //to keep track of changes, so that childAlgorithm array can be updated
	int m_compoundShapeRevision1;

	void removeChildAlgorithms();

	//	void	preallocateChildAlgorithms(const cbtCollisionObjectWrapper* body0Wrap,const cbtCollisionObjectWrapper* body1Wrap);

public:
	cbtCompoundCompoundCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped);

	virtual ~cbtCompoundCompoundCollisionAlgorithm();

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray);

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtCompoundCompoundCollisionAlgorithm));
			return new (mem) cbtCompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
		}
	};

	struct SwappedCreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtCompoundCompoundCollisionAlgorithm));
			return new (mem) cbtCompoundCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
		}
	};
};

#endif  //BT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H
