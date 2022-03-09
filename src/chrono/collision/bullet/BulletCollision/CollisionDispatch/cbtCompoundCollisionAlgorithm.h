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

#ifndef BT_COMPOUND_COLLISION_ALGORITHM_H
#define BT_COMPOUND_COLLISION_ALGORITHM_H

#include "cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseInterface.h"

#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
class cbtDispatcher;
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "cbtCollisionCreateFunc.h"
#include "LinearMath/cbtAlignedObjectArray.h"
#include "BulletCollision/BroadphaseCollision/cbtDbvt.h"
class cbtDispatcher;
class cbtCollisionObject;

class cbtCollisionShape;
typedef bool (*cbtShapePairCallback)(const cbtCollisionShape* pShape0, const cbtCollisionShape* pShape1);
extern cbtShapePairCallback gCompoundChildShapePairCallback;

/// cbtCompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes
class cbtCompoundCollisionAlgorithm : public cbtActivatingCollisionAlgorithm
{
	cbtNodeStack stack2;
	cbtManifoldArray manifoldArray;

protected:
	cbtAlignedObjectArray<cbtCollisionAlgorithm*> m_childCollisionAlgorithms;
	bool m_isSwapped;

	class cbtPersistentManifold* m_sharedManifold;
	bool m_ownsManifold;

	int m_compoundShapeRevision;  //to keep track of changes, so that childAlgorithm array can be updated

	void removeChildAlgorithms();

	void preallocateChildAlgorithms(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap);

public:
	cbtCompoundCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped);

	virtual ~cbtCompoundCollisionAlgorithm();

	cbtCollisionAlgorithm* getChildAlgorithm(int n) const
	{
		return m_childCollisionAlgorithms[n];
	}

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		int i;
		for (i = 0; i < m_childCollisionAlgorithms.size(); i++)
		{
			if (m_childCollisionAlgorithms[i])
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
		}
	}

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtCompoundCollisionAlgorithm));
			return new (mem) cbtCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
		}
	};

	struct SwappedCreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtCompoundCollisionAlgorithm));
			return new (mem) cbtCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
		}
	};
};

#endif  //BT_COMPOUND_COLLISION_ALGORITHM_H
