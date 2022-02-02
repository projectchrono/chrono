/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_GHOST_OBJECT_H
#define BT_GHOST_OBJECT_H

#include "cbtCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/cbtOverlappingPairCallback.h"
#include "LinearMath/cbtAlignedAllocator.h"
#include "BulletCollision/BroadphaseCollision/cbtOverlappingPairCache.h"
#include "cbtCollisionWorld.h"

class cbtConvexShape;

class cbtDispatcher;

///The cbtGhostObject can keep track of all objects that are overlapping
///By default, this overlap is based on the AABB
///This is useful for creating a character controller, collision sensors/triggers, explosions etc.
///We plan on adding rayTest and other queries for the cbtGhostObject
ATTRIBUTE_ALIGNED16(class)
cbtGhostObject : public cbtCollisionObject
{
protected:
	cbtAlignedObjectArray<cbtCollisionObject*> m_overlappingObjects;

public:
	cbtGhostObject();

	virtual ~cbtGhostObject();

	void convexSweepTest(const class cbtConvexShape* castShape, const cbtTransform& convexFromWorld, const cbtTransform& convexToWorld, cbtCollisionWorld::ConvexResultCallback& resultCallback, cbtScalar allowedCcdPenetration = 0.f) const;

	void rayTest(const cbtVector3& rayFromWorld, const cbtVector3& rayToWorld, cbtCollisionWorld::RayResultCallback& resultCallback) const;

	///this method is mainly for expert/internal use only.
	virtual void addOverlappingObjectInternal(cbtBroadphaseProxy * otherProxy, cbtBroadphaseProxy* thisProxy = 0);
	///this method is mainly for expert/internal use only.
	virtual void removeOverlappingObjectInternal(cbtBroadphaseProxy * otherProxy, cbtDispatcher * dispatcher, cbtBroadphaseProxy* thisProxy = 0);

	int getNumOverlappingObjects() const
	{
		return m_overlappingObjects.size();
	}

	cbtCollisionObject* getOverlappingObject(int index)
	{
		return m_overlappingObjects[index];
	}

	const cbtCollisionObject* getOverlappingObject(int index) const
	{
		return m_overlappingObjects[index];
	}

	cbtAlignedObjectArray<cbtCollisionObject*>& getOverlappingPairs()
	{
		return m_overlappingObjects;
	}

	const cbtAlignedObjectArray<cbtCollisionObject*> getOverlappingPairs() const
	{
		return m_overlappingObjects;
	}

	//
	// internal cast
	//

	static const cbtGhostObject* upcast(const cbtCollisionObject* colObj)
	{
		if (colObj->getInternalType() == CO_GHOST_OBJECT)
			return (const cbtGhostObject*)colObj;
		return 0;
	}
	static cbtGhostObject* upcast(cbtCollisionObject * colObj)
	{
		if (colObj->getInternalType() == CO_GHOST_OBJECT)
			return (cbtGhostObject*)colObj;
		return 0;
	}
};

class cbtPairCachingGhostObject : public cbtGhostObject
{
	cbtHashedOverlappingPairCache* m_hashPairCache;

public:
	cbtPairCachingGhostObject();

	virtual ~cbtPairCachingGhostObject();

	///this method is mainly for expert/internal use only.
	virtual void addOverlappingObjectInternal(cbtBroadphaseProxy* otherProxy, cbtBroadphaseProxy* thisProxy = 0);

	virtual void removeOverlappingObjectInternal(cbtBroadphaseProxy* otherProxy, cbtDispatcher* dispatcher, cbtBroadphaseProxy* thisProxy = 0);

	cbtHashedOverlappingPairCache* getOverlappingPairCache()
	{
		return m_hashPairCache;
	}
};

///The cbtGhostPairCallback interfaces and forwards adding and removal of overlapping pairs from the cbtBroadphaseInterface to cbtGhostObject.
class cbtGhostPairCallback : public cbtOverlappingPairCallback
{
public:
	cbtGhostPairCallback()
	{
	}

	virtual ~cbtGhostPairCallback()
	{
	}

	virtual cbtBroadphasePair* addOverlappingPair(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1)
	{
		cbtCollisionObject* colObj0 = (cbtCollisionObject*)proxy0->m_clientObject;
		cbtCollisionObject* colObj1 = (cbtCollisionObject*)proxy1->m_clientObject;
		cbtGhostObject* ghost0 = cbtGhostObject::upcast(colObj0);
		cbtGhostObject* ghost1 = cbtGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->addOverlappingObjectInternal(proxy1, proxy0);
		if (ghost1)
			ghost1->addOverlappingObjectInternal(proxy0, proxy1);
		return 0;
	}

	virtual void* removeOverlappingPair(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1, cbtDispatcher* dispatcher)
	{
		cbtCollisionObject* colObj0 = (cbtCollisionObject*)proxy0->m_clientObject;
		cbtCollisionObject* colObj1 = (cbtCollisionObject*)proxy1->m_clientObject;
		cbtGhostObject* ghost0 = cbtGhostObject::upcast(colObj0);
		cbtGhostObject* ghost1 = cbtGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->removeOverlappingObjectInternal(proxy1, dispatcher, proxy0);
		if (ghost1)
			ghost1->removeOverlappingObjectInternal(proxy0, dispatcher, proxy1);
		return 0;
	}

	virtual void removeOverlappingPairsContainingProxy(cbtBroadphaseProxy* /*proxy0*/, cbtDispatcher* /*dispatcher*/)
	{
		cbtAssert(0);
		//need to keep track of all ghost objects and call them here
		//m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
	}
};

#endif
