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

#ifndef BT_COLLISION__DISPATCHER_H
#define BT_COLLISION__DISPATCHER_H

#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"

#include "BulletCollision/CollisionDispatch/cbtManifoldResult.h"

#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "LinearMath/cbtAlignedObjectArray.h"

class cbtIDebugDraw;
class cbtOverlappingPairCache;
class cbtPoolAllocator;
class cbtCollisionConfiguration;

#include "cbtCollisionCreateFunc.h"

#define USE_DISPATCH_REGISTRY_ARRAY 1

class cbtCollisionDispatcher;
///user can override this nearcallback for collision filtering and more finegrained control over collision detection
typedef void (*cbtNearCallback)(cbtBroadphasePair& collisionPair, cbtCollisionDispatcher& dispatcher, const cbtDispatcherInfo& dispatchInfo);

///cbtCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
///Time of Impact, Closest Points and Penetration Depth.
class cbtCollisionDispatcher : public cbtDispatcher
{
protected:
	int m_dispatcherFlags;

	cbtAlignedObjectArray<cbtPersistentManifold*> m_manifoldsPtr;

	cbtManifoldResult m_defaultManifoldResult;

	cbtNearCallback m_nearCallback;

	cbtPoolAllocator* m_collisionAlgorithmPoolAllocator;

	cbtPoolAllocator* m_persistentManifoldPoolAllocator;

	cbtCollisionAlgorithmCreateFunc* m_doubleDispatchContactPoints[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];

	cbtCollisionAlgorithmCreateFunc* m_doubleDispatchClosestPoints[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];

	cbtCollisionConfiguration* m_collisionConfiguration;

public:
	enum DispatcherFlags
	{
		CD_STATIC_STATIC_REPORTED = 1,
		CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
		CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4
	};

	int getDispatcherFlags() const
	{
		return m_dispatcherFlags;
	}

	void setDispatcherFlags(int flags)
	{
		m_dispatcherFlags = flags;
	}

	///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
	void registerCollisionCreateFunc(int proxyType0, int proxyType1, cbtCollisionAlgorithmCreateFunc* createFunc);

	void registerClosestPointsCreateFunc(int proxyType0, int proxyType1, cbtCollisionAlgorithmCreateFunc* createFunc);

	int getNumManifolds() const
	{
		return int(m_manifoldsPtr.size());
	}

	cbtPersistentManifold** getInternalManifoldPointer()
	{
		return m_manifoldsPtr.size() ? &m_manifoldsPtr[0] : 0;
	}

	cbtPersistentManifold* getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	const cbtPersistentManifold* getManifoldByIndexInternal(int index) const
	{
		return m_manifoldsPtr[index];
	}

	cbtCollisionDispatcher(cbtCollisionConfiguration* collisionConfiguration);

	virtual ~cbtCollisionDispatcher();

	virtual cbtPersistentManifold* getNewManifold(const cbtCollisionObject* b0, const cbtCollisionObject* b1);

	virtual void releaseManifold(cbtPersistentManifold* manifold);

	virtual void clearManifold(cbtPersistentManifold* manifold);

	cbtCollisionAlgorithm* findAlgorithm(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, cbtPersistentManifold* sharedManifold, ecbtDispatcherQueryType queryType);

	virtual bool needsCollision(const cbtCollisionObject* body0, const cbtCollisionObject* body1);

	virtual bool needsResponse(const cbtCollisionObject* body0, const cbtCollisionObject* body1);

	virtual void dispatchAllCollisionPairs(cbtOverlappingPairCache* pairCache, const cbtDispatcherInfo& dispatchInfo, cbtDispatcher* dispatcher);

	void setNearCallback(cbtNearCallback nearCallback)
	{
		m_nearCallback = nearCallback;
	}

	cbtNearCallback getNearCallback() const
	{
		return m_nearCallback;
	}

	//by default, Bullet will use this near callback
	static void defaultNearCallback(cbtBroadphasePair& collisionPair, cbtCollisionDispatcher& dispatcher, const cbtDispatcherInfo& dispatchInfo);

	virtual void* allocateCollisionAlgorithm(int size);

	virtual void freeCollisionAlgorithm(void* ptr);

	cbtCollisionConfiguration* getCollisionConfiguration()
	{
		return m_collisionConfiguration;
	}

	const cbtCollisionConfiguration* getCollisionConfiguration() const
	{
		return m_collisionConfiguration;
	}

	void setCollisionConfiguration(cbtCollisionConfiguration* config)
	{
		m_collisionConfiguration = config;
	}

	virtual cbtPoolAllocator* getInternalManifoldPool()
	{
		return m_persistentManifoldPoolAllocator;
	}

	virtual const cbtPoolAllocator* getInternalManifoldPool() const
	{
		return m_persistentManifoldPoolAllocator;
	}
};

#endif  //BT_COLLISION__DISPATCHER_H
