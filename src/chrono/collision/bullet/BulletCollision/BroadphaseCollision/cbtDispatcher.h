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

#ifndef BT_DISPATCHER_H
#define BT_DISPATCHER_H
#include "LinearMath/cbtScalar.h"

class cbtCollisionAlgorithm;
struct cbtBroadphaseProxy;
class cbtRigidBody;
class cbtCollisionObject;
class cbtOverlappingPairCache;
struct cbtCollisionObjectWrapper;

class cbtPersistentManifold;
class cbtPoolAllocator;

struct cbtDispatcherInfo
{
	enum DispatchFunc
	{
		DISPATCH_DISCRETE = 1,
		DISPATCH_CONTINUOUS
	};
	cbtDispatcherInfo()
		: m_timeStep(cbtScalar(0.)),
		  m_stepCount(0),
		  m_dispatchFunc(DISPATCH_DISCRETE),
		  m_timeOfImpact(cbtScalar(1.)),
		  m_useContinuous(true),
		  m_debugDraw(0),
		  m_enableSatConvex(false),
		  m_enableSPU(true),
		  m_useEpa(true),
		  m_allowedCcdPenetration(cbtScalar(0.04)),
		  m_useConvexConservativeDistanceUtil(false),
		  m_convexConservativeDistanceThreshold(0.0f),
		  m_deterministicOverlappingPairs(false)
	{
	}
	cbtScalar m_timeStep;
	int m_stepCount;
	int m_dispatchFunc;
	mutable cbtScalar m_timeOfImpact;
	bool m_useContinuous;
	class cbtIDebugDraw* m_debugDraw;
	bool m_enableSatConvex;
	bool m_enableSPU;
	bool m_useEpa;
	cbtScalar m_allowedCcdPenetration;
	bool m_useConvexConservativeDistanceUtil;
	cbtScalar m_convexConservativeDistanceThreshold;
	bool m_deterministicOverlappingPairs;
};

enum ecbtDispatcherQueryType
{
	BT_CONTACT_POINT_ALGORITHMS = 1,
	BT_CLOSEST_POINT_ALGORITHMS = 2
};

///The cbtDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in cbtPersistentManifold or user callbacks (game logic).
class cbtDispatcher
{
public:
	virtual ~cbtDispatcher();

	virtual cbtCollisionAlgorithm* findAlgorithm(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, cbtPersistentManifold* sharedManifold, ecbtDispatcherQueryType queryType) = 0;

	virtual cbtPersistentManifold* getNewManifold(const cbtCollisionObject* b0, const cbtCollisionObject* b1) = 0;

	virtual void releaseManifold(cbtPersistentManifold* manifold) = 0;

	virtual void clearManifold(cbtPersistentManifold* manifold) = 0;

	virtual bool needsCollision(const cbtCollisionObject* body0, const cbtCollisionObject* body1) = 0;

	virtual bool needsResponse(const cbtCollisionObject* body0, const cbtCollisionObject* body1) = 0;

	virtual void dispatchAllCollisionPairs(cbtOverlappingPairCache* pairCache, const cbtDispatcherInfo& dispatchInfo, cbtDispatcher* dispatcher) = 0;

	virtual int getNumManifolds() const = 0;

	virtual cbtPersistentManifold* getManifoldByIndexInternal(int index) = 0;

	virtual cbtPersistentManifold** getInternalManifoldPointer() = 0;

	virtual cbtPoolAllocator* getInternalManifoldPool() = 0;

	virtual const cbtPoolAllocator* getInternalManifoldPool() const = 0;

	virtual void* allocateCollisionAlgorithm(int size) = 0;

	virtual void freeCollisionAlgorithm(void* ptr) = 0;
};

#endif  //BT_DISPATCHER_H
