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

#ifndef BT_COLLISION_ALGORITHM_H
#define BT_COLLISION_ALGORITHM_H

#include "LinearMath/cbtScalar.h"
#include "LinearMath/cbtAlignedObjectArray.h"

struct cbtBroadphaseProxy;
class cbtDispatcher;
class cbtManifoldResult;
class cbtCollisionObject;
struct cbtCollisionObjectWrapper;
struct cbtDispatcherInfo;
class cbtPersistentManifold;

typedef cbtAlignedObjectArray<cbtPersistentManifold*> cbtManifoldArray;

struct cbtCollisionAlgorithmConstructionInfo
{
	cbtCollisionAlgorithmConstructionInfo()
		: m_dispatcher1(0),
		  m_manifold(0)
	{
	}
	cbtCollisionAlgorithmConstructionInfo(cbtDispatcher* dispatcher, int temp)
		: m_dispatcher1(dispatcher)
	{
		(void)temp;
	}

	cbtDispatcher* m_dispatcher1;
	cbtPersistentManifold* m_manifold;

	//	int	getDispatcherId();
};

///cbtCollisionAlgorithm is an collision interface that is compatible with the Broadphase and cbtDispatcher.
///It is persistent over frames
class cbtCollisionAlgorithm
{
protected:
	cbtDispatcher* m_dispatcher;

protected:
	//	int	getDispatcherId();

public:
	cbtCollisionAlgorithm(){};

	cbtCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci);

	virtual ~cbtCollisionAlgorithm(){};

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut) = 0;

	virtual cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut) = 0;

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray) = 0;
};

#endif  //BT_COLLISION_ALGORITHM_H
