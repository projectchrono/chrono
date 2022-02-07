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

#ifndef BT_COLLISION_DISPATCHER_MT_H
#define BT_COLLISION_DISPATCHER_MT_H

#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "LinearMath/cbtThreads.h"

class cbtCollisionDispatcherMt : public cbtCollisionDispatcher
{
public:
	cbtCollisionDispatcherMt(cbtCollisionConfiguration* config, int grainSize = 40);

	virtual cbtPersistentManifold* getNewManifold(const cbtCollisionObject* body0, const cbtCollisionObject* body1) BT_OVERRIDE;
	virtual void releaseManifold(cbtPersistentManifold* manifold) BT_OVERRIDE;

	virtual void dispatchAllCollisionPairs(cbtOverlappingPairCache* pairCache, const cbtDispatcherInfo& info, cbtDispatcher* dispatcher) BT_OVERRIDE;

protected:
	bool m_batchUpdating;
	int m_grainSize;
};

#endif  //BT_COLLISION_DISPATCHER_MT_H
