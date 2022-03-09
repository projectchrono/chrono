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

#ifndef BT_BROADPHASE_INTERFACE_H
#define BT_BROADPHASE_INTERFACE_H

struct cbtDispatcherInfo;
class cbtDispatcher;
#include "cbtBroadphaseProxy.h"

class cbtOverlappingPairCache;

struct cbtBroadphaseAabbCallback
{
	virtual ~cbtBroadphaseAabbCallback() {}
	virtual bool process(const cbtBroadphaseProxy* proxy) = 0;
};

struct cbtBroadphaseRayCallback : public cbtBroadphaseAabbCallback
{
	///added some cached data to accelerate ray-AABB tests
	cbtVector3 m_rayDirectionInverse;
	unsigned int m_signs[3];
	cbtScalar m_lambda_max;

	virtual ~cbtBroadphaseRayCallback() {}

protected:
	cbtBroadphaseRayCallback() {}
};

#include "LinearMath/cbtVector3.h"

///The cbtBroadphaseInterface class provides an interface to detect aabb-overlapping object pairs.
///Some implementations for this broadphase interface include cbtAxisSweep3, bt32BitAxisSweep3 and cbtDbvtBroadphase.
///The actual overlapping pair management, storage, adding and removing of pairs is dealt by the cbtOverlappingPairCache class.
class cbtBroadphaseInterface
{
public:
	virtual ~cbtBroadphaseInterface() {}

	virtual cbtBroadphaseProxy* createProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher) = 0;
	virtual void destroyProxy(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher) = 0;
	virtual void setAabb(cbtBroadphaseProxy* proxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher) = 0;
	virtual void getAabb(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const = 0;

	virtual void rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtBroadphaseRayCallback& rayCallback, const cbtVector3& aabbMin = cbtVector3(0, 0, 0), const cbtVector3& aabbMax = cbtVector3(0, 0, 0)) = 0;

	virtual void aabbTest(const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtBroadphaseAabbCallback& callback) = 0;

	///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
	virtual void calculateOverlappingPairs(cbtDispatcher* dispatcher) = 0;

	virtual cbtOverlappingPairCache* getOverlappingPairCache() = 0;
	virtual const cbtOverlappingPairCache* getOverlappingPairCache() const = 0;

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(cbtVector3& aabbMin, cbtVector3& aabbMax) const = 0;

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(cbtDispatcher* dispatcher) { (void)dispatcher; };

	virtual void printStats() = 0;
};

#endif  //BT_BROADPHASE_INTERFACE_H
