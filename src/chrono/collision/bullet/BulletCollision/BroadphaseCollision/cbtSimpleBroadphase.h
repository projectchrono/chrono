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

#ifndef BT_SIMPLE_BROADPHASE_H
#define BT_SIMPLE_BROADPHASE_H

#include "cbtOverlappingPairCache.h"

struct cbtSimpleBroadphaseProxy : public cbtBroadphaseProxy
{
	int m_nextFree;

	//	int			m_handleId;

	cbtSimpleBroadphaseProxy(){};

	cbtSimpleBroadphaseProxy(const cbtVector3& minpt, const cbtVector3& maxpt, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask)
		: cbtBroadphaseProxy(minpt, maxpt, userPtr, collisionFilterGroup, collisionFilterMask)
	{
		(void)shapeType;
	}

	SIMD_FORCE_INLINE void SetNextFree(int next) { m_nextFree = next; }
	SIMD_FORCE_INLINE int GetNextFree() const { return m_nextFree; }
};

///The SimpleBroadphase is just a unit-test for cbtAxisSweep3, bt32BitAxisSweep3, or cbtDbvtBroadphase, so use those classes instead.
///It is a brute force aabb culling broadphase based on O(n^2) aabb checks
class cbtSimpleBroadphase : public cbtBroadphaseInterface
{
protected:
	int m_numHandles;  // number of active handles
	int m_maxHandles;  // max number of handles
	int m_LastHandleIndex;

	cbtSimpleBroadphaseProxy* m_pHandles;  // handles pool

	void* m_pHandlesRawPtr;
	int m_firstFreeHandle;  // free handles list

	int allocHandle()
	{
		cbtAssert(m_numHandles < m_maxHandles);
		int freeHandle = m_firstFreeHandle;
		m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
		m_numHandles++;
		if (freeHandle > m_LastHandleIndex)
		{
			m_LastHandleIndex = freeHandle;
		}
		return freeHandle;
	}

	void freeHandle(cbtSimpleBroadphaseProxy* proxy)
	{
		int handle = int(proxy - m_pHandles);
		cbtAssert(handle >= 0 && handle < m_maxHandles);
		if (handle == m_LastHandleIndex)
		{
			m_LastHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		proxy->m_clientObject = 0;

		m_numHandles--;
	}

	cbtOverlappingPairCache* m_pairCache;
	bool m_ownsPairCache;

	int m_invalidPair;

	inline cbtSimpleBroadphaseProxy* getSimpleProxyFromProxy(cbtBroadphaseProxy* proxy)
	{
		cbtSimpleBroadphaseProxy* proxy0 = static_cast<cbtSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	inline const cbtSimpleBroadphaseProxy* getSimpleProxyFromProxy(cbtBroadphaseProxy* proxy) const
	{
		const cbtSimpleBroadphaseProxy* proxy0 = static_cast<const cbtSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(cbtDispatcher* dispatcher);

	void validate();

protected:
public:
	cbtSimpleBroadphase(int maxProxies = 16384, cbtOverlappingPairCache* overlappingPairCache = 0);
	virtual ~cbtSimpleBroadphase();

	static bool aabbOverlap(cbtSimpleBroadphaseProxy* proxy0, cbtSimpleBroadphaseProxy* proxy1);

	virtual cbtBroadphaseProxy* createProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher);

	virtual void calculateOverlappingPairs(cbtDispatcher* dispatcher);

	virtual void destroyProxy(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher);
	virtual void setAabb(cbtBroadphaseProxy* proxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher);
	virtual void getAabb(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtBroadphaseRayCallback& rayCallback, const cbtVector3& aabbMin = cbtVector3(0, 0, 0), const cbtVector3& aabbMax = cbtVector3(0, 0, 0));
	virtual void aabbTest(const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtBroadphaseAabbCallback& callback);

	cbtOverlappingPairCache* getOverlappingPairCache()
	{
		return m_pairCache;
	}
	const cbtOverlappingPairCache* getOverlappingPairCache() const
	{
		return m_pairCache;
	}

	bool testAabbOverlap(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1);

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(cbtVector3& aabbMin, cbtVector3& aabbMax) const
	{
		aabbMin.setValue(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		aabbMax.setValue(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
	}

	virtual void printStats()
	{
		//		printf("cbtSimpleBroadphase.h\n");
		//		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
	}
};

#endif  //BT_SIMPLE_BROADPHASE_H
