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

#include "cbtSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.h"

#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtMatrix3x3.h"
#include "LinearMath/cbtAabbUtil2.h"

#include <new>

void cbtSimpleBroadphase::validate()
{
	for (int i = 0; i < m_numHandles; i++)
	{
		for (int j = i + 1; j < m_numHandles; j++)
		{
			cbtAssert(&m_pHandles[i] != &m_pHandles[j]);
		}
	}
}

cbtSimpleBroadphase::cbtSimpleBroadphase(int maxProxies, cbtOverlappingPairCache* overlappingPairCache)
	: m_pairCache(overlappingPairCache),
	  m_ownsPairCache(false),
	  m_invalidPair(0)
{
	if (!overlappingPairCache)
	{
		void* mem = cbtAlignedAlloc(sizeof(cbtHashedOverlappingPairCache), 16);
		m_pairCache = new (mem) cbtHashedOverlappingPairCache();
		m_ownsPairCache = true;
	}

	// allocate handles buffer and put all handles on free list
	m_pHandlesRawPtr = cbtAlignedAlloc(sizeof(cbtSimpleBroadphaseProxy) * maxProxies, 16);
	m_pHandles = new (m_pHandlesRawPtr) cbtSimpleBroadphaseProxy[maxProxies];
	m_maxHandles = maxProxies;
	m_numHandles = 0;
	m_firstFreeHandle = 0;
	m_LastHandleIndex = -1;

	{
		for (int i = m_firstFreeHandle; i < maxProxies; i++)
		{
			m_pHandles[i].SetNextFree(i + 1);
			m_pHandles[i].m_uniqueId = i + 2;  //any UID will do, we just avoid too trivial values (0,1) for debugging purposes
		}
		m_pHandles[maxProxies - 1].SetNextFree(0);
	}
}

cbtSimpleBroadphase::~cbtSimpleBroadphase()
{
	cbtAlignedFree(m_pHandlesRawPtr);

	if (m_ownsPairCache)
	{
		m_pairCache->~cbtOverlappingPairCache();
		cbtAlignedFree(m_pairCache);
	}
}

cbtBroadphaseProxy* cbtSimpleBroadphase::createProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* /*dispatcher*/)
{
	if (m_numHandles >= m_maxHandles)
	{
		cbtAssert(0);
		return 0;  //should never happen, but don't let the game crash ;-)
	}
	cbtAssert(aabbMin[0] <= aabbMax[0] && aabbMin[1] <= aabbMax[1] && aabbMin[2] <= aabbMax[2]);

	int newHandleIndex = allocHandle();
	cbtSimpleBroadphaseProxy* proxy = new (&m_pHandles[newHandleIndex]) cbtSimpleBroadphaseProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask);

	return proxy;
}

class RemovingOverlapCallback : public cbtOverlapCallback
{
protected:
	virtual bool processOverlap(cbtBroadphasePair& pair)
	{
		(void)pair;
		cbtAssert(0);
		return false;
	}
};

class RemovePairContainingProxy
{
	cbtBroadphaseProxy* m_targetProxy;

public:
	virtual ~RemovePairContainingProxy()
	{
	}

protected:
	virtual bool processOverlap(cbtBroadphasePair& pair)
	{
		cbtSimpleBroadphaseProxy* proxy0 = static_cast<cbtSimpleBroadphaseProxy*>(pair.m_pProxy0);
		cbtSimpleBroadphaseProxy* proxy1 = static_cast<cbtSimpleBroadphaseProxy*>(pair.m_pProxy1);

		return ((m_targetProxy == proxy0 || m_targetProxy == proxy1));
	};
};

void cbtSimpleBroadphase::destroyProxy(cbtBroadphaseProxy* proxyOrg, cbtDispatcher* dispatcher)
{
	cbtSimpleBroadphaseProxy* proxy0 = static_cast<cbtSimpleBroadphaseProxy*>(proxyOrg);
	freeHandle(proxy0);

	m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg, dispatcher);

	//validate();
}

void cbtSimpleBroadphase::getAabb(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	const cbtSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
	aabbMin = sbp->m_aabbMin;
	aabbMax = sbp->m_aabbMax;
}

void cbtSimpleBroadphase::setAabb(cbtBroadphaseProxy* proxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* /*dispatcher*/)
{
	cbtSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
	sbp->m_aabbMin = aabbMin;
	sbp->m_aabbMax = aabbMax;
}

void cbtSimpleBroadphase::rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtBroadphaseRayCallback& rayCallback, const cbtVector3& aabbMin, const cbtVector3& aabbMax)
{
	for (int i = 0; i <= m_LastHandleIndex; i++)
	{
		cbtSimpleBroadphaseProxy* proxy = &m_pHandles[i];
		if (!proxy->m_clientObject)
		{
			continue;
		}
		rayCallback.process(proxy);
	}
}

void cbtSimpleBroadphase::aabbTest(const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtBroadphaseAabbCallback& callback)
{
	for (int i = 0; i <= m_LastHandleIndex; i++)
	{
		cbtSimpleBroadphaseProxy* proxy = &m_pHandles[i];
		if (!proxy->m_clientObject)
		{
			continue;
		}
		if (TestAabbAgainstAabb2(aabbMin, aabbMax, proxy->m_aabbMin, proxy->m_aabbMax))
		{
			callback.process(proxy);
		}
	}
}

bool cbtSimpleBroadphase::aabbOverlap(cbtSimpleBroadphaseProxy* proxy0, cbtSimpleBroadphaseProxy* proxy1)
{
	return proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0] &&
		   proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1] &&
		   proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2];
}

//then remove non-overlapping ones
class CheckOverlapCallback : public cbtOverlapCallback
{
public:
	virtual bool processOverlap(cbtBroadphasePair& pair)
	{
		return (!cbtSimpleBroadphase::aabbOverlap(static_cast<cbtSimpleBroadphaseProxy*>(pair.m_pProxy0), static_cast<cbtSimpleBroadphaseProxy*>(pair.m_pProxy1)));
	}
};

void cbtSimpleBroadphase::calculateOverlappingPairs(cbtDispatcher* dispatcher)
{
	//first check for new overlapping pairs
	int i, j;
	if (m_numHandles >= 0)
	{
		int new_largest_index = -1;
		for (i = 0; i <= m_LastHandleIndex; i++)
		{
			cbtSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
			if (!proxy0->m_clientObject)
			{
				continue;
			}
			new_largest_index = i;
			for (j = i + 1; j <= m_LastHandleIndex; j++)
			{
				cbtSimpleBroadphaseProxy* proxy1 = &m_pHandles[j];
				cbtAssert(proxy0 != proxy1);
				if (!proxy1->m_clientObject)
				{
					continue;
				}

				cbtSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
				cbtSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);

				if (aabbOverlap(p0, p1))
				{
					if (!m_pairCache->findPair(proxy0, proxy1))
					{
						m_pairCache->addOverlappingPair(proxy0, proxy1);
					}
				}
				else
				{
					if (!m_pairCache->hasDeferredRemoval())
					{
						if (m_pairCache->findPair(proxy0, proxy1))
						{
							m_pairCache->removeOverlappingPair(proxy0, proxy1, dispatcher);
						}
					}
				}
			}
		}

		m_LastHandleIndex = new_largest_index;

		if (m_ownsPairCache && m_pairCache->hasDeferredRemoval())
		{
			cbtBroadphasePairArray& overlappingPairArray = m_pairCache->getOverlappingPairArray();

			//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
			overlappingPairArray.quickSort(cbtBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;

			cbtBroadphasePair previousPair;
			previousPair.m_pProxy0 = 0;
			previousPair.m_pProxy1 = 0;
			previousPair.m_algorithm = 0;

			for (i = 0; i < overlappingPairArray.size(); i++)
			{
				cbtBroadphasePair& pair = overlappingPairArray[i];

				bool isDuplicate = (pair == previousPair);

				previousPair = pair;

				bool needsRemoval = false;

				if (!isDuplicate)
				{
					bool hasOverlap = testAabbOverlap(pair.m_pProxy0, pair.m_pProxy1);

					if (hasOverlap)
					{
						needsRemoval = false;  //callback->processOverlap(pair);
					}
					else
					{
						needsRemoval = true;
					}
				}
				else
				{
					//remove duplicate
					needsRemoval = true;
					//should have no algorithm
					cbtAssert(!pair.m_algorithm);
				}

				if (needsRemoval)
				{
					m_pairCache->cleanOverlappingPair(pair, dispatcher);

					//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
					//		m_overlappingPairArray.pop_back();
					pair.m_pProxy0 = 0;
					pair.m_pProxy1 = 0;
					m_invalidPair++;
				}
			}

			///if you don't like to skip the invalid pairs in the array, execute following code:
#define CLEAN_INVALID_PAIRS 1
#ifdef CLEAN_INVALID_PAIRS

			//perform a sort, to sort 'invalid' pairs to the end
			overlappingPairArray.quickSort(cbtBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;
#endif  //CLEAN_INVALID_PAIRS
		}
	}
}

bool cbtSimpleBroadphase::testAabbOverlap(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1)
{
	cbtSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
	cbtSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
	return aabbOverlap(p0, p1);
}

void cbtSimpleBroadphase::resetPool(cbtDispatcher* dispatcher)
{
	//not yet
}
