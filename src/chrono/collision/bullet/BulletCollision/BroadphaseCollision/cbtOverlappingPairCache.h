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

#ifndef BT_OVERLAPPING_PAIR_CACHE_H
#define BT_OVERLAPPING_PAIR_CACHE_H

#include "cbtBroadphaseInterface.h"
#include "cbtBroadphaseProxy.h"
#include "cbtOverlappingPairCallback.h"

#include "LinearMath/cbtAlignedObjectArray.h"
class cbtDispatcher;

typedef cbtAlignedObjectArray<cbtBroadphasePair> cbtBroadphasePairArray;

struct cbtOverlapCallback
{
	virtual ~cbtOverlapCallback()
	{
	}
	//return true for deletion of the pair
	virtual bool processOverlap(cbtBroadphasePair& pair) = 0;
};

struct cbtOverlapFilterCallback
{
	virtual ~cbtOverlapFilterCallback()
	{
	}
	// return true when pairs need collision
	virtual bool needBroadphaseCollision(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1) const = 0;
};

const int BT_NULL_PAIR = 0xffffffff;

///The cbtOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the cbtBroadphaseInterface broadphases.
///The cbtHashedOverlappingPairCache and cbtSortedOverlappingPairCache classes are two implementations.
class cbtOverlappingPairCache : public cbtOverlappingPairCallback
{
public:
	virtual ~cbtOverlappingPairCache() {}  // this is needed so we can get to the derived class destructor

	virtual cbtBroadphasePair* getOverlappingPairArrayPtr() = 0;

	virtual const cbtBroadphasePair* getOverlappingPairArrayPtr() const = 0;

	virtual cbtBroadphasePairArray& getOverlappingPairArray() = 0;

	virtual void cleanOverlappingPair(cbtBroadphasePair& pair, cbtDispatcher* dispatcher) = 0;

	virtual int getNumOverlappingPairs() const = 0;

	virtual void cleanProxyFromPairs(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher) = 0;

	virtual void setOverlapFilterCallback(cbtOverlapFilterCallback* callback) = 0;

	virtual void processAllOverlappingPairs(cbtOverlapCallback*, cbtDispatcher* dispatcher) = 0;

	virtual void processAllOverlappingPairs(cbtOverlapCallback* callback, cbtDispatcher* dispatcher, const struct cbtDispatcherInfo& dispatchInfo)
	{
		processAllOverlappingPairs(callback, dispatcher);
	}
	virtual cbtBroadphasePair* findPair(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1) = 0;

	virtual bool hasDeferredRemoval() = 0;

	virtual void setInternalGhostPairCallback(cbtOverlappingPairCallback* ghostPairCallback) = 0;

	virtual void sortOverlappingPairs(cbtDispatcher* dispatcher) = 0;
};

/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com

ATTRIBUTE_ALIGNED16(class)
cbtHashedOverlappingPairCache : public cbtOverlappingPairCache
{
	cbtBroadphasePairArray m_overlappingPairArray;
	cbtOverlapFilterCallback* m_overlapFilterCallback;

protected:
	cbtAlignedObjectArray<int> m_hashTable;
	cbtAlignedObjectArray<int> m_next;
	cbtOverlappingPairCallback* m_ghostPairCallback;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtHashedOverlappingPairCache();
	virtual ~cbtHashedOverlappingPairCache();

	void removeOverlappingPairsContainingProxy(cbtBroadphaseProxy * proxy, cbtDispatcher * dispatcher);

	virtual void* removeOverlappingPair(cbtBroadphaseProxy * proxy0, cbtBroadphaseProxy * proxy1, cbtDispatcher * dispatcher);

	SIMD_FORCE_INLINE bool needsBroadphaseCollision(cbtBroadphaseProxy * proxy0, cbtBroadphaseProxy * proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback->needBroadphaseCollision(proxy0, proxy1);

		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

		return collides;
	}

	// Add a pair and return the new pair. If the pair already exists,
	// no new pair is created and the old one is returned.
	virtual cbtBroadphasePair* addOverlappingPair(cbtBroadphaseProxy * proxy0, cbtBroadphaseProxy * proxy1)
	{
		if (!needsBroadphaseCollision(proxy0, proxy1))
			return 0;

		return internalAddPair(proxy0, proxy1);
	}

	void cleanProxyFromPairs(cbtBroadphaseProxy * proxy, cbtDispatcher * dispatcher);

	virtual void processAllOverlappingPairs(cbtOverlapCallback*, cbtDispatcher * dispatcher);

	virtual void processAllOverlappingPairs(cbtOverlapCallback * callback, cbtDispatcher * dispatcher, const struct cbtDispatcherInfo& dispatchInfo);

	virtual cbtBroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	const cbtBroadphasePair* getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}

	cbtBroadphasePairArray& getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	const cbtBroadphasePairArray& getOverlappingPairArray() const
	{
		return m_overlappingPairArray;
	}

	void cleanOverlappingPair(cbtBroadphasePair & pair, cbtDispatcher * dispatcher);

	cbtBroadphasePair* findPair(cbtBroadphaseProxy * proxy0, cbtBroadphaseProxy * proxy1);

	int GetCount() const { return m_overlappingPairArray.size(); }
	//	cbtBroadphasePair* GetPairs() { return m_pairs; }

	cbtOverlapFilterCallback* getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	void setOverlapFilterCallback(cbtOverlapFilterCallback * callback)
	{
		m_overlapFilterCallback = callback;
	}

	int getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}

private:
	cbtBroadphasePair* internalAddPair(cbtBroadphaseProxy * proxy0, cbtBroadphaseProxy * proxy1);

	void growTables();

	SIMD_FORCE_INLINE bool equalsPair(const cbtBroadphasePair& pair, int proxyId1, int proxyId2)
	{
		return pair.m_pProxy0->getUid() == proxyId1 && pair.m_pProxy1->getUid() == proxyId2;
	}

	/*
	// Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
	// This assumes proxyId1 and proxyId2 are 16-bit.
	SIMD_FORCE_INLINE int getHash(int proxyId1, int proxyId2)
	{
		int key = (proxyId2 << 16) | proxyId1;
		key = ~key + (key << 15);
		key = key ^ (key >> 12);
		key = key + (key << 2);
		key = key ^ (key >> 4);
		key = key * 2057;
		key = key ^ (key >> 16);
		return key;
	}
	*/

	SIMD_FORCE_INLINE unsigned int getHash(unsigned int proxyId1, unsigned int proxyId2)
	{
		unsigned int key = proxyId1 | (proxyId2 << 16);
		// Thomas Wang's hash

		key += ~(key << 15);
		key ^= (key >> 10);
		key += (key << 3);
		key ^= (key >> 6);
		key += ~(key << 11);
		key ^= (key >> 16);
		return key;
	}

	SIMD_FORCE_INLINE cbtBroadphasePair* internalFindPair(cbtBroadphaseProxy * proxy0, cbtBroadphaseProxy * proxy1, int hash)
	{
		int proxyId1 = proxy0->getUid();
		int proxyId2 = proxy1->getUid();
#if 0  // wrong, 'equalsPair' use unsorted uids, copy-past devil striked again. Nat.
		if (proxyId1 > proxyId2) 
			cbtSwap(proxyId1, proxyId2);
#endif

		int index = m_hashTable[hash];

		while (index != BT_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
		{
			index = m_next[index];
		}

		if (index == BT_NULL_PAIR)
		{
			return NULL;
		}

		cbtAssert(index < m_overlappingPairArray.size());

		return &m_overlappingPairArray[index];
	}

	virtual bool hasDeferredRemoval()
	{
		return false;
	}

	virtual void setInternalGhostPairCallback(cbtOverlappingPairCallback * ghostPairCallback)
	{
		m_ghostPairCallback = ghostPairCallback;
	}

	virtual void sortOverlappingPairs(cbtDispatcher * dispatcher);
};

///cbtSortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or cbtSimpleBroadphase
class cbtSortedOverlappingPairCache : public cbtOverlappingPairCache
{
protected:
	//avoid brute-force finding all the time
	cbtBroadphasePairArray m_overlappingPairArray;

	//during the dispatch, check that user doesn't destroy/create proxy
	bool m_blockedForChanges;

	///by default, do the removal during the pair traversal
	bool m_hasDeferredRemoval;

	//if set, use the callback instead of the built in filter in needBroadphaseCollision
	cbtOverlapFilterCallback* m_overlapFilterCallback;

	cbtOverlappingPairCallback* m_ghostPairCallback;

public:
	cbtSortedOverlappingPairCache();
	virtual ~cbtSortedOverlappingPairCache();

	virtual void processAllOverlappingPairs(cbtOverlapCallback*, cbtDispatcher* dispatcher);

	void* removeOverlappingPair(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1, cbtDispatcher* dispatcher);

	void cleanOverlappingPair(cbtBroadphasePair& pair, cbtDispatcher* dispatcher);

	cbtBroadphasePair* addOverlappingPair(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1);

	cbtBroadphasePair* findPair(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1);

	void cleanProxyFromPairs(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher);

	void removeOverlappingPairsContainingProxy(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher);

	inline bool needsBroadphaseCollision(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1) const
	{
		if (m_overlapFilterCallback)
			return m_overlapFilterCallback->needBroadphaseCollision(proxy0, proxy1);

		bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

		return collides;
	}

	cbtBroadphasePairArray& getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	const cbtBroadphasePairArray& getOverlappingPairArray() const
	{
		return m_overlappingPairArray;
	}

	cbtBroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}

	const cbtBroadphasePair* getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}

	int getNumOverlappingPairs() const
	{
		return m_overlappingPairArray.size();
	}

	cbtOverlapFilterCallback* getOverlapFilterCallback()
	{
		return m_overlapFilterCallback;
	}

	void setOverlapFilterCallback(cbtOverlapFilterCallback* callback)
	{
		m_overlapFilterCallback = callback;
	}

	virtual bool hasDeferredRemoval()
	{
		return m_hasDeferredRemoval;
	}

	virtual void setInternalGhostPairCallback(cbtOverlappingPairCallback* ghostPairCallback)
	{
		m_ghostPairCallback = ghostPairCallback;
	}

	virtual void sortOverlappingPairs(cbtDispatcher* dispatcher);
};

///cbtNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
class cbtNullPairCache : public cbtOverlappingPairCache
{
	cbtBroadphasePairArray m_overlappingPairArray;

public:
	virtual cbtBroadphasePair* getOverlappingPairArrayPtr()
	{
		return &m_overlappingPairArray[0];
	}
	const cbtBroadphasePair* getOverlappingPairArrayPtr() const
	{
		return &m_overlappingPairArray[0];
	}
	cbtBroadphasePairArray& getOverlappingPairArray()
	{
		return m_overlappingPairArray;
	}

	virtual void cleanOverlappingPair(cbtBroadphasePair& /*pair*/, cbtDispatcher* /*dispatcher*/)
	{
	}

	virtual int getNumOverlappingPairs() const
	{
		return 0;
	}

	virtual void cleanProxyFromPairs(cbtBroadphaseProxy* /*proxy*/, cbtDispatcher* /*dispatcher*/)
	{
	}

	virtual void setOverlapFilterCallback(cbtOverlapFilterCallback* /*callback*/)
	{
	}

	virtual void processAllOverlappingPairs(cbtOverlapCallback*, cbtDispatcher* /*dispatcher*/)
	{
	}

	virtual cbtBroadphasePair* findPair(cbtBroadphaseProxy* /*proxy0*/, cbtBroadphaseProxy* /*proxy1*/)
	{
		return 0;
	}

	virtual bool hasDeferredRemoval()
	{
		return true;
	}

	virtual void setInternalGhostPairCallback(cbtOverlappingPairCallback* /* ghostPairCallback */)
	{
	}

	virtual cbtBroadphasePair* addOverlappingPair(cbtBroadphaseProxy* /*proxy0*/, cbtBroadphaseProxy* /*proxy1*/)
	{
		return 0;
	}

	virtual void* removeOverlappingPair(cbtBroadphaseProxy* /*proxy0*/, cbtBroadphaseProxy* /*proxy1*/, cbtDispatcher* /*dispatcher*/)
	{
		return 0;
	}

	virtual void removeOverlappingPairsContainingProxy(cbtBroadphaseProxy* /*proxy0*/, cbtDispatcher* /*dispatcher*/)
	{
	}

	virtual void sortOverlappingPairs(cbtDispatcher* dispatcher)
	{
		(void)dispatcher;
	}
};

#endif  //BT_OVERLAPPING_PAIR_CACHE_H
