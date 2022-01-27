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

#include "cbtCollisionDispatcherMt.h"
#include "LinearMath/cbtQuickprof.h"

#include "BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.h"

#include "BulletCollision/CollisionShapes/cbtCollisionShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/cbtOverlappingPairCache.h"
#include "LinearMath/cbtPoolAllocator.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

cbtCollisionDispatcherMt::cbtCollisionDispatcherMt(cbtCollisionConfiguration* config, int grainSize)
	: cbtCollisionDispatcher(config)
{
	m_batchUpdating = false;
	m_grainSize = grainSize;  // iterations per task
}

cbtPersistentManifold* cbtCollisionDispatcherMt::getNewManifold(const cbtCollisionObject* body0, const cbtCollisionObject* body1)
{
	//optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)

	cbtScalar contactBreakingThreshold = (m_dispatcherFlags & cbtCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD) ? cbtMin(body0->getCollisionShape()->getContactBreakingThreshold(gContactBreakingThreshold), body1->getCollisionShape()->getContactBreakingThreshold(gContactBreakingThreshold))
																																: gContactBreakingThreshold;

	cbtScalar contactProcessingThreshold = cbtMin(body0->getContactProcessingThreshold(), body1->getContactProcessingThreshold());

	void* mem = m_persistentManifoldPoolAllocator->allocate(sizeof(cbtPersistentManifold));
	if (NULL == mem)
	{
		//we got a pool memory overflow, by default we fallback to dynamically allocate memory. If we require a contiguous contact pool then assert.
		if ((m_dispatcherFlags & CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION) == 0)
		{
			mem = cbtAlignedAlloc(sizeof(cbtPersistentManifold), 16);
		}
		else
		{
			cbtAssert(0);
			//make sure to increase the m_defaultMaxPersistentManifoldPoolSize in the cbtDefaultCollisionConstructionInfo/cbtDefaultCollisionConfiguration
			return 0;
		}
	}
	cbtPersistentManifold* manifold = new (mem) cbtPersistentManifold(body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold);
	if (!m_batchUpdating)
	{
		// batch updater will update manifold pointers array after finishing, so
		// only need to update array when not batch-updating
		//cbtAssert( !cbtThreadsAreRunning() );
		manifold->m_index1a = m_manifoldsPtr.size();
		m_manifoldsPtr.push_back(manifold);
	}

	return manifold;
}

void cbtCollisionDispatcherMt::releaseManifold(cbtPersistentManifold* manifold)
{
	clearManifold(manifold);
	//cbtAssert( !cbtThreadsAreRunning() );
	if (!m_batchUpdating)
	{
		// batch updater will update manifold pointers array after finishing, so
		// only need to update array when not batch-updating
		int findIndex = manifold->m_index1a;
		cbtAssert(findIndex < m_manifoldsPtr.size());
		m_manifoldsPtr.swap(findIndex, m_manifoldsPtr.size() - 1);
		m_manifoldsPtr[findIndex]->m_index1a = findIndex;
		m_manifoldsPtr.pop_back();
	}

	manifold->~cbtPersistentManifold();
	if (m_persistentManifoldPoolAllocator->validPtr(manifold))
	{
		m_persistentManifoldPoolAllocator->freeMemory(manifold);
	}
	else
	{
		cbtAlignedFree(manifold);
	}
}

struct CollisionDispatcherUpdater : public cbtIParallelForBody
{
	cbtBroadphasePair* mPairArray;
	cbtNearCallback mCallback;
	cbtCollisionDispatcher* mDispatcher;
	const cbtDispatcherInfo* mInfo;

	CollisionDispatcherUpdater()
	{
		mPairArray = NULL;
		mCallback = NULL;
		mDispatcher = NULL;
		mInfo = NULL;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			cbtBroadphasePair* pair = &mPairArray[i];
			mCallback(*pair, *mDispatcher, *mInfo);
		}
	}
};

void cbtCollisionDispatcherMt::dispatchAllCollisionPairs(cbtOverlappingPairCache* pairCache, const cbtDispatcherInfo& info, cbtDispatcher* dispatcher)
{
	int pairCount = pairCache->getNumOverlappingPairs();
	if (pairCount == 0)
	{
		return;
	}
	CollisionDispatcherUpdater updater;
	updater.mCallback = getNearCallback();
	updater.mPairArray = pairCache->getOverlappingPairArrayPtr();
	updater.mDispatcher = this;
	updater.mInfo = &info;

	m_batchUpdating = true;
	cbtParallelFor(0, pairCount, m_grainSize, updater);
	m_batchUpdating = false;

	// reconstruct the manifolds array to ensure determinism
	m_manifoldsPtr.resizeNoInitialize(0);

	cbtBroadphasePair* pairs = pairCache->getOverlappingPairArrayPtr();
	for (int i = 0; i < pairCount; ++i)
	{
		if (cbtCollisionAlgorithm* algo = pairs[i].m_algorithm)
		{
			algo->getAllContactManifolds(m_manifoldsPtr);
		}
	}

	// update the indices (used when releasing manifolds)
	for (int i = 0; i < m_manifoldsPtr.size(); ++i)
	{
		m_manifoldsPtr[i]->m_index1a = i;
	}
}
