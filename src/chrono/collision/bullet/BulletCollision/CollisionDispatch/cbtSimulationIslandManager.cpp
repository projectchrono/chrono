
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

#include "LinearMath/cbtScalar.h"
#include "cbtSimulationIslandManager.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionWorld.h"

//#include <stdio.h>
#include "LinearMath/cbtQuickprof.h"

cbtSimulationIslandManager::cbtSimulationIslandManager() : m_splitIslands(true)
{
}

cbtSimulationIslandManager::~cbtSimulationIslandManager()
{
}

void cbtSimulationIslandManager::initUnionFind(int n)
{
	m_unionFind.reset(n);
}

void cbtSimulationIslandManager::findUnions(cbtDispatcher* /* dispatcher */, cbtCollisionWorld* colWorld)
{
	{
		cbtOverlappingPairCache* pairCachePtr = colWorld->getPairCache();
		const int numOverlappingPairs = pairCachePtr->getNumOverlappingPairs();
		if (numOverlappingPairs)
		{
			cbtBroadphasePair* pairPtr = pairCachePtr->getOverlappingPairArrayPtr();

			for (int i = 0; i < numOverlappingPairs; i++)
			{
				const cbtBroadphasePair& collisionPair = pairPtr[i];
				cbtCollisionObject* colObj0 = (cbtCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
				cbtCollisionObject* colObj1 = (cbtCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

				if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
					((colObj1) && ((colObj1)->mergesSimulationIslands())))
				{
					m_unionFind.unite((colObj0)->getIslandTag(),
									  (colObj1)->getIslandTag());
				}
			}
		}
	}
}

#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
void cbtSimulationIslandManager::updateActivationState(cbtCollisionWorld* colWorld, cbtDispatcher* dispatcher)
{
	// put the index into m_controllers into m_tag
	int index = 0;
	{
		int i;
		for (i = 0; i < colWorld->getCollisionObjectArray().size(); i++)
		{
			cbtCollisionObject* collisionObject = colWorld->getCollisionObjectArray()[i];
			//Adding filtering here
			if (!collisionObject->isStaticOrKinematicObject())
			{
				collisionObject->setIslandTag(index++);
			}
			collisionObject->setCompanionId(-1);
			collisionObject->setHitFraction(cbtScalar(1.));
		}
	}
	// do the union find

	initUnionFind(index);

	findUnions(dispatcher, colWorld);
}

void cbtSimulationIslandManager::storeIslandActivationState(cbtCollisionWorld* colWorld)
{
	// put the islandId ('find' value) into m_tag
	{
		int index = 0;
		int i;
		for (i = 0; i < colWorld->getCollisionObjectArray().size(); i++)
		{
			cbtCollisionObject* collisionObject = colWorld->getCollisionObjectArray()[i];
			if (!collisionObject->isStaticOrKinematicObject())
			{
				collisionObject->setIslandTag(m_unionFind.find(index));
				//Set the correct object offset in Collision Object Array
				m_unionFind.getElement(index).m_sz = i;
				collisionObject->setCompanionId(-1);
				index++;
			}
			else
			{
				collisionObject->setIslandTag(-1);
				collisionObject->setCompanionId(-2);
			}
		}
	}
}

#else  //STATIC_SIMULATION_ISLAND_OPTIMIZATION
void cbtSimulationIslandManager::updateActivationState(cbtCollisionWorld* colWorld, cbtDispatcher* dispatcher)
{
	initUnionFind(int(colWorld->getCollisionObjectArray().size()));

	// put the index into m_controllers into m_tag
	{
		int index = 0;
		int i;
		for (i = 0; i < colWorld->getCollisionObjectArray().size(); i++)
		{
			cbtCollisionObject* collisionObject = colWorld->getCollisionObjectArray()[i];
			collisionObject->setIslandTag(index);
			collisionObject->setCompanionId(-1);
			collisionObject->setHitFraction(cbtScalar(1.));
			index++;
		}
	}
	// do the union find

	findUnions(dispatcher, colWorld);
}

void cbtSimulationIslandManager::storeIslandActivationState(cbtCollisionWorld* colWorld)
{
	// put the islandId ('find' value) into m_tag
	{
		int index = 0;
		int i;
		for (i = 0; i < colWorld->getCollisionObjectArray().size(); i++)
		{
			cbtCollisionObject* collisionObject = colWorld->getCollisionObjectArray()[i];
			if (!collisionObject->isStaticOrKinematicObject())
			{
				collisionObject->setIslandTag(m_unionFind.find(index));
				collisionObject->setCompanionId(-1);
			}
			else
			{
				collisionObject->setIslandTag(-1);
				collisionObject->setCompanionId(-2);
			}
			index++;
		}
	}
}

#endif  //STATIC_SIMULATION_ISLAND_OPTIMIZATION

inline int getIslandId(const cbtPersistentManifold* lhs)
{
	int islandId;
	const cbtCollisionObject* rcolObj0 = static_cast<const cbtCollisionObject*>(lhs->getBody0());
	const cbtCollisionObject* rcolObj1 = static_cast<const cbtCollisionObject*>(lhs->getBody1());
	islandId = rcolObj0->getIslandTag() >= 0 ? rcolObj0->getIslandTag() : rcolObj1->getIslandTag();
	return islandId;
}

/// function object that routes calls to operator<
class cbtPersistentManifoldSortPredicate
{
public:
	SIMD_FORCE_INLINE bool operator()(const cbtPersistentManifold* lhs, const cbtPersistentManifold* rhs) const
	{
		return getIslandId(lhs) < getIslandId(rhs);
	}
};

class cbtPersistentManifoldSortPredicateDeterministic
{
public:
	SIMD_FORCE_INLINE bool operator()(const cbtPersistentManifold* lhs, const cbtPersistentManifold* rhs) const
	{
		return (
			(getIslandId(lhs) < getIslandId(rhs)) || ((getIslandId(lhs) == getIslandId(rhs)) && lhs->getBody0()->getBroadphaseHandle()->m_uniqueId < rhs->getBody0()->getBroadphaseHandle()->m_uniqueId) || ((getIslandId(lhs) == getIslandId(rhs)) && (lhs->getBody0()->getBroadphaseHandle()->m_uniqueId == rhs->getBody0()->getBroadphaseHandle()->m_uniqueId) && (lhs->getBody1()->getBroadphaseHandle()->m_uniqueId < rhs->getBody1()->getBroadphaseHandle()->m_uniqueId)));
	}
};

void cbtSimulationIslandManager::buildIslands(cbtDispatcher* dispatcher, cbtCollisionWorld* collisionWorld)
{
	BT_PROFILE("islandUnionFindAndQuickSort");

	cbtCollisionObjectArray& collisionObjects = collisionWorld->getCollisionObjectArray();

	m_islandmanifold.resize(0);

	//we are going to sort the unionfind array, and store the element id in the size
	//afterwards, we clean unionfind, to make sure no-one uses it anymore

	getUnionFind().sortIslands();
	int numElem = getUnionFind().getNumElements();

	int endIslandIndex = 1;
	int startIslandIndex;

	//update the sleeping state for bodies, if all are sleeping
	for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
	{
		int islandId = getUnionFind().getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex + 1; (endIslandIndex < numElem) && (getUnionFind().getElement(endIslandIndex).m_id == islandId); endIslandIndex++)
		{
		}

		//int numSleeping = 0;

		bool allSleeping = true;

		int idx;
		for (idx = startIslandIndex; idx < endIslandIndex; idx++)
		{
			int i = getUnionFind().getElement(idx).m_sz;

			cbtCollisionObject* colObj0 = collisionObjects[i];
			if ((colObj0->getIslandTag() != islandId) && (colObj0->getIslandTag() != -1))
			{
				//				printf("error in island management\n");
			}

			cbtAssert((colObj0->getIslandTag() == islandId) || (colObj0->getIslandTag() == -1));
			if (colObj0->getIslandTag() == islandId)
			{
				if (colObj0->getActivationState() == ACTIVE_TAG ||
					colObj0->getActivationState() == DISABLE_DEACTIVATION)
				{
					allSleeping = false;
					break;
				}
			}
		}

		if (allSleeping)
		{
			int idx;
			for (idx = startIslandIndex; idx < endIslandIndex; idx++)
			{
				int i = getUnionFind().getElement(idx).m_sz;
				cbtCollisionObject* colObj0 = collisionObjects[i];
				if ((colObj0->getIslandTag() != islandId) && (colObj0->getIslandTag() != -1))
				{
					//					printf("error in island management\n");
				}

				cbtAssert((colObj0->getIslandTag() == islandId) || (colObj0->getIslandTag() == -1));

				if (colObj0->getIslandTag() == islandId)
				{
					colObj0->setActivationState(ISLAND_SLEEPING);
				}
			}
		}
		else
		{
			int idx;
			for (idx = startIslandIndex; idx < endIslandIndex; idx++)
			{
				int i = getUnionFind().getElement(idx).m_sz;

				cbtCollisionObject* colObj0 = collisionObjects[i];
				if ((colObj0->getIslandTag() != islandId) && (colObj0->getIslandTag() != -1))
				{
					//					printf("error in island management\n");
				}

				cbtAssert((colObj0->getIslandTag() == islandId) || (colObj0->getIslandTag() == -1));

				if (colObj0->getIslandTag() == islandId)
				{
					if (colObj0->getActivationState() == ISLAND_SLEEPING)
					{
						colObj0->setActivationState(WANTS_DEACTIVATION);
						colObj0->setDeactivationTime(0.f);
					}
				}
			}
		}
	}

	int i;
	int maxNumManifolds = dispatcher->getNumManifolds();

	//#define SPLIT_ISLANDS 1
	//#ifdef SPLIT_ISLANDS

	//#endif //SPLIT_ISLANDS

	for (i = 0; i < maxNumManifolds; i++)
	{
		cbtPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
		if (collisionWorld->getDispatchInfo().m_deterministicOverlappingPairs)
		{
			if (manifold->getNumContacts() == 0)
				continue;
		}

		const cbtCollisionObject* colObj0 = static_cast<const cbtCollisionObject*>(manifold->getBody0());
		const cbtCollisionObject* colObj1 = static_cast<const cbtCollisionObject*>(manifold->getBody1());

		///@todo: check sleeping conditions!
		if (((colObj0) && colObj0->getActivationState() != ISLAND_SLEEPING) ||
			((colObj1) && colObj1->getActivationState() != ISLAND_SLEEPING))
		{
			//kinematic objects don't merge islands, but wake up all connected objects
			if (colObj0->isKinematicObject() && colObj0->getActivationState() != ISLAND_SLEEPING)
			{
				if (colObj0->hasContactResponse())
					colObj1->activate();
			}
			if (colObj1->isKinematicObject() && colObj1->getActivationState() != ISLAND_SLEEPING)
			{
				if (colObj1->hasContactResponse())
					colObj0->activate();
			}
			if (m_splitIslands)
			{
				//filtering for response
				if (dispatcher->needsResponse(colObj0, colObj1))
					m_islandmanifold.push_back(manifold);
			}
		}
	}
}

///@todo: this is random access, it can be walked 'cache friendly'!
void cbtSimulationIslandManager::buildAndProcessIslands(cbtDispatcher* dispatcher, cbtCollisionWorld* collisionWorld, IslandCallback* callback)
{
	cbtCollisionObjectArray& collisionObjects = collisionWorld->getCollisionObjectArray();

	buildIslands(dispatcher, collisionWorld);

	int endIslandIndex = 1;
	int startIslandIndex;
	int numElem = getUnionFind().getNumElements();

	BT_PROFILE("processIslands");

	if (!m_splitIslands)
	{
		cbtPersistentManifold** manifold = dispatcher->getInternalManifoldPointer();
		int maxNumManifolds = dispatcher->getNumManifolds();
		callback->processIsland(&collisionObjects[0], collisionObjects.size(), manifold, maxNumManifolds, -1);
	}
	else
	{
		// Sort manifolds, based on islands
		// Sort the vector using predicate and std::sort
		//std::sort(islandmanifold.begin(), islandmanifold.end(), cbtPersistentManifoldSortPredicate);

		int numManifolds = int(m_islandmanifold.size());

		//tried a radix sort, but quicksort/heapsort seems still faster
		//@todo rewrite island management
		//cbtPersistentManifoldSortPredicateDeterministic sorts contact manifolds based on islandid,
		//but also based on object0 unique id and object1 unique id
		if (collisionWorld->getDispatchInfo().m_deterministicOverlappingPairs)
		{
			m_islandmanifold.quickSort(cbtPersistentManifoldSortPredicateDeterministic());
		}
		else
		{
			m_islandmanifold.quickSort(cbtPersistentManifoldSortPredicate());
		}

		//m_islandmanifold.heapSort(cbtPersistentManifoldSortPredicate());

		//now process all active islands (sets of manifolds for now)

		int startManifoldIndex = 0;
		int endManifoldIndex = 1;

		//int islandId;

		//	printf("Start Islands\n");

		//traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
		for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
		{
			int islandId = getUnionFind().getElement(startIslandIndex).m_id;

			bool islandSleeping = true;

			for (endIslandIndex = startIslandIndex; (endIslandIndex < numElem) && (getUnionFind().getElement(endIslandIndex).m_id == islandId); endIslandIndex++)
			{
				int i = getUnionFind().getElement(endIslandIndex).m_sz;
				cbtCollisionObject* colObj0 = collisionObjects[i];
				m_islandBodies.push_back(colObj0);
				if (colObj0->isActive())
					islandSleeping = false;
			}

			//find the accompanying contact manifold for this islandId
			int numIslandManifolds = 0;
			cbtPersistentManifold** startManifold = 0;

			if (startManifoldIndex < numManifolds)
			{
				int curIslandId = getIslandId(m_islandmanifold[startManifoldIndex]);
				if (curIslandId == islandId)
				{
					startManifold = &m_islandmanifold[startManifoldIndex];

					for (endManifoldIndex = startManifoldIndex + 1; (endManifoldIndex < numManifolds) && (islandId == getIslandId(m_islandmanifold[endManifoldIndex])); endManifoldIndex++)
					{
					}
					/// Process the actual simulation, only if not sleeping/deactivated
					numIslandManifolds = endManifoldIndex - startManifoldIndex;
				}
			}

			if (!islandSleeping)
			{
				callback->processIsland(&m_islandBodies[0], m_islandBodies.size(), startManifold, numIslandManifolds, islandId);
				//			printf("Island callback of size:%d bodies, %d manifolds\n",islandBodies.size(),numIslandManifolds);
			}

			if (numIslandManifolds)
			{
				startManifoldIndex = endManifoldIndex;
			}

			m_islandBodies.resize(0);
		}
	}  // else if(!splitIslands)
}
