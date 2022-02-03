/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#include "cbtCompoundCompoundCollisionAlgorithm.h"
#include "LinearMath/cbtQuickprof.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/cbtDbvt.h"
#include "LinearMath/cbtIDebugDraw.h"
#include "LinearMath/cbtAabbUtil2.h"
#include "BulletCollision/CollisionDispatch/cbtManifoldResult.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

//USE_LOCAL_STACK will avoid most (often all) dynamic memory allocations due to resizing in processCollision and MycollideTT
#define USE_LOCAL_STACK 1

cbtShapePairCallback gCompoundCompoundChildShapePairCallback = 0;

cbtCompoundCompoundCollisionAlgorithm::cbtCompoundCompoundCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped)
	: cbtCompoundCollisionAlgorithm(ci, body0Wrap, body1Wrap, isSwapped)
{
	void* ptr = cbtAlignedAlloc(sizeof(cbtHashedSimplePairCache), 16);
	m_childCollisionAlgorithmCache = new (ptr) cbtHashedSimplePairCache();

	const cbtCollisionObjectWrapper* col0ObjWrap = body0Wrap;
	cbtAssert(col0ObjWrap->getCollisionShape()->isCompound());

	const cbtCollisionObjectWrapper* col1ObjWrap = body1Wrap;
	cbtAssert(col1ObjWrap->getCollisionShape()->isCompound());

	const cbtCompoundShape* compoundShape0 = static_cast<const cbtCompoundShape*>(col0ObjWrap->getCollisionShape());
	m_compoundShapeRevision0 = compoundShape0->getUpdateRevision();

	const cbtCompoundShape* compoundShape1 = static_cast<const cbtCompoundShape*>(col1ObjWrap->getCollisionShape());
	m_compoundShapeRevision1 = compoundShape1->getUpdateRevision();
}

cbtCompoundCompoundCollisionAlgorithm::~cbtCompoundCompoundCollisionAlgorithm()
{
	removeChildAlgorithms();
	m_childCollisionAlgorithmCache->~cbtHashedSimplePairCache();
	cbtAlignedFree(m_childCollisionAlgorithmCache);
}

void cbtCompoundCompoundCollisionAlgorithm::getAllContactManifolds(cbtManifoldArray& manifoldArray)
{
	int i;
	cbtSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();
	for (i = 0; i < pairs.size(); i++)
	{
		if (pairs[i].m_userPointer)
		{
			((cbtCollisionAlgorithm*)pairs[i].m_userPointer)->getAllContactManifolds(manifoldArray);
		}
	}
}

void cbtCompoundCompoundCollisionAlgorithm::removeChildAlgorithms()
{
	cbtSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();

	int numChildren = pairs.size();
	int i;
	for (i = 0; i < numChildren; i++)
	{
		if (pairs[i].m_userPointer)
		{
			cbtCollisionAlgorithm* algo = (cbtCollisionAlgorithm*)pairs[i].m_userPointer;
			algo->~cbtCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(algo);
		}
	}
	m_childCollisionAlgorithmCache->removeAllPairs();
}

struct cbtCompoundCompoundLeafCallback : cbtDbvt::ICollide
{
	int m_numOverlapPairs;

	const cbtCollisionObjectWrapper* m_compound0ColObjWrap;
	const cbtCollisionObjectWrapper* m_compound1ColObjWrap;
	cbtDispatcher* m_dispatcher;
	const cbtDispatcherInfo& m_dispatchInfo;
	cbtManifoldResult* m_resultOut;

	class cbtHashedSimplePairCache* m_childCollisionAlgorithmCache;

	cbtPersistentManifold* m_sharedManifold;

	cbtCompoundCompoundLeafCallback(const cbtCollisionObjectWrapper* compound1ObjWrap,
								   const cbtCollisionObjectWrapper* compound0ObjWrap,
								   cbtDispatcher* dispatcher,
								   const cbtDispatcherInfo& dispatchInfo,
								   cbtManifoldResult* resultOut,
								   cbtHashedSimplePairCache* childAlgorithmsCache,
								   cbtPersistentManifold* sharedManifold)
		: m_numOverlapPairs(0), m_compound0ColObjWrap(compound1ObjWrap), m_compound1ColObjWrap(compound0ObjWrap), m_dispatcher(dispatcher), m_dispatchInfo(dispatchInfo), m_resultOut(resultOut), m_childCollisionAlgorithmCache(childAlgorithmsCache), m_sharedManifold(sharedManifold)
	{
	}

	void Process(const cbtDbvtNode* leaf0, const cbtDbvtNode* leaf1)
	{
		BT_PROFILE("cbtCompoundCompoundLeafCallback::Process");
		m_numOverlapPairs++;

		int childIndex0 = leaf0->dataAsInt;
		int childIndex1 = leaf1->dataAsInt;

		cbtAssert(childIndex0 >= 0);
		cbtAssert(childIndex1 >= 0);

		const cbtCompoundShape* compoundShape0 = static_cast<const cbtCompoundShape*>(m_compound0ColObjWrap->getCollisionShape());
		cbtAssert(childIndex0 < compoundShape0->getNumChildShapes());

		const cbtCompoundShape* compoundShape1 = static_cast<const cbtCompoundShape*>(m_compound1ColObjWrap->getCollisionShape());
		cbtAssert(childIndex1 < compoundShape1->getNumChildShapes());

		const cbtCollisionShape* childShape0 = compoundShape0->getChildShape(childIndex0);
		const cbtCollisionShape* childShape1 = compoundShape1->getChildShape(childIndex1);

		//backup
		cbtTransform orgTrans0 = m_compound0ColObjWrap->getWorldTransform();
		const cbtTransform& childTrans0 = compoundShape0->getChildTransform(childIndex0);
		cbtTransform newChildWorldTrans0 = orgTrans0 * childTrans0;

		cbtTransform orgTrans1 = m_compound1ColObjWrap->getWorldTransform();
		const cbtTransform& childTrans1 = compoundShape1->getChildTransform(childIndex1);
		cbtTransform newChildWorldTrans1 = orgTrans1 * childTrans1;

		//perform an AABB check first
		cbtVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;
		childShape0->getAabb(newChildWorldTrans0, aabbMin0, aabbMax0);
		childShape1->getAabb(newChildWorldTrans1, aabbMin1, aabbMax1);

		cbtVector3 thresholdVec(m_resultOut->m_closestPointDistanceThreshold, m_resultOut->m_closestPointDistanceThreshold, m_resultOut->m_closestPointDistanceThreshold);

		aabbMin0 -= thresholdVec;
		aabbMax0 += thresholdVec;

		if (gCompoundCompoundChildShapePairCallback)
		{
			if (!gCompoundCompoundChildShapePairCallback(childShape0, childShape1))
				return;
		}

		if (TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
		{
			cbtCollisionObjectWrapper compoundWrap0(this->m_compound0ColObjWrap, childShape0, m_compound0ColObjWrap->getCollisionObject(), newChildWorldTrans0, -1, childIndex0);
			cbtCollisionObjectWrapper compoundWrap1(this->m_compound1ColObjWrap, childShape1, m_compound1ColObjWrap->getCollisionObject(), newChildWorldTrans1, -1, childIndex1);

			cbtSimplePair* pair = m_childCollisionAlgorithmCache->findPair(childIndex0, childIndex1);
			bool removePair = false;
			cbtCollisionAlgorithm* colAlgo = 0;
			if (m_resultOut->m_closestPointDistanceThreshold > 0)
			{
				colAlgo = m_dispatcher->findAlgorithm(&compoundWrap0, &compoundWrap1, 0, BT_CLOSEST_POINT_ALGORITHMS);
				removePair = true;
			}
			else
			{
				if (pair)
				{
					colAlgo = (cbtCollisionAlgorithm*)pair->m_userPointer;
				}
				else
				{
					colAlgo = m_dispatcher->findAlgorithm(&compoundWrap0, &compoundWrap1, m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS);
					pair = m_childCollisionAlgorithmCache->addOverlappingPair(childIndex0, childIndex1);
					cbtAssert(pair);
					pair->m_userPointer = colAlgo;
				}
			}

			cbtAssert(colAlgo);

			const cbtCollisionObjectWrapper* tmpWrap0 = 0;
			const cbtCollisionObjectWrapper* tmpWrap1 = 0;

			tmpWrap0 = m_resultOut->getBody0Wrap();
			tmpWrap1 = m_resultOut->getBody1Wrap();

			m_resultOut->setBody0Wrap(&compoundWrap0);
			m_resultOut->setBody1Wrap(&compoundWrap1);

			m_resultOut->setShapeIdentifiersA(-1, childIndex0);
			m_resultOut->setShapeIdentifiersB(-1, childIndex1);

			colAlgo->processCollision(&compoundWrap0, &compoundWrap1, m_dispatchInfo, m_resultOut);

			m_resultOut->setBody0Wrap(tmpWrap0);
			m_resultOut->setBody1Wrap(tmpWrap1);

			if (removePair)
			{
				colAlgo->~cbtCollisionAlgorithm();
				m_dispatcher->freeCollisionAlgorithm(colAlgo);
			}
		}
	}
};

static DBVT_INLINE bool MyIntersect(const cbtDbvtAabbMm& a,
									const cbtDbvtAabbMm& b, const cbtTransform& xform, cbtScalar distanceThreshold)
{
	cbtVector3 newmin, newmax;
	cbtTransformAabb(b.Mins(), b.Maxs(), 0.f, xform, newmin, newmax);
	newmin -= cbtVector3(distanceThreshold, distanceThreshold, distanceThreshold);
	newmax += cbtVector3(distanceThreshold, distanceThreshold, distanceThreshold);
	cbtDbvtAabbMm newb = cbtDbvtAabbMm::FromMM(newmin, newmax);
	return Intersect(a, newb);
}

static inline void MycollideTT(const cbtDbvtNode* root0,
							   const cbtDbvtNode* root1,
							   const cbtTransform& xform,
							   cbtCompoundCompoundLeafCallback* callback, cbtScalar distanceThreshold)
{
	if (root0 && root1)
	{
		int depth = 1;
		int treshold = cbtDbvt::DOUBLE_STACKSIZE - 4;
		cbtAlignedObjectArray<cbtDbvt::sStkNN> stkStack;
#ifdef USE_LOCAL_STACK
		ATTRIBUTE_ALIGNED16(cbtDbvt::sStkNN localStack[cbtDbvt::DOUBLE_STACKSIZE]);
		stkStack.initializeFromBuffer(&localStack, cbtDbvt::DOUBLE_STACKSIZE, cbtDbvt::DOUBLE_STACKSIZE);
#else
		stkStack.resize(cbtDbvt::DOUBLE_STACKSIZE);
#endif
		stkStack[0] = cbtDbvt::sStkNN(root0, root1);
		do
		{
			cbtDbvt::sStkNN p = stkStack[--depth];
			if (MyIntersect(p.a->volume, p.b->volume, xform, distanceThreshold))
			{
				if (depth > treshold)
				{
					stkStack.resize(stkStack.size() * 2);
					treshold = stkStack.size() - 4;
				}
				if (p.a->isinternal())
				{
					if (p.b->isinternal())
					{
						stkStack[depth++] = cbtDbvt::sStkNN(p.a->childs[0], p.b->childs[0]);
						stkStack[depth++] = cbtDbvt::sStkNN(p.a->childs[1], p.b->childs[0]);
						stkStack[depth++] = cbtDbvt::sStkNN(p.a->childs[0], p.b->childs[1]);
						stkStack[depth++] = cbtDbvt::sStkNN(p.a->childs[1], p.b->childs[1]);
					}
					else
					{
						stkStack[depth++] = cbtDbvt::sStkNN(p.a->childs[0], p.b);
						stkStack[depth++] = cbtDbvt::sStkNN(p.a->childs[1], p.b);
					}
				}
				else
				{
					if (p.b->isinternal())
					{
						stkStack[depth++] = cbtDbvt::sStkNN(p.a, p.b->childs[0]);
						stkStack[depth++] = cbtDbvt::sStkNN(p.a, p.b->childs[1]);
					}
					else
					{
						callback->Process(p.a, p.b);
					}
				}
			}
		} while (depth);
	}
}

void cbtCompoundCompoundCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	const cbtCollisionObjectWrapper* col0ObjWrap = body0Wrap;
	const cbtCollisionObjectWrapper* col1ObjWrap = body1Wrap;

	cbtAssert(col0ObjWrap->getCollisionShape()->isCompound());
	cbtAssert(col1ObjWrap->getCollisionShape()->isCompound());
	const cbtCompoundShape* compoundShape0 = static_cast<const cbtCompoundShape*>(col0ObjWrap->getCollisionShape());
	const cbtCompoundShape* compoundShape1 = static_cast<const cbtCompoundShape*>(col1ObjWrap->getCollisionShape());

	const cbtDbvt* tree0 = compoundShape0->getDynamicAabbTree();
	const cbtDbvt* tree1 = compoundShape1->getDynamicAabbTree();
	if (!tree0 || !tree1)
	{
		return cbtCompoundCollisionAlgorithm::processCollision(body0Wrap, body1Wrap, dispatchInfo, resultOut);
	}
	///cbtCompoundShape might have changed:
	////make sure the internal child collision algorithm caches are still valid
	if ((compoundShape0->getUpdateRevision() != m_compoundShapeRevision0) || (compoundShape1->getUpdateRevision() != m_compoundShapeRevision1))
	{
		///clear all
		removeChildAlgorithms();
		m_compoundShapeRevision0 = compoundShape0->getUpdateRevision();
		m_compoundShapeRevision1 = compoundShape1->getUpdateRevision();
	}

	///we need to refresh all contact manifolds
	///note that we should actually recursively traverse all children, cbtCompoundShape can nested more then 1 level deep
	///so we should add a 'refreshManifolds' in the cbtCollisionAlgorithm
	{
		int i;
		cbtManifoldArray manifoldArray;
#ifdef USE_LOCAL_STACK
		cbtPersistentManifold localManifolds[4];
		manifoldArray.initializeFromBuffer(&localManifolds, 0, 4);
#endif
		cbtSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();
		for (i = 0; i < pairs.size(); i++)
		{
			if (pairs[i].m_userPointer)
			{
				cbtCollisionAlgorithm* algo = (cbtCollisionAlgorithm*)pairs[i].m_userPointer;
				algo->getAllContactManifolds(manifoldArray);
				for (int m = 0; m < manifoldArray.size(); m++)
				{
					if (manifoldArray[m]->getNumContacts())
					{
						resultOut->setPersistentManifold(manifoldArray[m]);
						resultOut->refreshContactPoints();
						resultOut->setPersistentManifold(0);
					}
				}
				manifoldArray.resize(0);
			}
		}
	}

	cbtCompoundCompoundLeafCallback callback(col0ObjWrap, col1ObjWrap, this->m_dispatcher, dispatchInfo, resultOut, this->m_childCollisionAlgorithmCache, m_sharedManifold);

	const cbtTransform xform = col0ObjWrap->getWorldTransform().inverse() * col1ObjWrap->getWorldTransform();
	MycollideTT(tree0->m_root, tree1->m_root, xform, &callback, resultOut->m_closestPointDistanceThreshold);

	//printf("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);

	//remove non-overlapping child pairs

	{
		cbtAssert(m_removePairs.size() == 0);

		//iterate over all children, perform an AABB check inside ProcessChildShape
		cbtSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();

		int i;
		cbtManifoldArray manifoldArray;

		cbtVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;

		for (i = 0; i < pairs.size(); i++)
		{
			if (pairs[i].m_userPointer)
			{
				cbtCollisionAlgorithm* algo = (cbtCollisionAlgorithm*)pairs[i].m_userPointer;

				{
					const cbtCollisionShape* childShape0 = 0;

					cbtTransform newChildWorldTrans0;
					childShape0 = compoundShape0->getChildShape(pairs[i].m_indexA);
					const cbtTransform& childTrans0 = compoundShape0->getChildTransform(pairs[i].m_indexA);
					newChildWorldTrans0 = col0ObjWrap->getWorldTransform() * childTrans0;
					childShape0->getAabb(newChildWorldTrans0, aabbMin0, aabbMax0);
				}
				cbtVector3 thresholdVec(resultOut->m_closestPointDistanceThreshold, resultOut->m_closestPointDistanceThreshold, resultOut->m_closestPointDistanceThreshold);
				aabbMin0 -= thresholdVec;
				aabbMax0 += thresholdVec;
				{
					const cbtCollisionShape* childShape1 = 0;
					cbtTransform newChildWorldTrans1;

					childShape1 = compoundShape1->getChildShape(pairs[i].m_indexB);
					const cbtTransform& childTrans1 = compoundShape1->getChildTransform(pairs[i].m_indexB);
					newChildWorldTrans1 = col1ObjWrap->getWorldTransform() * childTrans1;
					childShape1->getAabb(newChildWorldTrans1, aabbMin1, aabbMax1);
				}

				aabbMin1 -= thresholdVec;
				aabbMax1 += thresholdVec;

				if (!TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
				{
					algo->~cbtCollisionAlgorithm();
					m_dispatcher->freeCollisionAlgorithm(algo);
					m_removePairs.push_back(cbtSimplePair(pairs[i].m_indexA, pairs[i].m_indexB));
				}
			}
		}
		for (int i = 0; i < m_removePairs.size(); i++)
		{
			m_childCollisionAlgorithmCache->removeOverlappingPair(m_removePairs[i].m_indexA, m_removePairs[i].m_indexB);
		}
		m_removePairs.clear();
	}
}

cbtScalar cbtCompoundCompoundCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	cbtAssert(0);
	return 0.f;
}
