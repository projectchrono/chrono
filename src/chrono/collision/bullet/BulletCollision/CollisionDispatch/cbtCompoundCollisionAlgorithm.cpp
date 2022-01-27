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

#include "BulletCollision/CollisionDispatch/cbtCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/cbtDbvt.h"
#include "LinearMath/cbtIDebugDraw.h"
#include "LinearMath/cbtAabbUtil2.h"
#include "cbtManifoldResult.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

cbtShapePairCallback gCompoundChildShapePairCallback = 0;

cbtCompoundCollisionAlgorithm::cbtCompoundCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped)
	: cbtActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_isSwapped(isSwapped),
	  m_sharedManifold(ci.m_manifold)
{
	m_ownsManifold = false;

	const cbtCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	cbtAssert(colObjWrap->getCollisionShape()->isCompound());

	const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(colObjWrap->getCollisionShape());
	m_compoundShapeRevision = compoundShape->getUpdateRevision();

	preallocateChildAlgorithms(body0Wrap, body1Wrap);
}

void cbtCompoundCollisionAlgorithm::preallocateChildAlgorithms(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
{
	const cbtCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const cbtCollisionObjectWrapper* otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;
	cbtAssert(colObjWrap->getCollisionShape()->isCompound());

	const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(colObjWrap->getCollisionShape());

	int numChildren = compoundShape->getNumChildShapes();
	int i;

	m_childCollisionAlgorithms.resize(numChildren);
	for (i = 0; i < numChildren; i++)
	{
		if (compoundShape->getDynamicAabbTree())
		{
			m_childCollisionAlgorithms[i] = 0;
		}
		else
		{
			const cbtCollisionShape* childShape = compoundShape->getChildShape(i);

			cbtCollisionObjectWrapper childWrap(colObjWrap, childShape, colObjWrap->getCollisionObject(), colObjWrap->getWorldTransform(), -1, i);  //wrong child trans, but unused (hopefully)
			m_childCollisionAlgorithms[i] = m_dispatcher->findAlgorithm(&childWrap, otherObjWrap, m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS);

			cbtAlignedObjectArray<cbtCollisionAlgorithm*> m_childCollisionAlgorithmsContact;
			cbtAlignedObjectArray<cbtCollisionAlgorithm*> m_childCollisionAlgorithmsClosestPoints;
		}
	}
}

void cbtCompoundCollisionAlgorithm::removeChildAlgorithms()
{
	int numChildren = m_childCollisionAlgorithms.size();
	int i;
	for (i = 0; i < numChildren; i++)
	{
		if (m_childCollisionAlgorithms[i])
		{
			m_childCollisionAlgorithms[i]->~cbtCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
		}
	}
}

cbtCompoundCollisionAlgorithm::~cbtCompoundCollisionAlgorithm()
{
	removeChildAlgorithms();
}

struct cbtCompoundLeafCallback : cbtDbvt::ICollide
{
public:
	const cbtCollisionObjectWrapper* m_compoundColObjWrap;
	const cbtCollisionObjectWrapper* m_otherObjWrap;
	cbtDispatcher* m_dispatcher;
	const cbtDispatcherInfo& m_dispatchInfo;
	cbtManifoldResult* m_resultOut;
	cbtCollisionAlgorithm** m_childCollisionAlgorithms;
	cbtPersistentManifold* m_sharedManifold;

	cbtCompoundLeafCallback(const cbtCollisionObjectWrapper* compoundObjWrap, const cbtCollisionObjectWrapper* otherObjWrap, cbtDispatcher* dispatcher, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut, cbtCollisionAlgorithm** childCollisionAlgorithms, cbtPersistentManifold* sharedManifold)
		: m_compoundColObjWrap(compoundObjWrap), m_otherObjWrap(otherObjWrap), m_dispatcher(dispatcher), m_dispatchInfo(dispatchInfo), m_resultOut(resultOut), m_childCollisionAlgorithms(childCollisionAlgorithms), m_sharedManifold(sharedManifold)
	{
	}

	void ProcessChildShape(const cbtCollisionShape* childShape, int index)
	{
		cbtAssert(index >= 0);
		const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(m_compoundColObjWrap->getCollisionShape());
		cbtAssert(index < compoundShape->getNumChildShapes());

		if (gCompoundChildShapePairCallback)
		{
			if (!gCompoundChildShapePairCallback(m_otherObjWrap->getCollisionShape(), childShape))
				return;
		}

		//backup
		cbtTransform orgTrans = m_compoundColObjWrap->getWorldTransform();

		const cbtTransform& childTrans = compoundShape->getChildTransform(index);
		cbtTransform newChildWorldTrans = orgTrans * childTrans;

		//perform an AABB check first
		cbtVector3 aabbMin0, aabbMax0;
		childShape->getAabb(newChildWorldTrans, aabbMin0, aabbMax0);

		cbtVector3 extendAabb(m_resultOut->m_closestPointDistanceThreshold, m_resultOut->m_closestPointDistanceThreshold, m_resultOut->m_closestPointDistanceThreshold);
		aabbMin0 -= extendAabb;
		aabbMax0 += extendAabb;

		cbtVector3 aabbMin1, aabbMax1;
		m_otherObjWrap->getCollisionShape()->getAabb(m_otherObjWrap->getWorldTransform(), aabbMin1, aabbMax1);


		if (TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
		{
			cbtCollisionObjectWrapper compoundWrap(this->m_compoundColObjWrap, childShape, m_compoundColObjWrap->getCollisionObject(), newChildWorldTrans, -1, index);

			cbtCollisionAlgorithm* algo = 0;
			bool allocatedAlgorithm = false;

			if (m_resultOut->m_closestPointDistanceThreshold > 0)
			{
				algo = m_dispatcher->findAlgorithm(&compoundWrap, m_otherObjWrap, 0, BT_CLOSEST_POINT_ALGORITHMS);
				allocatedAlgorithm = true;
			}
			else
			{
				//the contactpoint is still projected back using the original inverted worldtrans
				if (!m_childCollisionAlgorithms[index])
				{
					m_childCollisionAlgorithms[index] = m_dispatcher->findAlgorithm(&compoundWrap, m_otherObjWrap, m_sharedManifold, BT_CONTACT_POINT_ALGORITHMS);
				}
				algo = m_childCollisionAlgorithms[index];
			}

			const cbtCollisionObjectWrapper* tmpWrap = 0;

			///detect swapping case
			if (m_resultOut->getBody0Internal() == m_compoundColObjWrap->getCollisionObject())
			{
				tmpWrap = m_resultOut->getBody0Wrap();
				m_resultOut->setBody0Wrap(&compoundWrap);
				m_resultOut->setShapeIdentifiersA(-1, index);
			}
			else
			{
				tmpWrap = m_resultOut->getBody1Wrap();
				m_resultOut->setBody1Wrap(&compoundWrap);
				m_resultOut->setShapeIdentifiersB(-1, index);
			}

			algo->processCollision(&compoundWrap, m_otherObjWrap, m_dispatchInfo, m_resultOut);

#if 0
			if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & cbtIDebugDraw::DBG_DrawAabb))
			{
				cbtVector3 worldAabbMin,worldAabbMax;
				m_dispatchInfo.m_debugDraw->drawAabb(aabbMin0,aabbMax0,cbtVector3(1,1,1));
				m_dispatchInfo.m_debugDraw->drawAabb(aabbMin1,aabbMax1,cbtVector3(1,1,1));
			}
#endif

			if (m_resultOut->getBody0Internal() == m_compoundColObjWrap->getCollisionObject())
			{
				m_resultOut->setBody0Wrap(tmpWrap);
			}
			else
			{
				m_resultOut->setBody1Wrap(tmpWrap);
			}
			if (allocatedAlgorithm)
			{
				algo->~cbtCollisionAlgorithm();
				m_dispatcher->freeCollisionAlgorithm(algo);
			}
		}
	}
	void Process(const cbtDbvtNode* leaf)
	{
		int index = leaf->dataAsInt;

		const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(m_compoundColObjWrap->getCollisionShape());
		const cbtCollisionShape* childShape = compoundShape->getChildShape(index);

#if 0
		if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & cbtIDebugDraw::DBG_DrawAabb))
		{
			cbtVector3 worldAabbMin,worldAabbMax;
			cbtTransform	orgTrans = m_compoundColObjWrap->getWorldTransform();
			cbtTransformAabb(leaf->volume.Mins(),leaf->volume.Maxs(),0.,orgTrans,worldAabbMin,worldAabbMax);
			m_dispatchInfo.m_debugDraw->drawAabb(worldAabbMin,worldAabbMax,cbtVector3(1,0,0));
		}
#endif

		ProcessChildShape(childShape, index);
	}
};

void cbtCompoundCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	const cbtCollisionObjectWrapper* colObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const cbtCollisionObjectWrapper* otherObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

	cbtAssert(colObjWrap->getCollisionShape()->isCompound());
	const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(colObjWrap->getCollisionShape());

	///cbtCompoundShape might have changed:
	////make sure the internal child collision algorithm caches are still valid
	if (compoundShape->getUpdateRevision() != m_compoundShapeRevision)
	{
		///clear and update all
		removeChildAlgorithms();

		preallocateChildAlgorithms(body0Wrap, body1Wrap);
		m_compoundShapeRevision = compoundShape->getUpdateRevision();
	}

	if (m_childCollisionAlgorithms.size() == 0)
		return;

	const cbtDbvt* tree = compoundShape->getDynamicAabbTree();
	//use a dynamic aabb tree to cull potential child-overlaps
	cbtCompoundLeafCallback callback(colObjWrap, otherObjWrap, m_dispatcher, dispatchInfo, resultOut, &m_childCollisionAlgorithms[0], m_sharedManifold);

	///we need to refresh all contact manifolds
	///note that we should actually recursively traverse all children, cbtCompoundShape can nested more then 1 level deep
	///so we should add a 'refreshManifolds' in the cbtCollisionAlgorithm
	{
		int i;
		manifoldArray.resize(0);
		for (i = 0; i < m_childCollisionAlgorithms.size(); i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
				for (int m = 0; m < manifoldArray.size(); m++)
				{
					if (manifoldArray[m]->getNumContacts())
					{
						resultOut->setPersistentManifold(manifoldArray[m]);
						resultOut->refreshContactPoints();
						resultOut->setPersistentManifold(0);  //??necessary?
					}
				}
				manifoldArray.resize(0);
			}
		}
	}

	if (tree)
	{
		cbtVector3 localAabbMin, localAabbMax;
		cbtTransform otherInCompoundSpace;
		otherInCompoundSpace = colObjWrap->getWorldTransform().inverse() * otherObjWrap->getWorldTransform();
		otherObjWrap->getCollisionShape()->getAabb(otherInCompoundSpace, localAabbMin, localAabbMax);
		cbtVector3 extraExtends(resultOut->m_closestPointDistanceThreshold, resultOut->m_closestPointDistanceThreshold, resultOut->m_closestPointDistanceThreshold);
		localAabbMin -= extraExtends;
		localAabbMax += extraExtends;

		const ATTRIBUTE_ALIGNED16(cbtDbvtVolume) bounds = cbtDbvtVolume::FromMM(localAabbMin, localAabbMax);
		//process all children, that overlap with  the given AABB bounds
		tree->collideTVNoStackAlloc(tree->m_root, bounds, stack2, callback);
	}
	else
	{
		//iterate over all children, perform an AABB check inside ProcessChildShape
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		for (i = 0; i < numChildren; i++)
		{
			callback.ProcessChildShape(compoundShape->getChildShape(i), i);
		}
	}

	{
		//iterate over all children, perform an AABB check inside ProcessChildShape
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		manifoldArray.resize(0);
		const cbtCollisionShape* childShape = 0;
		cbtTransform orgTrans;

		cbtTransform newChildWorldTrans;
		cbtVector3 aabbMin0, aabbMax0, aabbMin1, aabbMax1;

		for (i = 0; i < numChildren; i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				childShape = compoundShape->getChildShape(i);
				//if not longer overlapping, remove the algorithm
				orgTrans = colObjWrap->getWorldTransform();

				const cbtTransform& childTrans = compoundShape->getChildTransform(i);
				newChildWorldTrans = orgTrans * childTrans;

				//perform an AABB check first
				childShape->getAabb(newChildWorldTrans, aabbMin0, aabbMax0);
				otherObjWrap->getCollisionShape()->getAabb(otherObjWrap->getWorldTransform(), aabbMin1, aabbMax1);

				if (!TestAabbAgainstAabb2(aabbMin0, aabbMax0, aabbMin1, aabbMax1))
				{
					m_childCollisionAlgorithms[i]->~cbtCollisionAlgorithm();
					m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
					m_childCollisionAlgorithms[i] = 0;
				}
			}
		}
	}
}

cbtScalar cbtCompoundCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	cbtAssert(0);
	//needs to be fixed, using cbtCollisionObjectWrapper and NOT modifying internal data structures
	cbtCollisionObject* colObj = m_isSwapped ? body1 : body0;
	cbtCollisionObject* otherObj = m_isSwapped ? body0 : body1;

	cbtAssert(colObj->getCollisionShape()->isCompound());

	cbtCompoundShape* compoundShape = static_cast<cbtCompoundShape*>(colObj->getCollisionShape());

	//We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
	//If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
	//given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
	//determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
	//then use each overlapping node AABB against Tree0
	//and vise versa.

	cbtScalar hitFraction = cbtScalar(1.);

	int numChildren = m_childCollisionAlgorithms.size();
	int i;
	cbtTransform orgTrans;
	cbtScalar frac;
	for (i = 0; i < numChildren; i++)
	{
		//cbtCollisionShape* childShape = compoundShape->getChildShape(i);

		//backup
		orgTrans = colObj->getWorldTransform();

		const cbtTransform& childTrans = compoundShape->getChildTransform(i);
		//cbtTransform	newChildWorldTrans = orgTrans*childTrans ;
		colObj->setWorldTransform(orgTrans * childTrans);

		//cbtCollisionShape* tmpShape = colObj->getCollisionShape();
		//colObj->internalSetTemporaryCollisionShape( childShape );
		frac = m_childCollisionAlgorithms[i]->calculateTimeOfImpact(colObj, otherObj, dispatchInfo, resultOut);
		if (frac < hitFraction)
		{
			hitFraction = frac;
		}
		//revert back
		//colObj->internalSetTemporaryCollisionShape( tmpShape);
		colObj->setWorldTransform(orgTrans);
	}
	return hitFraction;
}
