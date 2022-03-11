/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "cbtGhostObject.h"
#include "cbtCollisionWorld.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "LinearMath/cbtAabbUtil2.h"

cbtGhostObject::cbtGhostObject()
{
	m_internalType = CO_GHOST_OBJECT;
}

cbtGhostObject::~cbtGhostObject()
{
	///cbtGhostObject should have been removed from the world, so no overlapping objects
	cbtAssert(!m_overlappingObjects.size());
}

void cbtGhostObject::addOverlappingObjectInternal(cbtBroadphaseProxy* otherProxy, cbtBroadphaseProxy* thisProxy)
{
	cbtCollisionObject* otherObject = (cbtCollisionObject*)otherProxy->m_clientObject;
	cbtAssert(otherObject);
	///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index == m_overlappingObjects.size())
	{
		//not found
		m_overlappingObjects.push_back(otherObject);
	}
}

void cbtGhostObject::removeOverlappingObjectInternal(cbtBroadphaseProxy* otherProxy, cbtDispatcher* dispatcher, cbtBroadphaseProxy* thisProxy)
{
	cbtCollisionObject* otherObject = (cbtCollisionObject*)otherProxy->m_clientObject;
	cbtAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index < m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size() - 1];
		m_overlappingObjects.pop_back();
	}
}

cbtPairCachingGhostObject::cbtPairCachingGhostObject()
{
	m_hashPairCache = new (cbtAlignedAlloc(sizeof(cbtHashedOverlappingPairCache), 16)) cbtHashedOverlappingPairCache();
}

cbtPairCachingGhostObject::~cbtPairCachingGhostObject()
{
	m_hashPairCache->~cbtHashedOverlappingPairCache();
	cbtAlignedFree(m_hashPairCache);
}

void cbtPairCachingGhostObject::addOverlappingObjectInternal(cbtBroadphaseProxy* otherProxy, cbtBroadphaseProxy* thisProxy)
{
	cbtBroadphaseProxy* actualThisProxy = thisProxy ? thisProxy : getBroadphaseHandle();
	cbtAssert(actualThisProxy);

	cbtCollisionObject* otherObject = (cbtCollisionObject*)otherProxy->m_clientObject;
	cbtAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index == m_overlappingObjects.size())
	{
		m_overlappingObjects.push_back(otherObject);
		m_hashPairCache->addOverlappingPair(actualThisProxy, otherProxy);
	}
}

void cbtPairCachingGhostObject::removeOverlappingObjectInternal(cbtBroadphaseProxy* otherProxy, cbtDispatcher* dispatcher, cbtBroadphaseProxy* thisProxy1)
{
	cbtCollisionObject* otherObject = (cbtCollisionObject*)otherProxy->m_clientObject;
	cbtBroadphaseProxy* actualThisProxy = thisProxy1 ? thisProxy1 : getBroadphaseHandle();
	cbtAssert(actualThisProxy);

	cbtAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index < m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size() - 1];
		m_overlappingObjects.pop_back();
		m_hashPairCache->removeOverlappingPair(actualThisProxy, otherProxy, dispatcher);
	}
}

void cbtGhostObject::convexSweepTest(const cbtConvexShape* castShape, const cbtTransform& convexFromWorld, const cbtTransform& convexToWorld, cbtCollisionWorld::ConvexResultCallback& resultCallback, cbtScalar allowedCcdPenetration) const
{
	cbtTransform convexFromTrans, convexToTrans;
	convexFromTrans = convexFromWorld;
	convexToTrans = convexToWorld;
	cbtVector3 castShapeAabbMin, castShapeAabbMax;
	/* Compute AABB that encompasses angular movement */
	{
		cbtVector3 linVel, angVel;
		cbtTransformUtil::calculateVelocity(convexFromTrans, convexToTrans, 1.0, linVel, angVel);
		cbtTransform R;
		R.setIdentity();
		R.setRotation(convexFromTrans.getRotation());
		castShape->calculateTemporalAabb(R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
	}

	/// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for (i = 0; i < m_overlappingObjects.size(); i++)
	{
		cbtCollisionObject* collisionObject = m_overlappingObjects[i];
		//only perform raycast if filterMask matches
		if (resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			cbtVector3 collisionObjectAabbMin, collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(), collisionObjectAabbMin, collisionObjectAabbMax);
			AabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
			cbtScalar hitLambda = cbtScalar(1.);  //could use resultCallback.m_closestHitFraction, but needs testing
			cbtVector3 hitNormal;
			if (cbtRayAabb(convexFromWorld.getOrigin(), convexToWorld.getOrigin(), collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal))
			{
				cbtCollisionWorld::objectQuerySingle(castShape, convexFromTrans, convexToTrans,
													collisionObject,
													collisionObject->getCollisionShape(),
													collisionObject->getWorldTransform(),
													resultCallback,
													allowedCcdPenetration);
			}
		}
	}
}

void cbtGhostObject::rayTest(const cbtVector3& rayFromWorld, const cbtVector3& rayToWorld, cbtCollisionWorld::RayResultCallback& resultCallback) const
{
	cbtTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFromWorld);
	cbtTransform rayToTrans;
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(rayToWorld);

	int i;
	for (i = 0; i < m_overlappingObjects.size(); i++)
	{
		cbtCollisionObject* collisionObject = m_overlappingObjects[i];
		//only perform raycast if filterMask matches
		if (resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			cbtCollisionWorld::rayTestSingle(rayFromTrans, rayToTrans,
											collisionObject,
											collisionObject->getCollisionShape(),
											collisionObject->getWorldTransform(),
											resultCallback);
		}
	}
}
