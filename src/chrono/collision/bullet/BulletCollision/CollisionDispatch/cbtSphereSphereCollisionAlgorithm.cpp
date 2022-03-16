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
#define CLEAR_MANIFOLD 1

#include "cbtSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

cbtSphereSphereCollisionAlgorithm::cbtSphereSphereCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap)
	: cbtActivatingCollisionAlgorithm(ci, col0Wrap, col1Wrap),
	  m_ownManifold(false),
	  m_manifoldPtr(mf)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(col0Wrap->getCollisionObject(), col1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

cbtSphereSphereCollisionAlgorithm::~cbtSphereSphereCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void cbtSphereSphereCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)dispatchInfo;

	if (!m_manifoldPtr)
		return;

	resultOut->setPersistentManifold(m_manifoldPtr);

	cbtSphereShape* sphere0 = (cbtSphereShape*)col0Wrap->getCollisionShape();
	cbtSphereShape* sphere1 = (cbtSphereShape*)col1Wrap->getCollisionShape();

	cbtVector3 diff = col0Wrap->getWorldTransform().getOrigin() - col1Wrap->getWorldTransform().getOrigin();
	cbtScalar len = diff.length();
	cbtScalar radius0 = sphere0->getRadius();
	cbtScalar radius1 = sphere1->getRadius();

#ifdef CLEAR_MANIFOLD
	m_manifoldPtr->clearManifold();  //don't do this, it disables warmstarting
#endif

	///iff distance positive, don't generate a new contact
	if (len > (radius0 + radius1 + resultOut->m_closestPointDistanceThreshold))
	{
#ifndef CLEAR_MANIFOLD
		resultOut->refreshContactPoints();
#endif  //CLEAR_MANIFOLD
		return;
	}
	///distance (negative means penetration)
	cbtScalar dist = len - (radius0 + radius1);

	cbtVector3 normalOnSurfaceB(1, 0, 0);
	if (len > SIMD_EPSILON)
	{
		normalOnSurfaceB = diff / len;
	}

	///point on A (worldspace)
	///cbtVector3 pos0 = col0->getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
	///point on B (worldspace)
	cbtVector3 pos1 = col1Wrap->getWorldTransform().getOrigin() + radius1 * normalOnSurfaceB;

	/// report a contact. internally this will be kept persistent, and contact reduction is done

	resultOut->addContactPoint(normalOnSurfaceB, pos1, dist);

#ifndef CLEAR_MANIFOLD
	resultOut->refreshContactPoints();
#endif  //CLEAR_MANIFOLD
}

cbtScalar cbtSphereSphereCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* col0, cbtCollisionObject* col1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)col0;
	(void)col1;
	(void)dispatchInfo;
	(void)resultOut;

	//not yet
	return cbtScalar(1.);
}
