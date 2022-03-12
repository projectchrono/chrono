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

#include "cbtBoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "cbtBoxBoxDetector.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"
#define USE_PERSISTENT_CONTACTS 1

cbtBoxBoxCollisionAlgorithm::cbtBoxBoxCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
	: cbtActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_ownManifold(false),
	  m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->needsCollision(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

cbtBoxBoxCollisionAlgorithm::~cbtBoxBoxCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void cbtBoxBoxCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	const cbtBoxShape* box0 = (cbtBoxShape*)body0Wrap->getCollisionShape();
	const cbtBoxShape* box1 = (cbtBoxShape*)body1Wrap->getCollisionShape();

	/// report a contact. internally this will be kept persistent, and contact reduction is done
	resultOut->setPersistentManifold(m_manifoldPtr);
#ifndef USE_PERSISTENT_CONTACTS
	m_manifoldPtr->clearManifold();
#endif  //USE_PERSISTENT_CONTACTS

	cbtDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
	input.m_transformA = body0Wrap->getWorldTransform();
	input.m_transformB = body1Wrap->getWorldTransform();

	cbtBoxBoxDetector detector(box0, box1);
	detector.getClosestPoints(input, *resultOut, dispatchInfo.m_debugDraw);

#ifdef USE_PERSISTENT_CONTACTS
	//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
#endif  //USE_PERSISTENT_CONTACTS
}

cbtScalar cbtBoxBoxCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* /*body0*/, cbtCollisionObject* /*body1*/, const cbtDispatcherInfo& /*dispatchInfo*/, cbtManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}
