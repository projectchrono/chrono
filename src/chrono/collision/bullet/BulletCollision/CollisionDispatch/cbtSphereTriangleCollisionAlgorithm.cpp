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

#include "cbtSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

cbtSphereTriangleCollisionAlgorithm::cbtSphereTriangleCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool swapped)
	: cbtActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_ownManifold(false),
	  m_manifoldPtr(mf),
	  m_swapped(swapped)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

cbtSphereTriangleCollisionAlgorithm::~cbtSphereTriangleCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void cbtSphereTriangleCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	const cbtCollisionObjectWrapper* sphereObjWrap = m_swapped ? col1Wrap : col0Wrap;
	const cbtCollisionObjectWrapper* triObjWrap = m_swapped ? col0Wrap : col1Wrap;

	cbtSphereShape* sphere = (cbtSphereShape*)sphereObjWrap->getCollisionShape();
	cbtTriangleShape* triangle = (cbtTriangleShape*)triObjWrap->getCollisionShape();

	/// report a contact. internally this will be kept persistent, and contact reduction is done
	resultOut->setPersistentManifold(m_manifoldPtr);
	SphereTriangleDetector detector(sphere, triangle, m_manifoldPtr->getContactBreakingThreshold() + resultOut->m_closestPointDistanceThreshold);

	cbtDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_maximumDistanceSquared = cbtScalar(BT_LARGE_FLOAT);  ///@todo: tighter bounds
	input.m_transformA = sphereObjWrap->getWorldTransform();
	input.m_transformB = triObjWrap->getWorldTransform();

	bool swapResults = m_swapped;

	detector.getClosestPoints(input, *resultOut, dispatchInfo.m_debugDraw, swapResults);

	if (m_ownManifold)
		resultOut->refreshContactPoints();
}

cbtScalar cbtSphereTriangleCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* col0, cbtCollisionObject* col1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return cbtScalar(1.);
}
