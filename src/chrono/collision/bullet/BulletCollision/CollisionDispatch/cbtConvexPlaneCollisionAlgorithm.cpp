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

#include "cbtConvexPlaneCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "BulletCollision/CollisionShapes/cbtStaticPlaneShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

//#include <stdio.h>

cbtConvexPlaneCollisionAlgorithm::cbtConvexPlaneCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
	: cbtCollisionAlgorithm(ci),
	  m_ownManifold(false),
	  m_manifoldPtr(mf),
	  m_isSwapped(isSwapped),
	  m_numPerturbationIterations(numPerturbationIterations),
	  m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
	const cbtCollisionObjectWrapper* convexObjWrap = m_isSwapped ? col1Wrap : col0Wrap;
	const cbtCollisionObjectWrapper* planeObjWrap = m_isSwapped ? col0Wrap : col1Wrap;

	if (!m_manifoldPtr && m_dispatcher->needsCollision(convexObjWrap->getCollisionObject(), planeObjWrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(convexObjWrap->getCollisionObject(), planeObjWrap->getCollisionObject());
		m_ownManifold = true;
	}
}

cbtConvexPlaneCollisionAlgorithm::~cbtConvexPlaneCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void cbtConvexPlaneCollisionAlgorithm::collideSingleContact(const cbtQuaternion& perturbeRot, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	const cbtCollisionObjectWrapper* convexObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const cbtCollisionObjectWrapper* planeObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

	cbtConvexShape* convexShape = (cbtConvexShape*)convexObjWrap->getCollisionShape();
	cbtStaticPlaneShape* planeShape = (cbtStaticPlaneShape*)planeObjWrap->getCollisionShape();

	bool hasCollision = false;
	const cbtVector3& planeNormal = planeShape->getPlaneNormal();
	const cbtScalar& planeConstant = planeShape->getPlaneConstant();

	cbtTransform convexWorldTransform = convexObjWrap->getWorldTransform();
	cbtTransform convexInPlaneTrans;
	convexInPlaneTrans = planeObjWrap->getWorldTransform().inverse() * convexWorldTransform;
	//now perturbe the convex-world transform
	convexWorldTransform.getBasis() *= cbtMatrix3x3(perturbeRot);
	cbtTransform planeInConvex;
	planeInConvex = convexWorldTransform.inverse() * planeObjWrap->getWorldTransform();

	cbtVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis() * -planeNormal);

	cbtVector3 vtxInPlane = convexInPlaneTrans(vtx);
	cbtScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

	cbtVector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
	cbtVector3 vtxInPlaneWorld = planeObjWrap->getWorldTransform() * vtxInPlaneProjected;

	hasCollision = distance < m_manifoldPtr->getContactBreakingThreshold();
	resultOut->setPersistentManifold(m_manifoldPtr);
	if (hasCollision)
	{
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		cbtVector3 normalOnSurfaceB = planeObjWrap->getWorldTransform().getBasis() * planeNormal;
		cbtVector3 pOnB = vtxInPlaneWorld;
		resultOut->addContactPoint(normalOnSurfaceB, pOnB, distance);
	}
}

void cbtConvexPlaneCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)dispatchInfo;
	if (!m_manifoldPtr)
		return;

	const cbtCollisionObjectWrapper* convexObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const cbtCollisionObjectWrapper* planeObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

	cbtConvexShape* convexShape = (cbtConvexShape*)convexObjWrap->getCollisionShape();
	cbtStaticPlaneShape* planeShape = (cbtStaticPlaneShape*)planeObjWrap->getCollisionShape();

	bool hasCollision = false;
	const cbtVector3& planeNormal = planeShape->getPlaneNormal();
	const cbtScalar& planeConstant = planeShape->getPlaneConstant();
	cbtTransform planeInConvex;
	planeInConvex = convexObjWrap->getWorldTransform().inverse() * planeObjWrap->getWorldTransform();
	cbtTransform convexInPlaneTrans;
	convexInPlaneTrans = planeObjWrap->getWorldTransform().inverse() * convexObjWrap->getWorldTransform();

	cbtVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis() * -planeNormal);
	cbtVector3 vtxInPlane = convexInPlaneTrans(vtx);
	cbtScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

	cbtVector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
	cbtVector3 vtxInPlaneWorld = planeObjWrap->getWorldTransform() * vtxInPlaneProjected;

	hasCollision = distance < m_manifoldPtr->getContactBreakingThreshold()+ resultOut->m_closestPointDistanceThreshold;
	resultOut->setPersistentManifold(m_manifoldPtr);
	if (hasCollision)
	{
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		cbtVector3 normalOnSurfaceB = planeObjWrap->getWorldTransform().getBasis() * planeNormal;
		cbtVector3 pOnB = vtxInPlaneWorld;
		resultOut->addContactPoint(normalOnSurfaceB, pOnB, distance);
	}

	//the perturbation algorithm doesn't work well with implicit surfaces such as spheres, cylinder and cones:
	//they keep on rolling forever because of the additional off-center contact points
	//so only enable the feature for polyhedral shapes (cbtBoxShape, cbtConvexHullShape etc)
	if (convexShape->isPolyhedral() && resultOut->getPersistentManifold()->getNumContacts() < m_minimumPointsPerturbationThreshold)
	{
		cbtVector3 v0, v1;
		cbtPlaneSpace1(planeNormal, v0, v1);
		//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

		const cbtScalar angleLimit = 0.125f * SIMD_PI;
		cbtScalar perturbeAngle;
		cbtScalar radius = convexShape->getAngularMotionDisc();
		perturbeAngle = gContactBreakingThreshold / radius;
		if (perturbeAngle > angleLimit)
			perturbeAngle = angleLimit;

		cbtQuaternion perturbeRot(v0, perturbeAngle);
		for (int i = 0; i < m_numPerturbationIterations; i++)
		{
			cbtScalar iterationAngle = i * (SIMD_2_PI / cbtScalar(m_numPerturbationIterations));
			cbtQuaternion rotq(planeNormal, iterationAngle);
			collideSingleContact(rotq.inverse() * perturbeRot * rotq, body0Wrap, body1Wrap, dispatchInfo, resultOut);
		}
	}

	if (m_ownManifold)
	{
		if (m_manifoldPtr->getNumContacts())
		{
			resultOut->refreshContactPoints();
		}
	}
}

cbtScalar cbtConvexPlaneCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* col0, cbtCollisionObject* col1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return cbtScalar(1.);
}
