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

#include "cbtSphereBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"
//#include <stdio.h>

cbtSphereBoxCollisionAlgorithm::cbtSphereBoxCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* col0Wrap, const cbtCollisionObjectWrapper* col1Wrap, bool isSwapped)
	: cbtActivatingCollisionAlgorithm(ci, col0Wrap, col1Wrap),
	  m_ownManifold(false),
	  m_manifoldPtr(mf),
	  m_isSwapped(isSwapped)
{
	const cbtCollisionObjectWrapper* sphereObjWrap = m_isSwapped ? col1Wrap : col0Wrap;
	const cbtCollisionObjectWrapper* boxObjWrap = m_isSwapped ? col0Wrap : col1Wrap;

	if (!m_manifoldPtr && m_dispatcher->needsCollision(sphereObjWrap->getCollisionObject(), boxObjWrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(sphereObjWrap->getCollisionObject(), boxObjWrap->getCollisionObject());
		m_ownManifold = true;
	}
}

cbtSphereBoxCollisionAlgorithm::~cbtSphereBoxCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void cbtSphereBoxCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)dispatchInfo;
	(void)resultOut;
	if (!m_manifoldPtr)
		return;

	const cbtCollisionObjectWrapper* sphereObjWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const cbtCollisionObjectWrapper* boxObjWrap = m_isSwapped ? body0Wrap : body1Wrap;

	cbtVector3 pOnBox;

	cbtVector3 normalOnSurfaceB;
	cbtScalar penetrationDepth;
	cbtVector3 sphereCenter = sphereObjWrap->getWorldTransform().getOrigin();
	const cbtSphereShape* sphere0 = (const cbtSphereShape*)sphereObjWrap->getCollisionShape();
	cbtScalar radius = sphere0->getRadius();
	cbtScalar maxContactDistance = m_manifoldPtr->getContactBreakingThreshold();

	resultOut->setPersistentManifold(m_manifoldPtr);

	if (getSphereDistance(boxObjWrap, pOnBox, normalOnSurfaceB, penetrationDepth, sphereCenter, radius, maxContactDistance))
	{
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		resultOut->addContactPoint(normalOnSurfaceB, pOnBox, penetrationDepth);
	}

	if (m_ownManifold)
	{
		if (m_manifoldPtr->getNumContacts())
		{
			resultOut->refreshContactPoints();
		}
	}
}

cbtScalar cbtSphereBoxCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* col0, cbtCollisionObject* col1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return cbtScalar(1.);
}

bool cbtSphereBoxCollisionAlgorithm::getSphereDistance(const cbtCollisionObjectWrapper* boxObjWrap, cbtVector3& pointOnBox, cbtVector3& normal, cbtScalar& penetrationDepth, const cbtVector3& sphereCenter, cbtScalar fRadius, cbtScalar maxContactDistance)
{
	const cbtBoxShape* boxShape = (const cbtBoxShape*)boxObjWrap->getCollisionShape();
	cbtVector3 const& boxHalfExtent = boxShape->getHalfExtentsWithoutMargin();
	cbtScalar boxMargin = boxShape->getMargin();
	penetrationDepth = 1.0f;

	// convert the sphere position to the box's local space
	cbtTransform const& m44T = boxObjWrap->getWorldTransform();
	cbtVector3 sphereRelPos = m44T.invXform(sphereCenter);

	// Determine the closest point to the sphere center in the box
	cbtVector3 closestPoint = sphereRelPos;
	closestPoint.setX(cbtMin(boxHalfExtent.getX(), closestPoint.getX()));
	closestPoint.setX(cbtMax(-boxHalfExtent.getX(), closestPoint.getX()));
	closestPoint.setY(cbtMin(boxHalfExtent.getY(), closestPoint.getY()));
	closestPoint.setY(cbtMax(-boxHalfExtent.getY(), closestPoint.getY()));
	closestPoint.setZ(cbtMin(boxHalfExtent.getZ(), closestPoint.getZ()));
	closestPoint.setZ(cbtMax(-boxHalfExtent.getZ(), closestPoint.getZ()));

	cbtScalar intersectionDist = fRadius + boxMargin;
	cbtScalar contactDist = intersectionDist + maxContactDistance;
	normal = sphereRelPos - closestPoint;

	//if there is no penetration, we are done
	cbtScalar dist2 = normal.length2();
	if (dist2 > contactDist * contactDist)
	{
		return false;
	}

	cbtScalar distance;

	//special case if the sphere center is inside the box
	if (dist2 <= SIMD_EPSILON)
	{
		distance = -getSpherePenetration(boxHalfExtent, sphereRelPos, closestPoint, normal);
	}
	else  //compute the penetration details
	{
		distance = normal.length();
		normal /= distance;
	}

	pointOnBox = closestPoint + normal * boxMargin;
	//	v3PointOnSphere = sphereRelPos - (normal * fRadius);
	penetrationDepth = distance - intersectionDist;

	// transform back in world space
	cbtVector3 tmp = m44T(pointOnBox);
	pointOnBox = tmp;
	//	tmp = m44T(v3PointOnSphere);
	//	v3PointOnSphere = tmp;
	tmp = m44T.getBasis() * normal;
	normal = tmp;

	return true;
}

cbtScalar cbtSphereBoxCollisionAlgorithm::getSpherePenetration(cbtVector3 const& boxHalfExtent, cbtVector3 const& sphereRelPos, cbtVector3& closestPoint, cbtVector3& normal)
{
	//project the center of the sphere on the closest face of the box
	cbtScalar faceDist = boxHalfExtent.getX() - sphereRelPos.getX();
	cbtScalar minDist = faceDist;
	closestPoint.setX(boxHalfExtent.getX());
	normal.setValue(cbtScalar(1.0f), cbtScalar(0.0f), cbtScalar(0.0f));

	faceDist = boxHalfExtent.getX() + sphereRelPos.getX();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setX(-boxHalfExtent.getX());
		normal.setValue(cbtScalar(-1.0f), cbtScalar(0.0f), cbtScalar(0.0f));
	}

	faceDist = boxHalfExtent.getY() - sphereRelPos.getY();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setY(boxHalfExtent.getY());
		normal.setValue(cbtScalar(0.0f), cbtScalar(1.0f), cbtScalar(0.0f));
	}

	faceDist = boxHalfExtent.getY() + sphereRelPos.getY();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setY(-boxHalfExtent.getY());
		normal.setValue(cbtScalar(0.0f), cbtScalar(-1.0f), cbtScalar(0.0f));
	}

	faceDist = boxHalfExtent.getZ() - sphereRelPos.getZ();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setZ(boxHalfExtent.getZ());
		normal.setValue(cbtScalar(0.0f), cbtScalar(0.0f), cbtScalar(1.0f));
	}

	faceDist = boxHalfExtent.getZ() + sphereRelPos.getZ();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setZ(-boxHalfExtent.getZ());
		normal.setValue(cbtScalar(0.0f), cbtScalar(0.0f), cbtScalar(-1.0f));
	}

	return minDist;
}
