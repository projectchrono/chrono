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

#include "cbtConvexConcaveCollisionAlgorithm.h"
#include "LinearMath/cbtQuickprof.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionShapes/cbtMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/cbtConcaveShape.h"
#include "BulletCollision/CollisionDispatch/cbtManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/cbtRaycastCallback.h"
#include "BulletCollision/CollisionShapes/cbtTriangleShape.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "LinearMath/cbtIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"
#include "BulletCollision/CollisionShapes/cbtSdfCollisionShape.h"

cbtConvexConcaveCollisionAlgorithm::cbtConvexConcaveCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped)
	: cbtActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_cbtConvexTriangleCallback(ci.m_dispatcher1, body0Wrap, body1Wrap, isSwapped),
	  m_isSwapped(isSwapped)
{
}

cbtConvexConcaveCollisionAlgorithm::~cbtConvexConcaveCollisionAlgorithm()
{
}

void cbtConvexConcaveCollisionAlgorithm::getAllContactManifolds(cbtManifoldArray& manifoldArray)
{
	if (m_cbtConvexTriangleCallback.m_manifoldPtr)
	{
		manifoldArray.push_back(m_cbtConvexTriangleCallback.m_manifoldPtr);
	}
}

cbtConvexTriangleCallback::cbtConvexTriangleCallback(cbtDispatcher* dispatcher, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped) : m_dispatcher(dispatcher),
																																													 m_dispatchInfoPtr(0)
{
	m_convexBodyWrap = isSwapped ? body1Wrap : body0Wrap;
	m_triBodyWrap = isSwapped ? body0Wrap : body1Wrap;

	//
	// create the manifold from the dispatcher 'manifold pool'
	//
	m_manifoldPtr = m_dispatcher->getNewManifold(m_convexBodyWrap->getCollisionObject(), m_triBodyWrap->getCollisionObject());

	clearCache();
}

cbtConvexTriangleCallback::~cbtConvexTriangleCallback()
{
	clearCache();
	m_dispatcher->releaseManifold(m_manifoldPtr);
}

void cbtConvexTriangleCallback::clearCache()
{
	m_dispatcher->clearManifold(m_manifoldPtr);
}

void cbtConvexTriangleCallback::processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
{
	BT_PROFILE("cbtConvexTriangleCallback::processTriangle");

	if (!TestTriangleAgainstAabb2(triangle, m_aabbMin, m_aabbMax))
	{
		return;
	}

	//just for debugging purposes
	//printf("triangle %d",m_triangleCount++);

	cbtCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher1 = m_dispatcher;

#if 0	
	
	///debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && (m_dispatchInfoPtr->m_debugDraw->getDebugMode() &cbtIDebugDraw::DBG_DrawWireframe ))
	{
		const cbtCollisionObject* ob = const_cast<cbtCollisionObject*>(m_triBodyWrap->getCollisionObject());
		cbtVector3 color(1,1,0);
		cbtTransform& tr = ob->getWorldTransform();
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);
	}
#endif

	if (m_convexBodyWrap->getCollisionShape()->isConvex())
	{
		cbtTriangleShape tm(triangle[0], triangle[1], triangle[2]);
		tm.setMargin(m_collisionMarginTriangle);

		cbtCollisionObjectWrapper triObWrap(m_triBodyWrap, &tm, m_triBodyWrap->getCollisionObject(), m_triBodyWrap->getWorldTransform(), partId, triangleIndex);  //correct transform?
		cbtCollisionAlgorithm* colAlgo = 0;

		if (m_resultOut->m_closestPointDistanceThreshold > 0)
		{
			colAlgo = ci.m_dispatcher1->findAlgorithm(m_convexBodyWrap, &triObWrap, 0, BT_CLOSEST_POINT_ALGORITHMS);
		}
		else
		{
			colAlgo = ci.m_dispatcher1->findAlgorithm(m_convexBodyWrap, &triObWrap, m_manifoldPtr, BT_CONTACT_POINT_ALGORITHMS);
		}
		const cbtCollisionObjectWrapper* tmpWrap = 0;

		if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
		{
			tmpWrap = m_resultOut->getBody0Wrap();
			m_resultOut->setBody0Wrap(&triObWrap);
			m_resultOut->setShapeIdentifiersA(partId, triangleIndex);
		}
		else
		{
			tmpWrap = m_resultOut->getBody1Wrap();
			m_resultOut->setBody1Wrap(&triObWrap);
			m_resultOut->setShapeIdentifiersB(partId, triangleIndex);
		}

		colAlgo->processCollision(m_convexBodyWrap, &triObWrap, *m_dispatchInfoPtr, m_resultOut);

		if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
		{
			m_resultOut->setBody0Wrap(tmpWrap);
		}
		else
		{
			m_resultOut->setBody1Wrap(tmpWrap);
		}

		colAlgo->~cbtCollisionAlgorithm();
		ci.m_dispatcher1->freeCollisionAlgorithm(colAlgo);
	}
}

void cbtConvexTriangleCallback::setTimeStepAndCounters(cbtScalar collisionMarginTriangle, const cbtDispatcherInfo& dispatchInfo, const cbtCollisionObjectWrapper* convexBodyWrap, const cbtCollisionObjectWrapper* triBodyWrap, cbtManifoldResult* resultOut)
{
	m_convexBodyWrap = convexBodyWrap;
	m_triBodyWrap = triBodyWrap;

	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle;
	m_resultOut = resultOut;

	//recalc aabbs
	cbtTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triBodyWrap->getWorldTransform().inverse() * m_convexBodyWrap->getWorldTransform();
	const cbtCollisionShape* convexShape = static_cast<const cbtCollisionShape*>(m_convexBodyWrap->getCollisionShape());
	//CollisionShape* triangleShape = static_cast<cbtCollisionShape*>(triBody->m_collisionShape);
	convexShape->getAabb(convexInTriangleSpace, m_aabbMin, m_aabbMax);
	cbtScalar extraMargin = collisionMarginTriangle + resultOut->m_closestPointDistanceThreshold;

	cbtVector3 extra(extraMargin, extraMargin, extraMargin);

	m_aabbMax += extra;
	m_aabbMin -= extra;
}

void cbtConvexConcaveCollisionAlgorithm::clearCache()
{
	m_cbtConvexTriangleCallback.clearCache();
}

void cbtConvexConcaveCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	BT_PROFILE("cbtConvexConcaveCollisionAlgorithm::processCollision");

	const cbtCollisionObjectWrapper* convexBodyWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const cbtCollisionObjectWrapper* triBodyWrap = m_isSwapped ? body0Wrap : body1Wrap;

	if (triBodyWrap->getCollisionShape()->isConcave())
	{
		if (triBodyWrap->getCollisionShape()->getShapeType() == SDF_SHAPE_PROXYTYPE)
		{
			cbtSdfCollisionShape* sdfShape = (cbtSdfCollisionShape*)triBodyWrap->getCollisionShape();
			if (convexBodyWrap->getCollisionShape()->isConvex())
			{
				cbtConvexShape* convex = (cbtConvexShape*)convexBodyWrap->getCollisionShape();
				cbtAlignedObjectArray<cbtVector3> queryVertices;

				if (convex->isPolyhedral())
				{
					cbtPolyhedralConvexShape* poly = (cbtPolyhedralConvexShape*)convex;
					for (int v = 0; v < poly->getNumVertices(); v++)
					{
						cbtVector3 vtx;
						poly->getVertex(v, vtx);
						queryVertices.push_back(vtx);
					}
				}
				cbtScalar maxDist = SIMD_EPSILON;

				if (convex->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
				{
					queryVertices.push_back(cbtVector3(0, 0, 0));
					cbtSphereShape* sphere = (cbtSphereShape*)convex;
					maxDist = sphere->getRadius() + SIMD_EPSILON;
				}
				if (queryVertices.size())
				{
					resultOut->setPersistentManifold(m_cbtConvexTriangleCallback.m_manifoldPtr);
					//m_cbtConvexTriangleCallback.m_manifoldPtr->clearManifold();

					cbtPolyhedralConvexShape* poly = (cbtPolyhedralConvexShape*)convex;
					for (int v = 0; v < queryVertices.size(); v++)
					{
						const cbtVector3& vtx = queryVertices[v];
						cbtVector3 vtxWorldSpace = convexBodyWrap->getWorldTransform() * vtx;
						cbtVector3 vtxInSdf = triBodyWrap->getWorldTransform().invXform(vtxWorldSpace);

						cbtVector3 normalLocal;
						cbtScalar dist;
						if (sdfShape->queryPoint(vtxInSdf, dist, normalLocal))
						{
							if (dist <= maxDist)
							{
								normalLocal.safeNormalize();
								cbtVector3 normal = triBodyWrap->getWorldTransform().getBasis() * normalLocal;

								if (convex->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
								{
									cbtSphereShape* sphere = (cbtSphereShape*)convex;
									dist -= sphere->getRadius();
									vtxWorldSpace -= sphere->getRadius() * normal;
								}
								resultOut->addContactPoint(normal, vtxWorldSpace - normal * dist, dist);
							}
						}
					}
					resultOut->refreshContactPoints();
				}
			}
		}
		else
		{
			const cbtConcaveShape* concaveShape = static_cast<const cbtConcaveShape*>(triBodyWrap->getCollisionShape());

			if (convexBodyWrap->getCollisionShape()->isConvex())
			{
				cbtScalar collisionMarginTriangle = concaveShape->getMargin();

				resultOut->setPersistentManifold(m_cbtConvexTriangleCallback.m_manifoldPtr);
				m_cbtConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, convexBodyWrap, triBodyWrap, resultOut);

				m_cbtConvexTriangleCallback.m_manifoldPtr->setBodies(convexBodyWrap->getCollisionObject(), triBodyWrap->getCollisionObject());

				concaveShape->processAllTriangles(&m_cbtConvexTriangleCallback, m_cbtConvexTriangleCallback.getAabbMin(), m_cbtConvexTriangleCallback.getAabbMax());

				resultOut->refreshContactPoints();

				m_cbtConvexTriangleCallback.clearWrapperData();
			}
		}
	}
}

cbtScalar cbtConvexConcaveCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	cbtCollisionObject* convexbody = m_isSwapped ? body1 : body0;
	cbtCollisionObject* triBody = m_isSwapped ? body0 : body1;

	//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the cbtConvexCast)

	//only perform CCD above a certain threshold, this prevents blocking on the long run
	//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
	cbtScalar squareMot0 = (convexbody->getInterpolationWorldTransform().getOrigin() - convexbody->getWorldTransform().getOrigin()).length2();
	if (squareMot0 < convexbody->getCcdSquareMotionThreshold())
	{
		return cbtScalar(1.);
	}

	//const cbtVector3& from = convexbody->m_worldTransform.getOrigin();
	//cbtVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
	//todo: only do if the motion exceeds the 'radius'

	cbtTransform triInv = triBody->getWorldTransform().inverse();
	cbtTransform convexFromLocal = triInv * convexbody->getWorldTransform();
	cbtTransform convexToLocal = triInv * convexbody->getInterpolationWorldTransform();

	struct LocalTriangleSphereCastCallback : public cbtTriangleCallback
	{
		cbtTransform m_ccdSphereFromTrans;
		cbtTransform m_ccdSphereToTrans;
		cbtTransform m_meshTransform;

		cbtScalar m_ccdSphereRadius;
		cbtScalar m_hitFraction;

		LocalTriangleSphereCastCallback(const cbtTransform& from, const cbtTransform& to, cbtScalar ccdSphereRadius, cbtScalar hitFraction)
			: m_ccdSphereFromTrans(from),
			  m_ccdSphereToTrans(to),
			  m_ccdSphereRadius(ccdSphereRadius),
			  m_hitFraction(hitFraction)
		{
		}

		virtual void processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
		{
			BT_PROFILE("processTriangle");
			(void)partId;
			(void)triangleIndex;
			//do a swept sphere for now
			cbtTransform ident;
			ident.setIdentity();
			cbtConvexCast::CastResult castResult;
			castResult.m_fraction = m_hitFraction;
			cbtSphereShape pointShape(m_ccdSphereRadius);
			cbtTriangleShape triShape(triangle[0], triangle[1], triangle[2]);
			cbtVoronoiSimplexSolver simplexSolver;
			cbtSubsimplexConvexCast convexCaster(&pointShape, &triShape, &simplexSolver);
			//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
			//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			//local space?

			if (convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans, m_ccdSphereToTrans,
											  ident, ident, castResult))
			{
				if (m_hitFraction > castResult.m_fraction)
					m_hitFraction = castResult.m_fraction;
			}
		}
	};

	if (triBody->getCollisionShape()->isConcave())
	{
		cbtVector3 rayAabbMin = convexFromLocal.getOrigin();
		rayAabbMin.setMin(convexToLocal.getOrigin());
		cbtVector3 rayAabbMax = convexFromLocal.getOrigin();
		rayAabbMax.setMax(convexToLocal.getOrigin());
		cbtScalar ccdRadius0 = convexbody->getCcdSweptSphereRadius();
		rayAabbMin -= cbtVector3(ccdRadius0, ccdRadius0, ccdRadius0);
		rayAabbMax += cbtVector3(ccdRadius0, ccdRadius0, ccdRadius0);

		cbtScalar curHitFraction = cbtScalar(1.);  //is this available?
		LocalTriangleSphereCastCallback raycastCallback(convexFromLocal, convexToLocal,
														convexbody->getCcdSweptSphereRadius(), curHitFraction);

		raycastCallback.m_hitFraction = convexbody->getHitFraction();

		cbtCollisionObject* concavebody = triBody;

		cbtConcaveShape* triangleMesh = (cbtConcaveShape*)concavebody->getCollisionShape();

		if (triangleMesh)
		{
			triangleMesh->processAllTriangles(&raycastCallback, rayAabbMin, rayAabbMax);
		}

		if (raycastCallback.m_hitFraction < convexbody->getHitFraction())
		{
			convexbody->setHitFraction(raycastCallback.m_hitFraction);
			return raycastCallback.m_hitFraction;
		}
	}

	return cbtScalar(1.);
}
