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

#include "cbtCollisionWorld.h"
#include "cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionShapes/cbtCollisionShape.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"                 //for raycasting
#include "BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.h"        //for raycasting
#include "BulletCollision/CollisionShapes/cbtScaledBvhTriangleMeshShape.h"  //for raycasting
#include "BulletCollision/NarrowPhaseCollision/cbtRaycastCallback.h"
#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/cbtContinuousConvexCollision.h"
#include "BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/cbtDbvt.h"
#include "BulletCollision/CollisionShapes/cbtCEtriangleShape.h" //***CHRONO***
#include "LinearMath/cbtAabbUtil2.h"
#include "LinearMath/cbtQuickprof.h"
#include "LinearMath/cbtSerializer.h"
#include "BulletCollision/CollisionShapes/cbtConvexPolyhedron.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

//#define DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION

//#define USE_BRUTEFORCE_RAYBROADPHASE 1
//RECALCULATE_AABB is slower, but benefit is that you don't need to call 'stepSimulation'  or 'updateAabbs' before using a rayTest
//#define RECALCULATE_AABB_RAYCAST 1

//When the user doesn't provide dispatcher or broadphase, create basic versions (and delete them in destructor)
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtSimpleBroadphase.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionConfiguration.h"

///for debug drawing

//for debug rendering
#include "BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "BulletCollision/CollisionShapes/cbtCapsuleShape.h"
#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "BulletCollision/CollisionShapes/cbtConeShape.h"
#include "BulletCollision/CollisionShapes/cbtConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtCylinderShape.h"
#include "BulletCollision/CollisionShapes/cbtCylindricalShellShape.h"  /* ***CHRONO*** */
#include "BulletCollision/CollisionShapes/cbtMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtTriangleCallback.h"
#include "BulletCollision/CollisionShapes/cbtTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtStaticPlaneShape.h"

cbtCollisionWorld::cbtCollisionWorld(cbtDispatcher* dispatcher, cbtBroadphaseInterface* pairCache, cbtCollisionConfiguration* collisionConfiguration)
	: m_dispatcher1(dispatcher),
	  m_broadphasePairCache(pairCache),
	  m_debugDrawer(0),
	  m_forceUpdateAllAabbs(true)
{
}

cbtCollisionWorld::~cbtCollisionWorld()
{
	//clean up remaining objects
	int i;
	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		cbtCollisionObject* collisionObject = m_collisionObjects[i];

		cbtBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// only clear the cached algorithms
			//
			getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp, m_dispatcher1);
			getBroadphase()->destroyProxy(bp, m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}
}

void cbtCollisionWorld::refreshBroadphaseProxy(cbtCollisionObject* collisionObject)
{
	if (collisionObject->getBroadphaseHandle())
	{
		int collisionFilterGroup = collisionObject->getBroadphaseHandle()->m_collisionFilterGroup;
		int collisionFilterMask = collisionObject->getBroadphaseHandle()->m_collisionFilterMask;

		getBroadphase()->destroyProxy(collisionObject->getBroadphaseHandle(), getDispatcher());

		//calculate new AABB
		cbtTransform trans = collisionObject->getWorldTransform();

		cbtVector3 minAabb;
		cbtVector3 maxAabb;
		collisionObject->getCollisionShape()->getAabb(trans, minAabb, maxAabb);

		int type = collisionObject->getCollisionShape()->getShapeType();
		collisionObject->setBroadphaseHandle(getBroadphase()->createProxy(
			minAabb,
			maxAabb,
			type,
			collisionObject,
			collisionFilterGroup,
			collisionFilterMask,
			m_dispatcher1));
	}
}

void cbtCollisionWorld::addCollisionObject(cbtCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
{
	cbtAssert(collisionObject);

	//check that the object isn't already added
	cbtAssert(m_collisionObjects.findLinearSearch(collisionObject) == m_collisionObjects.size());
	cbtAssert(collisionObject->getWorldArrayIndex() == -1);  // do not add the same object to more than one collision world

	collisionObject->setWorldArrayIndex(m_collisionObjects.size());
	m_collisionObjects.push_back(collisionObject);

	//calculate new AABB
	cbtTransform trans = collisionObject->getWorldTransform();

	cbtVector3 minAabb;
	cbtVector3 maxAabb;
	collisionObject->getCollisionShape()->getAabb(trans, minAabb, maxAabb);

	int type = collisionObject->getCollisionShape()->getShapeType();
	collisionObject->setBroadphaseHandle(getBroadphase()->createProxy(
		minAabb,
		maxAabb,
		type,
		collisionObject,
		collisionFilterGroup,
		collisionFilterMask,
		m_dispatcher1));
}

void cbtCollisionWorld::updateSingleAabb(cbtCollisionObject* colObj)
{
	cbtVector3 minAabb, maxAabb;
	colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb, maxAabb);
	//need to increase the aabb for contact thresholds
	cbtVector3 contactThreshold(gContactBreakingThreshold, gContactBreakingThreshold, gContactBreakingThreshold);
    /* ***CHRONO*** minAabb and maxAabb not needed because the margin in Chrono also icludes the contact braking layer */
	////minAabb -= contactThreshold;
	////maxAabb += contactThreshold;

	if (getDispatchInfo().m_useContinuous && colObj->getInternalType() == cbtCollisionObject::CO_RIGID_BODY && !colObj->isStaticOrKinematicObject())
	{
		cbtVector3 minAabb2, maxAabb2;
		colObj->getCollisionShape()->getAabb(colObj->getInterpolationWorldTransform(), minAabb2, maxAabb2);
		minAabb2 -= contactThreshold;
		maxAabb2 += contactThreshold;
		minAabb.setMin(minAabb2);
		maxAabb.setMax(maxAabb2);
	}

	cbtBroadphaseInterface* bp = (cbtBroadphaseInterface*)m_broadphasePairCache;

	//moving objects should be moderately sized, probably something wrong if not
	if (colObj->isStaticObject() || ((maxAabb - minAabb).length2() < cbtScalar(1e12)))
	{
		bp->setAabb(colObj->getBroadphaseHandle(), minAabb, maxAabb, m_dispatcher1);
	}
	else
	{
		//something went wrong, investigate
		//this assert is unwanted in 3D modelers (danger of loosing work)
		colObj->setActivationState(DISABLE_SIMULATION);

		static bool reportMe = true;
		if (reportMe && m_debugDrawer)
		{
			reportMe = false;
			m_debugDrawer->reportErrorWarning("Overflow in AABB, object removed from simulation");
			m_debugDrawer->reportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n");
			m_debugDrawer->reportErrorWarning("Please include above information, your Platform, version of OS.\n");
			m_debugDrawer->reportErrorWarning("Thanks.\n");
		}
	}
}

void cbtCollisionWorld::updateAabbs()
{
	BT_PROFILE("updateAabbs");

	cbtTransform predictedTrans;
	for (int i = 0; i < m_collisionObjects.size(); i++)
	{
		cbtCollisionObject* colObj = m_collisionObjects[i];
		cbtAssert(colObj->getWorldArrayIndex() == i);

		//only update aabb of active objects
		if (m_forceUpdateAllAabbs || colObj->isActive())
		{
			updateSingleAabb(colObj);
		}
	}
}

void cbtCollisionWorld::computeOverlappingPairs()
{
	BT_PROFILE("calculateOverlappingPairs");
	m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
}

void cbtCollisionWorld::performDiscreteCollisionDetection()
{
	BT_PROFILE("performDiscreteCollisionDetection");

	cbtDispatcherInfo& dispatchInfo = getDispatchInfo();

	updateAabbs();

	{
        /* ***CHRONO*** Add Chrono-specific timers */
		BT_PROFILE("computeOverlappingPairs");
        CH_PROFILE("Broad-phase");
        timer_collision_broad.start();
		computeOverlappingPairs();
        timer_collision_broad.stop();
	}

	cbtDispatcher* dispatcher = getDispatcher();
	{
        /* ***CHRONO*** Add Chrono-specific timers */
        BT_PROFILE("dispatchAllCollisionPairs");
		CH_PROFILE("Narrow-phase");
        timer_collision_narrow.start();
		if (dispatcher)
			dispatcher->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(), dispatchInfo, m_dispatcher1);
		timer_collision_narrow.stop();
	}
}

void cbtCollisionWorld::removeCollisionObject(cbtCollisionObject* collisionObject)
{
	//bool removeFromBroadphase = false;

	{
		cbtBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// only clear the cached algorithms
			//
			getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp, m_dispatcher1);
			getBroadphase()->destroyProxy(bp, m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}

	int iObj = collisionObject->getWorldArrayIndex();
	//    cbtAssert(iObj >= 0 && iObj < m_collisionObjects.size()); // trying to remove an object that was never added or already removed previously?
	if (iObj >= 0 && iObj < m_collisionObjects.size())
	{
		cbtAssert(collisionObject == m_collisionObjects[iObj]);
		m_collisionObjects.swap(iObj, m_collisionObjects.size() - 1);
		m_collisionObjects.pop_back();
		if (iObj < m_collisionObjects.size())
		{
			m_collisionObjects[iObj]->setWorldArrayIndex(iObj);
		}
	}
	else
	{
		// slow linear search
		//swapremove
		m_collisionObjects.remove(collisionObject);
	}
	collisionObject->setWorldArrayIndex(-1);
}

void cbtCollisionWorld::rayTestSingle(const cbtTransform& rayFromTrans, const cbtTransform& rayToTrans,
									 cbtCollisionObject* collisionObject,
									 const cbtCollisionShape* collisionShape,
									 const cbtTransform& colObjWorldTransform,
									 RayResultCallback& resultCallback)
{
	cbtCollisionObjectWrapper colObWrap(0, collisionShape, collisionObject, colObjWorldTransform, -1, -1);
	cbtCollisionWorld::rayTestSingleInternal(rayFromTrans, rayToTrans, &colObWrap, resultCallback);
}

void cbtCollisionWorld::rayTestSingleInternal(const cbtTransform& rayFromTrans, const cbtTransform& rayToTrans,
											 const cbtCollisionObjectWrapper* collisionObjectWrap,
											 RayResultCallback& resultCallback)
{
	cbtSphereShape pointShape(cbtScalar(0.0));
	pointShape.setMargin(0.f);
	const cbtConvexShape* castShape = &pointShape;
	const cbtCollisionShape* collisionShape = collisionObjectWrap->getCollisionShape();
	const cbtTransform& colObjWorldTransform = collisionObjectWrap->getWorldTransform();

	if (collisionShape->isConvex())
	{
		//		BT_PROFILE("rayTestConvex");
		cbtConvexCast::CastResult castResult;
		castResult.m_fraction = resultCallback.m_closestHitFraction;

		cbtConvexShape* convexShape = (cbtConvexShape*)collisionShape;
		cbtVoronoiSimplexSolver simplexSolver;
		cbtSubsimplexConvexCast subSimplexConvexCaster(castShape, convexShape, &simplexSolver);

		cbtGjkConvexCast gjkConvexCaster(castShape, convexShape, &simplexSolver);

		//cbtContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);

		cbtConvexCast* convexCasterPtr = 0;
		//use kF_UseSubSimplexConvexCastRaytest by default
		if (resultCallback.m_flags & cbtTriangleRaycastCallback::kF_UseGjkConvexCastRaytest)
			convexCasterPtr = &gjkConvexCaster;
		else
			convexCasterPtr = &subSimplexConvexCaster;

		cbtConvexCast& convexCaster = *convexCasterPtr;

		if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, colObjWorldTransform, colObjWorldTransform, castResult))
		{
			//add hit
			if (castResult.m_normal.length2() > cbtScalar(0.0001))
			{
				if (castResult.m_fraction < resultCallback.m_closestHitFraction)
				{
					//todo: figure out what this is about. When is rayFromTest.getBasis() not identity?
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
					//rotate normal into worldspace
					castResult.m_normal = rayFromTrans.getBasis() * castResult.m_normal;
#endif  //USE_SUBSIMPLEX_CONVEX_CAST

					castResult.m_normal.normalize();
					cbtCollisionWorld::LocalRayResult localRayResult(
						collisionObjectWrap->getCollisionObject(),
						0,
						castResult.m_normal,
						castResult.m_fraction);

					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(localRayResult, normalInWorldSpace);
				}
			}
		}
	}
	else
	{
		if (collisionShape->isConcave())
		{
			//ConvexCast::CastResult
			struct BridgeTriangleRaycastCallback : public cbtTriangleRaycastCallback
			{
				cbtCollisionWorld::RayResultCallback* m_resultCallback;
				const cbtCollisionObject* m_collisionObject;
				const cbtConcaveShape* m_triangleMesh;

				cbtTransform m_colObjWorldTransform;

				BridgeTriangleRaycastCallback(const cbtVector3& from, const cbtVector3& to,
											  cbtCollisionWorld::RayResultCallback* resultCallback, const cbtCollisionObject* collisionObject, const cbtConcaveShape* triangleMesh, const cbtTransform& colObjWorldTransform) :  //@BP Mod
																																																							cbtTriangleRaycastCallback(from, to, resultCallback->m_flags),
																																																							m_resultCallback(resultCallback),
																																																							m_collisionObject(collisionObject),
																																																							m_triangleMesh(triangleMesh),
																																																							m_colObjWorldTransform(colObjWorldTransform)
				{
				}

				virtual cbtScalar reportHit(const cbtVector3& hitNormalLocal, cbtScalar hitFraction, int partId, int triangleIndex)
				{
					cbtCollisionWorld::LocalShapeInfo shapeInfo;
					shapeInfo.m_shapePart = partId;
					shapeInfo.m_triangleIndex = triangleIndex;

					cbtVector3 hitNormalWorld = m_colObjWorldTransform.getBasis() * hitNormalLocal;

					cbtCollisionWorld::LocalRayResult rayResult(m_collisionObject,
															   &shapeInfo,
															   hitNormalWorld,
															   hitFraction);

					bool normalInWorldSpace = true;
					return m_resultCallback->addSingleResult(rayResult, normalInWorldSpace);
				}
			};

			cbtTransform worldTocollisionObject = colObjWorldTransform.inverse();
			cbtVector3 rayFromLocal = worldTocollisionObject * rayFromTrans.getOrigin();
			cbtVector3 rayToLocal = worldTocollisionObject * rayToTrans.getOrigin();

			//			BT_PROFILE("rayTestConcave");
			if (collisionShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				///optimized version for cbtBvhTriangleMeshShape
				cbtBvhTriangleMeshShape* triangleMesh = (cbtBvhTriangleMeshShape*)collisionShape;

				BridgeTriangleRaycastCallback rcb(rayFromLocal, rayToLocal, &resultCallback, collisionObjectWrap->getCollisionObject(), triangleMesh, colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;
				triangleMesh->performRaycast(&rcb, rayFromLocal, rayToLocal);
			}
			else if (collisionShape->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				///optimized version for cbtScaledBvhTriangleMeshShape
				cbtScaledBvhTriangleMeshShape* scaledTriangleMesh = (cbtScaledBvhTriangleMeshShape*)collisionShape;
				cbtBvhTriangleMeshShape* triangleMesh = (cbtBvhTriangleMeshShape*)scaledTriangleMesh->getChildShape();

				//scale the ray positions
				cbtVector3 scale = scaledTriangleMesh->getLocalScaling();
				cbtVector3 rayFromLocalScaled = rayFromLocal / scale;
				cbtVector3 rayToLocalScaled = rayToLocal / scale;

				//perform raycast in the underlying cbtBvhTriangleMeshShape
				BridgeTriangleRaycastCallback rcb(rayFromLocalScaled, rayToLocalScaled, &resultCallback, collisionObjectWrap->getCollisionObject(), triangleMesh, colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;
				triangleMesh->performRaycast(&rcb, rayFromLocalScaled, rayToLocalScaled);
			}
			else
			{
				//generic (slower) case
				cbtConcaveShape* concaveShape = (cbtConcaveShape*)collisionShape;

				cbtTransform worldTocollisionObject = colObjWorldTransform.inverse();

				cbtVector3 rayFromLocal = worldTocollisionObject * rayFromTrans.getOrigin();
				cbtVector3 rayToLocal = worldTocollisionObject * rayToTrans.getOrigin();

				//ConvexCast::CastResult

				struct BridgeTriangleRaycastCallback : public cbtTriangleRaycastCallback
				{
					cbtCollisionWorld::RayResultCallback* m_resultCallback;
					const cbtCollisionObject* m_collisionObject;
					cbtConcaveShape* m_triangleMesh;

					cbtTransform m_colObjWorldTransform;

					BridgeTriangleRaycastCallback(const cbtVector3& from, const cbtVector3& to,
												  cbtCollisionWorld::RayResultCallback* resultCallback, const cbtCollisionObject* collisionObject, cbtConcaveShape* triangleMesh, const cbtTransform& colObjWorldTransform) :  //@BP Mod
																																																						  cbtTriangleRaycastCallback(from, to, resultCallback->m_flags),
																																																						  m_resultCallback(resultCallback),
																																																						  m_collisionObject(collisionObject),
																																																						  m_triangleMesh(triangleMesh),
																																																						  m_colObjWorldTransform(colObjWorldTransform)
					{
					}

					virtual cbtScalar reportHit(const cbtVector3& hitNormalLocal, cbtScalar hitFraction, int partId, int triangleIndex)
					{
						cbtCollisionWorld::LocalShapeInfo shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;

						cbtVector3 hitNormalWorld = m_colObjWorldTransform.getBasis() * hitNormalLocal;

						cbtCollisionWorld::LocalRayResult rayResult(m_collisionObject,
																   &shapeInfo,
																   hitNormalWorld,
																   hitFraction);

						bool normalInWorldSpace = true;
						return m_resultCallback->addSingleResult(rayResult, normalInWorldSpace);
					}
				};

				BridgeTriangleRaycastCallback rcb(rayFromLocal, rayToLocal, &resultCallback, collisionObjectWrap->getCollisionObject(), concaveShape, colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;

				cbtVector3 rayAabbMinLocal = rayFromLocal;
				rayAabbMinLocal.setMin(rayToLocal);
				cbtVector3 rayAabbMaxLocal = rayFromLocal;
				rayAabbMaxLocal.setMax(rayToLocal);

				concaveShape->processAllTriangles(&rcb, rayAabbMinLocal, rayAabbMaxLocal);
			}
		}
		else
		{
			//			BT_PROFILE("rayTestCompound");
			if (collisionShape->isCompound())
			{
				struct LocalInfoAdder2 : public RayResultCallback
				{
					RayResultCallback* m_userCallback;
					int m_i;

					LocalInfoAdder2(int i, RayResultCallback* user)
						: m_userCallback(user), m_i(i)
					{
						m_closestHitFraction = m_userCallback->m_closestHitFraction;
						m_flags = m_userCallback->m_flags;
					}
					virtual bool needsCollision(cbtBroadphaseProxy* p) const
					{
						return m_userCallback->needsCollision(p);
					}

					virtual cbtScalar addSingleResult(cbtCollisionWorld::LocalRayResult& r, bool b)
					{
						cbtCollisionWorld::LocalShapeInfo shapeInfo;
						shapeInfo.m_shapePart = -1;
						shapeInfo.m_triangleIndex = m_i;
						if (r.m_localShapeInfo == NULL)
							r.m_localShapeInfo = &shapeInfo;

						const cbtScalar result = m_userCallback->addSingleResult(r, b);
						m_closestHitFraction = m_userCallback->m_closestHitFraction;
						return result;
					}
				};

				struct RayTester : cbtDbvt::ICollide
				{
					const cbtCollisionObject* m_collisionObject;
					const cbtCompoundShape* m_compoundShape;
					const cbtTransform& m_colObjWorldTransform;
					const cbtTransform& m_rayFromTrans;
					const cbtTransform& m_rayToTrans;
					RayResultCallback& m_resultCallback;

					RayTester(const cbtCollisionObject* collisionObject,
							  const cbtCompoundShape* compoundShape,
							  const cbtTransform& colObjWorldTransform,
							  const cbtTransform& rayFromTrans,
							  const cbtTransform& rayToTrans,
							  RayResultCallback& resultCallback) : m_collisionObject(collisionObject),
																   m_compoundShape(compoundShape),
																   m_colObjWorldTransform(colObjWorldTransform),
																   m_rayFromTrans(rayFromTrans),
																   m_rayToTrans(rayToTrans),
																   m_resultCallback(resultCallback)
					{
					}

					void ProcessLeaf(int i)
					{
						const cbtCollisionShape* childCollisionShape = m_compoundShape->getChildShape(i);
						const cbtTransform& childTrans = m_compoundShape->getChildTransform(i);
						cbtTransform childWorldTrans = m_colObjWorldTransform * childTrans;

						cbtCollisionObjectWrapper tmpOb(0, childCollisionShape, m_collisionObject, childWorldTrans, -1, i);
						// replace collision shape so that callback can determine the triangle

						LocalInfoAdder2 my_cb(i, &m_resultCallback);

						rayTestSingleInternal(
							m_rayFromTrans,
							m_rayToTrans,
							&tmpOb,
							my_cb);
					}

					void Process(const cbtDbvtNode* leaf)
					{
						ProcessLeaf(leaf->dataAsInt);
					}
				};

				const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(collisionShape);
				const cbtDbvt* dbvt = compoundShape->getDynamicAabbTree();

				RayTester rayCB(
					collisionObjectWrap->getCollisionObject(),
					compoundShape,
					colObjWorldTransform,
					rayFromTrans,
					rayToTrans,
					resultCallback);
#ifndef DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
				if (dbvt)
				{
					cbtVector3 localRayFrom = colObjWorldTransform.inverseTimes(rayFromTrans).getOrigin();
					cbtVector3 localRayTo = colObjWorldTransform.inverseTimes(rayToTrans).getOrigin();
					cbtDbvt::rayTest(dbvt->m_root, localRayFrom, localRayTo, rayCB);
				}
				else
#endif  //DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
				{
					for (int i = 0, n = compoundShape->getNumChildShapes(); i < n; ++i)
					{
						rayCB.ProcessLeaf(i);
					}
				}
			}
		}
	}
}

void cbtCollisionWorld::objectQuerySingle(const cbtConvexShape* castShape, const cbtTransform& convexFromTrans, const cbtTransform& convexToTrans,
										 cbtCollisionObject* collisionObject,
										 const cbtCollisionShape* collisionShape,
										 const cbtTransform& colObjWorldTransform,
										 ConvexResultCallback& resultCallback, cbtScalar allowedPenetration)
{
	cbtCollisionObjectWrapper tmpOb(0, collisionShape, collisionObject, colObjWorldTransform, -1, -1);
	cbtCollisionWorld::objectQuerySingleInternal(castShape, convexFromTrans, convexToTrans, &tmpOb, resultCallback, allowedPenetration);
}

void cbtCollisionWorld::objectQuerySingleInternal(const cbtConvexShape* castShape, const cbtTransform& convexFromTrans, const cbtTransform& convexToTrans,
												 const cbtCollisionObjectWrapper* colObjWrap,
												 ConvexResultCallback& resultCallback, cbtScalar allowedPenetration)
{
	const cbtCollisionShape* collisionShape = colObjWrap->getCollisionShape();
	const cbtTransform& colObjWorldTransform = colObjWrap->getWorldTransform();

	if (collisionShape->isConvex())
	{
		//BT_PROFILE("convexSweepConvex");
		cbtConvexCast::CastResult castResult;
		castResult.m_allowedPenetration = allowedPenetration;
		castResult.m_fraction = resultCallback.m_closestHitFraction;  //cbtScalar(1.);//??

		cbtConvexShape* convexShape = (cbtConvexShape*)collisionShape;
		cbtVoronoiSimplexSolver simplexSolver;
		cbtGjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver;

		cbtContinuousConvexCollision convexCaster1(castShape, convexShape, &simplexSolver, &gjkEpaPenetrationSolver);
		//cbtGjkConvexCast convexCaster2(castShape,convexShape,&simplexSolver);
		//cbtSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

		cbtConvexCast* castPtr = &convexCaster1;

		if (castPtr->calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult))
		{
			//add hit
			if (castResult.m_normal.length2() > cbtScalar(0.0001))
			{
				if (castResult.m_fraction < resultCallback.m_closestHitFraction)
				{
					castResult.m_normal.normalize();
					cbtCollisionWorld::LocalConvexResult localConvexResult(
						colObjWrap->getCollisionObject(),
						0,
						castResult.m_normal,
						castResult.m_hitPoint,
						castResult.m_fraction);

					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
				}
			}
		}
	}
	else
	{
		if (collisionShape->isConcave())
		{
			if (collisionShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				//BT_PROFILE("convexSweepcbtBvhTriangleMesh");
				cbtBvhTriangleMeshShape* triangleMesh = (cbtBvhTriangleMeshShape*)collisionShape;
				cbtTransform worldTocollisionObject = colObjWorldTransform.inverse();
				cbtVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
				cbtVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
				// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
				cbtTransform rotationXform = cbtTransform(worldTocollisionObject.getBasis() * convexToTrans.getBasis());

				//ConvexCast::CastResult
				struct BridgeTriangleConvexcastCallback : public cbtTriangleConvexcastCallback
				{
					cbtCollisionWorld::ConvexResultCallback* m_resultCallback;
					const cbtCollisionObject* m_collisionObject;
					cbtTriangleMeshShape* m_triangleMesh;

					BridgeTriangleConvexcastCallback(const cbtConvexShape* castShape, const cbtTransform& from, const cbtTransform& to,
													 cbtCollisionWorld::ConvexResultCallback* resultCallback, const cbtCollisionObject* collisionObject, cbtTriangleMeshShape* triangleMesh, const cbtTransform& triangleToWorld) : cbtTriangleConvexcastCallback(castShape, from, to, triangleToWorld, triangleMesh->getMargin()),
																																																								m_resultCallback(resultCallback),
																																																								m_collisionObject(collisionObject),
																																																								m_triangleMesh(triangleMesh)
					{
					}

					virtual cbtScalar reportHit(const cbtVector3& hitNormalLocal, const cbtVector3& hitPointLocal, cbtScalar hitFraction, int partId, int triangleIndex)
					{
						cbtCollisionWorld::LocalShapeInfo shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;
						if (hitFraction <= m_resultCallback->m_closestHitFraction)
						{
							cbtCollisionWorld::LocalConvexResult convexResult(m_collisionObject,
																			 &shapeInfo,
																			 hitNormalLocal,
																			 hitPointLocal,
																			 hitFraction);

							bool normalInWorldSpace = true;

							return m_resultCallback->addSingleResult(convexResult, normalInWorldSpace);
						}
						return hitFraction;
					}
				};

				BridgeTriangleConvexcastCallback tccb(castShape, convexFromTrans, convexToTrans, &resultCallback, colObjWrap->getCollisionObject(), triangleMesh, colObjWorldTransform);
				tccb.m_hitFraction = resultCallback.m_closestHitFraction;
				tccb.m_allowedPenetration = allowedPenetration;
				cbtVector3 boxMinLocal, boxMaxLocal;
				castShape->getAabb(rotationXform, boxMinLocal, boxMaxLocal);
				triangleMesh->performConvexcast(&tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal);
			}
			else
			{
				if (collisionShape->getShapeType() == STATIC_PLANE_PROXYTYPE)
				{
					cbtConvexCast::CastResult castResult;
					castResult.m_allowedPenetration = allowedPenetration;
					castResult.m_fraction = resultCallback.m_closestHitFraction;
					cbtStaticPlaneShape* planeShape = (cbtStaticPlaneShape*)collisionShape;
					cbtContinuousConvexCollision convexCaster1(castShape, planeShape);
					cbtConvexCast* castPtr = &convexCaster1;

					if (castPtr->calcTimeOfImpact(convexFromTrans, convexToTrans, colObjWorldTransform, colObjWorldTransform, castResult))
					{
						//add hit
						if (castResult.m_normal.length2() > cbtScalar(0.0001))
						{
							if (castResult.m_fraction < resultCallback.m_closestHitFraction)
							{
								castResult.m_normal.normalize();
								cbtCollisionWorld::LocalConvexResult localConvexResult(
									colObjWrap->getCollisionObject(),
									0,
									castResult.m_normal,
									castResult.m_hitPoint,
									castResult.m_fraction);

								bool normalInWorldSpace = true;
								resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
							}
						}
					}
				}
				else
				{
					//BT_PROFILE("convexSweepConcave");
					cbtConcaveShape* concaveShape = (cbtConcaveShape*)collisionShape;
					cbtTransform worldTocollisionObject = colObjWorldTransform.inverse();
					cbtVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
					cbtVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					cbtTransform rotationXform = cbtTransform(worldTocollisionObject.getBasis() * convexToTrans.getBasis());

					//ConvexCast::CastResult
					struct BridgeTriangleConvexcastCallback : public cbtTriangleConvexcastCallback
					{
						cbtCollisionWorld::ConvexResultCallback* m_resultCallback;
						const cbtCollisionObject* m_collisionObject;
						cbtConcaveShape* m_triangleMesh;

						BridgeTriangleConvexcastCallback(const cbtConvexShape* castShape, const cbtTransform& from, const cbtTransform& to,
														 cbtCollisionWorld::ConvexResultCallback* resultCallback, const cbtCollisionObject* collisionObject, cbtConcaveShape* triangleMesh, const cbtTransform& triangleToWorld) : cbtTriangleConvexcastCallback(castShape, from, to, triangleToWorld, triangleMesh->getMargin()),
																																																							   m_resultCallback(resultCallback),
																																																							   m_collisionObject(collisionObject),
																																																							   m_triangleMesh(triangleMesh)
						{
						}

						virtual cbtScalar reportHit(const cbtVector3& hitNormalLocal, const cbtVector3& hitPointLocal, cbtScalar hitFraction, int partId, int triangleIndex)
						{
							cbtCollisionWorld::LocalShapeInfo shapeInfo;
							shapeInfo.m_shapePart = partId;
							shapeInfo.m_triangleIndex = triangleIndex;
							if (hitFraction <= m_resultCallback->m_closestHitFraction)
							{
								cbtCollisionWorld::LocalConvexResult convexResult(m_collisionObject,
																				 &shapeInfo,
																				 hitNormalLocal,
																				 hitPointLocal,
																				 hitFraction);

								bool normalInWorldSpace = true;

								return m_resultCallback->addSingleResult(convexResult, normalInWorldSpace);
							}
							return hitFraction;
						}
					};

					BridgeTriangleConvexcastCallback tccb(castShape, convexFromTrans, convexToTrans, &resultCallback, colObjWrap->getCollisionObject(), concaveShape, colObjWorldTransform);
					tccb.m_hitFraction = resultCallback.m_closestHitFraction;
					tccb.m_allowedPenetration = allowedPenetration;
					cbtVector3 boxMinLocal, boxMaxLocal;
					castShape->getAabb(rotationXform, boxMinLocal, boxMaxLocal);

					cbtVector3 rayAabbMinLocal = convexFromLocal;
					rayAabbMinLocal.setMin(convexToLocal);
					cbtVector3 rayAabbMaxLocal = convexFromLocal;
					rayAabbMaxLocal.setMax(convexToLocal);
					rayAabbMinLocal += boxMinLocal;
					rayAabbMaxLocal += boxMaxLocal;
					concaveShape->processAllTriangles(&tccb, rayAabbMinLocal, rayAabbMaxLocal);
				}
			}
		}
		else
		{
			if (collisionShape->isCompound())
			{
				struct cbtCompoundLeafCallback : cbtDbvt::ICollide
				{
					cbtCompoundLeafCallback(
						const cbtCollisionObjectWrapper* colObjWrap,
						const cbtConvexShape* castShape,
						const cbtTransform& convexFromTrans,
						const cbtTransform& convexToTrans,
						cbtScalar allowedPenetration,
						const cbtCompoundShape* compoundShape,
						const cbtTransform& colObjWorldTransform,
						ConvexResultCallback& resultCallback)
						: m_colObjWrap(colObjWrap),
						  m_castShape(castShape),
						  m_convexFromTrans(convexFromTrans),
						  m_convexToTrans(convexToTrans),
						  m_allowedPenetration(allowedPenetration),
						  m_compoundShape(compoundShape),
						  m_colObjWorldTransform(colObjWorldTransform),
						  m_resultCallback(resultCallback)
					{
					}

					const cbtCollisionObjectWrapper* m_colObjWrap;
					const cbtConvexShape* m_castShape;
					const cbtTransform& m_convexFromTrans;
					const cbtTransform& m_convexToTrans;
					cbtScalar m_allowedPenetration;
					const cbtCompoundShape* m_compoundShape;
					const cbtTransform& m_colObjWorldTransform;
					ConvexResultCallback& m_resultCallback;

				public:
					void ProcessChild(int index, const cbtTransform& childTrans, const cbtCollisionShape* childCollisionShape)
					{
						cbtTransform childWorldTrans = m_colObjWorldTransform * childTrans;

						struct LocalInfoAdder : public ConvexResultCallback
						{
							ConvexResultCallback* m_userCallback;
							int m_i;

							LocalInfoAdder(int i, ConvexResultCallback* user)
								: m_userCallback(user), m_i(i)
							{
								m_closestHitFraction = m_userCallback->m_closestHitFraction;
							}
							virtual bool needsCollision(cbtBroadphaseProxy* p) const
							{
								return m_userCallback->needsCollision(p);
							}
							virtual cbtScalar addSingleResult(cbtCollisionWorld::LocalConvexResult& r, bool b)
							{
								cbtCollisionWorld::LocalShapeInfo shapeInfo;
								shapeInfo.m_shapePart = -1;
								shapeInfo.m_triangleIndex = m_i;
								if (r.m_localShapeInfo == NULL)
									r.m_localShapeInfo = &shapeInfo;
								const cbtScalar result = m_userCallback->addSingleResult(r, b);
								m_closestHitFraction = m_userCallback->m_closestHitFraction;
								return result;
							}
						};

						LocalInfoAdder my_cb(index, &m_resultCallback);

						cbtCollisionObjectWrapper tmpObj(m_colObjWrap, childCollisionShape, m_colObjWrap->getCollisionObject(), childWorldTrans, -1, index);

						objectQuerySingleInternal(m_castShape, m_convexFromTrans, m_convexToTrans, &tmpObj, my_cb, m_allowedPenetration);
					}

					void Process(const cbtDbvtNode* leaf)
					{
						// Processing leaf node
						int index = leaf->dataAsInt;

						cbtTransform childTrans = m_compoundShape->getChildTransform(index);
						const cbtCollisionShape* childCollisionShape = m_compoundShape->getChildShape(index);

						ProcessChild(index, childTrans, childCollisionShape);
					}
				};

				BT_PROFILE("convexSweepCompound");
				const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(collisionShape);

				cbtVector3 fromLocalAabbMin, fromLocalAabbMax;
				cbtVector3 toLocalAabbMin, toLocalAabbMax;

				castShape->getAabb(colObjWorldTransform.inverse() * convexFromTrans, fromLocalAabbMin, fromLocalAabbMax);
				castShape->getAabb(colObjWorldTransform.inverse() * convexToTrans, toLocalAabbMin, toLocalAabbMax);

				fromLocalAabbMin.setMin(toLocalAabbMin);
				fromLocalAabbMax.setMax(toLocalAabbMax);

				cbtCompoundLeafCallback callback(colObjWrap, castShape, convexFromTrans, convexToTrans,
												allowedPenetration, compoundShape, colObjWorldTransform, resultCallback);

				const cbtDbvt* tree = compoundShape->getDynamicAabbTree();
				if (tree)
				{
					const ATTRIBUTE_ALIGNED16(cbtDbvtVolume) bounds = cbtDbvtVolume::FromMM(fromLocalAabbMin, fromLocalAabbMax);
					tree->collideTV(tree->m_root, bounds, callback);
				}
				else
				{
					int i;
					for (i = 0; i < compoundShape->getNumChildShapes(); i++)
					{
						const cbtCollisionShape* childCollisionShape = compoundShape->getChildShape(i);
						cbtTransform childTrans = compoundShape->getChildTransform(i);
						callback.ProcessChild(i, childTrans, childCollisionShape);
					}
				}
			}
		}
	}
}

struct cbtSingleRayCallback : public cbtBroadphaseRayCallback
{
	cbtVector3 m_rayFromWorld;
	cbtVector3 m_rayToWorld;
	cbtTransform m_rayFromTrans;
	cbtTransform m_rayToTrans;
	cbtVector3 m_hitNormal;

	const cbtCollisionWorld* m_world;
	cbtCollisionWorld::RayResultCallback& m_resultCallback;

	cbtSingleRayCallback(const cbtVector3& rayFromWorld, const cbtVector3& rayToWorld, const cbtCollisionWorld* world, cbtCollisionWorld::RayResultCallback& resultCallback)
		: m_rayFromWorld(rayFromWorld),
		  m_rayToWorld(rayToWorld),
		  m_world(world),
		  m_resultCallback(resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin(m_rayFromWorld);
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin(m_rayToWorld);

		cbtVector3 rayDir = (rayToWorld - rayFromWorld);

		rayDir.normalize();
		///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == cbtScalar(0.0) ? cbtScalar(BT_LARGE_FLOAT) : cbtScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == cbtScalar(0.0) ? cbtScalar(BT_LARGE_FLOAT) : cbtScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == cbtScalar(0.0) ? cbtScalar(BT_LARGE_FLOAT) : cbtScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(m_rayToWorld - m_rayFromWorld);
	}

	virtual bool process(const cbtBroadphaseProxy* proxy)
	{
		///terminate further ray tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == cbtScalar(0.f))
			return false;

		cbtCollisionObject* collisionObject = (cbtCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			//cbtVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if 0
#ifdef RECALCULATE_AABB
			cbtVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
			//getBroadphase()->getAabb(collisionObject->getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
			const cbtVector3& collisionObjectAabbMin = collisionObject->getBroadphaseHandle()->m_aabbMin;
			const cbtVector3& collisionObjectAabbMax = collisionObject->getBroadphaseHandle()->m_aabbMax;
#endif
#endif
			//cbtScalar hitLambda = m_resultCallback.m_closestHitFraction;
			//culling already done by broadphase
			//if (cbtRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
			{
				m_world->rayTestSingle(m_rayFromTrans, m_rayToTrans,
									   collisionObject,
									   collisionObject->getCollisionShape(),
									   collisionObject->getWorldTransform(),
									   m_resultCallback);
			}
		}
		return true;
	}
};

void cbtCollisionWorld::rayTest(const cbtVector3& rayFromWorld, const cbtVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	//BT_PROFILE("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	cbtSingleRayCallback rayCB(rayFromWorld, rayToWorld, this, resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache->rayTest(rayFromWorld, rayToWorld, rayCB);
#else
	for (int i = 0; i < this->getNumCollisionObjects(); i++)
	{
		rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
	}
#endif  //USE_BRUTEFORCE_RAYBROADPHASE
}

struct cbtSingleSweepCallback : public cbtBroadphaseRayCallback
{
	cbtTransform m_convexFromTrans;
	cbtTransform m_convexToTrans;
	cbtVector3 m_hitNormal;
	const cbtCollisionWorld* m_world;
	cbtCollisionWorld::ConvexResultCallback& m_resultCallback;
	cbtScalar m_allowedCcdPenetration;
	const cbtConvexShape* m_castShape;

	cbtSingleSweepCallback(const cbtConvexShape* castShape, const cbtTransform& convexFromTrans, const cbtTransform& convexToTrans, const cbtCollisionWorld* world, cbtCollisionWorld::ConvexResultCallback& resultCallback, cbtScalar allowedPenetration)
		: m_convexFromTrans(convexFromTrans),
		  m_convexToTrans(convexToTrans),
		  m_world(world),
		  m_resultCallback(resultCallback),
		  m_allowedCcdPenetration(allowedPenetration),
		  m_castShape(castShape)
	{
		cbtVector3 unnormalizedRayDir = (m_convexToTrans.getOrigin() - m_convexFromTrans.getOrigin());
		cbtVector3 rayDir = unnormalizedRayDir.normalized();
		///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == cbtScalar(0.0) ? cbtScalar(BT_LARGE_FLOAT) : cbtScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == cbtScalar(0.0) ? cbtScalar(BT_LARGE_FLOAT) : cbtScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == cbtScalar(0.0) ? cbtScalar(BT_LARGE_FLOAT) : cbtScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(unnormalizedRayDir);
	}

	virtual bool process(const cbtBroadphaseProxy* proxy)
	{
		///terminate further convex sweep tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == cbtScalar(0.f))
			return false;

		cbtCollisionObject* collisionObject = (cbtCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			m_world->objectQuerySingle(m_castShape, m_convexFromTrans, m_convexToTrans,
									   collisionObject,
									   collisionObject->getCollisionShape(),
									   collisionObject->getWorldTransform(),
									   m_resultCallback,
									   m_allowedCcdPenetration);
		}

		return true;
	}
};

void cbtCollisionWorld::convexSweepTest(const cbtConvexShape* castShape, const cbtTransform& convexFromWorld, const cbtTransform& convexToWorld, ConvexResultCallback& resultCallback, cbtScalar allowedCcdPenetration) const
{
	BT_PROFILE("convexSweepTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	/// unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical

	cbtTransform convexFromTrans, convexToTrans;
	convexFromTrans = convexFromWorld;
	convexToTrans = convexToWorld;
	cbtVector3 castShapeAabbMin, castShapeAabbMax;
	/* Compute AABB that encompasses angular movement */
	{
		cbtVector3 linVel, angVel;
		cbtTransformUtil::calculateVelocity(convexFromTrans, convexToTrans, 1.0f, linVel, angVel);
		cbtVector3 zeroLinVel;
		zeroLinVel.setValue(0, 0, 0);
		cbtTransform R;
		R.setIdentity();
		R.setRotation(convexFromTrans.getRotation());
		castShape->calculateTemporalAabb(R, zeroLinVel, angVel, 1.0f, castShapeAabbMin, castShapeAabbMax);
	}

#ifndef USE_BRUTEFORCE_RAYBROADPHASE

	cbtSingleSweepCallback convexCB(castShape, convexFromWorld, convexToWorld, this, resultCallback, allowedCcdPenetration);

	m_broadphasePairCache->rayTest(convexFromTrans.getOrigin(), convexToTrans.getOrigin(), convexCB, castShapeAabbMin, castShapeAabbMax);

#else
	/// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		cbtCollisionObject* collisionObject = m_collisionObjects[i];
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
				objectQuerySingle(castShape, convexFromTrans, convexToTrans,
								  collisionObject,
								  collisionObject->getCollisionShape(),
								  collisionObject->getWorldTransform(),
								  resultCallback,
								  allowedCcdPenetration);
			}
		}
	}
#endif  //USE_BRUTEFORCE_RAYBROADPHASE
}

struct cbtBridgedManifoldResult : public cbtManifoldResult
{
	cbtCollisionWorld::ContactResultCallback& m_resultCallback;

	cbtBridgedManifoldResult(const cbtCollisionObjectWrapper* obj0Wrap, const cbtCollisionObjectWrapper* obj1Wrap, cbtCollisionWorld::ContactResultCallback& resultCallback)
		: cbtManifoldResult(obj0Wrap, obj1Wrap),
		  m_resultCallback(resultCallback)
	{
	}

	virtual void addContactPoint(const cbtVector3& normalOnBInWorld, const cbtVector3& pointInWorld, cbtScalar depth)
	{
		bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
		cbtVector3 pointA = pointInWorld + normalOnBInWorld * depth;
		cbtVector3 localA;
		cbtVector3 localB;
		if (isSwapped)
		{
			localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
			localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
		}
		else
		{
			localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA);
			localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
		}

		cbtManifoldPoint newPt(localA, localB, normalOnBInWorld, depth);
		newPt.m_positionWorldOnA = pointA;
		newPt.m_positionWorldOnB = pointInWorld;

		//BP mod, store contact triangles.
		if (isSwapped)
		{
			newPt.m_partId0 = m_partId1;
			newPt.m_partId1 = m_partId0;
			newPt.m_index0 = m_index1;
			newPt.m_index1 = m_index0;
		}
		else
		{
			newPt.m_partId0 = m_partId0;
			newPt.m_partId1 = m_partId1;
			newPt.m_index0 = m_index0;
			newPt.m_index1 = m_index1;
		}

		//experimental feature info, for per-triangle material etc.
		const cbtCollisionObjectWrapper* obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
		const cbtCollisionObjectWrapper* obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
		m_resultCallback.addSingleResult(newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap, newPt.m_partId1, newPt.m_index1);
	}
};

struct cbtSingleContactCallback : public cbtBroadphaseAabbCallback
{
	cbtCollisionObject* m_collisionObject;
	cbtCollisionWorld* m_world;
	cbtCollisionWorld::ContactResultCallback& m_resultCallback;

	cbtSingleContactCallback(cbtCollisionObject* collisionObject, cbtCollisionWorld* world, cbtCollisionWorld::ContactResultCallback& resultCallback)
		: m_collisionObject(collisionObject),
		  m_world(world),
		  m_resultCallback(resultCallback)
	{
	}

	virtual bool process(const cbtBroadphaseProxy* proxy)
	{
		cbtCollisionObject* collisionObject = (cbtCollisionObject*)proxy->m_clientObject;
		if (collisionObject == m_collisionObject)
			return true;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle()))
		{
			cbtCollisionObjectWrapper ob0(0, m_collisionObject->getCollisionShape(), m_collisionObject, m_collisionObject->getWorldTransform(), -1, -1);
			cbtCollisionObjectWrapper ob1(0, collisionObject->getCollisionShape(), collisionObject, collisionObject->getWorldTransform(), -1, -1);

			cbtCollisionAlgorithm* algorithm = m_world->getDispatcher()->findAlgorithm(&ob0, &ob1, 0, BT_CLOSEST_POINT_ALGORITHMS);
			if (algorithm)
			{
				cbtBridgedManifoldResult contactPointResult(&ob0, &ob1, m_resultCallback);
				//discrete collision detection query

				algorithm->processCollision(&ob0, &ob1, m_world->getDispatchInfo(), &contactPointResult);

				algorithm->~cbtCollisionAlgorithm();
				m_world->getDispatcher()->freeCollisionAlgorithm(algorithm);
			}
		}
		return true;
	}
};

///contactTest performs a discrete collision test against all objects in the cbtCollisionWorld, and calls the resultCallback.
///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
void cbtCollisionWorld::contactTest(cbtCollisionObject* colObj, ContactResultCallback& resultCallback)
{
	cbtVector3 aabbMin, aabbMax;
	colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), aabbMin, aabbMax);
	cbtSingleContactCallback contactCB(colObj, this, resultCallback);

	m_broadphasePairCache->aabbTest(aabbMin, aabbMax, contactCB);
}

///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
///it reports one or more contact points (including the one with deepest penetration)
void cbtCollisionWorld::contactPairTest(cbtCollisionObject* colObjA, cbtCollisionObject* colObjB, ContactResultCallback& resultCallback)
{
	cbtCollisionObjectWrapper obA(0, colObjA->getCollisionShape(), colObjA, colObjA->getWorldTransform(), -1, -1);
	cbtCollisionObjectWrapper obB(0, colObjB->getCollisionShape(), colObjB, colObjB->getWorldTransform(), -1, -1);

	cbtCollisionAlgorithm* algorithm = getDispatcher()->findAlgorithm(&obA, &obB, 0, BT_CLOSEST_POINT_ALGORITHMS);
	if (algorithm)
	{
		cbtBridgedManifoldResult contactPointResult(&obA, &obB, resultCallback);
		contactPointResult.m_closestPointDistanceThreshold = resultCallback.m_closestDistanceThreshold;
		//discrete collision detection query
		algorithm->processCollision(&obA, &obB, getDispatchInfo(), &contactPointResult);

		algorithm->~cbtCollisionAlgorithm();
		getDispatcher()->freeCollisionAlgorithm(algorithm);
	}
}

class DebugDrawcallback : public cbtTriangleCallback, public cbtInternalTriangleIndexCallback
{
	cbtIDebugDraw* m_debugDrawer;
	cbtVector3 m_color;
	cbtTransform m_worldTrans;

public:
	DebugDrawcallback(cbtIDebugDraw* debugDrawer, const cbtTransform& worldTrans, const cbtVector3& color) : m_debugDrawer(debugDrawer),
																										  m_color(color),
																										  m_worldTrans(worldTrans)
	{
	}

	virtual void internalProcessTriangleIndex(cbtVector3* triangle, int partId, int triangleIndex)
	{
		processTriangle(triangle, partId, triangleIndex);
	}

	virtual void processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
	{
		(void)partId;
		(void)triangleIndex;

		cbtVector3 wv0, wv1, wv2;
		wv0 = m_worldTrans * triangle[0];
		wv1 = m_worldTrans * triangle[1];
		wv2 = m_worldTrans * triangle[2];
		cbtVector3 center = (wv0 + wv1 + wv2) * cbtScalar(1. / 3.);

		if (m_debugDrawer->getDebugMode() & cbtIDebugDraw::DBG_DrawNormals)
		{
			cbtVector3 normal = (wv1 - wv0).cross(wv2 - wv0);
			normal.normalize();
			cbtVector3 normalColor(1, 1, 0);
			m_debugDrawer->drawLine(center, center + normal, normalColor);
		}
		m_debugDrawer->drawLine(wv0, wv1, m_color);
		m_debugDrawer->drawLine(wv1, wv2, m_color);
		m_debugDrawer->drawLine(wv2, wv0, m_color);
	}
};

void cbtCollisionWorld::debugDrawObject(const cbtTransform& worldTransform, const cbtCollisionShape* shape, const cbtVector3& color)
{
	// Draw a small simplex at the center of the object
	if (getDebugDrawer() && getDebugDrawer()->getDebugMode() & cbtIDebugDraw::DBG_DrawFrames)
	{
        /* ***CHRONO*** Comment out */
		////getDebugDrawer()->drawTransform(worldTransform, .1);
	}

	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const cbtCompoundShape* compoundShape = static_cast<const cbtCompoundShape*>(shape);
		for (int i = compoundShape->getNumChildShapes() - 1; i >= 0; i--)
		{
			cbtTransform childTrans = compoundShape->getChildTransform(i);
			const cbtCollisionShape* colShape = compoundShape->getChildShape(i);
			debugDrawObject(worldTransform * childTrans, colShape, color);
		}
	}
	else
	{
		switch (shape->getShapeType())
		{
			case CE_TRIANGLE_SHAPE_PROXYTYPE:	// ***CHRONO***
			{
				const cbtCEtriangleShape* triShape = static_cast<const cbtCEtriangleShape*>(shape);
				chrono::ChVector<>* v1 = triShape->get_p1();
				chrono::ChVector<>* v2 = triShape->get_p2();
				chrono::ChVector<>* v3 = triShape->get_p3();
				auto origin = worldTransform.getOrigin();
				auto vt1 = origin + worldTransform.getBasis() *  cbtVector3((cbtScalar)v1->x(), (cbtScalar)v1->y(), (cbtScalar)v1->z());
				auto vt2 = origin + worldTransform.getBasis() *  cbtVector3((cbtScalar)v2->x(), (cbtScalar)v2->y(), (cbtScalar)v2->z());
				auto vt3 = origin + worldTransform.getBasis() *  cbtVector3((cbtScalar)v3->x(), (cbtScalar)v3->y(), (cbtScalar)v3->z());
				getDebugDrawer()->drawTriangle(vt1, vt2, vt3,color,1);
				break;
			}
			case BOX_SHAPE_PROXYTYPE:
			{
				const cbtBoxShape* boxShape = static_cast<const cbtBoxShape*>(shape);
				cbtVector3 halfExtents = boxShape->getHalfExtentsWithMargin();
				getDebugDrawer()->drawBox(-halfExtents, halfExtents, worldTransform, color);
				break;
			}

			case SPHERE_SHAPE_PROXYTYPE:
			{
				const cbtSphereShape* sphereShape = static_cast<const cbtSphereShape*>(shape);
				cbtScalar radius = sphereShape->getMargin();  //radius doesn't include the margin, so draw with margin

				getDebugDrawer()->drawSphere(radius, worldTransform, color);
				break;
			}
			case MULTI_SPHERE_SHAPE_PROXYTYPE:
			{
				const cbtMultiSphereShape* multiSphereShape = static_cast<const cbtMultiSphereShape*>(shape);

				cbtTransform childTransform;
				childTransform.setIdentity();

				for (int i = multiSphereShape->getSphereCount() - 1; i >= 0; i--)
				{
					childTransform.setOrigin(multiSphereShape->getSpherePosition(i));
					getDebugDrawer()->drawSphere(multiSphereShape->getSphereRadius(i), worldTransform * childTransform, color);
				}

				break;
			}
			case CAPSULE_SHAPE_PROXYTYPE:
			{
				const cbtCapsuleShape* capsuleShape = static_cast<const cbtCapsuleShape*>(shape);

				cbtScalar radius = capsuleShape->getRadius();
				cbtScalar halfHeight = capsuleShape->getHalfHeight();

				int upAxis = capsuleShape->getUpAxis();
				getDebugDrawer()->drawCapsule(radius, halfHeight, upAxis, worldTransform, color);
				break;
			}
			case CONE_SHAPE_PROXYTYPE:
			{
				const cbtConeShape* coneShape = static_cast<const cbtConeShape*>(shape);
				cbtScalar radius = coneShape->getRadius();  //+coneShape->getMargin();
				cbtScalar height = coneShape->getHeight();  //+coneShape->getMargin();

				int upAxis = coneShape->getConeUpIndex();
				getDebugDrawer()->drawCone(radius, height, upAxis, worldTransform, color);
				break;
			}
			case CYLINDER_SHAPE_PROXYTYPE:
			{
				const cbtCylinderShape* cylinder = static_cast<const cbtCylinderShape*>(shape);
				int upAxis = cylinder->getUpAxis();
				cbtScalar radius = cylinder->getRadius();
				cbtScalar halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];
				getDebugDrawer()->drawCylinder(radius, halfHeight, upAxis, worldTransform, color);
				break;
			}
			case CYLSHELL_SHAPE_PROXYTYPE: { /* ***CHRONO*** */
				const cbtCylindricalShellShape* cylinder = static_cast<const cbtCylindricalShellShape*>(shape);
				cbtScalar radius = cylinder->getRadius();
				cbtScalar halfHeight = cylinder->getHalfExtentsWithMargin()[1];
				getDebugDrawer()->drawCylinder(radius, halfHeight, 1, worldTransform, color);
				break;
			}
			case STATIC_PLANE_PROXYTYPE:
			{
				const cbtStaticPlaneShape* staticPlaneShape = static_cast<const cbtStaticPlaneShape*>(shape);
				cbtScalar planeConst = staticPlaneShape->getPlaneConstant();
				const cbtVector3& planeNormal = staticPlaneShape->getPlaneNormal();
				getDebugDrawer()->drawPlane(planeNormal, planeConst, worldTransform, color);
				break;
			}
			default:
			{
				/// for polyhedral shapes
				if (shape->isPolyhedral())
				{
					cbtPolyhedralConvexShape* polyshape = (cbtPolyhedralConvexShape*)shape;

					int i;
					if (polyshape->getConvexPolyhedron())
					{
						const cbtConvexPolyhedron* poly = polyshape->getConvexPolyhedron();
						for (i = 0; i < poly->m_faces.size(); i++)
						{
							cbtVector3 centroid(0, 0, 0);
							int numVerts = poly->m_faces[i].m_indices.size();
							if (numVerts)
							{
								int lastV = poly->m_faces[i].m_indices[numVerts - 1];
								for (int v = 0; v < poly->m_faces[i].m_indices.size(); v++)
								{
									int curVert = poly->m_faces[i].m_indices[v];
									centroid += poly->m_vertices[curVert];
									getDebugDrawer()->drawLine(worldTransform * poly->m_vertices[lastV], worldTransform * poly->m_vertices[curVert], color);
									lastV = curVert;
								}
							}
							centroid *= cbtScalar(1.f) / cbtScalar(numVerts);
							if (getDebugDrawer()->getDebugMode() & cbtIDebugDraw::DBG_DrawNormals)
							{
								cbtVector3 normalColor(1, 1, 0);
								cbtVector3 faceNormal(poly->m_faces[i].m_plane[0], poly->m_faces[i].m_plane[1], poly->m_faces[i].m_plane[2]);
								getDebugDrawer()->drawLine(worldTransform * centroid, worldTransform * (centroid + faceNormal), normalColor);
							}
						}
					}
					else
					{
						for (i = 0; i < polyshape->getNumEdges(); i++)
						{
							cbtVector3 a, b;
							polyshape->getEdge(i, a, b);
							cbtVector3 wa = worldTransform * a;
							cbtVector3 wb = worldTransform * b;
							getDebugDrawer()->drawLine(wa, wb, color);
						}
					}
				}

				if (shape->isConcave())
				{
					cbtConcaveShape* concaveMesh = (cbtConcaveShape*)shape;

					///@todo pass camera, for some culling? no -> we are not a graphics lib
					cbtVector3 aabbMax(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
					cbtVector3 aabbMin(cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT));

					DebugDrawcallback drawCallback(getDebugDrawer(), worldTransform, color);
					concaveMesh->processAllTriangles(&drawCallback, aabbMin, aabbMax);
				}

				if (shape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
				{
					cbtConvexTriangleMeshShape* convexMesh = (cbtConvexTriangleMeshShape*)shape;
					//todo: pass camera for some culling
					cbtVector3 aabbMax(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
					cbtVector3 aabbMin(cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT));
					//DebugDrawcallback drawCallback;
					DebugDrawcallback drawCallback(getDebugDrawer(), worldTransform, color);
					convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback, aabbMin, aabbMax);
				}
			}
		}
	}
}

void cbtCollisionWorld::debugDrawWorld()
{
	if (getDebugDrawer())
	{
		getDebugDrawer()->clearLines();

		cbtIDebugDraw::DefaultColors defaultColors = getDebugDrawer()->getDefaultColors();

		if (getDebugDrawer()->getDebugMode() & cbtIDebugDraw::DBG_DrawContactPoints)
		{
			if (getDispatcher())
			{
				int numManifolds = getDispatcher()->getNumManifolds();

				for (int i = 0; i < numManifolds; i++)
				{
					cbtPersistentManifold* contactManifold = getDispatcher()->getManifoldByIndexInternal(i);
					//cbtCollisionObject* obA = static_cast<cbtCollisionObject*>(contactManifold->getBody0());
					//cbtCollisionObject* obB = static_cast<cbtCollisionObject*>(contactManifold->getBody1());

					int numContacts = contactManifold->getNumContacts();
					for (int j = 0; j < numContacts; j++)
					{
						cbtManifoldPoint& cp = contactManifold->getContactPoint(j);
						getDebugDrawer()->drawContactPoint(cp.m_positionWorldOnB, cp.m_normalWorldOnB, cp.getDistance(), cp.getLifeTime(), defaultColors.m_contactPoint);
					}
				}
			}
		}

		if ((getDebugDrawer()->getDebugMode() & (cbtIDebugDraw::DBG_DrawWireframe | cbtIDebugDraw::DBG_DrawAabb)))
		{
			int i;

			for (i = 0; i < m_collisionObjects.size(); i++)
			{
				cbtCollisionObject* colObj = m_collisionObjects[i];
				if ((colObj->getCollisionFlags() & cbtCollisionObject::CF_DISABLE_VISUALIZE_OBJECT) == 0)
				{
					if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & cbtIDebugDraw::DBG_DrawWireframe))
					{
						cbtVector3 color(cbtScalar(0.4), cbtScalar(0.4), cbtScalar(0.4));

						switch (colObj->getActivationState())
						{
							case ACTIVE_TAG:
								color = defaultColors.m_activeObject;
								break;
							case ISLAND_SLEEPING:
								color = defaultColors.m_deactivatedObject;
								break;
							case WANTS_DEACTIVATION:
								color = defaultColors.m_wantsDeactivationObject;
								break;
							case DISABLE_DEACTIVATION:
								color = defaultColors.m_disabledDeactivationObject;
								break;
							case DISABLE_SIMULATION:
								color = defaultColors.m_disabledSimulationObject;
								break;
							default:
							{
								color = cbtVector3(cbtScalar(.3), cbtScalar(0.3), cbtScalar(0.3));
							}
						};

						colObj->getCustomDebugColor(color);

						debugDrawObject(colObj->getWorldTransform(), colObj->getCollisionShape(), color);
					}
					if (m_debugDrawer && (m_debugDrawer->getDebugMode() & cbtIDebugDraw::DBG_DrawAabb))
					{
						cbtVector3 minAabb, maxAabb;
						cbtVector3 colorvec = defaultColors.m_aabb;
						colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb, maxAabb);
						cbtVector3 contactThreshold(gContactBreakingThreshold, gContactBreakingThreshold, gContactBreakingThreshold);
						minAabb -= contactThreshold;
						maxAabb += contactThreshold;

						cbtVector3 minAabb2, maxAabb2;

						if (getDispatchInfo().m_useContinuous && colObj->getInternalType() == cbtCollisionObject::CO_RIGID_BODY && !colObj->isStaticOrKinematicObject())
						{
							colObj->getCollisionShape()->getAabb(colObj->getInterpolationWorldTransform(), minAabb2, maxAabb2);
							minAabb2 -= contactThreshold;
							maxAabb2 += contactThreshold;
							minAabb.setMin(minAabb2);
							maxAabb.setMax(maxAabb2);
						}

						m_debugDrawer->drawAabb(minAabb, maxAabb, colorvec);
					}
				}
			}
		}
	}
}

void cbtCollisionWorld::serializeCollisionObjects(cbtSerializer* serializer)
{
	int i;

	///keep track of shapes already serialized
	cbtHashMap<cbtHashPtr, cbtCollisionShape*> serializedShapes;

	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		cbtCollisionObject* colObj = m_collisionObjects[i];
		cbtCollisionShape* shape = colObj->getCollisionShape();

		if (!serializedShapes.find(shape))
		{
			serializedShapes.insert(shape, shape);
			shape->serializeSingleShape(serializer);
		}
	}

	//serialize all collision objects
	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		cbtCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() == cbtCollisionObject::CO_COLLISION_OBJECT)
		{
			colObj->serializeSingleObject(serializer);
		}
	}
}

void cbtCollisionWorld::serializeContactManifolds(cbtSerializer* serializer)
{
	if (serializer->getSerializationFlags() & BT_SERIALIZE_CONTACT_MANIFOLDS)
	{
		int numManifolds = getDispatcher()->getNumManifolds();
		for (int i = 0; i < numManifolds; i++)
		{
			const cbtPersistentManifold* manifold = getDispatcher()->getInternalManifoldPointer()[i];
			//don't serialize empty manifolds, they just take space
			//(may have to do it anyway if it destroys determinism)
			if (manifold->getNumContacts() == 0)
				continue;

			cbtChunk* chunk = serializer->allocate(manifold->calculateSerializeBufferSize(), 1);
			const char* structType = manifold->serialize(manifold, chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_CONTACTMANIFOLD_CODE, (void*)manifold);
		}
	}
}

void cbtCollisionWorld::serialize(cbtSerializer* serializer)
{
	serializer->startSerialization();

	serializeCollisionObjects(serializer);

	serializeContactManifolds(serializer);

	serializer->finishSerialization();
}
