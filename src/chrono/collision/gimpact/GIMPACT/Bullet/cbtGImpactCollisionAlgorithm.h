/*! \file cbtGImpactShape.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_GIMPACT_BVH_CONCAVE_COLLISION_ALGORITHM_H
#define BT_GIMPACT_BVH_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseInterface.h"
#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
class cbtDispatcher;
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"

#include "LinearMath/cbtAlignedObjectArray.h"

#include "cbtGImpactShape.h"
#include "BulletCollision/CollisionShapes/cbtStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "BulletCollision/CollisionDispatch/cbtConvexConvexAlgorithm.h"
#include "LinearMath/cbtIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

//! Collision Algorithm for GImpact Shapes
/*!
For register this algorithm in Bullet, proceed as following:
 \code
cbtCollisionDispatcher * dispatcher = static_cast<cbtCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
cbtGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
 \endcode
*/
class cbtGImpactCollisionAlgorithm : public cbtActivatingCollisionAlgorithm
{
protected:
	cbtCollisionAlgorithm* m_convex_algorithm;
	cbtPersistentManifold* m_manifoldPtr;
	cbtManifoldResult* m_resultOut;
	const cbtDispatcherInfo* m_dispatchInfo;
	int m_triface0;
	int m_part0;
	int m_triface1;
	int m_part1;

	//! Creates a new contact point
	SIMD_FORCE_INLINE cbtPersistentManifold* newContactManifold(const cbtCollisionObject* body0, const cbtCollisionObject* body1)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0, body1);
		return m_manifoldPtr;
	}

	SIMD_FORCE_INLINE void destroyConvexAlgorithm()
	{
		if (m_convex_algorithm)
		{
			m_convex_algorithm->~cbtCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(m_convex_algorithm);
			m_convex_algorithm = NULL;
		}
	}

	SIMD_FORCE_INLINE void destroyContactManifolds()
	{
		if (m_manifoldPtr == NULL) return;
		m_dispatcher->releaseManifold(m_manifoldPtr);
		m_manifoldPtr = NULL;
	}

	SIMD_FORCE_INLINE void clearCache()
	{
		destroyContactManifolds();
		destroyConvexAlgorithm();

		m_triface0 = -1;
		m_part0 = -1;
		m_triface1 = -1;
		m_part1 = -1;
	}

	SIMD_FORCE_INLINE cbtPersistentManifold* getLastManifold()
	{
		return m_manifoldPtr;
	}

	// Call before process collision
	SIMD_FORCE_INLINE void checkManifold(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
	{
		if (getLastManifold() == 0)
		{
			newContactManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		}

		m_resultOut->setPersistentManifold(getLastManifold());
	}

	// Call before process collision
	SIMD_FORCE_INLINE cbtCollisionAlgorithm* newAlgorithm(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
	{
		checkManifold(body0Wrap, body1Wrap);

		cbtCollisionAlgorithm* convex_algorithm = m_dispatcher->findAlgorithm(
			body0Wrap, body1Wrap, getLastManifold(), BT_CONTACT_POINT_ALGORITHMS);
		return convex_algorithm;
	}

	// Call before process collision
	SIMD_FORCE_INLINE void checkConvexAlgorithm(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
	{
		if (m_convex_algorithm) return;
		m_convex_algorithm = newAlgorithm(body0Wrap, body1Wrap);
	}

	void addContactPoint(const cbtCollisionObjectWrapper* body0Wrap,
						 const cbtCollisionObjectWrapper* body1Wrap,
						 const cbtVector3& point,
						 const cbtVector3& normal,
						 cbtScalar distance);

	//! Collision routines
	//!@{

	void collide_gjk_triangles(const cbtCollisionObjectWrapper* body0Wrap,
							   const cbtCollisionObjectWrapper* body1Wrap,
							   const cbtGImpactMeshShapePart* shape0,
							   const cbtGImpactMeshShapePart* shape1,
							   const int* pairs, int pair_count);

	void collide_sat_triangles(const cbtCollisionObjectWrapper* body0Wrap,
							   const cbtCollisionObjectWrapper* body1Wrap,
							   const cbtGImpactMeshShapePart* shape0,
							   const cbtGImpactMeshShapePart* shape1,
							   const int* pairs, int pair_count);

	void shape_vs_shape_collision(
		const cbtCollisionObjectWrapper* body0,
		const cbtCollisionObjectWrapper* body1,
		const cbtCollisionShape* shape0,
		const cbtCollisionShape* shape1);

	void convex_vs_convex_collision(const cbtCollisionObjectWrapper* body0Wrap,
									const cbtCollisionObjectWrapper* body1Wrap,
									const cbtCollisionShape* shape0,
									const cbtCollisionShape* shape1);

	void gimpact_vs_gimpact_find_pairs(
		const cbtTransform& trans0,
		const cbtTransform& trans1,
		const cbtGImpactShapeInterface* shape0,
		const cbtGImpactShapeInterface* shape1, cbtPairSet& pairset);

	void gimpact_vs_shape_find_pairs(
		const cbtTransform& trans0,
		const cbtTransform& trans1,
		const cbtGImpactShapeInterface* shape0,
		const cbtCollisionShape* shape1,
		cbtAlignedObjectArray<int>& collided_primitives);

	void gimpacttrimeshpart_vs_plane_collision(
		const cbtCollisionObjectWrapper* body0Wrap,
		const cbtCollisionObjectWrapper* body1Wrap,
		const cbtGImpactMeshShapePart* shape0,
		const cbtStaticPlaneShape* shape1, bool swapped);

public:
	cbtGImpactCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap);

	virtual ~cbtGImpactCollisionAlgorithm();

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	cbtScalar calculateTimeOfImpact(cbtCollisionObject* body0, cbtCollisionObject* body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray& manifoldArray)
	{
		if (m_manifoldPtr)
			manifoldArray.push_back(m_manifoldPtr);
	}

	cbtManifoldResult* internalGetResultOut()
	{
		return m_resultOut;
	}

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtGImpactCollisionAlgorithm));
			return new (mem) cbtGImpactCollisionAlgorithm(ci, body0Wrap, body1Wrap);
		}
	};

	//! Use this function for register the algorithm externally
	static void registerAlgorithm(cbtCollisionDispatcher* dispatcher);
#ifdef TRI_COLLISION_PROFILING
	//! Gets the average time in miliseconds of tree collisions
	static float getAverageTreeCollisionTime();

	//! Gets the average time in miliseconds of triangle collisions
	static float getAverageTriangleCollisionTime();
#endif  //TRI_COLLISION_PROFILING

	//! Collides two gimpact shapes
	/*!
	\pre shape0 and shape1 couldn't be cbtGImpactMeshShape objects
	*/

	void gimpact_vs_gimpact(const cbtCollisionObjectWrapper* body0Wrap,
							const cbtCollisionObjectWrapper* body1Wrap,
							const cbtGImpactShapeInterface* shape0,
							const cbtGImpactShapeInterface* shape1);

	void gimpact_vs_shape(const cbtCollisionObjectWrapper* body0Wrap,
						  const cbtCollisionObjectWrapper* body1Wrap,
						  const cbtGImpactShapeInterface* shape0,
						  const cbtCollisionShape* shape1, bool swapped);

	void gimpact_vs_compoundshape(const cbtCollisionObjectWrapper* body0Wrap,
								  const cbtCollisionObjectWrapper* body1Wrap,
								  const cbtGImpactShapeInterface* shape0,
								  const cbtCompoundShape* shape1, bool swapped);

	void gimpact_vs_concave(
		const cbtCollisionObjectWrapper* body0Wrap,
		const cbtCollisionObjectWrapper* body1Wrap,
		const cbtGImpactShapeInterface* shape0,
		const cbtConcaveShape* shape1, bool swapped);

	/// Accessor/Mutator pairs for Part and triangleID
	void setFace0(int value)
	{
		m_triface0 = value;
	}
	int getFace0()
	{
		return m_triface0;
	}
	void setFace1(int value)
	{
		m_triface1 = value;
	}
	int getFace1()
	{
		return m_triface1;
	}
	void setPart0(int value)
	{
		m_part0 = value;
	}
	int getPart0()
	{
		return m_part0;
	}
	void setPart1(int value)
	{
		m_part1 = value;
	}
	int getPart1()
	{
		return m_part1;
	}
};

//algorithm details
//#define BULLET_TRIANGLE_COLLISION 1
#define GIMPACT_VS_PLANE_COLLISION 1

#endif  //BT_GIMPACT_BVH_CONCAVE_COLLISION_ALGORITHM_H
