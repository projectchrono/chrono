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

#ifndef BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H
#define BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H

#include "cbtActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/cbtDispatcher.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseInterface.h"
#include "BulletCollision/CollisionShapes/cbtTriangleCallback.h"
#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
class cbtDispatcher;
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "cbtCollisionCreateFunc.h"

///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), processTriangle is called.
ATTRIBUTE_ALIGNED16(class)
cbtConvexTriangleCallback : public cbtTriangleCallback
{
	cbtVector3 m_aabbMin;
	cbtVector3 m_aabbMax;

	const cbtCollisionObjectWrapper* m_convexBodyWrap;
	const cbtCollisionObjectWrapper* m_triBodyWrap;

	cbtManifoldResult* m_resultOut;
	cbtDispatcher* m_dispatcher;
	const cbtDispatcherInfo* m_dispatchInfoPtr;
	cbtScalar m_collisionMarginTriangle;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_triangleCount;

	cbtPersistentManifold* m_manifoldPtr;

	cbtConvexTriangleCallback(cbtDispatcher * dispatcher, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped);

	void setTimeStepAndCounters(cbtScalar collisionMarginTriangle, const cbtDispatcherInfo& dispatchInfo, const cbtCollisionObjectWrapper* convexBodyWrap, const cbtCollisionObjectWrapper* triBodyWrap, cbtManifoldResult* resultOut);

	void clearWrapperData()
	{
		m_convexBodyWrap = 0;
		m_triBodyWrap = 0;
	}
	virtual ~cbtConvexTriangleCallback();

	virtual void processTriangle(cbtVector3 * triangle, int partId, int triangleIndex);

	void clearCache();

	SIMD_FORCE_INLINE const cbtVector3& getAabbMin() const
	{
		return m_aabbMin;
	}
	SIMD_FORCE_INLINE const cbtVector3& getAabbMax() const
	{
		return m_aabbMax;
	}
};

/// cbtConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.
ATTRIBUTE_ALIGNED16(class)
cbtConvexConcaveCollisionAlgorithm : public cbtActivatingCollisionAlgorithm
{
	cbtConvexTriangleCallback m_cbtConvexTriangleCallback;

	bool m_isSwapped;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConvexConcaveCollisionAlgorithm(const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, bool isSwapped);

	virtual ~cbtConvexConcaveCollisionAlgorithm();

	virtual void processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	cbtScalar calculateTimeOfImpact(cbtCollisionObject * body0, cbtCollisionObject * body1, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut);

	virtual void getAllContactManifolds(cbtManifoldArray & manifoldArray);

	void clearCache();

	struct CreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtConvexConcaveCollisionAlgorithm));
			return new (mem) cbtConvexConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, false);
		}
	};

	struct SwappedCreateFunc : public cbtCollisionAlgorithmCreateFunc
	{
		virtual cbtCollisionAlgorithm* CreateCollisionAlgorithm(cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(cbtConvexConcaveCollisionAlgorithm));
			return new (mem) cbtConvexConcaveCollisionAlgorithm(ci, body0Wrap, body1Wrap, true);
		}
	};
};

#endif  //BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H
