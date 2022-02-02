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

#include "cbtDefaultCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/cbtConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCompoundCompoundCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/cbtConvexPlaneCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtBoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtSphereSphereCollisionAlgorithm.h"
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/cbtSphereBoxCollisionAlgorithm.h"
#endif  //USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/cbtSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.h"

#include "LinearMath/cbtPoolAllocator.h"

cbtDefaultCollisionConfiguration::cbtDefaultCollisionConfiguration(const cbtDefaultCollisionConstructionInfo& constructionInfo)
//cbtDefaultCollisionConfiguration::cbtDefaultCollisionConfiguration(cbtStackAlloc*	stackAlloc,cbtPoolAllocator*	persistentManifoldPool,cbtPoolAllocator*	collisionAlgorithmPool)
{
	void* mem = NULL;
	if (constructionInfo.m_useEpaPenetrationAlgorithm)
	{
		mem = cbtAlignedAlloc(sizeof(cbtGjkEpaPenetrationDepthSolver), 16);
		m_pdSolver = new (mem) cbtGjkEpaPenetrationDepthSolver;
	}
	else
	{
		mem = cbtAlignedAlloc(sizeof(cbtMinkowskiPenetrationDepthSolver), 16);
		m_pdSolver = new (mem) cbtMinkowskiPenetrationDepthSolver;
	}

	//default CreationFunctions, filling the m_doubleDispatch table
	mem = cbtAlignedAlloc(sizeof(cbtConvexConvexAlgorithm::CreateFunc), 16);
	m_convexConvexCreateFunc = new (mem) cbtConvexConvexAlgorithm::CreateFunc(m_pdSolver);
	mem = cbtAlignedAlloc(sizeof(cbtConvexConcaveCollisionAlgorithm::CreateFunc), 16);
	m_convexConcaveCreateFunc = new (mem) cbtConvexConcaveCollisionAlgorithm::CreateFunc;
	mem = cbtAlignedAlloc(sizeof(cbtConvexConcaveCollisionAlgorithm::CreateFunc), 16);
	m_swappedConvexConcaveCreateFunc = new (mem) cbtConvexConcaveCollisionAlgorithm::SwappedCreateFunc;
	mem = cbtAlignedAlloc(sizeof(cbtCompoundCollisionAlgorithm::CreateFunc), 16);
	m_compoundCreateFunc = new (mem) cbtCompoundCollisionAlgorithm::CreateFunc;

	mem = cbtAlignedAlloc(sizeof(cbtCompoundCompoundCollisionAlgorithm::CreateFunc), 16);
	m_compoundCompoundCreateFunc = new (mem) cbtCompoundCompoundCollisionAlgorithm::CreateFunc;

	mem = cbtAlignedAlloc(sizeof(cbtCompoundCollisionAlgorithm::SwappedCreateFunc), 16);
	m_swappedCompoundCreateFunc = new (mem) cbtCompoundCollisionAlgorithm::SwappedCreateFunc;
	mem = cbtAlignedAlloc(sizeof(cbtEmptyAlgorithm::CreateFunc), 16);
	m_emptyCreateFunc = new (mem) cbtEmptyAlgorithm::CreateFunc;

	mem = cbtAlignedAlloc(sizeof(cbtSphereSphereCollisionAlgorithm::CreateFunc), 16);
	m_sphereSphereCF = new (mem) cbtSphereSphereCollisionAlgorithm::CreateFunc;
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	mem = cbtAlignedAlloc(sizeof(cbtSphereBoxCollisionAlgorithm::CreateFunc), 16);
	m_sphereBoxCF = new (mem) cbtSphereBoxCollisionAlgorithm::CreateFunc;
	mem = cbtAlignedAlloc(sizeof(cbtSphereBoxCollisionAlgorithm::CreateFunc), 16);
	m_boxSphereCF = new (mem) cbtSphereBoxCollisionAlgorithm::CreateFunc;
	m_boxSphereCF->m_swapped = true;
#endif  //USE_BUGGY_SPHERE_BOX_ALGORITHM

	mem = cbtAlignedAlloc(sizeof(cbtSphereTriangleCollisionAlgorithm::CreateFunc), 16);
	m_sphereTriangleCF = new (mem) cbtSphereTriangleCollisionAlgorithm::CreateFunc;
	mem = cbtAlignedAlloc(sizeof(cbtSphereTriangleCollisionAlgorithm::CreateFunc), 16);
	m_triangleSphereCF = new (mem) cbtSphereTriangleCollisionAlgorithm::CreateFunc;
	m_triangleSphereCF->m_swapped = true;

	mem = cbtAlignedAlloc(sizeof(cbtBoxBoxCollisionAlgorithm::CreateFunc), 16);
	m_boxBoxCF = new (mem) cbtBoxBoxCollisionAlgorithm::CreateFunc;

	//convex versus plane
	mem = cbtAlignedAlloc(sizeof(cbtConvexPlaneCollisionAlgorithm::CreateFunc), 16);
	m_convexPlaneCF = new (mem) cbtConvexPlaneCollisionAlgorithm::CreateFunc;
	mem = cbtAlignedAlloc(sizeof(cbtConvexPlaneCollisionAlgorithm::CreateFunc), 16);
	m_planeConvexCF = new (mem) cbtConvexPlaneCollisionAlgorithm::CreateFunc;
	m_planeConvexCF->m_swapped = true;

	///calculate maximum element size, big enough to fit any collision algorithm in the memory pool
	int maxSize = sizeof(cbtConvexConvexAlgorithm);
	int maxSize2 = sizeof(cbtConvexConcaveCollisionAlgorithm);
	int maxSize3 = sizeof(cbtCompoundCollisionAlgorithm);
	int maxSize4 = sizeof(cbtCompoundCompoundCollisionAlgorithm);

	int collisionAlgorithmMaxElementSize = cbtMax(maxSize, constructionInfo.m_customCollisionAlgorithmMaxElementSize);
	collisionAlgorithmMaxElementSize = cbtMax(collisionAlgorithmMaxElementSize, maxSize2);
	collisionAlgorithmMaxElementSize = cbtMax(collisionAlgorithmMaxElementSize, maxSize3);
	collisionAlgorithmMaxElementSize = cbtMax(collisionAlgorithmMaxElementSize, maxSize4);

	if (constructionInfo.m_persistentManifoldPool)
	{
		m_ownsPersistentManifoldPool = false;
		m_persistentManifoldPool = constructionInfo.m_persistentManifoldPool;
	}
	else
	{
		m_ownsPersistentManifoldPool = true;
		void* mem = cbtAlignedAlloc(sizeof(cbtPoolAllocator), 16);
		m_persistentManifoldPool = new (mem) cbtPoolAllocator(sizeof(cbtPersistentManifold), constructionInfo.m_defaultMaxPersistentManifoldPoolSize);
	}

	collisionAlgorithmMaxElementSize = (collisionAlgorithmMaxElementSize + 16) & 0xffffffffffff0;
	if (constructionInfo.m_collisionAlgorithmPool)
	{
		m_ownsCollisionAlgorithmPool = false;
		m_collisionAlgorithmPool = constructionInfo.m_collisionAlgorithmPool;
	}
	else
	{
		m_ownsCollisionAlgorithmPool = true;
		void* mem = cbtAlignedAlloc(sizeof(cbtPoolAllocator), 16);
		m_collisionAlgorithmPool = new (mem) cbtPoolAllocator(collisionAlgorithmMaxElementSize, constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
	}
}

cbtDefaultCollisionConfiguration::~cbtDefaultCollisionConfiguration()
{
	if (m_ownsCollisionAlgorithmPool)
	{
		m_collisionAlgorithmPool->~cbtPoolAllocator();
		cbtAlignedFree(m_collisionAlgorithmPool);
	}
	if (m_ownsPersistentManifoldPool)
	{
		m_persistentManifoldPool->~cbtPoolAllocator();
		cbtAlignedFree(m_persistentManifoldPool);
	}

	m_convexConvexCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_convexConvexCreateFunc);

	m_convexConcaveCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_convexConcaveCreateFunc);
	m_swappedConvexConcaveCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_swappedConvexConcaveCreateFunc);

	m_compoundCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_compoundCreateFunc);

	m_compoundCompoundCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_compoundCompoundCreateFunc);

	m_swappedCompoundCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_swappedCompoundCreateFunc);

	m_emptyCreateFunc->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_emptyCreateFunc);

	m_sphereSphereCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_sphereSphereCF);

#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	m_sphereBoxCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_sphereBoxCF);
	m_boxSphereCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_boxSphereCF);
#endif  //USE_BUGGY_SPHERE_BOX_ALGORITHM

	m_sphereTriangleCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_sphereTriangleCF);
	m_triangleSphereCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_triangleSphereCF);
	m_boxBoxCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_boxBoxCF);

	m_convexPlaneCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_convexPlaneCF);
	m_planeConvexCF->~cbtCollisionAlgorithmCreateFunc();
	cbtAlignedFree(m_planeConvexCF);

	m_pdSolver->~cbtConvexPenetrationDepthSolver();

	cbtAlignedFree(m_pdSolver);
}

cbtCollisionAlgorithmCreateFunc* cbtDefaultCollisionConfiguration::getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1)
{
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE))
	{
		return m_sphereSphereCF;
	}
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == BOX_SHAPE_PROXYTYPE))
	{
		return m_sphereBoxCF;
	}

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE))
	{
		return m_boxSphereCF;
	}
#endif  //USE_BUGGY_SPHERE_BOX_ALGORITHM

	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == TRIANGLE_SHAPE_PROXYTYPE))
	{
		return m_sphereTriangleCF;
	}

	if ((proxyType0 == TRIANGLE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE))
	{
		return m_triangleSphereCF;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType0) && (proxyType1 == STATIC_PLANE_PROXYTYPE))
	{
		return m_convexPlaneCF;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType1) && (proxyType0 == STATIC_PLANE_PROXYTYPE))
	{
		return m_planeConvexCF;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType0) && cbtBroadphaseProxy::isConvex(proxyType1))
	{
		return m_convexConvexCreateFunc;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType0) && cbtBroadphaseProxy::isConcave(proxyType1))
	{
		return m_convexConcaveCreateFunc;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType1) && cbtBroadphaseProxy::isConcave(proxyType0))
	{
		return m_swappedConvexConcaveCreateFunc;
	}

	if (cbtBroadphaseProxy::isCompound(proxyType0) && cbtBroadphaseProxy::isCompound(proxyType1))
	{
		return m_compoundCompoundCreateFunc;
	}

	if (cbtBroadphaseProxy::isCompound(proxyType0))
	{
		return m_compoundCreateFunc;
	}
	else
	{
		if (cbtBroadphaseProxy::isCompound(proxyType1))
		{
			return m_swappedCompoundCreateFunc;
		}
	}

	//failed to find an algorithm
	return m_emptyCreateFunc;
}

cbtCollisionAlgorithmCreateFunc* cbtDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1)
{
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE))
	{
		return m_sphereSphereCF;
	}
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == BOX_SHAPE_PROXYTYPE))
	{
		return m_sphereBoxCF;
	}

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE))
	{
		return m_boxSphereCF;
	}
#endif  //USE_BUGGY_SPHERE_BOX_ALGORITHM

	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == TRIANGLE_SHAPE_PROXYTYPE))
	{
		return m_sphereTriangleCF;
	}

	if ((proxyType0 == TRIANGLE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE))
	{
		return m_triangleSphereCF;
	}

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE) && (proxyType1 == BOX_SHAPE_PROXYTYPE))
	{
		return m_boxBoxCF;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType0) && (proxyType1 == STATIC_PLANE_PROXYTYPE))
	{
		return m_convexPlaneCF;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType1) && (proxyType0 == STATIC_PLANE_PROXYTYPE))
	{
		return m_planeConvexCF;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType0) && cbtBroadphaseProxy::isConvex(proxyType1))
	{
		return m_convexConvexCreateFunc;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType0) && cbtBroadphaseProxy::isConcave(proxyType1))
	{
		return m_convexConcaveCreateFunc;
	}

	if (cbtBroadphaseProxy::isConvex(proxyType1) && cbtBroadphaseProxy::isConcave(proxyType0))
	{
		return m_swappedConvexConcaveCreateFunc;
	}

	if (cbtBroadphaseProxy::isCompound(proxyType0) && cbtBroadphaseProxy::isCompound(proxyType1))
	{
		return m_compoundCompoundCreateFunc;
	}

	if (cbtBroadphaseProxy::isCompound(proxyType0))
	{
		return m_compoundCreateFunc;
	}
	else
	{
		if (cbtBroadphaseProxy::isCompound(proxyType1))
		{
			return m_swappedCompoundCreateFunc;
		}
	}

	//failed to find an algorithm
	return m_emptyCreateFunc;
}

void cbtDefaultCollisionConfiguration::setConvexConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	cbtConvexConvexAlgorithm::CreateFunc* convexConvex = (cbtConvexConvexAlgorithm::CreateFunc*)m_convexConvexCreateFunc;
	convexConvex->m_numPerturbationIterations = numPerturbationIterations;
	convexConvex->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}

void cbtDefaultCollisionConfiguration::setPlaneConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	cbtConvexPlaneCollisionAlgorithm::CreateFunc* cpCF = (cbtConvexPlaneCollisionAlgorithm::CreateFunc*)m_convexPlaneCF;
	cpCF->m_numPerturbationIterations = numPerturbationIterations;
	cpCF->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;

	cbtConvexPlaneCollisionAlgorithm::CreateFunc* pcCF = (cbtConvexPlaneCollisionAlgorithm::CreateFunc*)m_planeConvexCF;
	pcCF->m_numPerturbationIterations = numPerturbationIterations;
	pcCF->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}
