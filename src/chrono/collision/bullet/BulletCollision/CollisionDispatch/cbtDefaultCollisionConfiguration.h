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

#ifndef BT_DEFAULT_COLLISION_CONFIGURATION
#define BT_DEFAULT_COLLISION_CONFIGURATION

#include "cbtCollisionConfiguration.h"
class cbtVoronoiSimplexSolver;
class cbtConvexPenetrationDepthSolver;

struct cbtDefaultCollisionConstructionInfo
{
	cbtPoolAllocator* m_persistentManifoldPool;
	cbtPoolAllocator* m_collisionAlgorithmPool;
	int m_defaultMaxPersistentManifoldPoolSize;
	int m_defaultMaxCollisionAlgorithmPoolSize;
	int m_customCollisionAlgorithmMaxElementSize;
	int m_useEpaPenetrationAlgorithm;

	cbtDefaultCollisionConstructionInfo()
		: m_persistentManifoldPool(0),
		  m_collisionAlgorithmPool(0),
		  m_defaultMaxPersistentManifoldPoolSize(4096),
		  m_defaultMaxCollisionAlgorithmPoolSize(4096),
		  m_customCollisionAlgorithmMaxElementSize(0),
		  m_useEpaPenetrationAlgorithm(true)
	{
	}
};

///cbtCollisionConfiguration allows to configure Bullet collision detection
///stack allocator, pool memory allocators
///@todo: describe the meaning
class cbtDefaultCollisionConfiguration : public cbtCollisionConfiguration
{
protected:
	int m_persistentManifoldPoolSize;

	cbtPoolAllocator* m_persistentManifoldPool;
	bool m_ownsPersistentManifoldPool;

	cbtPoolAllocator* m_collisionAlgorithmPool;
	bool m_ownsCollisionAlgorithmPool;

	//default penetration depth solver
	cbtConvexPenetrationDepthSolver* m_pdSolver;

	//default CreationFunctions, filling the m_doubleDispatch table
	cbtCollisionAlgorithmCreateFunc* m_convexConvexCreateFunc;
	cbtCollisionAlgorithmCreateFunc* m_convexConcaveCreateFunc;
	cbtCollisionAlgorithmCreateFunc* m_swappedConvexConcaveCreateFunc;
	cbtCollisionAlgorithmCreateFunc* m_compoundCreateFunc;
	cbtCollisionAlgorithmCreateFunc* m_compoundCompoundCreateFunc;

	cbtCollisionAlgorithmCreateFunc* m_swappedCompoundCreateFunc;
	cbtCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
	cbtCollisionAlgorithmCreateFunc* m_sphereSphereCF;
	cbtCollisionAlgorithmCreateFunc* m_sphereBoxCF;
	cbtCollisionAlgorithmCreateFunc* m_boxSphereCF;

	cbtCollisionAlgorithmCreateFunc* m_boxBoxCF;
	cbtCollisionAlgorithmCreateFunc* m_sphereTriangleCF;
	cbtCollisionAlgorithmCreateFunc* m_triangleSphereCF;
	cbtCollisionAlgorithmCreateFunc* m_planeConvexCF;
	cbtCollisionAlgorithmCreateFunc* m_convexPlaneCF;

public:
	cbtDefaultCollisionConfiguration(const cbtDefaultCollisionConstructionInfo& constructionInfo = cbtDefaultCollisionConstructionInfo());

	virtual ~cbtDefaultCollisionConfiguration();

	///memory pools
	virtual cbtPoolAllocator* getPersistentManifoldPool()
	{
		return m_persistentManifoldPool;
	}

	virtual cbtPoolAllocator* getCollisionAlgorithmPool()
	{
		return m_collisionAlgorithmPool;
	}

	virtual cbtCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1);

	virtual cbtCollisionAlgorithmCreateFunc* getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1);

	///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
	///By default, this feature is disabled for best performance.
	///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
	///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
	///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
	///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
	///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
	void setConvexConvexMultipointIterations(int numPerturbationIterations = 3, int minimumPointsPerturbationThreshold = 3);

	void setPlaneConvexMultipointIterations(int numPerturbationIterations = 3, int minimumPointsPerturbationThreshold = 3);
};

#endif  //BT_DEFAULT_COLLISION_CONFIGURATION
