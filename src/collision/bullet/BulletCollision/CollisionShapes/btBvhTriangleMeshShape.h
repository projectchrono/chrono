/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BVH_TRIANGLE_MESH_SHAPE_H
#define BVH_TRIANGLE_MESH_SHAPE_H

#include "btTriangleMeshShape.h"
#include "btOptimizedBvh.h"
#include "LinearMath/btAlignedAllocator.h"


///The btBvhTriangleMeshShape is a static-triangle mesh shape with several optimizations, such as bounding volume hierarchy and cache friendly traversal for PlayStation 3 Cell SPU. It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
ATTRIBUTE_ALIGNED16(class) btBvhTriangleMeshShape : public btTriangleMeshShape
{

	btOptimizedBvh*	m_bvh;
	bool m_useQuantizedAabbCompression;
	bool m_ownsBvh;
	bool m_pad[11];////need padding due to alignment

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btBvhTriangleMeshShape() : btTriangleMeshShape(0),m_bvh(0),m_ownsBvh(false) {m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;};
	btBvhTriangleMeshShape(btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true);

	///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
	btBvhTriangleMeshShape(btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression,const btVector3& bvhAabbMin,const btVector3& bvhAabbMax, bool buildBvh = true);
	
	virtual ~btBvhTriangleMeshShape();

	bool getOwnsBvh () const
	{
		return m_ownsBvh;
	}


	
	void performRaycast (btTriangleCallback* callback, const btVector3& raySource, const btVector3& rayTarget);
	void performConvexcast (btTriangleCallback* callback, const btVector3& boxSource, const btVector3& boxTarget, const btVector3& boxMin, const btVector3& boxMax);

	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

	void	refitTree(const btVector3& aabbMin,const btVector3& aabbMax);

	///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
	void	partialRefitTree(const btVector3& aabbMin,const btVector3& aabbMax);

	//debugging
	virtual const char*	getName()const {return "BVHTRIANGLEMESH";}


	virtual void	setLocalScaling(const btVector3& scaling);
	
	btOptimizedBvh*	getOptimizedBvh()
	{
		return m_bvh;
	}


	void	setOptimizedBvh(btOptimizedBvh* bvh, const btVector3& localScaling=btVector3(1,1,1));

	bool	usesQuantizedAabbCompression() const
	{
		return	m_useQuantizedAabbCompression;
	}
}
;

#endif //BVH_TRIANGLE_MESH_SHAPE_H
