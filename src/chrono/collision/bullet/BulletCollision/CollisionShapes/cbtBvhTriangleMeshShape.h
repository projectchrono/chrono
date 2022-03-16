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

#ifndef BT_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_BVH_TRIANGLE_MESH_SHAPE_H

#include "cbtTriangleMeshShape.h"
#include "cbtOptimizedBvh.h"
#include "LinearMath/cbtAlignedAllocator.h"
#include "cbtTriangleInfoMap.h"

///The cbtBvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
///If you required moving concave triangle meshes, it is recommended to perform convex decomposition
///using HACD, see Bullet/Demos/ConvexDecompositionDemo.
///Alternatively, you can use cbtGimpactMeshShape for moving concave triangle meshes.
///cbtBvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and
///cache friendly traversal for PlayStation 3 Cell SPU.
///It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a cbtTriangleMesh or cbtTriangleIndexVertexArray. The cbtBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
ATTRIBUTE_ALIGNED16(class)
cbtBvhTriangleMeshShape : public cbtTriangleMeshShape
{
	cbtOptimizedBvh* m_bvh;
	cbtTriangleInfoMap* m_triangleInfoMap;

	bool m_useQuantizedAabbCompression;
	bool m_ownsBvh;
#ifdef __clang__
	bool m_pad[11] __attribute__((unused));  ////need padding due to alignment
#else
	bool m_pad[11];  ////need padding due to alignment
#endif

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtBvhTriangleMeshShape(cbtStridingMeshInterface * meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true);

	///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
	cbtBvhTriangleMeshShape(cbtStridingMeshInterface * meshInterface, bool useQuantizedAabbCompression, const cbtVector3& bvhAabbMin, const cbtVector3& bvhAabbMax, bool buildBvh = true);

	virtual ~cbtBvhTriangleMeshShape();

	bool getOwnsBvh() const
	{
		return m_ownsBvh;
	}

	void performRaycast(cbtTriangleCallback * callback, const cbtVector3& raySource, const cbtVector3& rayTarget);
	void performConvexcast(cbtTriangleCallback * callback, const cbtVector3& boxSource, const cbtVector3& boxTarget, const cbtVector3& boxMin, const cbtVector3& boxMax);

	virtual void processAllTriangles(cbtTriangleCallback * callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const;

	void refitTree(const cbtVector3& aabbMin, const cbtVector3& aabbMax);

	///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
	void partialRefitTree(const cbtVector3& aabbMin, const cbtVector3& aabbMax);

	//debugging
	virtual const char* getName() const { return "BVHTRIANGLEMESH"; }

	virtual void setLocalScaling(const cbtVector3& scaling);

	cbtOptimizedBvh* getOptimizedBvh()
	{
		return m_bvh;
	}

	void setOptimizedBvh(cbtOptimizedBvh * bvh, const cbtVector3& localScaling = cbtVector3(1, 1, 1));

	void buildOptimizedBvh();

	bool usesQuantizedAabbCompression() const
	{
		return m_useQuantizedAabbCompression;
	}

	void setTriangleInfoMap(cbtTriangleInfoMap * triangleInfoMap)
	{
		m_triangleInfoMap = triangleInfoMap;
	}

	const cbtTriangleInfoMap* getTriangleInfoMap() const
	{
		return m_triangleInfoMap;
	}

	cbtTriangleInfoMap* getTriangleInfoMap()
	{
		return m_triangleInfoMap;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;

	virtual void serializeSingleBvh(cbtSerializer * serializer) const;

	virtual void serializeSingleTriangleInfoMap(cbtSerializer * serializer) const;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	cbtTriangleMeshShapeData
{
	cbtCollisionShapeData	m_collisionShapeData;

	cbtStridingMeshInterfaceData m_meshInterface;

	cbtQuantizedBvhFloatData		*m_quantizedFloatBvh;
	cbtQuantizedBvhDoubleData	*m_quantizedDoubleBvh;

	cbtTriangleInfoMapData	*m_triangleInfoMap;
	
	float	m_collisionMargin;

	char m_pad3[4];
	
};

// clang-format on

SIMD_FORCE_INLINE int cbtBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtTriangleMeshShapeData);
}

#endif  //BT_BVH_TRIANGLE_MESH_SHAPE_H
