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

#ifndef BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H

#include "BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.h"

///The cbtScaledBvhTriangleMeshShape allows to instance a scaled version of an existing cbtBvhTriangleMeshShape.
///Note that each cbtBvhTriangleMeshShape still can have its own local scaling, independent from this cbtScaledBvhTriangleMeshShape 'localScaling'
ATTRIBUTE_ALIGNED16(class)
cbtScaledBvhTriangleMeshShape : public cbtConcaveShape
{
	cbtVector3 m_localScaling;

	cbtBvhTriangleMeshShape* m_bvhTriMeshShape;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtScaledBvhTriangleMeshShape(cbtBvhTriangleMeshShape * childShape, const cbtVector3& localScaling);

	virtual ~cbtScaledBvhTriangleMeshShape();

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;
	virtual void setLocalScaling(const cbtVector3& scaling);
	virtual const cbtVector3& getLocalScaling() const;
	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual void processAllTriangles(cbtTriangleCallback * callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const;

	cbtBvhTriangleMeshShape* getChildShape()
	{
		return m_bvhTriMeshShape;
	}

	const cbtBvhTriangleMeshShape* getChildShape() const
	{
		return m_bvhTriMeshShape;
	}

	//debugging
	virtual const char* getName() const { return "SCALEDBVHTRIANGLEMESH"; }

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtScaledTriangleMeshShapeData
{
	cbtTriangleMeshShapeData m_trimeshShapeData;

	cbtVector3FloatData m_localScaling;
};

SIMD_FORCE_INLINE int cbtScaledBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtScaledTriangleMeshShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtScaledBvhTriangleMeshShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtScaledTriangleMeshShapeData* scaledMeshData = (cbtScaledTriangleMeshShapeData*)dataBuffer;
	m_bvhTriMeshShape->serialize(&scaledMeshData->m_trimeshShapeData, serializer);
	scaledMeshData->m_trimeshShapeData.m_collisionShapeData.m_shapeType = SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
	m_localScaling.serializeFloat(scaledMeshData->m_localScaling);
	return "cbtScaledTriangleMeshShapeData";
}

#endif  //BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
