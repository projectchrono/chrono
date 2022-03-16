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

#ifndef BT_CAPSULE_SHAPE_H
#define BT_CAPSULE_SHAPE_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types

///The cbtCapsuleShape represents a capsule around the Y axis, there is also the cbtCapsuleShapeX aligned around the X axis and cbtCapsuleShapeZ around the Z axis.
///The total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
///The cbtCapsuleShape is a convex hull of two spheres. The cbtMultiSphereShape is a more general collision shape that takes the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.
ATTRIBUTE_ALIGNED16(class)
cbtCapsuleShape : public cbtConvexInternalShape
{
protected:
	int m_upAxis;

protected:
	///only used for cbtCapsuleShapeZ and cbtCapsuleShapeX subclasses.
	cbtCapsuleShape() : cbtConvexInternalShape() { m_shapeType = CAPSULE_SHAPE_PROXYTYPE; };

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtCapsuleShape(cbtScalar radius, cbtScalar height);

	///CollisionShape Interface
	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	/// cbtConvexShape Interface
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void setMargin(cbtScalar collisionMargin)
	{
		//don't override the margin for capsules, their entire radius == margin
		(void)collisionMargin;
	}

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
	{
		cbtVector3 halfExtents(getRadius(), getRadius(), getRadius());
		halfExtents[m_upAxis] = getRadius() + getHalfHeight();
		cbtMatrix3x3 abs_b = t.getBasis().absolute();
		cbtVector3 center = t.getOrigin();
		cbtVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);

		aabbMin = center - extent;
		aabbMax = center + extent;
	}

	virtual const char* getName() const
	{
		return "CapsuleShape";
	}

	int getUpAxis() const
	{
		return m_upAxis;
	}

	cbtScalar getRadius() const
	{
		int radiusAxis = (m_upAxis + 2) % 3;
		return m_implicitShapeDimensions[radiusAxis];
	}

	cbtScalar getHalfHeight() const
	{
		return m_implicitShapeDimensions[m_upAxis];
	}

	virtual void setLocalScaling(const cbtVector3& scaling)
	{
		cbtVector3 unScaledImplicitShapeDimensions = m_implicitShapeDimensions / m_localScaling;
		cbtConvexInternalShape::setLocalScaling(scaling);
		m_implicitShapeDimensions = (unScaledImplicitShapeDimensions * scaling);
		//update m_collisionMargin, since entire radius==margin
		int radiusAxis = (m_upAxis + 2) % 3;
		m_collisionMargin = m_implicitShapeDimensions[radiusAxis];
	}

	virtual cbtVector3 getAnisotropicRollingFrictionDirection() const
	{
		cbtVector3 aniDir(0, 0, 0);
		aniDir[getUpAxis()] = 1;
		return aniDir;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;

	SIMD_FORCE_INLINE void deSerializeFloat(struct cbtCapsuleShapeData * dataBuffer);
};

///cbtCapsuleShapeX represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class cbtCapsuleShapeX : public cbtCapsuleShape
{
public:
	cbtCapsuleShapeX(cbtScalar radius, cbtScalar height);

	//debugging
	virtual const char* getName() const
	{
		return "CapsuleX";
	}
};

///cbtCapsuleShapeZ represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class cbtCapsuleShapeZ : public cbtCapsuleShape
{
public:
	cbtCapsuleShapeZ(cbtScalar radius, cbtScalar height);

	//debugging
	virtual const char* getName() const
	{
		return "CapsuleZ";
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtCapsuleShapeData
{
	cbtConvexInternalShapeData m_convexInternalShapeData;

	int m_upAxis;

	char m_padding[4];
};

SIMD_FORCE_INLINE int cbtCapsuleShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtCapsuleShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtCapsuleShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtCapsuleShapeData* shapeData = (cbtCapsuleShapeData*)dataBuffer;

	cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	shapeData->m_upAxis = m_upAxis;

	// Fill padding with zeros to appease msan.
	shapeData->m_padding[0] = 0;
	shapeData->m_padding[1] = 0;
	shapeData->m_padding[2] = 0;
	shapeData->m_padding[3] = 0;

	return "cbtCapsuleShapeData";
}

SIMD_FORCE_INLINE void cbtCapsuleShape::deSerializeFloat(cbtCapsuleShapeData* dataBuffer)
{
	m_implicitShapeDimensions.deSerializeFloat(dataBuffer->m_convexInternalShapeData.m_implicitShapeDimensions);
	m_collisionMargin = dataBuffer->m_convexInternalShapeData.m_collisionMargin;
	m_localScaling.deSerializeFloat(dataBuffer->m_convexInternalShapeData.m_localScaling);
	//it is best to already pre-allocate the matching cbtCapsuleShape*(X/Z) version to match m_upAxis
	m_upAxis = dataBuffer->m_upAxis;
}

#endif  //BT_CAPSULE_SHAPE_H
