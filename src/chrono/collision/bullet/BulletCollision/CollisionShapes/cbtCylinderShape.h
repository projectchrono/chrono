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

#ifndef BT_CYLINDER_MINKOWSKI_H
#define BT_CYLINDER_MINKOWSKI_H

#include "cbtBoxShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "LinearMath/cbtVector3.h"

/// The cbtCylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y axis. cbtCylinderShapeX is aligned with the X axis and cbtCylinderShapeZ around the Z axis.
ATTRIBUTE_ALIGNED16(class)
cbtCylinderShape : public cbtConvexInternalShape

{
protected:
	int m_upAxis;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtVector3 getHalfExtentsWithMargin() const
	{
		cbtVector3 halfExtents = getHalfExtentsWithoutMargin();
		cbtVector3 margin(getMargin(), getMargin(), getMargin());
		halfExtents += margin;
		return halfExtents;
	}

	const cbtVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;  //changed in Bullet 2.63: assume the scaling and margin are included
	}

	cbtCylinderShape(const cbtVector3& halfExtents);

	void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void setMargin(cbtScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
		cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

		cbtConvexInternalShape::setMargin(collisionMargin);
		cbtVector3 newMargin(getMargin(), getMargin(), getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	}

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const
	{
		cbtVector3 supVertex;
		supVertex = localGetSupportingVertexWithoutMargin(vec);

		if (getMargin() != cbtScalar(0.))
		{
			cbtVector3 vecnorm = vec;
			if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
			{
				vecnorm.setValue(cbtScalar(-1.), cbtScalar(-1.), cbtScalar(-1.));
			}
			vecnorm.normalize();
			supVertex += getMargin() * vecnorm;
		}
		return supVertex;
	}

	//use box inertia
	//	virtual void	calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const;

	int getUpAxis() const
	{
		return m_upAxis;
	}

	virtual cbtVector3 getAnisotropicRollingFrictionDirection() const
	{
		cbtVector3 aniDir(0, 0, 0);
		aniDir[getUpAxis()] = 1;
		return aniDir;
	}

	virtual cbtScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}

	virtual void setLocalScaling(const cbtVector3& scaling)
	{
		cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
		cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
		cbtVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		cbtConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
	}

	//debugging
	virtual const char* getName() const
	{
		return "CylinderY";
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

class cbtCylinderShapeX : public cbtCylinderShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtCylinderShapeX(const cbtVector3& halfExtents);

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	//debugging
	virtual const char* getName() const
	{
		return "CylinderX";
	}

	virtual cbtScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getY();
	}
};

class cbtCylinderShapeZ : public cbtCylinderShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtCylinderShapeZ(const cbtVector3& halfExtents);

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	//debugging
	virtual const char* getName() const
	{
		return "CylinderZ";
	}

	virtual cbtScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtCylinderShapeData
{
	cbtConvexInternalShapeData m_convexInternalShapeData;

	int m_upAxis;

	char m_padding[4];
};

SIMD_FORCE_INLINE int cbtCylinderShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtCylinderShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtCylinderShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtCylinderShapeData* shapeData = (cbtCylinderShapeData*)dataBuffer;

	cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	shapeData->m_upAxis = m_upAxis;

	// Fill padding with zeros to appease msan.
	shapeData->m_padding[0] = 0;
	shapeData->m_padding[1] = 0;
	shapeData->m_padding[2] = 0;
	shapeData->m_padding[3] = 0;

	return "cbtCylinderShapeData";
}

#endif  //BT_CYLINDER_MINKOWSKI_H
