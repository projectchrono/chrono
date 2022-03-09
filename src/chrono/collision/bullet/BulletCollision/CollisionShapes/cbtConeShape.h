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

#ifndef BT_CONE_MINKOWSKI_H
#define BT_CONE_MINKOWSKI_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types

///The cbtConeShape implements a cone shape primitive, centered around the origin and aligned with the Y axis. The cbtConeShapeX is aligned around the X axis and cbtConeShapeZ around the Z axis.
ATTRIBUTE_ALIGNED16(class)
cbtConeShape : public cbtConvexInternalShape

{
	cbtScalar m_sinAngle;
	cbtScalar m_radius;
	cbtScalar m_height;
	int m_coneIndices[3];
	cbtVector3 coneLocalSupport(const cbtVector3& v) const;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConeShape(cbtScalar radius, cbtScalar height);

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	cbtScalar getRadius() const { return m_radius; }
	cbtScalar getHeight() const { return m_height; }

	void setRadius(const cbtScalar radius)
	{
		m_radius = radius;
	}
	void setHeight(const cbtScalar height)
	{
		m_height = height;
	}

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const
	{
		cbtTransform identity;
		identity.setIdentity();
		cbtVector3 aabbMin, aabbMax;
		getAabb(identity, aabbMin, aabbMax);

		cbtVector3 halfExtents = (aabbMax - aabbMin) * cbtScalar(0.5);

		cbtScalar margin = getMargin();

		cbtScalar lx = cbtScalar(2.) * (halfExtents.x() + margin);
		cbtScalar ly = cbtScalar(2.) * (halfExtents.y() + margin);
		cbtScalar lz = cbtScalar(2.) * (halfExtents.z() + margin);
		const cbtScalar x2 = lx * lx;
		const cbtScalar y2 = ly * ly;
		const cbtScalar z2 = lz * lz;
		const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

		inertia = scaledmass * (cbtVector3(y2 + z2, x2 + z2, x2 + y2));

		//		inertia.x() = scaledmass * (y2+z2);
		//		inertia.y() = scaledmass * (x2+z2);
		//		inertia.z() = scaledmass * (x2+y2);
	}

	virtual const char* getName() const
	{
		return "Cone";
	}

	///choose upAxis index
	void setConeUpIndex(int upIndex);

	int getConeUpIndex() const
	{
		return m_coneIndices[1];
	}

	virtual cbtVector3 getAnisotropicRollingFrictionDirection() const
	{
		return cbtVector3(0, 1, 0);
	}

	virtual void setLocalScaling(const cbtVector3& scaling);

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

///cbtConeShape implements a Cone shape, around the X axis
class cbtConeShapeX : public cbtConeShape
{
public:
	cbtConeShapeX(cbtScalar radius, cbtScalar height);

	virtual cbtVector3 getAnisotropicRollingFrictionDirection() const
	{
		return cbtVector3(1, 0, 0);
	}

	//debugging
	virtual const char* getName() const
	{
		return "ConeX";
	}
};

///cbtConeShapeZ implements a Cone shape, around the Z axis
class cbtConeShapeZ : public cbtConeShape
{
public:
	cbtConeShapeZ(cbtScalar radius, cbtScalar height);

	virtual cbtVector3 getAnisotropicRollingFrictionDirection() const
	{
		return cbtVector3(0, 0, 1);
	}

	//debugging
	virtual const char* getName() const
	{
		return "ConeZ";
	}
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtConeShapeData
{
	cbtConvexInternalShapeData m_convexInternalShapeData;

	int m_upIndex;

	char m_padding[4];
};

SIMD_FORCE_INLINE int cbtConeShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtConeShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtConeShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtConeShapeData* shapeData = (cbtConeShapeData*)dataBuffer;

	cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	shapeData->m_upIndex = m_coneIndices[1];

	// Fill padding with zeros to appease msan.
	shapeData->m_padding[0] = 0;
	shapeData->m_padding[1] = 0;
	shapeData->m_padding[2] = 0;
	shapeData->m_padding[3] = 0;

	return "cbtConeShapeData";
}

#endif  //BT_CONE_MINKOWSKI_H
