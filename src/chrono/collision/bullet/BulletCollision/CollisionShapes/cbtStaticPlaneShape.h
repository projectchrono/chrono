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

#ifndef BT_STATIC_PLANE_SHAPE_H
#define BT_STATIC_PLANE_SHAPE_H

#include "cbtConcaveShape.h"

///The cbtStaticPlaneShape simulates an infinite non-moving (static) collision plane.
ATTRIBUTE_ALIGNED16(class)
cbtStaticPlaneShape : public cbtConcaveShape
{
protected:
	cbtVector3 m_localAabbMin;
	cbtVector3 m_localAabbMax;

	cbtVector3 m_planeNormal;
	cbtScalar m_planeConstant;
	cbtVector3 m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtStaticPlaneShape(const cbtVector3& planeNormal, cbtScalar planeConstant);

	virtual ~cbtStaticPlaneShape();

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void processAllTriangles(cbtTriangleCallback * callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual void setLocalScaling(const cbtVector3& scaling);
	virtual const cbtVector3& getLocalScaling() const;

	const cbtVector3& getPlaneNormal() const
	{
		return m_planeNormal;
	}

	const cbtScalar& getPlaneConstant() const
	{
		return m_planeConstant;
	}

	//debugging
	virtual const char* getName() const { return "STATICPLANE"; }

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtStaticPlaneShapeData
{
	cbtCollisionShapeData m_collisionShapeData;

	cbtVector3FloatData m_localScaling;
	cbtVector3FloatData m_planeNormal;
	float m_planeConstant;
	char m_pad[4];
};

SIMD_FORCE_INLINE int cbtStaticPlaneShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtStaticPlaneShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtStaticPlaneShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtStaticPlaneShapeData* planeData = (cbtStaticPlaneShapeData*)dataBuffer;
	cbtCollisionShape::serialize(&planeData->m_collisionShapeData, serializer);

	m_localScaling.serializeFloat(planeData->m_localScaling);
	m_planeNormal.serializeFloat(planeData->m_planeNormal);
	planeData->m_planeConstant = float(m_planeConstant);

	// Fill padding with zeros to appease msan.
	planeData->m_pad[0] = 0;
	planeData->m_pad[1] = 0;
	planeData->m_pad[2] = 0;
	planeData->m_pad[3] = 0;

	return "cbtStaticPlaneShapeData";
}

#endif  //BT_STATIC_PLANE_SHAPE_H
