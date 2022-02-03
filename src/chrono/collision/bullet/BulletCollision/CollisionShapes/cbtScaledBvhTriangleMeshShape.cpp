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

#include "cbtScaledBvhTriangleMeshShape.h"

cbtScaledBvhTriangleMeshShape::cbtScaledBvhTriangleMeshShape(cbtBvhTriangleMeshShape* childShape, const cbtVector3& localScaling)
	: m_localScaling(localScaling), m_bvhTriMeshShape(childShape)
{
	m_shapeType = SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
}

cbtScaledBvhTriangleMeshShape::~cbtScaledBvhTriangleMeshShape()
{
}

class cbtScaledTriangleCallback : public cbtTriangleCallback
{
	cbtTriangleCallback* m_originalCallback;

	cbtVector3 m_localScaling;

public:
	cbtScaledTriangleCallback(cbtTriangleCallback* originalCallback, const cbtVector3& localScaling)
		: m_originalCallback(originalCallback),
		  m_localScaling(localScaling)
	{
	}

	virtual void processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
	{
		cbtVector3 newTriangle[3];
		newTriangle[0] = triangle[0] * m_localScaling;
		newTriangle[1] = triangle[1] * m_localScaling;
		newTriangle[2] = triangle[2] * m_localScaling;
		m_originalCallback->processTriangle(&newTriangle[0], partId, triangleIndex);
	}
};

void cbtScaledBvhTriangleMeshShape::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
	cbtScaledTriangleCallback scaledCallback(callback, m_localScaling);

	cbtVector3 invLocalScaling(1.f / m_localScaling.getX(), 1.f / m_localScaling.getY(), 1.f / m_localScaling.getZ());
	cbtVector3 scaledAabbMin, scaledAabbMax;

	///support negative scaling
	scaledAabbMin[0] = m_localScaling.getX() >= 0. ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
	scaledAabbMin[1] = m_localScaling.getY() >= 0. ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
	scaledAabbMin[2] = m_localScaling.getZ() >= 0. ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];
	scaledAabbMin[3] = 0.f;

	scaledAabbMax[0] = m_localScaling.getX() <= 0. ? aabbMin[0] * invLocalScaling[0] : aabbMax[0] * invLocalScaling[0];
	scaledAabbMax[1] = m_localScaling.getY() <= 0. ? aabbMin[1] * invLocalScaling[1] : aabbMax[1] * invLocalScaling[1];
	scaledAabbMax[2] = m_localScaling.getZ() <= 0. ? aabbMin[2] * invLocalScaling[2] : aabbMax[2] * invLocalScaling[2];
	scaledAabbMax[3] = 0.f;

	m_bvhTriMeshShape->processAllTriangles(&scaledCallback, scaledAabbMin, scaledAabbMax);
}

void cbtScaledBvhTriangleMeshShape::getAabb(const cbtTransform& trans, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	cbtVector3 localAabbMin = m_bvhTriMeshShape->getLocalAabbMin();
	cbtVector3 localAabbMax = m_bvhTriMeshShape->getLocalAabbMax();

	cbtVector3 tmpLocalAabbMin = localAabbMin * m_localScaling;
	cbtVector3 tmpLocalAabbMax = localAabbMax * m_localScaling;

	localAabbMin[0] = (m_localScaling.getX() >= 0.) ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
	localAabbMin[1] = (m_localScaling.getY() >= 0.) ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
	localAabbMin[2] = (m_localScaling.getZ() >= 0.) ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];
	localAabbMax[0] = (m_localScaling.getX() <= 0.) ? tmpLocalAabbMin[0] : tmpLocalAabbMax[0];
	localAabbMax[1] = (m_localScaling.getY() <= 0.) ? tmpLocalAabbMin[1] : tmpLocalAabbMax[1];
	localAabbMax[2] = (m_localScaling.getZ() <= 0.) ? tmpLocalAabbMin[2] : tmpLocalAabbMax[2];

	cbtVector3 localHalfExtents = cbtScalar(0.5) * (localAabbMax - localAabbMin);
	cbtScalar margin = m_bvhTriMeshShape->getMargin();
	localHalfExtents += cbtVector3(margin, margin, margin);
	cbtVector3 localCenter = cbtScalar(0.5) * (localAabbMax + localAabbMin);

	cbtMatrix3x3 abs_b = trans.getBasis().absolute();

	cbtVector3 center = trans(localCenter);

	cbtVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
	aabbMin = center - extent;
	aabbMax = center + extent;
}

void cbtScaledBvhTriangleMeshShape::setLocalScaling(const cbtVector3& scaling)
{
	m_localScaling = scaling;
}

const cbtVector3& cbtScaledBvhTriangleMeshShape::getLocalScaling() const
{
	return m_localScaling;
}

void cbtScaledBvhTriangleMeshShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	///don't make this a movable object!
	//	cbtAssert(0);
}
