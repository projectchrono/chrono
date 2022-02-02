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

#include "cbtConvexPointCloudShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"

#include "LinearMath/cbtQuaternion.h"

void cbtConvexPointCloudShape::setLocalScaling(const cbtVector3& scaling)
{
	m_localScaling = scaling;
	recalcLocalAabb();
}

#ifndef __SPU__
cbtVector3 cbtConvexPointCloudShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const
{
	cbtVector3 supVec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	cbtScalar maxDot = cbtScalar(-BT_LARGE_FLOAT);

	cbtVector3 vec = vec0;
	cbtScalar lenSqr = vec.length2();
	if (lenSqr < cbtScalar(0.0001))
	{
		vec.setValue(1, 0, 0);
	}
	else
	{
		cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
		vec *= rlen;
	}

	if (m_numPoints > 0)
	{
		// Here we take advantage of dot(a*b, c) = dot( a, b*c) to do less work. Note this transformation is true mathematically, not numerically.
		//    cbtVector3 scaled = vec * m_localScaling;
		int index = (int)vec.maxDot(&m_unscaledPoints[0], m_numPoints, maxDot);  //FIXME: may violate encapsulation of m_unscaledPoints
		return getScaledPoint(index);
	}

	return supVec;
}

void cbtConvexPointCloudShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	for (int j = 0; j < numVectors; j++)
	{
		const cbtVector3& vec = vectors[j] * m_localScaling;  // dot( a*c, b) = dot(a, b*c)
		cbtScalar maxDot;
		int index = (int)vec.maxDot(&m_unscaledPoints[0], m_numPoints, maxDot);
		supportVerticesOut[j][3] = cbtScalar(-BT_LARGE_FLOAT);
		if (0 <= index)
		{
			//WARNING: don't swap next lines, the w component would get overwritten!
			supportVerticesOut[j] = getScaledPoint(index);
			supportVerticesOut[j][3] = maxDot;
		}
	}
}

cbtVector3 cbtConvexPointCloudShape::localGetSupportingVertex(const cbtVector3& vec) const
{
	cbtVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

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

#endif

//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw cbtConvexHullShape with the Raytracer Demo
int cbtConvexPointCloudShape::getNumVertices() const
{
	return m_numPoints;
}

int cbtConvexPointCloudShape::getNumEdges() const
{
	return 0;
}

void cbtConvexPointCloudShape::getEdge(int i, cbtVector3& pa, cbtVector3& pb) const
{
	cbtAssert(0);
}

void cbtConvexPointCloudShape::getVertex(int i, cbtVector3& vtx) const
{
	vtx = m_unscaledPoints[i] * m_localScaling;
}

int cbtConvexPointCloudShape::getNumPlanes() const
{
	return 0;
}

void cbtConvexPointCloudShape::getPlane(cbtVector3&, cbtVector3&, int) const
{
	cbtAssert(0);
}

//not yet
bool cbtConvexPointCloudShape::isInside(const cbtVector3&, cbtScalar) const
{
	cbtAssert(0);
	return false;
}
