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

#ifndef BT_CONVEX_POINT_CLOUD_SHAPE_H
#define BT_CONVEX_POINT_CLOUD_SHAPE_H

#include "cbtPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "LinearMath/cbtAlignedObjectArray.h"

///The cbtConvexPointCloudShape implements an implicit convex hull of an array of vertices.
ATTRIBUTE_ALIGNED16(class)
cbtConvexPointCloudShape : public cbtPolyhedralConvexAabbCachingShape
{
	cbtVector3* m_unscaledPoints;
	int m_numPoints;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConvexPointCloudShape()
	{
		m_localScaling.setValue(1.f, 1.f, 1.f);
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = 0;
		m_numPoints = 0;
	}

	cbtConvexPointCloudShape(cbtVector3 * points, int numPoints, const cbtVector3& localScaling, bool computeAabb = true)
	{
		m_localScaling = localScaling;
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = points;
		m_numPoints = numPoints;

		if (computeAabb)
			recalcLocalAabb();
	}

	void setPoints(cbtVector3 * points, int numPoints, bool computeAabb = true, const cbtVector3& localScaling = cbtVector3(1.f, 1.f, 1.f))
	{
		m_unscaledPoints = points;
		m_numPoints = numPoints;
		m_localScaling = localScaling;

		if (computeAabb)
			recalcLocalAabb();
	}

	SIMD_FORCE_INLINE cbtVector3* getUnscaledPoints()
	{
		return m_unscaledPoints;
	}

	SIMD_FORCE_INLINE const cbtVector3* getUnscaledPoints() const
	{
		return m_unscaledPoints;
	}

	SIMD_FORCE_INLINE int getNumPoints() const
	{
		return m_numPoints;
	}

	SIMD_FORCE_INLINE cbtVector3 getScaledPoint(int index) const
	{
		return m_unscaledPoints[index] * m_localScaling;
	}

#ifndef __SPU__
	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;
#endif

	//debugging
	virtual const char* getName() const { return "ConvexPointCloud"; }

	virtual int getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const;
	virtual void getVertex(int i, cbtVector3& vtx) const;
	virtual int getNumPlanes() const;
	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const;
	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const;

	///in case we receive negative scaling
	virtual void setLocalScaling(const cbtVector3& scaling);
};

#endif  //BT_CONVEX_POINT_CLOUD_SHAPE_H
