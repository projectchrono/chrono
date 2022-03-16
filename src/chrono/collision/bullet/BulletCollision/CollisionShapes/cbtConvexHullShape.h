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

#ifndef BT_CONVEX_HULL_SHAPE_H
#define BT_CONVEX_HULL_SHAPE_H

#include "cbtPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "LinearMath/cbtAlignedObjectArray.h"

///The cbtConvexHullShape implements an implicit convex hull of an array of vertices.
///Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.
ATTRIBUTE_ALIGNED16(class)
cbtConvexHullShape : public cbtPolyhedralConvexAabbCachingShape
{
	cbtAlignedObjectArray<cbtVector3> m_unscaledPoints;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive cbtScalar (x,y,z), the striding defines the number of bytes between each point, in memory.
	///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
	///cbtConvexHullShape make an internal copy of the points.
	cbtConvexHullShape(const cbtScalar* points = 0, int numPoints = 0, int stride = sizeof(cbtVector3));

	void addPoint(const cbtVector3& point, bool recalculateLocalAabb = true);

	cbtVector3* getUnscaledPoints()
	{
		return &m_unscaledPoints[0];
	}

	const cbtVector3* getUnscaledPoints() const
	{
		return &m_unscaledPoints[0];
	}

	///getPoints is obsolete, please use getUnscaledPoints
	const cbtVector3* getPoints() const
	{
		return getUnscaledPoints();
	}

	void optimizeConvexHull();

	SIMD_FORCE_INLINE cbtVector3 getScaledPoint(int i) const
	{
		return m_unscaledPoints[i] * m_localScaling;
	}

	SIMD_FORCE_INLINE int getNumPoints() const
	{
		return m_unscaledPoints.size();
	}

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void project(const cbtTransform& trans, const cbtVector3& dir, cbtScalar& minProj, cbtScalar& maxProj, cbtVector3& witnesPtMin, cbtVector3& witnesPtMax) const;

	//debugging
	virtual const char* getName() const { return "Convex"; }

	virtual int getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const;
	virtual void getVertex(int i, cbtVector3& vtx) const;
	virtual int getNumPlanes() const;
	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const;
	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const;

	///in case we receive negative scaling
	virtual void setLocalScaling(const cbtVector3& scaling);

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	cbtConvexHullShapeData
{
	cbtConvexInternalShapeData	m_convexInternalShapeData;

	cbtVector3FloatData	*m_unscaledPointsFloatPtr;
	cbtVector3DoubleData	*m_unscaledPointsDoublePtr;

	int		m_numUnscaledPoints;
	char m_padding3[4];

};

// clang-format on

SIMD_FORCE_INLINE int cbtConvexHullShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtConvexHullShapeData);
}

#endif  //BT_CONVEX_HULL_SHAPE_H
