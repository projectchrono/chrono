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

#ifndef BT_POLYHEDRAL_CONVEX_SHAPE_H
#define BT_POLYHEDRAL_CONVEX_SHAPE_H

#include "LinearMath/cbtMatrix3x3.h"
#include "cbtConvexInternalShape.h"
class cbtConvexPolyhedron;

///The cbtPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
ATTRIBUTE_ALIGNED16(class)
cbtPolyhedralConvexShape : public cbtConvexInternalShape
{
protected:
	cbtConvexPolyhedron* m_polyhedron;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtPolyhedralConvexShape();

	virtual ~cbtPolyhedralConvexShape();

	///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
	///experimental/work-in-progress
	virtual bool initializePolyhedralFeatures(int shiftVerticesByMargin = 0);

	virtual void setPolyhedralFeatures(cbtConvexPolyhedron & polyhedron);

	const cbtConvexPolyhedron* getConvexPolyhedron() const
	{
		return m_polyhedron;
	}

	//brute force implementations

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual int getNumVertices() const = 0;
	virtual int getNumEdges() const = 0;
	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const = 0;
	virtual void getVertex(int i, cbtVector3& vtx) const = 0;
	virtual int getNumPlanes() const = 0;
	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const = 0;
	//	virtual int getIndex(int i) const = 0 ;

	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const = 0;
};

///The cbtPolyhedralConvexAabbCachingShape adds aabb caching to the cbtPolyhedralConvexShape
class cbtPolyhedralConvexAabbCachingShape : public cbtPolyhedralConvexShape
{
	cbtVector3 m_localAabbMin;
	cbtVector3 m_localAabbMax;
	bool m_isLocalAabbValid;

protected:
	void setCachedLocalAabb(const cbtVector3& aabbMin, const cbtVector3& aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	inline void getCachedLocalAabb(cbtVector3& aabbMin, cbtVector3& aabbMax) const
	{
		cbtAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

protected:
	cbtPolyhedralConvexAabbCachingShape();

public:
	inline void getNonvirtualAabb(const cbtTransform& trans, cbtVector3& aabbMin, cbtVector3& aabbMax, cbtScalar margin) const
	{
		//lazy evaluation of local aabb
		cbtAssert(m_isLocalAabbValid);
		cbtTransformAabb(m_localAabbMin, m_localAabbMax, margin, trans, aabbMin, aabbMax);
	}

	virtual void setLocalScaling(const cbtVector3& scaling);

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	void recalcLocalAabb();
};

#endif  //BT_POLYHEDRAL_CONVEX_SHAPE_H
