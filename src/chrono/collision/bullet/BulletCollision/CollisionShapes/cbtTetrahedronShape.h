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

#ifndef BT_SIMPLEX_1TO4_SHAPE
#define BT_SIMPLEX_1TO4_SHAPE

#include "cbtPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"

///The cbtBU_Simplex1to4 implements tetrahedron, triangle, line, vertex collision shapes. In most cases it is better to use cbtConvexHullShape instead.
ATTRIBUTE_ALIGNED16(class)
cbtBU_Simplex1to4 : public cbtPolyhedralConvexAabbCachingShape
{
protected:
	int m_numVertices;
	cbtVector3 m_vertices[4];

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtBU_Simplex1to4();

	cbtBU_Simplex1to4(const cbtVector3& pt0);
	cbtBU_Simplex1to4(const cbtVector3& pt0, const cbtVector3& pt1);
	cbtBU_Simplex1to4(const cbtVector3& pt0, const cbtVector3& pt1, const cbtVector3& pt2);
	cbtBU_Simplex1to4(const cbtVector3& pt0, const cbtVector3& pt1, const cbtVector3& pt2, const cbtVector3& pt3);

	void reset()
	{
		m_numVertices = 0;
	}

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	void addVertex(const cbtVector3& pt);

	//PolyhedralConvexShape interface

	virtual int getNumVertices() const;

	virtual int getNumEdges() const;

	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const;

	virtual void getVertex(int i, cbtVector3& vtx) const;

	virtual int getNumPlanes() const;

	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const;

	virtual int getIndex(int i) const;

	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const;

	///getName is for debugging
	virtual const char* getName() const { return "cbtBU_Simplex1to4"; }
};

#endif  //BT_SIMPLEX_1TO4_SHAPE
