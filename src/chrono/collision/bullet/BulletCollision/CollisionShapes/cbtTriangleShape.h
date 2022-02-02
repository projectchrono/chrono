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

#ifndef BT_OBB_TRIANGLE_MINKOWSKI_H
#define BT_OBB_TRIANGLE_MINKOWSKI_H

#include "cbtConvexShape.h"
#include "cbtBoxShape.h"

ATTRIBUTE_ALIGNED16(class)
cbtTriangleShape : public cbtPolyhedralConvexShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtVector3 m_vertices1[3];

	virtual int getNumVertices() const
	{
		return 3;
	}

	cbtVector3& getVertexPtr(int index)
	{
		return m_vertices1[index];
	}

	const cbtVector3& getVertexPtr(int index) const
	{
		return m_vertices1[index];
	}
	virtual void getVertex(int index, cbtVector3& vert) const
	{
		vert = m_vertices1[index];
	}

	virtual int getNumEdges() const
	{
		return 3;
	}

	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const
	{
		getVertex(i, pa);
		getVertex((i + 1) % 3, pb);
	}

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
	{
		//		cbtAssert(0);
		getAabbSlow(t, aabbMin, aabbMax);
	}

	cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& dir) const
	{
		cbtVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
		return m_vertices1[dots.maxAxis()];
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
	{
		for (int i = 0; i < numVectors; i++)
		{
			const cbtVector3& dir = vectors[i];
			cbtVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
			supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
		}
	}

	cbtTriangleShape() : cbtPolyhedralConvexShape()
	{
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
	}

	cbtTriangleShape(const cbtVector3& p0, const cbtVector3& p1, const cbtVector3& p2) : cbtPolyhedralConvexShape()
	{
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
		m_vertices1[0] = p0;
		m_vertices1[1] = p1;
		m_vertices1[2] = p2;
	}

	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const
	{
		getPlaneEquation(i, planeNormal, planeSupport);
	}

	virtual int getNumPlanes() const
	{
		return 1;
	}

	void calcNormal(cbtVector3 & normal) const
	{
		normal = (m_vertices1[1] - m_vertices1[0]).cross(m_vertices1[2] - m_vertices1[0]);
		normal.normalize();
	}

	virtual void getPlaneEquation(int i, cbtVector3& planeNormal, cbtVector3& planeSupport) const
	{
		(void)i;
		calcNormal(planeNormal);
		planeSupport = m_vertices1[0];
	}

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const
	{
		(void)mass;
		cbtAssert(0);
		inertia.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	}

	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const
	{
		cbtVector3 normal;
		calcNormal(normal);
		//distance to plane
		cbtScalar dist = pt.dot(normal);
		cbtScalar planeconst = m_vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance)
		{
			//inside check on edge-planes
			int i;
			for (i = 0; i < 3; i++)
			{
				cbtVector3 pa, pb;
				getEdge(i, pa, pb);
				cbtVector3 edge = pb - pa;
				cbtVector3 edgeNormal = edge.cross(normal);
				edgeNormal.normalize();
				cbtScalar dist = pt.dot(edgeNormal);
				cbtScalar edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance)
					return false;
			}

			return true;
		}

		return false;
	}
	//debugging
	virtual const char* getName() const
	{
		return "Triangle";
	}

	virtual int getNumPreferredPenetrationDirections() const
	{
		return 2;
	}

	virtual void getPreferredPenetrationDirection(int index, cbtVector3& penetrationVector) const
	{
		calcNormal(penetrationVector);
		if (index)
			penetrationVector *= cbtScalar(-1.);
	}
};

#endif  //BT_OBB_TRIANGLE_MINKOWSKI_H
