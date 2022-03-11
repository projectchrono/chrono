/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_OBB_BOX_2D_SHAPE_H
#define BT_OBB_BOX_2D_SHAPE_H

#include "BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtMinMax.h"

///The cbtBox2dShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a cbtCollisionObject or cbtRigidBody it will be an oriented box in world space.
ATTRIBUTE_ALIGNED16(class)
cbtBox2dShape : public cbtPolyhedralConvexShape
{
	//cbtVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead

	cbtVector3 m_centroid;
	cbtVector3 m_vertices[4];
	cbtVector3 m_normals[4];

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

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const
	{
		cbtVector3 halfExtents = getHalfExtentsWithoutMargin();
		cbtVector3 margin(getMargin(), getMargin(), getMargin());
		halfExtents += margin;

		return cbtVector3(cbtFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
						 cbtFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
						 cbtFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	SIMD_FORCE_INLINE cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
	{
		const cbtVector3& halfExtents = getHalfExtentsWithoutMargin();

		return cbtVector3(cbtFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
						 cbtFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
						 cbtFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
	{
		const cbtVector3& halfExtents = getHalfExtentsWithoutMargin();

		for (int i = 0; i < numVectors; i++)
		{
			const cbtVector3& vec = vectors[i];
			supportVerticesOut[i].setValue(cbtFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
										   cbtFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
										   cbtFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
		}
	}

	///a cbtBox2dShape is a flat 2D box in the X-Y plane (Z extents are zero)
	cbtBox2dShape(const cbtVector3& boxHalfExtents)
		: cbtPolyhedralConvexShape(),
		  m_centroid(0, 0, 0)
	{
		m_vertices[0].setValue(-boxHalfExtents.getX(), -boxHalfExtents.getY(), 0);
		m_vertices[1].setValue(boxHalfExtents.getX(), -boxHalfExtents.getY(), 0);
		m_vertices[2].setValue(boxHalfExtents.getX(), boxHalfExtents.getY(), 0);
		m_vertices[3].setValue(-boxHalfExtents.getX(), boxHalfExtents.getY(), 0);

		m_normals[0].setValue(0, -1, 0);
		m_normals[1].setValue(1, 0, 0);
		m_normals[2].setValue(0, 1, 0);
		m_normals[3].setValue(-1, 0, 0);

		cbtScalar minDimension = boxHalfExtents.getX();
		if (minDimension > boxHalfExtents.getY())
			minDimension = boxHalfExtents.getY();

		m_shapeType = BOX_2D_SHAPE_PROXYTYPE;
		cbtVector3 margin(getMargin(), getMargin(), getMargin());
		m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;

		setSafeMargin(minDimension);
	};

	virtual void setMargin(cbtScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
		cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

		cbtConvexInternalShape::setMargin(collisionMargin);
		cbtVector3 newMargin(getMargin(), getMargin(), getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	}
	virtual void setLocalScaling(const cbtVector3& scaling)
	{
		cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
		cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
		cbtVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		cbtConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
	}

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	int getVertexCount() const
	{
		return 4;
	}

	virtual int getNumVertices() const
	{
		return 4;
	}

	const cbtVector3* getVertices() const
	{
		return &m_vertices[0];
	}

	const cbtVector3* getNormals() const
	{
		return &m_normals[0];
	}

	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const
	{
		//this plane might not be aligned...
		cbtVector4 plane;
		getPlaneEquation(plane, i);
		planeNormal = cbtVector3(plane.getX(), plane.getY(), plane.getZ());
		planeSupport = localGetSupportingVertex(-planeNormal);
	}

	const cbtVector3& getCentroid() const
	{
		return m_centroid;
	}

	virtual int getNumPlanes() const
	{
		return 6;
	}

	virtual int getNumEdges() const
	{
		return 12;
	}

	virtual void getVertex(int i, cbtVector3& vtx) const
	{
		cbtVector3 halfExtents = getHalfExtentsWithoutMargin();

		vtx = cbtVector3(
			halfExtents.x() * (1 - (i & 1)) - halfExtents.x() * (i & 1),
			halfExtents.y() * (1 - ((i & 2) >> 1)) - halfExtents.y() * ((i & 2) >> 1),
			halfExtents.z() * (1 - ((i & 4) >> 2)) - halfExtents.z() * ((i & 4) >> 2));
	}

	virtual void getPlaneEquation(cbtVector4 & plane, int i) const
	{
		cbtVector3 halfExtents = getHalfExtentsWithoutMargin();

		switch (i)
		{
			case 0:
				plane.setValue(cbtScalar(1.), cbtScalar(0.), cbtScalar(0.), -halfExtents.x());
				break;
			case 1:
				plane.setValue(cbtScalar(-1.), cbtScalar(0.), cbtScalar(0.), -halfExtents.x());
				break;
			case 2:
				plane.setValue(cbtScalar(0.), cbtScalar(1.), cbtScalar(0.), -halfExtents.y());
				break;
			case 3:
				plane.setValue(cbtScalar(0.), cbtScalar(-1.), cbtScalar(0.), -halfExtents.y());
				break;
			case 4:
				plane.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(1.), -halfExtents.z());
				break;
			case 5:
				plane.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(-1.), -halfExtents.z());
				break;
			default:
				cbtAssert(0);
		}
	}

	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const
	//virtual void getEdge(int i,Edge& edge) const
	{
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i)
		{
			case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
				break;
			case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
				break;
			case 2:
				edgeVert0 = 1;
				edgeVert1 = 3;

				break;
			case 3:
				edgeVert0 = 2;
				edgeVert1 = 3;
				break;
			case 4:
				edgeVert0 = 0;
				edgeVert1 = 4;
				break;
			case 5:
				edgeVert0 = 1;
				edgeVert1 = 5;

				break;
			case 6:
				edgeVert0 = 2;
				edgeVert1 = 6;
				break;
			case 7:
				edgeVert0 = 3;
				edgeVert1 = 7;
				break;
			case 8:
				edgeVert0 = 4;
				edgeVert1 = 5;
				break;
			case 9:
				edgeVert0 = 4;
				edgeVert1 = 6;
				break;
			case 10:
				edgeVert0 = 5;
				edgeVert1 = 7;
				break;
			case 11:
				edgeVert0 = 6;
				edgeVert1 = 7;
				break;
			default:
				cbtAssert(0);
		}

		getVertex(edgeVert0, pa);
		getVertex(edgeVert1, pb);
	}

	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const
	{
		cbtVector3 halfExtents = getHalfExtentsWithoutMargin();

		//cbtScalar minDist = 2*tolerance;

		bool result = (pt.x() <= (halfExtents.x() + tolerance)) &&
					  (pt.x() >= (-halfExtents.x() - tolerance)) &&
					  (pt.y() <= (halfExtents.y() + tolerance)) &&
					  (pt.y() >= (-halfExtents.y() - tolerance)) &&
					  (pt.z() <= (halfExtents.z() + tolerance)) &&
					  (pt.z() >= (-halfExtents.z() - tolerance));

		return result;
	}

	//debugging
	virtual const char* getName() const
	{
		return "Box2d";
	}

	virtual int getNumPreferredPenetrationDirections() const
	{
		return 6;
	}

	virtual void getPreferredPenetrationDirection(int index, cbtVector3& penetrationVector) const
	{
		switch (index)
		{
			case 0:
				penetrationVector.setValue(cbtScalar(1.), cbtScalar(0.), cbtScalar(0.));
				break;
			case 1:
				penetrationVector.setValue(cbtScalar(-1.), cbtScalar(0.), cbtScalar(0.));
				break;
			case 2:
				penetrationVector.setValue(cbtScalar(0.), cbtScalar(1.), cbtScalar(0.));
				break;
			case 3:
				penetrationVector.setValue(cbtScalar(0.), cbtScalar(-1.), cbtScalar(0.));
				break;
			case 4:
				penetrationVector.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(1.));
				break;
			case 5:
				penetrationVector.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(-1.));
				break;
			default:
				cbtAssert(0);
		}
	}
};

#endif  //BT_OBB_BOX_2D_SHAPE_H
