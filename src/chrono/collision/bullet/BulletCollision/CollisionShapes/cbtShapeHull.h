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

///cbtShapeHull implemented by John McCutchan.

#ifndef BT_SHAPE_HULL_H
#define BT_SHAPE_HULL_H

#include "LinearMath/cbtAlignedObjectArray.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"

///The cbtShapeHull class takes a cbtConvexShape, builds a simplified convex hull using cbtConvexHull and provides triangle indices and vertices.
///It can be useful for to simplify a complex convex object and for visualization of a non-polyhedral convex object.
///It approximates the convex hull using the supporting vertex of 42 directions.
ATTRIBUTE_ALIGNED16(class)
cbtShapeHull
{
protected:
	cbtAlignedObjectArray<cbtVector3> m_vertices;
	cbtAlignedObjectArray<unsigned int> m_indices;
	unsigned int m_numIndices;
	const cbtConvexShape* m_shape;

	static cbtVector3* getUnitSpherePoints(int highres = 0);

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtShapeHull(const cbtConvexShape* shape);
	~cbtShapeHull();

	bool buildHull(cbtScalar margin, int highres = 0);

	int numTriangles() const;
	int numVertices() const;
	int numIndices() const;

	const cbtVector3* getVertexPointer() const
	{
		return &m_vertices[0];
	}
	const unsigned int* getIndexPointer() const
	{
		return &m_indices[0];
	}
};

#endif  //BT_SHAPE_HULL_H
