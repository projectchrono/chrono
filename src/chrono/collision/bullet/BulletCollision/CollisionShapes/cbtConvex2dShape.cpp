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

#include "cbtConvex2dShape.h"

cbtConvex2dShape::cbtConvex2dShape(cbtConvexShape* convexChildShape) : cbtConvexShape(), m_childConvexShape(convexChildShape)
{
	m_shapeType = CONVEX_2D_SHAPE_PROXYTYPE;
}

cbtConvex2dShape::~cbtConvex2dShape()
{
}

cbtVector3 cbtConvex2dShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
{
	return m_childConvexShape->localGetSupportingVertexWithoutMargin(vec);
}

void cbtConvex2dShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	m_childConvexShape->batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
}

cbtVector3 cbtConvex2dShape::localGetSupportingVertex(const cbtVector3& vec) const
{
	return m_childConvexShape->localGetSupportingVertex(vec);
}

void cbtConvex2dShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	///this linear upscaling is not realistic, but we don't deal with large mass ratios...
	m_childConvexShape->calculateLocalInertia(mass, inertia);
}

///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
void cbtConvex2dShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	m_childConvexShape->getAabb(t, aabbMin, aabbMax);
}

void cbtConvex2dShape::getAabbSlow(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	m_childConvexShape->getAabbSlow(t, aabbMin, aabbMax);
}

void cbtConvex2dShape::setLocalScaling(const cbtVector3& scaling)
{
	m_childConvexShape->setLocalScaling(scaling);
}

const cbtVector3& cbtConvex2dShape::getLocalScaling() const
{
	return m_childConvexShape->getLocalScaling();
}

void cbtConvex2dShape::setMargin(cbtScalar margin)
{
	m_childConvexShape->setMargin(margin);
}
cbtScalar cbtConvex2dShape::getMargin() const
{
	return m_childConvexShape->getMargin();
}

int cbtConvex2dShape::getNumPreferredPenetrationDirections() const
{
	return m_childConvexShape->getNumPreferredPenetrationDirections();
}

void cbtConvex2dShape::getPreferredPenetrationDirection(int index, cbtVector3& penetrationVector) const
{
	m_childConvexShape->getPreferredPenetrationDirection(index, penetrationVector);
}
