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

#include "cbtMinkowskiSumShape.h"

cbtMinkowskiSumShape::cbtMinkowskiSumShape(const cbtConvexShape* shapeA, const cbtConvexShape* shapeB)
	: cbtConvexInternalShape(),
	  m_shapeA(shapeA),
	  m_shapeB(shapeB)
{
	m_shapeType = MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE;
	m_transA.setIdentity();
	m_transB.setIdentity();
}

cbtVector3 cbtMinkowskiSumShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
{
	cbtVector3 supVertexA = m_transA(m_shapeA->localGetSupportingVertexWithoutMargin(vec * m_transA.getBasis()));
	cbtVector3 supVertexB = m_transB(m_shapeB->localGetSupportingVertexWithoutMargin(-vec * m_transB.getBasis()));
	return supVertexA - supVertexB;
}

void cbtMinkowskiSumShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	///@todo: could make recursive use of batching. probably this shape is not used frequently.
	for (int i = 0; i < numVectors; i++)
	{
		supportVerticesOut[i] = localGetSupportingVertexWithoutMargin(vectors[i]);
	}
}

cbtScalar cbtMinkowskiSumShape::getMargin() const
{
	return m_shapeA->getMargin() + m_shapeB->getMargin();
}

void cbtMinkowskiSumShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	(void)mass;
	//inertia of the AABB of the Minkowski sum
	cbtTransform identity;
	identity.setIdentity();
	cbtVector3 aabbMin, aabbMax;
	getAabb(identity, aabbMin, aabbMax);

	cbtVector3 halfExtents = (aabbMax - aabbMin) * cbtScalar(0.5);

	cbtScalar margin = getMargin();

	cbtScalar lx = cbtScalar(2.) * (halfExtents.x() + margin);
	cbtScalar ly = cbtScalar(2.) * (halfExtents.y() + margin);
	cbtScalar lz = cbtScalar(2.) * (halfExtents.z() + margin);
	const cbtScalar x2 = lx * lx;
	const cbtScalar y2 = ly * ly;
	const cbtScalar z2 = lz * lz;
	const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

	inertia = scaledmass * (cbtVector3(y2 + z2, x2 + z2, x2 + y2));
}
