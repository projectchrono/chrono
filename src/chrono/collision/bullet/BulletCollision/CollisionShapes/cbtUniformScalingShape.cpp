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

#include "cbtUniformScalingShape.h"

cbtUniformScalingShape::cbtUniformScalingShape(cbtConvexShape* convexChildShape, cbtScalar uniformScalingFactor) : cbtConvexShape(), m_childConvexShape(convexChildShape), m_uniformScalingFactor(uniformScalingFactor)
{
	m_shapeType = UNIFORM_SCALING_SHAPE_PROXYTYPE;
}

cbtUniformScalingShape::~cbtUniformScalingShape()
{
}

cbtVector3 cbtUniformScalingShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
{
	cbtVector3 tmpVertex;
	tmpVertex = m_childConvexShape->localGetSupportingVertexWithoutMargin(vec);
	return tmpVertex * m_uniformScalingFactor;
}

void cbtUniformScalingShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	m_childConvexShape->batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
	int i;
	for (i = 0; i < numVectors; i++)
	{
		supportVerticesOut[i] = supportVerticesOut[i] * m_uniformScalingFactor;
	}
}

cbtVector3 cbtUniformScalingShape::localGetSupportingVertex(const cbtVector3& vec) const
{
	cbtVector3 tmpVertex;
	tmpVertex = m_childConvexShape->localGetSupportingVertex(vec);
	return tmpVertex * m_uniformScalingFactor;
}

void cbtUniformScalingShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	///this linear upscaling is not realistic, but we don't deal with large mass ratios...
	cbtVector3 tmpInertia;
	m_childConvexShape->calculateLocalInertia(mass, tmpInertia);
	inertia = tmpInertia * m_uniformScalingFactor;
}

///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
void cbtUniformScalingShape::getAabb(const cbtTransform& trans, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	getAabbSlow(trans, aabbMin, aabbMax);
}

void cbtUniformScalingShape::getAabbSlow(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
#if 1
	cbtVector3 _directions[] =
		{
			cbtVector3(1., 0., 0.),
			cbtVector3(0., 1., 0.),
			cbtVector3(0., 0., 1.),
			cbtVector3(-1., 0., 0.),
			cbtVector3(0., -1., 0.),
			cbtVector3(0., 0., -1.)};

	cbtVector3 _supporting[] =
		{
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.)};

	for (int i = 0; i < 6; i++)
	{
		_directions[i] = _directions[i] * t.getBasis();
	}

	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

	cbtVector3 aabbMin1(0, 0, 0), aabbMax1(0, 0, 0);

	for (int i = 0; i < 3; ++i)
	{
		aabbMax1[i] = t(_supporting[i])[i];
		aabbMin1[i] = t(_supporting[i + 3])[i];
	}
	cbtVector3 marginVec(getMargin(), getMargin(), getMargin());
	aabbMin = aabbMin1 - marginVec;
	aabbMax = aabbMax1 + marginVec;

#else

	cbtScalar margin = getMargin();
	for (int i = 0; i < 3; i++)
	{
		cbtVector3 vec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
		vec[i] = cbtScalar(1.);
		cbtVector3 sv = localGetSupportingVertex(vec * t.getBasis());
		cbtVector3 tmp = t(sv);
		aabbMax[i] = tmp[i] + margin;
		vec[i] = cbtScalar(-1.);
		sv = localGetSupportingVertex(vec * t.getBasis());
		tmp = t(sv);
		aabbMin[i] = tmp[i] - margin;
	}

#endif
}

void cbtUniformScalingShape::setLocalScaling(const cbtVector3& scaling)
{
	m_childConvexShape->setLocalScaling(scaling);
}

const cbtVector3& cbtUniformScalingShape::getLocalScaling() const
{
	return m_childConvexShape->getLocalScaling();
}

void cbtUniformScalingShape::setMargin(cbtScalar margin)
{
	m_childConvexShape->setMargin(margin);
}
cbtScalar cbtUniformScalingShape::getMargin() const
{
	return m_childConvexShape->getMargin() * m_uniformScalingFactor;
}

int cbtUniformScalingShape::getNumPreferredPenetrationDirections() const
{
	return m_childConvexShape->getNumPreferredPenetrationDirections();
}

void cbtUniformScalingShape::getPreferredPenetrationDirection(int index, cbtVector3& penetrationVector) const
{
	m_childConvexShape->getPreferredPenetrationDirection(index, penetrationVector);
}
