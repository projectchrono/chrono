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

#include "cbtStaticPlaneShape.h"

#include "LinearMath/cbtTransformUtil.h"

cbtStaticPlaneShape::cbtStaticPlaneShape(const cbtVector3& planeNormal, cbtScalar planeConstant)
	: cbtConcaveShape(), m_planeNormal(planeNormal.normalized()), m_planeConstant(planeConstant), m_localScaling(cbtScalar(1.), cbtScalar(1.), cbtScalar(1.))
{
	m_shapeType = STATIC_PLANE_PROXYTYPE;
	//	cbtAssert( cbtFuzzyZero(m_planeNormal.length() - cbtScalar(1.)) );
}

cbtStaticPlaneShape::~cbtStaticPlaneShape()
{
}

void cbtStaticPlaneShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	(void)t;
	/*
	cbtVector3 infvec (cbtScalar(BT_LARGE_FLOAT),cbtScalar(BT_LARGE_FLOAT),cbtScalar(BT_LARGE_FLOAT));

	cbtVector3 center = m_planeNormal*m_planeConstant;
	aabbMin = center + infvec*m_planeNormal;
	aabbMax = aabbMin;
	aabbMin.setMin(center - infvec*m_planeNormal);
	aabbMax.setMax(center - infvec*m_planeNormal); 
	*/

	aabbMin.setValue(cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT));
	aabbMax.setValue(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
}

void cbtStaticPlaneShape::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
	cbtVector3 halfExtents = (aabbMax - aabbMin) * cbtScalar(0.5);
	cbtScalar radius = halfExtents.length();
	cbtVector3 center = (aabbMax + aabbMin) * cbtScalar(0.5);

	//this is where the triangles are generated, given AABB and plane equation (normal/constant)

	cbtVector3 tangentDir0, tangentDir1;

	//tangentDir0/tangentDir1 can be precalculated
	cbtPlaneSpace1(m_planeNormal, tangentDir0, tangentDir1);

	cbtVector3 projectedCenter = center - (m_planeNormal.dot(center) - m_planeConstant) * m_planeNormal;

	cbtVector3 triangle[3];
	triangle[0] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
	triangle[1] = projectedCenter + tangentDir0 * radius - tangentDir1 * radius;
	triangle[2] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;

	callback->processTriangle(triangle, 0, 0);

	triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
	triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius;
	triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;

	callback->processTriangle(triangle, 0, 1);
}

void cbtStaticPlaneShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	(void)mass;

	//moving concave objects not supported

	inertia.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
}

void cbtStaticPlaneShape::setLocalScaling(const cbtVector3& scaling)
{
	m_localScaling = scaling;
}
const cbtVector3& cbtStaticPlaneShape::getLocalScaling() const
{
	return m_localScaling;
}
