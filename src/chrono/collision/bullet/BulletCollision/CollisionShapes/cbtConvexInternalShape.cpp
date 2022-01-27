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

#include "cbtConvexInternalShape.h"

cbtConvexInternalShape::cbtConvexInternalShape()
	: m_localScaling(cbtScalar(1.), cbtScalar(1.), cbtScalar(1.)),
	  m_collisionMargin(CONVEX_DISTANCE_MARGIN)
{
}

void cbtConvexInternalShape::setLocalScaling(const cbtVector3& scaling)
{
	m_localScaling = scaling.absolute();
}

void cbtConvexInternalShape::getAabbSlow(const cbtTransform& trans, cbtVector3& minAabb, cbtVector3& maxAabb) const
{
#ifndef __SPU__
	//use localGetSupportingVertexWithoutMargin?
	cbtScalar margin = getMargin();
	for (int i = 0; i < 3; i++)
	{
		cbtVector3 vec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
		vec[i] = cbtScalar(1.);

		cbtVector3 sv = localGetSupportingVertex(vec * trans.getBasis());

		cbtVector3 tmp = trans(sv);
		maxAabb[i] = tmp[i] + margin;
		vec[i] = cbtScalar(-1.);
		tmp = trans(localGetSupportingVertex(vec * trans.getBasis()));
		minAabb[i] = tmp[i] - margin;
	}
#endif
}

cbtVector3 cbtConvexInternalShape::localGetSupportingVertex(const cbtVector3& vec) const
{
#ifndef __SPU__

	cbtVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

	if (getMargin() != cbtScalar(0.))
	{
		cbtVector3 vecnorm = vec;
		if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
		{
			vecnorm.setValue(cbtScalar(-1.), cbtScalar(-1.), cbtScalar(-1.));
		}
		vecnorm.normalize();
		supVertex += getMargin() * vecnorm;
	}
	return supVertex;

#else
	cbtAssert(0);
	return cbtVector3(0, 0, 0);
#endif  //__SPU__
}

cbtConvexInternalAabbCachingShape::cbtConvexInternalAabbCachingShape()
	: cbtConvexInternalShape(),
	  m_localAabbMin(1, 1, 1),
	  m_localAabbMax(-1, -1, -1),
	  m_isLocalAabbValid(false)
{
}

void cbtConvexInternalAabbCachingShape::getAabb(const cbtTransform& trans, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
}

void cbtConvexInternalAabbCachingShape::setLocalScaling(const cbtVector3& scaling)
{
	cbtConvexInternalShape::setLocalScaling(scaling);
	recalcLocalAabb();
}

void cbtConvexInternalAabbCachingShape::recalcLocalAabb()
{
	m_isLocalAabbValid = true;

#if 1
	static const cbtVector3 _directions[] =
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

	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

	for (int i = 0; i < 3; ++i)
	{
		m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
		m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
	}

#else

	for (int i = 0; i < 3; i++)
	{
		cbtVector3 vec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
		vec[i] = cbtScalar(1.);
		cbtVector3 tmp = localGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i] + m_collisionMargin;
		vec[i] = cbtScalar(-1.);
		tmp = localGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i] - m_collisionMargin;
	}
#endif
}
