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
#ifndef BT_SPHERE_MINKOWSKI_H
#define BT_SPHERE_MINKOWSKI_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types

///The cbtSphereShape implements an implicit sphere, centered around a local origin with radius.
ATTRIBUTE_ALIGNED16(class)
cbtSphereShape : public cbtConvexInternalShape

{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtSphereShape(cbtScalar radius) : cbtConvexInternalShape()
	{
		m_shapeType = SPHERE_SHAPE_PROXYTYPE;
		m_localScaling.setValue(1.0, 1.0, 1.0);
		m_implicitShapeDimensions.setZero();
		m_implicitShapeDimensions.setX(radius);
		m_collisionMargin = radius;
		m_padding = 0;
	}

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	//notice that the vectors should be unit length
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	cbtScalar getRadius() const { return m_implicitShapeDimensions.getX() * m_localScaling.getX(); }

	void setUnscaledRadius(cbtScalar radius)
	{
		m_implicitShapeDimensions.setX(radius);
		cbtConvexInternalShape::setMargin(radius);
	}

	//debugging
	virtual const char* getName() const { return "SPHERE"; }

	virtual void setMargin(cbtScalar margin)
	{
		cbtConvexInternalShape::setMargin(margin);
	}
	virtual cbtScalar getMargin() const
	{
		//to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		//this means, non-uniform scaling is not supported anymore
		return getRadius();
	}
};

#endif  //BT_SPHERE_MINKOWSKI_H
