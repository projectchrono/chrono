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

#ifndef BT_UNIFORM_SCALING_SHAPE_H
#define BT_UNIFORM_SCALING_SHAPE_H

#include "cbtConvexShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types

///The cbtUniformScalingShape allows to re-use uniform scaled instances of cbtConvexShape in a memory efficient way.
///Istead of using cbtUniformScalingShape, it is better to use the non-uniform setLocalScaling method on convex shapes that implement it.
ATTRIBUTE_ALIGNED16(class)
cbtUniformScalingShape : public cbtConvexShape
{
	cbtConvexShape* m_childConvexShape;

	cbtScalar m_uniformScalingFactor;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtUniformScalingShape(cbtConvexShape * convexChildShape, cbtScalar uniformScalingFactor);

	virtual ~cbtUniformScalingShape();

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	cbtScalar getUniformScalingFactor() const
	{
		return m_uniformScalingFactor;
	}

	cbtConvexShape* getChildShape()
	{
		return m_childConvexShape;
	}

	const cbtConvexShape* getChildShape() const
	{
		return m_childConvexShape;
	}

	virtual const char* getName() const
	{
		return "UniformScalingShape";
	}

	///////////////////////////

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void getAabbSlow(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void setLocalScaling(const cbtVector3& scaling);
	virtual const cbtVector3& getLocalScaling() const;

	virtual void setMargin(cbtScalar margin);
	virtual cbtScalar getMargin() const;

	virtual int getNumPreferredPenetrationDirections() const;

	virtual void getPreferredPenetrationDirection(int index, cbtVector3& penetrationVector) const;
};

#endif  //BT_UNIFORM_SCALING_SHAPE_H
