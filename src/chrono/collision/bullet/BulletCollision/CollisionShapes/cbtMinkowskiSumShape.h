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

#ifndef BT_MINKOWSKI_SUM_SHAPE_H
#define BT_MINKOWSKI_SUM_SHAPE_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types

/// The cbtMinkowskiSumShape is only for advanced users. This shape represents implicit based minkowski sum of two convex implicit shapes.
ATTRIBUTE_ALIGNED16(class)
cbtMinkowskiSumShape : public cbtConvexInternalShape
{
	cbtTransform m_transA;
	cbtTransform m_transB;
	const cbtConvexShape* m_shapeA;
	const cbtConvexShape* m_shapeB;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtMinkowskiSumShape(const cbtConvexShape* shapeA, const cbtConvexShape* shapeB);

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	void setTransformA(const cbtTransform& transA) { m_transA = transA; }
	void setTransformB(const cbtTransform& transB) { m_transB = transB; }

	const cbtTransform& getTransformA() const { return m_transA; }
	const cbtTransform& GetTransformB() const { return m_transB; }

	virtual cbtScalar getMargin() const;

	const cbtConvexShape* getShapeA() const { return m_shapeA; }
	const cbtConvexShape* getShapeB() const { return m_shapeB; }

	virtual const char* getName() const
	{
		return "MinkowskiSum";
	}
};

#endif  //BT_MINKOWSKI_SUM_SHAPE_H
