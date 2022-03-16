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

#ifndef BT_CONCAVE_SHAPE_H
#define BT_CONCAVE_SHAPE_H

#include "cbtCollisionShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "cbtTriangleCallback.h"

/// PHY_ScalarType enumerates possible scalar types.
/// See the cbtStridingMeshInterface or cbtHeightfieldTerrainShape for its use
typedef enum PHY_ScalarType
{
	PHY_FLOAT,
	PHY_DOUBLE,
	PHY_INTEGER,
	PHY_SHORT,
	PHY_FIXEDPOINT88,
	PHY_UCHAR
} PHY_ScalarType;

///The cbtConcaveShape class provides an interface for non-moving (static) concave shapes.
///It has been implemented by the cbtStaticPlaneShape, cbtBvhTriangleMeshShape and cbtHeightfieldTerrainShape.
ATTRIBUTE_ALIGNED16(class)
cbtConcaveShape : public cbtCollisionShape
{
protected:
	cbtScalar m_collisionMargin;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConcaveShape();

	virtual ~cbtConcaveShape();

	virtual void processAllTriangles(cbtTriangleCallback * callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const = 0;

	virtual cbtScalar getMargin() const
	{
		return m_collisionMargin;
	}
	virtual void setMargin(cbtScalar collisionMargin)
	{
		m_collisionMargin = collisionMargin;
	}
};

#endif  //BT_CONCAVE_SHAPE_H
