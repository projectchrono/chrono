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

#include "cbtSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"

#include "LinearMath/cbtQuaternion.h"

cbtVector3 cbtSphereShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
{
	(void)vec;
	return cbtVector3(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
}

void cbtSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	(void)vectors;

	for (int i = 0; i < numVectors; i++)
	{
		supportVerticesOut[i].setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	}
}

cbtVector3 cbtSphereShape::localGetSupportingVertex(const cbtVector3& vec) const
{
	cbtVector3 supVertex;
	supVertex = localGetSupportingVertexWithoutMargin(vec);

	cbtVector3 vecnorm = vec;
	if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
	{
		vecnorm.setValue(cbtScalar(-1.), cbtScalar(-1.), cbtScalar(-1.));
	}
	vecnorm.normalize();
	supVertex += getMargin() * vecnorm;
	return supVertex;
}

//broken due to scaling
void cbtSphereShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	const cbtVector3& center = t.getOrigin();
	cbtVector3 extent(getMargin(), getMargin(), getMargin());
	aabbMin = center - extent;
	aabbMax = center + extent;
}

void cbtSphereShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	cbtScalar elem = cbtScalar(0.4) * mass * getMargin() * getMargin();
	inertia.setValue(elem, elem, elem);
}
