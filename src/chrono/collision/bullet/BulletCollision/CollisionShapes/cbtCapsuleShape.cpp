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

#include "cbtCapsuleShape.h"

#include "LinearMath/cbtQuaternion.h"

cbtCapsuleShape::cbtCapsuleShape(cbtScalar radius, cbtScalar height) : cbtConvexInternalShape()
{
	m_collisionMargin = radius;
	m_shapeType = CAPSULE_SHAPE_PROXYTYPE;
	m_upAxis = 1;
	m_implicitShapeDimensions.setValue(radius, 0.5f * height, radius);
}

cbtVector3 cbtCapsuleShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const
{
	cbtVector3 supVec(0, 0, 0);

	cbtScalar maxDot(cbtScalar(-BT_LARGE_FLOAT));

	cbtVector3 vec = vec0;
	cbtScalar lenSqr = vec.length2();
	if (lenSqr < cbtScalar(0.0001))
	{
		vec.setValue(1, 0, 0);
	}
	else
	{
		cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
		vec *= rlen;
	}

	cbtVector3 vtx;
	cbtScalar newDot;

	{
		cbtVector3 pos(0, 0, 0);
		pos[getUpAxis()] = getHalfHeight();

		vtx = pos;
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}
	{
		cbtVector3 pos(0, 0, 0);
		pos[getUpAxis()] = -getHalfHeight();

		vtx = pos;
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	return supVec;
}

void cbtCapsuleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	for (int j = 0; j < numVectors; j++)
	{
		cbtScalar maxDot(cbtScalar(-BT_LARGE_FLOAT));
		const cbtVector3& vec = vectors[j];

		cbtVector3 vtx;
		cbtScalar newDot;
		{
			cbtVector3 pos(0, 0, 0);
			pos[getUpAxis()] = getHalfHeight();
			vtx = pos;
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
		{
			cbtVector3 pos(0, 0, 0);
			pos[getUpAxis()] = -getHalfHeight();
			vtx = pos;
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
	}
}

void cbtCapsuleShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	cbtTransform ident;
	ident.setIdentity();

	cbtScalar radius = getRadius();

	cbtVector3 halfExtents(radius, radius, radius);
	halfExtents[getUpAxis()] += getHalfHeight();

	cbtScalar lx = cbtScalar(2.) * (halfExtents[0]);
	cbtScalar ly = cbtScalar(2.) * (halfExtents[1]);
	cbtScalar lz = cbtScalar(2.) * (halfExtents[2]);
	const cbtScalar x2 = lx * lx;
	const cbtScalar y2 = ly * ly;
	const cbtScalar z2 = lz * lz;
	const cbtScalar scaledmass = mass * cbtScalar(.08333333);

	inertia[0] = scaledmass * (y2 + z2);
	inertia[1] = scaledmass * (x2 + z2);
	inertia[2] = scaledmass * (x2 + y2);
}

cbtCapsuleShapeX::cbtCapsuleShapeX(cbtScalar radius, cbtScalar height)
{
	m_collisionMargin = radius;
	m_upAxis = 0;
	m_implicitShapeDimensions.setValue(0.5f * height, radius, radius);
}

cbtCapsuleShapeZ::cbtCapsuleShapeZ(cbtScalar radius, cbtScalar height)
{
	m_collisionMargin = radius;
	m_upAxis = 2;
	m_implicitShapeDimensions.setValue(radius, radius, 0.5f * height);
}
