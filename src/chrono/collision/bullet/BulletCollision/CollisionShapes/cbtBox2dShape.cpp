/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "cbtBox2dShape.h"

//{

void cbtBox2dShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	cbtTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
}

void cbtBox2dShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	//cbtScalar margin = cbtScalar(0.);
	cbtVector3 halfExtents = getHalfExtentsWithMargin();

	cbtScalar lx = cbtScalar(2.) * (halfExtents.x());
	cbtScalar ly = cbtScalar(2.) * (halfExtents.y());
	cbtScalar lz = cbtScalar(2.) * (halfExtents.z());

	inertia.setValue(mass / (cbtScalar(12.0)) * (ly * ly + lz * lz),
					 mass / (cbtScalar(12.0)) * (lx * lx + lz * lz),
					 mass / (cbtScalar(12.0)) * (lx * lx + ly * ly));
}
