/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_AABB_UTIL2
#define BT_AABB_UTIL2

#include "cbtTransform.h"
#include "cbtVector3.h"
#include "cbtMinMax.h"

SIMD_FORCE_INLINE void AabbExpand(cbtVector3& aabbMin,
								  cbtVector3& aabbMax,
								  const cbtVector3& expansionMin,
								  const cbtVector3& expansionMax)
{
	aabbMin = aabbMin + expansionMin;
	aabbMax = aabbMax + expansionMax;
}

/// conservative test for overlap between two aabbs
SIMD_FORCE_INLINE bool TestPointAgainstAabb2(const cbtVector3& aabbMin1, const cbtVector3& aabbMax1,
											 const cbtVector3& point)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
	return overlap;
}

/// conservative test for overlap between two aabbs
SIMD_FORCE_INLINE bool TestAabbAgainstAabb2(const cbtVector3& aabbMin1, const cbtVector3& aabbMax1,
											const cbtVector3& aabbMin2, const cbtVector3& aabbMax2)
{
	bool overlap = true;
	overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false : overlap;
	overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false : overlap;
	overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false : overlap;
	return overlap;
}

/// conservative test for overlap between triangle and aabb
SIMD_FORCE_INLINE bool TestTriangleAgainstAabb2(const cbtVector3* vertices,
												const cbtVector3& aabbMin, const cbtVector3& aabbMax)
{
	const cbtVector3& p1 = vertices[0];
	const cbtVector3& p2 = vertices[1];
	const cbtVector3& p3 = vertices[2];

	if (cbtMin(cbtMin(p1[0], p2[0]), p3[0]) > aabbMax[0]) return false;
	if (cbtMax(cbtMax(p1[0], p2[0]), p3[0]) < aabbMin[0]) return false;

	if (cbtMin(cbtMin(p1[2], p2[2]), p3[2]) > aabbMax[2]) return false;
	if (cbtMax(cbtMax(p1[2], p2[2]), p3[2]) < aabbMin[2]) return false;

	if (cbtMin(cbtMin(p1[1], p2[1]), p3[1]) > aabbMax[1]) return false;
	if (cbtMax(cbtMax(p1[1], p2[1]), p3[1]) < aabbMin[1]) return false;
	return true;
}

SIMD_FORCE_INLINE int cbtOutcode(const cbtVector3& p, const cbtVector3& halfExtent)
{
	return (p.getX() < -halfExtent.getX() ? 0x01 : 0x0) |
		   (p.getX() > halfExtent.getX() ? 0x08 : 0x0) |
		   (p.getY() < -halfExtent.getY() ? 0x02 : 0x0) |
		   (p.getY() > halfExtent.getY() ? 0x10 : 0x0) |
		   (p.getZ() < -halfExtent.getZ() ? 0x4 : 0x0) |
		   (p.getZ() > halfExtent.getZ() ? 0x20 : 0x0);
}

SIMD_FORCE_INLINE bool cbtRayAabb2(const cbtVector3& rayFrom,
								  const cbtVector3& rayInvDirection,
								  const unsigned int raySign[3],
								  const cbtVector3 bounds[2],
								  cbtScalar& tmin,
								  cbtScalar lambda_min,
								  cbtScalar lambda_max)
{
	cbtScalar tmax, tymin, tymax, tzmin, tzmax;
	tmin = (bounds[raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
	tmax = (bounds[1 - raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
	tymin = (bounds[raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();
	tymax = (bounds[1 - raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();

	if ((tmin > tymax) || (tymin > tmax))
		return false;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	tzmin = (bounds[raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();
	tzmax = (bounds[1 - raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;
	return ((tmin < lambda_max) && (tmax > lambda_min));
}

SIMD_FORCE_INLINE bool cbtRayAabb(const cbtVector3& rayFrom,
								 const cbtVector3& rayTo,
								 const cbtVector3& aabbMin,
								 const cbtVector3& aabbMax,
								 cbtScalar& param, cbtVector3& normal)
{
	cbtVector3 aabbHalfExtent = (aabbMax - aabbMin) * cbtScalar(0.5);
	cbtVector3 aabbCenter = (aabbMax + aabbMin) * cbtScalar(0.5);
	cbtVector3 source = rayFrom - aabbCenter;
	cbtVector3 target = rayTo - aabbCenter;
	int sourceOutcode = cbtOutcode(source, aabbHalfExtent);
	int targetOutcode = cbtOutcode(target, aabbHalfExtent);
	if ((sourceOutcode & targetOutcode) == 0x0)
	{
		cbtScalar lambda_enter = cbtScalar(0.0);
		cbtScalar lambda_exit = param;
		cbtVector3 r = target - source;
		int i;
		cbtScalar normSign = 1;
		cbtVector3 hitNormal(0, 0, 0);
		int bit = 1;

		for (int j = 0; j < 2; j++)
		{
			for (i = 0; i != 3; ++i)
			{
				if (sourceOutcode & bit)
				{
					cbtScalar lambda = (-source[i] - aabbHalfExtent[i] * normSign) / r[i];
					if (lambda_enter <= lambda)
					{
						lambda_enter = lambda;
						hitNormal.setValue(0, 0, 0);
						hitNormal[i] = normSign;
					}
				}
				else if (targetOutcode & bit)
				{
					cbtScalar lambda = (-source[i] - aabbHalfExtent[i] * normSign) / r[i];
					cbtSetMin(lambda_exit, lambda);
				}
				bit <<= 1;
			}
			normSign = cbtScalar(-1.);
		}
		if (lambda_enter <= lambda_exit)
		{
			param = lambda_enter;
			normal = hitNormal;
			return true;
		}
	}
	return false;
}

SIMD_FORCE_INLINE void cbtTransformAabb(const cbtVector3& halfExtents, cbtScalar margin, const cbtTransform& t, cbtVector3& aabbMinOut, cbtVector3& aabbMaxOut)
{
	cbtVector3 halfExtentsWithMargin = halfExtents + cbtVector3(margin, margin, margin);
	cbtMatrix3x3 abs_b = t.getBasis().absolute();
	cbtVector3 center = t.getOrigin();
	cbtVector3 extent = halfExtentsWithMargin.dot3(abs_b[0], abs_b[1], abs_b[2]);
	aabbMinOut = center - extent;
	aabbMaxOut = center + extent;
}

SIMD_FORCE_INLINE void cbtTransformAabb(const cbtVector3& localAabbMin, const cbtVector3& localAabbMax, cbtScalar margin, const cbtTransform& trans, cbtVector3& aabbMinOut, cbtVector3& aabbMaxOut)
{
	cbtAssert(localAabbMin.getX() <= localAabbMax.getX());
	cbtAssert(localAabbMin.getY() <= localAabbMax.getY());
	cbtAssert(localAabbMin.getZ() <= localAabbMax.getZ());
	cbtVector3 localHalfExtents = cbtScalar(0.5) * (localAabbMax - localAabbMin);
	localHalfExtents += cbtVector3(margin, margin, margin);

	cbtVector3 localCenter = cbtScalar(0.5) * (localAabbMax + localAabbMin);
	cbtMatrix3x3 abs_b = trans.getBasis().absolute();
	cbtVector3 center = trans(localCenter);
	cbtVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
	aabbMinOut = center - extent;
	aabbMaxOut = center + extent;
}

#define USE_BANCHLESS 1
#ifdef USE_BANCHLESS
//This block replaces the block below and uses no branches, and replaces the 8 bit return with a 32 bit return for improved performance (~3x on XBox 360)
SIMD_FORCE_INLINE unsigned testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1, const unsigned short int* aabbMax1, const unsigned short int* aabbMin2, const unsigned short int* aabbMax2)
{
	return static_cast<unsigned int>(cbtSelect((unsigned)((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0]) & (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2]) & (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])),
											  1, 0));
}
#else
SIMD_FORCE_INLINE bool testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1, const unsigned short int* aabbMax1, const unsigned short int* aabbMin2, const unsigned short int* aabbMax2)
{
	bool overlap = true;
	overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? false : overlap;
	overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? false : overlap;
	overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? false : overlap;
	return overlap;
}
#endif  //USE_BANCHLESS

#endif  //BT_AABB_UTIL2
