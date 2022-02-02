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

#include "LinearMath/cbtScalar.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionShapes/cbtTriangleShape.h"
#include "BulletCollision/CollisionShapes/cbtSphereShape.h"

SphereTriangleDetector::SphereTriangleDetector(cbtSphereShape* sphere, cbtTriangleShape* triangle, cbtScalar contactBreakingThreshold)
	: m_sphere(sphere),
	  m_triangle(triangle),
	  m_contactBreakingThreshold(contactBreakingThreshold)
{
}

void SphereTriangleDetector::getClosestPoints(const ClosestPointInput& input, Result& output, class cbtIDebugDraw* debugDraw, bool swapResults)
{
	(void)debugDraw;
	const cbtTransform& transformA = input.m_transformA;
	const cbtTransform& transformB = input.m_transformB;

	cbtVector3 point, normal;
	cbtScalar timeOfImpact = cbtScalar(1.);
	cbtScalar depth = cbtScalar(0.);
	//	output.m_distance = cbtScalar(BT_LARGE_FLOAT);
	//move sphere into triangle space
	cbtTransform sphereInTr = transformB.inverseTimes(transformA);

	if (collide(sphereInTr.getOrigin(), point, normal, depth, timeOfImpact, m_contactBreakingThreshold))
	{
		if (swapResults)
		{
			cbtVector3 normalOnB = transformB.getBasis() * normal;
			cbtVector3 normalOnA = -normalOnB;
			cbtVector3 pointOnA = transformB * point + normalOnB * depth;
			output.addContactPoint(normalOnA, pointOnA, depth);
		}
		else
		{
			output.addContactPoint(transformB.getBasis() * normal, transformB * point, depth);
		}
	}
}

// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
cbtScalar SegmentSqrDistance(const cbtVector3& from, const cbtVector3& to, const cbtVector3& p, cbtVector3& nearest);

cbtScalar SegmentSqrDistance(const cbtVector3& from, const cbtVector3& to, const cbtVector3& p, cbtVector3& nearest)
{
	cbtVector3 diff = p - from;
	cbtVector3 v = to - from;
	cbtScalar t = v.dot(diff);

	if (t > 0)
	{
		cbtScalar dotVV = v.dot(v);
		if (t < dotVV)
		{
			t /= dotVV;
			diff -= t * v;
		}
		else
		{
			t = 1;
			diff -= v;
		}
	}
	else
		t = 0;

	nearest = from + t * v;
	return diff.dot(diff);
}

bool SphereTriangleDetector::facecontains(const cbtVector3& p, const cbtVector3* vertices, cbtVector3& normal)
{
	cbtVector3 lp(p);
	cbtVector3 lnormal(normal);

	return pointInTriangle(vertices, lnormal, &lp);
}

bool SphereTriangleDetector::collide(const cbtVector3& sphereCenter, cbtVector3& point, cbtVector3& resultNormal, cbtScalar& depth, cbtScalar& timeOfImpact, cbtScalar contactBreakingThreshold)
{
	const cbtVector3* vertices = &m_triangle->getVertexPtr(0);

	cbtScalar radius = m_sphere->getRadius();
	cbtScalar radiusWithThreshold = radius + contactBreakingThreshold;

	cbtVector3 normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);

	cbtScalar l2 = normal.length2();
	bool hasContact = false;
	cbtVector3 contactPoint;

	if (l2 >= SIMD_EPSILON * SIMD_EPSILON)
	{
		normal /= cbtSqrt(l2);

		cbtVector3 p1ToCentre = sphereCenter - vertices[0];
		cbtScalar distanceFromPlane = p1ToCentre.dot(normal);

		if (distanceFromPlane < cbtScalar(0.))
		{
			//triangle facing the other way
			distanceFromPlane *= cbtScalar(-1.);
			normal *= cbtScalar(-1.);
		}

		bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;

		// Check for contact / intersection

		if (isInsideContactPlane)
		{
			if (facecontains(sphereCenter, vertices, normal))
			{
				// Inside the contact wedge - touches a point on the shell plane
				hasContact = true;
				contactPoint = sphereCenter - normal * distanceFromPlane;
			}
			else
			{
				// Could be inside one of the contact capsules
				cbtScalar contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
				cbtScalar minDistSqr = contactCapsuleRadiusSqr;
				cbtVector3 nearestOnEdge;
				for (int i = 0; i < m_triangle->getNumEdges(); i++)
				{
					cbtVector3 pa;
					cbtVector3 pb;

					m_triangle->getEdge(i, pa, pb);

					cbtScalar distanceSqr = SegmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge);
					if (distanceSqr < minDistSqr)
					{
						// Yep, we're inside a capsule, and record the capsule with smallest distance
						minDistSqr = distanceSqr;
						hasContact = true;
						contactPoint = nearestOnEdge;
					}
				}
			}
		}
	}

	if (hasContact)
	{
		cbtVector3 contactToCentre = sphereCenter - contactPoint;
		cbtScalar distanceSqr = contactToCentre.length2();

		if (distanceSqr < radiusWithThreshold * radiusWithThreshold)
		{
			if (distanceSqr > SIMD_EPSILON)
			{
				cbtScalar distance = cbtSqrt(distanceSqr);
				resultNormal = contactToCentre;
				resultNormal.normalize();
				point = contactPoint;
				depth = -(radius - distance);
			}
			else
			{
				resultNormal = normal;
				point = contactPoint;
				depth = -radius;
			}
			return true;
		}
	}

	return false;
}

bool SphereTriangleDetector::pointInTriangle(const cbtVector3 vertices[], const cbtVector3& normal, cbtVector3* p)
{
	const cbtVector3* p1 = &vertices[0];
	const cbtVector3* p2 = &vertices[1];
	const cbtVector3* p3 = &vertices[2];

	cbtVector3 edge1(*p2 - *p1);
	cbtVector3 edge2(*p3 - *p2);
	cbtVector3 edge3(*p1 - *p3);

	cbtVector3 p1_to_p(*p - *p1);
	cbtVector3 p2_to_p(*p - *p2);
	cbtVector3 p3_to_p(*p - *p3);

	cbtVector3 edge1_normal(edge1.cross(normal));
	cbtVector3 edge2_normal(edge2.cross(normal));
	cbtVector3 edge3_normal(edge3.cross(normal));

	cbtScalar r1, r2, r3;
	r1 = edge1_normal.dot(p1_to_p);
	r2 = edge2_normal.dot(p2_to_p);
	r3 = edge3_normal.dot(p3_to_p);
	if ((r1 > 0 && r2 > 0 && r3 > 0) ||
		(r1 <= 0 && r2 <= 0 && r3 <= 0))
		return true;
	return false;
}
