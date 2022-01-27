/*
Bullet Continuous Collision Detection and Physics Library
* The b2CollidePolygons routines are Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///cbtBox2dBox2dCollisionAlgorithm, with modified b2CollidePolygons routines from the Box2D library.
///The modifications include: switching from b2Vec to cbtVector3, redefinition of b2Dot, b2Cross

#include "cbtBox2dBox2dCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/cbtBoxShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/CollisionDispatch/cbtBoxBoxDetector.h"
#include "BulletCollision/CollisionShapes/cbtBox2dShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

#define USE_PERSISTENT_CONTACTS 1

cbtBox2dBox2dCollisionAlgorithm::cbtBox2dBox2dCollisionAlgorithm(cbtPersistentManifold* mf, const cbtCollisionAlgorithmConstructionInfo& ci, const cbtCollisionObjectWrapper* obj0Wrap, const cbtCollisionObjectWrapper* obj1Wrap)
	: cbtActivatingCollisionAlgorithm(ci, obj0Wrap, obj1Wrap),
	  m_ownManifold(false),
	  m_manifoldPtr(mf)
{
	if (!m_manifoldPtr && m_dispatcher->needsCollision(obj0Wrap->getCollisionObject(), obj1Wrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(obj0Wrap->getCollisionObject(), obj1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

cbtBox2dBox2dCollisionAlgorithm::~cbtBox2dBox2dCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void b2CollidePolygons(cbtManifoldResult* manifold, const cbtBox2dShape* polyA, const cbtTransform& xfA, const cbtBox2dShape* polyB, const cbtTransform& xfB);

//#include <stdio.h>
void cbtBox2dBox2dCollisionAlgorithm::processCollision(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap, const cbtDispatcherInfo& dispatchInfo, cbtManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	const cbtBox2dShape* box0 = (const cbtBox2dShape*)body0Wrap->getCollisionShape();
	const cbtBox2dShape* box1 = (const cbtBox2dShape*)body1Wrap->getCollisionShape();

	resultOut->setPersistentManifold(m_manifoldPtr);

	b2CollidePolygons(resultOut, box0, body0Wrap->getWorldTransform(), box1, body1Wrap->getWorldTransform());

	//  refreshContactPoints is only necessary when using persistent contact points. otherwise all points are newly added
	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
}

cbtScalar cbtBox2dBox2dCollisionAlgorithm::calculateTimeOfImpact(cbtCollisionObject* /*body0*/, cbtCollisionObject* /*body1*/, const cbtDispatcherInfo& /*dispatchInfo*/, cbtManifoldResult* /*resultOut*/)
{
	//not yet
	return 1.f;
}

struct ClipVertex
{
	cbtVector3 v;
	int id;
	//b2ContactID id;
	//b2ContactID id;
};

#define b2Dot(a, b) (a).dot(b)
#define b2Mul(a, b) (a) * (b)
#define b2MulT(a, b) (a).transpose() * (b)
#define b2Cross(a, b) (a).cross(b)
#define cbtCrossS(a, s) cbtVector3(s* a.getY(), -s* a.getX(), 0.f)

int b2_maxManifoldPoints = 2;

static int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
							 const cbtVector3& normal, cbtScalar offset)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	cbtScalar distance0 = b2Dot(normal, vIn[0].v) - offset;
	cbtScalar distance1 = b2Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		cbtScalar interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].id = vIn[0].id;
		}
		else
		{
			vOut[numOut].id = vIn[1].id;
		}
		++numOut;
	}

	return numOut;
}

// Find the separation between poly1 and poly2 for a give edge normal on poly1.
static cbtScalar EdgeSeparation(const cbtBox2dShape* poly1, const cbtTransform& xf1, int edge1,
							   const cbtBox2dShape* poly2, const cbtTransform& xf2)
{
	const cbtVector3* vertices1 = poly1->getVertices();
	const cbtVector3* normals1 = poly1->getNormals();

	int count2 = poly2->getVertexCount();
	const cbtVector3* vertices2 = poly2->getVertices();

	cbtAssert(0 <= edge1 && edge1 < poly1->getVertexCount());

	// Convert normal from poly1's frame into poly2's frame.
	cbtVector3 normal1World = b2Mul(xf1.getBasis(), normals1[edge1]);
	cbtVector3 normal1 = b2MulT(xf2.getBasis(), normal1World);

	// Find support vertex on poly2 for -normal.
	int index = 0;
	cbtScalar minDot = BT_LARGE_FLOAT;

	if (count2 > 0)
		index = (int)normal1.minDot(vertices2, count2, minDot);

	cbtVector3 v1 = b2Mul(xf1, vertices1[edge1]);
	cbtVector3 v2 = b2Mul(xf2, vertices2[index]);
	cbtScalar separation = b2Dot(v2 - v1, normal1World);
	return separation;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static cbtScalar FindMaxSeparation(int* edgeIndex,
								  const cbtBox2dShape* poly1, const cbtTransform& xf1,
								  const cbtBox2dShape* poly2, const cbtTransform& xf2)
{
	int count1 = poly1->getVertexCount();
	const cbtVector3* normals1 = poly1->getNormals();

	// Vector pointing from the centroid of poly1 to the centroid of poly2.
	cbtVector3 d = b2Mul(xf2, poly2->getCentroid()) - b2Mul(xf1, poly1->getCentroid());
	cbtVector3 dLocal1 = b2MulT(xf1.getBasis(), d);

	// Find edge normal on poly1 that has the largest projection onto d.
	int edge = 0;
	cbtScalar maxDot;
	if (count1 > 0)
		edge = (int)dLocal1.maxDot(normals1, count1, maxDot);

	// Get the separation for the edge normal.
	cbtScalar s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
	if (s > 0.0f)
	{
		return s;
	}

	// Check the separation for the previous edge normal.
	int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	cbtScalar sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
	if (sPrev > 0.0f)
	{
		return sPrev;
	}

	// Check the separation for the next edge normal.
	int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	cbtScalar sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
	if (sNext > 0.0f)
	{
		return sNext;
	}

	// Find the best edge and the search direction.
	int bestEdge;
	cbtScalar bestSeparation;
	int increment;
	if (sPrev > s && sPrev > sNext)
	{
		increment = -1;
		bestEdge = prevEdge;
		bestSeparation = sPrev;
	}
	else if (sNext > s)
	{
		increment = 1;
		bestEdge = nextEdge;
		bestSeparation = sNext;
	}
	else
	{
		*edgeIndex = edge;
		return s;
	}

	// Perform a local search for the best edge normal.
	for (;;)
	{
		if (increment == -1)
			edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
		else
			edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

		s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);
		if (s > 0.0f)
		{
			return s;
		}

		if (s > bestSeparation)
		{
			bestEdge = edge;
			bestSeparation = s;
		}
		else
		{
			break;
		}
	}

	*edgeIndex = bestEdge;
	return bestSeparation;
}

static void FindIncidentEdge(ClipVertex c[2],
							 const cbtBox2dShape* poly1, const cbtTransform& xf1, int edge1,
							 const cbtBox2dShape* poly2, const cbtTransform& xf2)
{
	const cbtVector3* normals1 = poly1->getNormals();

	int count2 = poly2->getVertexCount();
	const cbtVector3* vertices2 = poly2->getVertices();
	const cbtVector3* normals2 = poly2->getNormals();

	cbtAssert(0 <= edge1 && edge1 < poly1->getVertexCount());

	// Get the normal of the reference edge in poly2's frame.
	cbtVector3 normal1 = b2MulT(xf2.getBasis(), b2Mul(xf1.getBasis(), normals1[edge1]));

	// Find the incident edge on poly2.
	int index = 0;
	cbtScalar minDot = BT_LARGE_FLOAT;
	for (int i = 0; i < count2; ++i)
	{
		cbtScalar dot = b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int i1 = index;
	int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = b2Mul(xf2, vertices2[i1]);
	//	c[0].id.features.referenceEdge = (unsigned char)edge1;
	//	c[0].id.features.incidentEdge = (unsigned char)i1;
	//	c[0].id.features.incidentVertex = 0;

	c[1].v = b2Mul(xf2, vertices2[i2]);
	//	c[1].id.features.referenceEdge = (unsigned char)edge1;
	//	c[1].id.features.incidentEdge = (unsigned char)i2;
	//	c[1].id.features.incidentVertex = 1;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
void b2CollidePolygons(cbtManifoldResult* manifold,
					   const cbtBox2dShape* polyA, const cbtTransform& xfA,
					   const cbtBox2dShape* polyB, const cbtTransform& xfB)
{
	int edgeA = 0;
	cbtScalar separationA = FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
	if (separationA > 0.0f)
		return;

	int edgeB = 0;
	cbtScalar separationB = FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
	if (separationB > 0.0f)
		return;

	const cbtBox2dShape* poly1;  // reference poly
	const cbtBox2dShape* poly2;  // incident poly
	cbtTransform xf1, xf2;
	int edge1;  // reference edge
	unsigned char flip;
	const cbtScalar k_relativeTol = 0.98f;
	const cbtScalar k_absoluteTol = 0.001f;

	// TODO_ERIN use "radius" of poly for absolute tolerance.
	if (separationB > k_relativeTol * separationA + k_absoluteTol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		flip = 0;
	}

	ClipVertex incidentEdge[2];
	FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	int count1 = poly1->getVertexCount();
	const cbtVector3* vertices1 = poly1->getVertices();

	cbtVector3 v11 = vertices1[edge1];
	cbtVector3 v12 = edge1 + 1 < count1 ? vertices1[edge1 + 1] : vertices1[0];

	//cbtVector3 dv = v12 - v11;
	cbtVector3 sideNormal = b2Mul(xf1.getBasis(), v12 - v11);
	sideNormal.normalize();
	cbtVector3 frontNormal = cbtCrossS(sideNormal, 1.0f);

	v11 = b2Mul(xf1, v11);
	v12 = b2Mul(xf1, v12);

	cbtScalar frontOffset = b2Dot(frontNormal, v11);
	cbtScalar sideOffset1 = -b2Dot(sideNormal, v11);
	cbtScalar sideOffset2 = b2Dot(sideNormal, v12);

	// Clip incident edge against extruded edge1 side edges.
	ClipVertex clipPoints1[2];
	clipPoints1[0].v.setValue(0, 0, 0);
	clipPoints1[1].v.setValue(0, 0, 0);

	ClipVertex clipPoints2[2];
	clipPoints2[0].v.setValue(0, 0, 0);
	clipPoints2[1].v.setValue(0, 0, 0);

	int np;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);

	if (np < 2)
		return;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, sideOffset2);

	if (np < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	cbtVector3 manifoldNormal = flip ? -frontNormal : frontNormal;

	int pointCount = 0;
	for (int i = 0; i < b2_maxManifoldPoints; ++i)
	{
		cbtScalar separation = b2Dot(frontNormal, clipPoints2[i].v) - frontOffset;

		if (separation <= 0.0f)
		{
			//b2ManifoldPoint* cp = manifold->points + pointCount;
			//cbtScalar separation = separation;
			//cp->localPoint1 = b2MulT(xfA, clipPoints2[i].v);
			//cp->localPoint2 = b2MulT(xfB, clipPoints2[i].v);

			manifold->addContactPoint(-manifoldNormal, clipPoints2[i].v, separation);

			//			cp->id = clipPoints2[i].id;
			//			cp->id.features.flip = flip;
			++pointCount;
		}
	}

	//	manifold->pointCount = pointCount;}
}
