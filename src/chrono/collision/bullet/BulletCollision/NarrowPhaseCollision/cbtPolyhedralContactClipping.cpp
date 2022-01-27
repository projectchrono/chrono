/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This file was written by Erwin Coumans
///Separating axis rest based on work from Pierre Terdiman, see
///And contact clipping based on work from Simon Hobbs

#include "cbtPolyhedralContactClipping.h"
#include "BulletCollision/CollisionShapes/cbtConvexPolyhedron.h"

#include <float.h>  //for FLT_MAX

int gExpectedNbTests = 0;
int gActualNbTests = 0;
bool gUseInternalObject = true;

// Clips a face to the back of a plane
void cbtPolyhedralContactClipping::clipFace(const cbtVertexArray& pVtxIn, cbtVertexArray& ppVtxOut, const cbtVector3& planeNormalWS, cbtScalar planeEqWS)
{
	int ve;
	cbtScalar ds, de;
	int numVerts = pVtxIn.size();
	if (numVerts < 2)
		return;

	cbtVector3 firstVertex = pVtxIn[pVtxIn.size() - 1];
	cbtVector3 endVertex = pVtxIn[0];

	ds = planeNormalWS.dot(firstVertex) + planeEqWS;

	for (ve = 0; ve < numVerts; ve++)
	{
		endVertex = pVtxIn[ve];

		de = planeNormalWS.dot(endVertex) + planeEqWS;

		if (ds < 0)
		{
			if (de < 0)
			{
				// Start < 0, end < 0, so output endVertex
				ppVtxOut.push_back(endVertex);
			}
			else
			{
				// Start < 0, end >= 0, so output intersection
				ppVtxOut.push_back(firstVertex.lerp(endVertex, cbtScalar(ds * 1.f / (ds - de))));
			}
		}
		else
		{
			if (de < 0)
			{
				// Start >= 0, end < 0 so output intersection and end
				ppVtxOut.push_back(firstVertex.lerp(endVertex, cbtScalar(ds * 1.f / (ds - de))));
				ppVtxOut.push_back(endVertex);
			}
		}
		firstVertex = endVertex;
		ds = de;
	}
}

static bool TestSepAxis(const cbtConvexPolyhedron& hullA, const cbtConvexPolyhedron& hullB, const cbtTransform& transA, const cbtTransform& transB, const cbtVector3& sep_axis, cbtScalar& depth, cbtVector3& witnessPointA, cbtVector3& witnessPointB)
{
	cbtScalar Min0, Max0;
	cbtScalar Min1, Max1;
	cbtVector3 witnesPtMinA, witnesPtMaxA;
	cbtVector3 witnesPtMinB, witnesPtMaxB;

	hullA.project(transA, sep_axis, Min0, Max0, witnesPtMinA, witnesPtMaxA);
	hullB.project(transB, sep_axis, Min1, Max1, witnesPtMinB, witnesPtMaxB);

	if (Max0 < Min1 || Max1 < Min0)
		return false;

	cbtScalar d0 = Max0 - Min1;
	cbtAssert(d0 >= 0.0f);
	cbtScalar d1 = Max1 - Min0;
	cbtAssert(d1 >= 0.0f);
	if (d0 < d1)
	{
		depth = d0;
		witnessPointA = witnesPtMaxA;
		witnessPointB = witnesPtMinB;
	}
	else
	{
		depth = d1;
		witnessPointA = witnesPtMinA;
		witnessPointB = witnesPtMaxB;
	}

	return true;
}

static int gActualSATPairTests = 0;

inline bool IsAlmostZero(const cbtVector3& v)
{
	if (cbtFabs(v.x()) > 1e-6 || cbtFabs(v.y()) > 1e-6 || cbtFabs(v.z()) > 1e-6) return false;
	return true;
}

#ifdef TEST_INTERNAL_OBJECTS

inline void BoxSupport(const cbtScalar extents[3], const cbtScalar sv[3], cbtScalar p[3])
{
	// This version is ~11.000 cycles (4%) faster overall in one of the tests.
	//	IR(p[0]) = IR(extents[0])|(IR(sv[0])&SIGN_BITMASK);
	//	IR(p[1]) = IR(extents[1])|(IR(sv[1])&SIGN_BITMASK);
	//	IR(p[2]) = IR(extents[2])|(IR(sv[2])&SIGN_BITMASK);
	p[0] = sv[0] < 0.0f ? -extents[0] : extents[0];
	p[1] = sv[1] < 0.0f ? -extents[1] : extents[1];
	p[2] = sv[2] < 0.0f ? -extents[2] : extents[2];
}

void InverseTransformPoint3x3(cbtVector3& out, const cbtVector3& in, const cbtTransform& tr)
{
	const cbtMatrix3x3& rot = tr.getBasis();
	const cbtVector3& r0 = rot[0];
	const cbtVector3& r1 = rot[1];
	const cbtVector3& r2 = rot[2];

	const cbtScalar x = r0.x() * in.x() + r1.x() * in.y() + r2.x() * in.z();
	const cbtScalar y = r0.y() * in.x() + r1.y() * in.y() + r2.y() * in.z();
	const cbtScalar z = r0.z() * in.x() + r1.z() * in.y() + r2.z() * in.z();

	out.setValue(x, y, z);
}

bool TestInternalObjects(const cbtTransform& trans0, const cbtTransform& trans1, const cbtVector3& delta_c, const cbtVector3& axis, const cbtConvexPolyhedron& convex0, const cbtConvexPolyhedron& convex1, cbtScalar dmin)
{
	const cbtScalar dp = delta_c.dot(axis);

	cbtVector3 localAxis0;
	InverseTransformPoint3x3(localAxis0, axis, trans0);
	cbtVector3 localAxis1;
	InverseTransformPoint3x3(localAxis1, axis, trans1);

	cbtScalar p0[3];
	BoxSupport(convex0.m_extents, localAxis0, p0);
	cbtScalar p1[3];
	BoxSupport(convex1.m_extents, localAxis1, p1);

	const cbtScalar Radius0 = p0[0] * localAxis0.x() + p0[1] * localAxis0.y() + p0[2] * localAxis0.z();
	const cbtScalar Radius1 = p1[0] * localAxis1.x() + p1[1] * localAxis1.y() + p1[2] * localAxis1.z();

	const cbtScalar MinRadius = Radius0 > convex0.m_radius ? Radius0 : convex0.m_radius;
	const cbtScalar MaxRadius = Radius1 > convex1.m_radius ? Radius1 : convex1.m_radius;

	const cbtScalar MinMaxRadius = MaxRadius + MinRadius;
	const cbtScalar d0 = MinMaxRadius + dp;
	const cbtScalar d1 = MinMaxRadius - dp;

	const cbtScalar depth = d0 < d1 ? d0 : d1;
	if (depth > dmin)
		return false;
	return true;
}
#endif  //TEST_INTERNAL_OBJECTS

SIMD_FORCE_INLINE void cbtSegmentsClosestPoints(
	cbtVector3& ptsVector,
	cbtVector3& offsetA,
	cbtVector3& offsetB,
	cbtScalar& tA, cbtScalar& tB,
	const cbtVector3& translation,
	const cbtVector3& dirA, cbtScalar hlenA,
	const cbtVector3& dirB, cbtScalar hlenB)
{
	// compute the parameters of the closest points on each line segment

	cbtScalar dirA_dot_dirB = cbtDot(dirA, dirB);
	cbtScalar dirA_dot_trans = cbtDot(dirA, translation);
	cbtScalar dirB_dot_trans = cbtDot(dirB, translation);

	cbtScalar denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	if (denom == 0.0f)
	{
		tA = 0.0f;
	}
	else
	{
		tA = (dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB) / denom;
		if (tA < -hlenA)
			tA = -hlenA;
		else if (tA > hlenA)
			tA = hlenA;
	}

	tB = tA * dirA_dot_dirB - dirB_dot_trans;

	if (tB < -hlenB)
	{
		tB = -hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if (tA < -hlenA)
			tA = -hlenA;
		else if (tA > hlenA)
			tA = hlenA;
	}
	else if (tB > hlenB)
	{
		tB = hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if (tA < -hlenA)
			tA = -hlenA;
		else if (tA > hlenA)
			tA = hlenA;
	}

	// compute the closest points relative to segment centers.

	offsetA = dirA * tA;
	offsetB = dirB * tB;

	ptsVector = translation - offsetA + offsetB;
}

bool cbtPolyhedralContactClipping::findSeparatingAxis(const cbtConvexPolyhedron& hullA, const cbtConvexPolyhedron& hullB, const cbtTransform& transA, const cbtTransform& transB, cbtVector3& sep, cbtDiscreteCollisionDetectorInterface::Result& resultOut)
{
	gActualSATPairTests++;

	//#ifdef TEST_INTERNAL_OBJECTS
	const cbtVector3 c0 = transA * hullA.m_localCenter;
	const cbtVector3 c1 = transB * hullB.m_localCenter;
	const cbtVector3 DeltaC2 = c0 - c1;
	//#endif

	cbtScalar dmin = FLT_MAX;
	int curPlaneTests = 0;

	int numFacesA = hullA.m_faces.size();
	// Test normals from hullA
	for (int i = 0; i < numFacesA; i++)
	{
		const cbtVector3 Normal(hullA.m_faces[i].m_plane[0], hullA.m_faces[i].m_plane[1], hullA.m_faces[i].m_plane[2]);
		cbtVector3 faceANormalWS = transA.getBasis() * Normal;
		if (DeltaC2.dot(faceANormalWS) < 0)
			faceANormalWS *= -1.f;

		curPlaneTests++;
#ifdef TEST_INTERNAL_OBJECTS
		gExpectedNbTests++;
		if (gUseInternalObject && !TestInternalObjects(transA, transB, DeltaC2, faceANormalWS, hullA, hullB, dmin))
			continue;
		gActualNbTests++;
#endif

		cbtScalar d;
		cbtVector3 wA, wB;
		if (!TestSepAxis(hullA, hullB, transA, transB, faceANormalWS, d, wA, wB))
			return false;

		if (d < dmin)
		{
			dmin = d;
			sep = faceANormalWS;
		}
	}

	int numFacesB = hullB.m_faces.size();
	// Test normals from hullB
	for (int i = 0; i < numFacesB; i++)
	{
		const cbtVector3 Normal(hullB.m_faces[i].m_plane[0], hullB.m_faces[i].m_plane[1], hullB.m_faces[i].m_plane[2]);
		cbtVector3 WorldNormal = transB.getBasis() * Normal;
		if (DeltaC2.dot(WorldNormal) < 0)
			WorldNormal *= -1.f;

		curPlaneTests++;
#ifdef TEST_INTERNAL_OBJECTS
		gExpectedNbTests++;
		if (gUseInternalObject && !TestInternalObjects(transA, transB, DeltaC2, WorldNormal, hullA, hullB, dmin))
			continue;
		gActualNbTests++;
#endif

		cbtScalar d;
		cbtVector3 wA, wB;
		if (!TestSepAxis(hullA, hullB, transA, transB, WorldNormal, d, wA, wB))
			return false;

		if (d < dmin)
		{
			dmin = d;
			sep = WorldNormal;
		}
	}

	cbtVector3 edgeAstart, edgeAend, edgeBstart, edgeBend;
	int edgeA = -1;
	int edgeB = -1;
	cbtVector3 worldEdgeA;
	cbtVector3 worldEdgeB;
	cbtVector3 witnessPointA(0, 0, 0), witnessPointB(0, 0, 0);

	int curEdgeEdge = 0;
	// Test edges
	for (int e0 = 0; e0 < hullA.m_uniqueEdges.size(); e0++)
	{
		const cbtVector3 edge0 = hullA.m_uniqueEdges[e0];
		const cbtVector3 WorldEdge0 = transA.getBasis() * edge0;
		for (int e1 = 0; e1 < hullB.m_uniqueEdges.size(); e1++)
		{
			const cbtVector3 edge1 = hullB.m_uniqueEdges[e1];
			const cbtVector3 WorldEdge1 = transB.getBasis() * edge1;

			cbtVector3 Cross = WorldEdge0.cross(WorldEdge1);
			curEdgeEdge++;
			if (!IsAlmostZero(Cross))
			{
				Cross = Cross.normalize();
				if (DeltaC2.dot(Cross) < 0)
					Cross *= -1.f;

#ifdef TEST_INTERNAL_OBJECTS
				gExpectedNbTests++;
				if (gUseInternalObject && !TestInternalObjects(transA, transB, DeltaC2, Cross, hullA, hullB, dmin))
					continue;
				gActualNbTests++;
#endif

				cbtScalar dist;
				cbtVector3 wA, wB;
				if (!TestSepAxis(hullA, hullB, transA, transB, Cross, dist, wA, wB))
					return false;

				if (dist < dmin)
				{
					dmin = dist;
					sep = Cross;
					edgeA = e0;
					edgeB = e1;
					worldEdgeA = WorldEdge0;
					worldEdgeB = WorldEdge1;
					witnessPointA = wA;
					witnessPointB = wB;
				}
			}
		}
	}

	if (edgeA >= 0 && edgeB >= 0)
	{
		//		printf("edge-edge\n");
		//add an edge-edge contact

		cbtVector3 ptsVector;
		cbtVector3 offsetA;
		cbtVector3 offsetB;
		cbtScalar tA;
		cbtScalar tB;

		cbtVector3 translation = witnessPointB - witnessPointA;

		cbtVector3 dirA = worldEdgeA;
		cbtVector3 dirB = worldEdgeB;

		cbtScalar hlenB = 1e30f;
		cbtScalar hlenA = 1e30f;

		cbtSegmentsClosestPoints(ptsVector, offsetA, offsetB, tA, tB,
								translation,
								dirA, hlenA,
								dirB, hlenB);

		cbtScalar nlSqrt = ptsVector.length2();
		if (nlSqrt > SIMD_EPSILON)
		{
			cbtScalar nl = cbtSqrt(nlSqrt);
			ptsVector *= 1.f / nl;
			if (ptsVector.dot(DeltaC2) < 0.f)
			{
				ptsVector *= -1.f;
			}
			cbtVector3 ptOnB = witnessPointB + offsetB;
			cbtScalar distance = nl;
			resultOut.addContactPoint(ptsVector, ptOnB, -distance);
		}
	}

	if ((DeltaC2.dot(sep)) < 0.0f)
		sep = -sep;

	return true;
}

void cbtPolyhedralContactClipping::clipFaceAgainstHull(const cbtVector3& separatingNormal, const cbtConvexPolyhedron& hullA, const cbtTransform& transA, cbtVertexArray& worldVertsB1, cbtVertexArray& worldVertsB2, const cbtScalar minDist, cbtScalar maxDist, cbtDiscreteCollisionDetectorInterface::Result& resultOut)
{
	worldVertsB2.resize(0);
	cbtVertexArray* pVtxIn = &worldVertsB1;
	cbtVertexArray* pVtxOut = &worldVertsB2;
	pVtxOut->reserve(pVtxIn->size());

	int closestFaceA = -1;
	{
		cbtScalar dmin = FLT_MAX;
		for (int face = 0; face < hullA.m_faces.size(); face++)
		{
			const cbtVector3 Normal(hullA.m_faces[face].m_plane[0], hullA.m_faces[face].m_plane[1], hullA.m_faces[face].m_plane[2]);
			const cbtVector3 faceANormalWS = transA.getBasis() * Normal;

			cbtScalar d = faceANormalWS.dot(separatingNormal);
			if (d < dmin)
			{
				dmin = d;
				closestFaceA = face;
			}
		}
	}
	if (closestFaceA < 0)
		return;

	const cbtFace& polyA = hullA.m_faces[closestFaceA];

	// clip polygon to back of planes of all faces of hull A that are adjacent to witness face
	int numVerticesA = polyA.m_indices.size();
	for (int e0 = 0; e0 < numVerticesA; e0++)
	{
		const cbtVector3& a = hullA.m_vertices[polyA.m_indices[e0]];
		const cbtVector3& b = hullA.m_vertices[polyA.m_indices[(e0 + 1) % numVerticesA]];
		const cbtVector3 edge0 = a - b;
		const cbtVector3 WorldEdge0 = transA.getBasis() * edge0;
		cbtVector3 worldPlaneAnormal1 = transA.getBasis() * cbtVector3(polyA.m_plane[0], polyA.m_plane[1], polyA.m_plane[2]);

		cbtVector3 planeNormalWS1 = -WorldEdge0.cross(worldPlaneAnormal1);  //.cross(WorldEdge0);
		cbtVector3 worldA1 = transA * a;
		cbtScalar planeEqWS1 = -worldA1.dot(planeNormalWS1);

//int otherFace=0;
#ifdef BLA1
		int otherFace = polyA.m_connectedFaces[e0];
		cbtVector3 localPlaneNormal(hullA.m_faces[otherFace].m_plane[0], hullA.m_faces[otherFace].m_plane[1], hullA.m_faces[otherFace].m_plane[2]);
		cbtScalar localPlaneEq = hullA.m_faces[otherFace].m_plane[3];

		cbtVector3 planeNormalWS = transA.getBasis() * localPlaneNormal;
		cbtScalar planeEqWS = localPlaneEq - planeNormalWS.dot(transA.getOrigin());
#else
		cbtVector3 planeNormalWS = planeNormalWS1;
		cbtScalar planeEqWS = planeEqWS1;

#endif
		//clip face

		clipFace(*pVtxIn, *pVtxOut, planeNormalWS, planeEqWS);
		cbtSwap(pVtxIn, pVtxOut);
		pVtxOut->resize(0);
	}

	//#define ONLY_REPORT_DEEPEST_POINT

	cbtVector3 point;

	// only keep points that are behind the witness face
	{
		cbtVector3 localPlaneNormal(polyA.m_plane[0], polyA.m_plane[1], polyA.m_plane[2]);
		cbtScalar localPlaneEq = polyA.m_plane[3];
		cbtVector3 planeNormalWS = transA.getBasis() * localPlaneNormal;
		cbtScalar planeEqWS = localPlaneEq - planeNormalWS.dot(transA.getOrigin());
		for (int i = 0; i < pVtxIn->size(); i++)
		{
			cbtVector3 vtx = pVtxIn->at(i);
			cbtScalar depth = planeNormalWS.dot(vtx) + planeEqWS;
			if (depth <= minDist)
			{
				//				printf("clamped: depth=%f to minDist=%f\n",depth,minDist);
				depth = minDist;
			}

			if (depth <= maxDist)
			{
				cbtVector3 point = pVtxIn->at(i);
#ifdef ONLY_REPORT_DEEPEST_POINT
				curMaxDist = depth;
#else
#if 0
				if (depth<-3)
				{
					printf("error in cbtPolyhedralContactClipping depth = %f\n", depth);
					printf("likely wrong separatingNormal passed in\n");
				}
#endif
				resultOut.addContactPoint(separatingNormal, point, depth);
#endif
			}
		}
	}
#ifdef ONLY_REPORT_DEEPEST_POINT
	if (curMaxDist < maxDist)
	{
		resultOut.addContactPoint(separatingNormal, point, curMaxDist);
	}
#endif  //ONLY_REPORT_DEEPEST_POINT
}

void cbtPolyhedralContactClipping::clipHullAgainstHull(const cbtVector3& separatingNormal1, const cbtConvexPolyhedron& hullA, const cbtConvexPolyhedron& hullB, const cbtTransform& transA, const cbtTransform& transB, const cbtScalar minDist, cbtScalar maxDist, cbtVertexArray& worldVertsB1, cbtVertexArray& worldVertsB2, cbtDiscreteCollisionDetectorInterface::Result& resultOut)
{
	cbtVector3 separatingNormal = separatingNormal1.normalized();
	//	const cbtVector3 c0 = transA * hullA.m_localCenter;
	//	const cbtVector3 c1 = transB * hullB.m_localCenter;
	//const cbtVector3 DeltaC2 = c0 - c1;

	int closestFaceB = -1;
	cbtScalar dmax = -FLT_MAX;
	{
		for (int face = 0; face < hullB.m_faces.size(); face++)
		{
			const cbtVector3 Normal(hullB.m_faces[face].m_plane[0], hullB.m_faces[face].m_plane[1], hullB.m_faces[face].m_plane[2]);
			const cbtVector3 WorldNormal = transB.getBasis() * Normal;
			cbtScalar d = WorldNormal.dot(separatingNormal);
			if (d > dmax)
			{
				dmax = d;
				closestFaceB = face;
			}
		}
	}
	worldVertsB1.resize(0);
	{
		const cbtFace& polyB = hullB.m_faces[closestFaceB];
		const int numVertices = polyB.m_indices.size();
		for (int e0 = 0; e0 < numVertices; e0++)
		{
			const cbtVector3& b = hullB.m_vertices[polyB.m_indices[e0]];
			worldVertsB1.push_back(transB * b);
		}
	}

	if (closestFaceB >= 0)
		clipFaceAgainstHull(separatingNormal, hullA, transA, worldVertsB1, worldVertsB2, minDist, maxDist, resultOut);
}
