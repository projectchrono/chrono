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
#if defined(_WIN32) || defined(__i386__)
#define BT_USE_SSE_IN_API
#endif

#include "BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.h"
#include "cbtConvexPolyhedron.h"
#include "LinearMath/cbtConvexHullComputer.h"
#include <new>
#include "LinearMath/cbtGeometryUtil.h"
#include "LinearMath/cbtGrahamScan2dConvexHull.h"

cbtPolyhedralConvexShape::cbtPolyhedralConvexShape() : cbtConvexInternalShape(),
													 m_polyhedron(0)
{
}

cbtPolyhedralConvexShape::~cbtPolyhedralConvexShape()
{
	if (m_polyhedron)
	{
		m_polyhedron->~cbtConvexPolyhedron();
		cbtAlignedFree(m_polyhedron);
	}
}

void cbtPolyhedralConvexShape::setPolyhedralFeatures(cbtConvexPolyhedron& polyhedron)
{
	if (m_polyhedron)
	{
		*m_polyhedron = polyhedron;
	}
	else
	{
		void* mem = cbtAlignedAlloc(sizeof(cbtConvexPolyhedron), 16);
		m_polyhedron = new (mem) cbtConvexPolyhedron(polyhedron);
	}
}

bool cbtPolyhedralConvexShape::initializePolyhedralFeatures(int shiftVerticesByMargin)
{
	if (m_polyhedron)
	{
		m_polyhedron->~cbtConvexPolyhedron();
		cbtAlignedFree(m_polyhedron);
	}

	void* mem = cbtAlignedAlloc(sizeof(cbtConvexPolyhedron), 16);
	m_polyhedron = new (mem) cbtConvexPolyhedron;

	cbtAlignedObjectArray<cbtVector3> orgVertices;

	for (int i = 0; i < getNumVertices(); i++)
	{
		cbtVector3& newVertex = orgVertices.expand();
		getVertex(i, newVertex);
	}

	cbtConvexHullComputer conv;

	if (shiftVerticesByMargin)
	{
		cbtAlignedObjectArray<cbtVector3> planeEquations;
		cbtGeometryUtil::getPlaneEquationsFromVertices(orgVertices, planeEquations);

		cbtAlignedObjectArray<cbtVector3> shiftedPlaneEquations;
		for (int p = 0; p < planeEquations.size(); p++)
		{
			cbtVector3 plane = planeEquations[p];
			//	   cbtScalar margin = getMargin();
			plane[3] -= getMargin();
			shiftedPlaneEquations.push_back(plane);
		}

		cbtAlignedObjectArray<cbtVector3> tmpVertices;

		cbtGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations, tmpVertices);

		conv.compute(&tmpVertices[0].getX(), sizeof(cbtVector3), tmpVertices.size(), 0.f, 0.f);
	}
	else
	{
		conv.compute(&orgVertices[0].getX(), sizeof(cbtVector3), orgVertices.size(), 0.f, 0.f);
	}

#ifndef BT_RECONSTRUCT_FACES

	int numVertices = conv.vertices.size();
	m_polyhedron->m_vertices.resize(numVertices);
	for (int p = 0; p < numVertices; p++)
	{
		m_polyhedron->m_vertices[p] = conv.vertices[p];
	}

	int v0, v1;
	for (int j = 0; j < conv.faces.size(); j++)
	{
		cbtVector3 edges[3];
		int numEdges = 0;
		cbtFace combinedFace;
		const cbtConvexHullComputer::Edge* edge = &conv.edges[conv.faces[j]];
		v0 = edge->getSourceVertex();
		int prevVertex = v0;
		combinedFace.m_indices.push_back(v0);
		v1 = edge->getTargetVertex();
		while (v1 != v0)
		{
			cbtVector3 wa = conv.vertices[prevVertex];
			cbtVector3 wb = conv.vertices[v1];
			cbtVector3 newEdge = wb - wa;
			newEdge.normalize();
			if (numEdges < 2)
				edges[numEdges++] = newEdge;

			//face->addIndex(v1);
			combinedFace.m_indices.push_back(v1);
			edge = edge->getNextEdgeOfFace();
			prevVertex = v1;
			int v01 = edge->getSourceVertex();
			v1 = edge->getTargetVertex();
		}

		cbtAssert(combinedFace.m_indices.size() > 2);

		cbtVector3 faceNormal = edges[0].cross(edges[1]);
		faceNormal.normalize();

		cbtScalar planeEq = 1e30f;

		for (int v = 0; v < combinedFace.m_indices.size(); v++)
		{
			cbtScalar eq = m_polyhedron->m_vertices[combinedFace.m_indices[v]].dot(faceNormal);
			if (planeEq > eq)
			{
				planeEq = eq;
			}
		}
		combinedFace.m_plane[0] = faceNormal.getX();
		combinedFace.m_plane[1] = faceNormal.getY();
		combinedFace.m_plane[2] = faceNormal.getZ();
		combinedFace.m_plane[3] = -planeEq;

		m_polyhedron->m_faces.push_back(combinedFace);
	}

#else  //BT_RECONSTRUCT_FACES

	cbtAlignedObjectArray<cbtVector3> faceNormals;
	int numFaces = conv.faces.size();
	faceNormals.resize(numFaces);
	cbtConvexHullComputer* convexUtil = &conv;

	cbtAlignedObjectArray<cbtFace> tmpFaces;
	tmpFaces.resize(numFaces);

	int numVertices = convexUtil->vertices.size();
	m_polyhedron->m_vertices.resize(numVertices);
	for (int p = 0; p < numVertices; p++)
	{
		m_polyhedron->m_vertices[p] = convexUtil->vertices[p];
	}

	for (int i = 0; i < numFaces; i++)
	{
		int face = convexUtil->faces[i];
		//printf("face=%d\n",face);
		const cbtConvexHullComputer::Edge* firstEdge = &convexUtil->edges[face];
		const cbtConvexHullComputer::Edge* edge = firstEdge;

		cbtVector3 edges[3];
		int numEdges = 0;
		//compute face normals

		do
		{
			int src = edge->getSourceVertex();
			tmpFaces[i].m_indices.push_back(src);
			int targ = edge->getTargetVertex();
			cbtVector3 wa = convexUtil->vertices[src];

			cbtVector3 wb = convexUtil->vertices[targ];
			cbtVector3 newEdge = wb - wa;
			newEdge.normalize();
			if (numEdges < 2)
				edges[numEdges++] = newEdge;

			edge = edge->getNextEdgeOfFace();
		} while (edge != firstEdge);

		cbtScalar planeEq = 1e30f;

		if (numEdges == 2)
		{
			faceNormals[i] = edges[0].cross(edges[1]);
			faceNormals[i].normalize();
			tmpFaces[i].m_plane[0] = faceNormals[i].getX();
			tmpFaces[i].m_plane[1] = faceNormals[i].getY();
			tmpFaces[i].m_plane[2] = faceNormals[i].getZ();
			tmpFaces[i].m_plane[3] = planeEq;
		}
		else
		{
			cbtAssert(0);  //degenerate?
			faceNormals[i].setZero();
		}

		for (int v = 0; v < tmpFaces[i].m_indices.size(); v++)
		{
			cbtScalar eq = m_polyhedron->m_vertices[tmpFaces[i].m_indices[v]].dot(faceNormals[i]);
			if (planeEq > eq)
			{
				planeEq = eq;
			}
		}
		tmpFaces[i].m_plane[3] = -planeEq;
	}

	//merge coplanar faces and copy them to m_polyhedron

	cbtScalar faceWeldThreshold = 0.999f;
	cbtAlignedObjectArray<int> todoFaces;
	for (int i = 0; i < tmpFaces.size(); i++)
		todoFaces.push_back(i);

	while (todoFaces.size())
	{
		cbtAlignedObjectArray<int> coplanarFaceGroup;
		int refFace = todoFaces[todoFaces.size() - 1];

		coplanarFaceGroup.push_back(refFace);
		cbtFace& faceA = tmpFaces[refFace];
		todoFaces.pop_back();

		cbtVector3 faceNormalA(faceA.m_plane[0], faceA.m_plane[1], faceA.m_plane[2]);
		for (int j = todoFaces.size() - 1; j >= 0; j--)
		{
			int i = todoFaces[j];
			cbtFace& faceB = tmpFaces[i];
			cbtVector3 faceNormalB(faceB.m_plane[0], faceB.m_plane[1], faceB.m_plane[2]);
			if (faceNormalA.dot(faceNormalB) > faceWeldThreshold)
			{
				coplanarFaceGroup.push_back(i);
				todoFaces.remove(i);
			}
		}

		bool did_merge = false;
		if (coplanarFaceGroup.size() > 1)
		{
			//do the merge: use Graham Scan 2d convex hull

			cbtAlignedObjectArray<GrahamVector3> orgpoints;
			cbtVector3 averageFaceNormal(0, 0, 0);

			for (int i = 0; i < coplanarFaceGroup.size(); i++)
			{
				//				m_polyhedron->m_faces.push_back(tmpFaces[coplanarFaceGroup[i]]);

				cbtFace& face = tmpFaces[coplanarFaceGroup[i]];
				cbtVector3 faceNormal(face.m_plane[0], face.m_plane[1], face.m_plane[2]);
				averageFaceNormal += faceNormal;
				for (int f = 0; f < face.m_indices.size(); f++)
				{
					int orgIndex = face.m_indices[f];
					cbtVector3 pt = m_polyhedron->m_vertices[orgIndex];

					bool found = false;

					for (int i = 0; i < orgpoints.size(); i++)
					{
						//if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).length2()<0.0001))
						if (orgpoints[i].m_orgIndex == orgIndex)
						{
							found = true;
							break;
						}
					}
					if (!found)
						orgpoints.push_back(GrahamVector3(pt, orgIndex));
				}
			}

			cbtFace combinedFace;
			for (int i = 0; i < 4; i++)
				combinedFace.m_plane[i] = tmpFaces[coplanarFaceGroup[0]].m_plane[i];

			cbtAlignedObjectArray<GrahamVector3> hull;

			averageFaceNormal.normalize();
			GrahamScanConvexHull2D(orgpoints, hull, averageFaceNormal);

			for (int i = 0; i < hull.size(); i++)
			{
				combinedFace.m_indices.push_back(hull[i].m_orgIndex);
				for (int k = 0; k < orgpoints.size(); k++)
				{
					if (orgpoints[k].m_orgIndex == hull[i].m_orgIndex)
					{
						orgpoints[k].m_orgIndex = -1;  // invalidate...
						break;
					}
				}
			}

			// are there rejected vertices?
			bool reject_merge = false;

			for (int i = 0; i < orgpoints.size(); i++)
			{
				if (orgpoints[i].m_orgIndex == -1)
					continue;  // this is in the hull...
				// this vertex is rejected -- is anybody else using this vertex?
				for (int j = 0; j < tmpFaces.size(); j++)
				{
					cbtFace& face = tmpFaces[j];
					// is this a face of the current coplanar group?
					bool is_in_current_group = false;
					for (int k = 0; k < coplanarFaceGroup.size(); k++)
					{
						if (coplanarFaceGroup[k] == j)
						{
							is_in_current_group = true;
							break;
						}
					}
					if (is_in_current_group)  // ignore this face...
						continue;
					// does this face use this rejected vertex?
					for (int v = 0; v < face.m_indices.size(); v++)
					{
						if (face.m_indices[v] == orgpoints[i].m_orgIndex)
						{
							// this rejected vertex is used in another face -- reject merge
							reject_merge = true;
							break;
						}
					}
					if (reject_merge)
						break;
				}
				if (reject_merge)
					break;
			}

			if (!reject_merge)
			{
				// do this merge!
				did_merge = true;
				m_polyhedron->m_faces.push_back(combinedFace);
			}
		}
		if (!did_merge)
		{
			for (int i = 0; i < coplanarFaceGroup.size(); i++)
			{
				cbtFace face = tmpFaces[coplanarFaceGroup[i]];
				m_polyhedron->m_faces.push_back(face);
			}
		}
	}

#endif  //BT_RECONSTRUCT_FACES

	m_polyhedron->initialize();

	return true;
}

#ifndef MIN
#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#endif

cbtVector3 cbtPolyhedralConvexShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const
{
	cbtVector3 supVec(0, 0, 0);
#ifndef __SPU__
	int i;
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

	for (int k = 0; k < getNumVertices(); k += 128)
	{
		cbtVector3 temp[128];
		int inner_count = MIN(getNumVertices() - k, 128);
		for (i = 0; i < inner_count; i++)
			getVertex(i, temp[i]);
		i = (int)vec.maxDot(temp, inner_count, newDot);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = temp[i];
		}
	}

#endif  //__SPU__
	return supVec;
}

void cbtPolyhedralConvexShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
#ifndef __SPU__
	int i;

	cbtVector3 vtx;
	cbtScalar newDot;

	for (i = 0; i < numVectors; i++)
	{
		supportVerticesOut[i][3] = cbtScalar(-BT_LARGE_FLOAT);
	}

	for (int j = 0; j < numVectors; j++)
	{
		const cbtVector3& vec = vectors[j];

		for (int k = 0; k < getNumVertices(); k += 128)
		{
			cbtVector3 temp[128];
			int inner_count = MIN(getNumVertices() - k, 128);
			for (i = 0; i < inner_count; i++)
				getVertex(i, temp[i]);
			i = (int)vec.maxDot(temp, inner_count, newDot);
			if (newDot > supportVerticesOut[j][3])
			{
				supportVerticesOut[j] = temp[i];
				supportVerticesOut[j][3] = newDot;
			}
		}
	}

#endif  //__SPU__
}

void cbtPolyhedralConvexShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
#ifndef __SPU__
	//not yet, return box inertia

	cbtScalar margin = getMargin();

	cbtTransform ident;
	ident.setIdentity();
	cbtVector3 aabbMin, aabbMax;
	getAabb(ident, aabbMin, aabbMax);
	cbtVector3 halfExtents = (aabbMax - aabbMin) * cbtScalar(0.5);

	cbtScalar lx = cbtScalar(2.) * (halfExtents.x() + margin);
	cbtScalar ly = cbtScalar(2.) * (halfExtents.y() + margin);
	cbtScalar lz = cbtScalar(2.) * (halfExtents.z() + margin);
	const cbtScalar x2 = lx * lx;
	const cbtScalar y2 = ly * ly;
	const cbtScalar z2 = lz * lz;
	const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

	inertia = scaledmass * (cbtVector3(y2 + z2, x2 + z2, x2 + y2));
#endif  //__SPU__
}

void cbtPolyhedralConvexAabbCachingShape::setLocalScaling(const cbtVector3& scaling)
{
	cbtConvexInternalShape::setLocalScaling(scaling);
	recalcLocalAabb();
}

cbtPolyhedralConvexAabbCachingShape::cbtPolyhedralConvexAabbCachingShape()
	: cbtPolyhedralConvexShape(),
	  m_localAabbMin(1, 1, 1),
	  m_localAabbMax(-1, -1, -1),
	  m_isLocalAabbValid(false)
{
}

void cbtPolyhedralConvexAabbCachingShape::getAabb(const cbtTransform& trans, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
}

void cbtPolyhedralConvexAabbCachingShape::recalcLocalAabb()
{
	m_isLocalAabbValid = true;

#if 1
	static const cbtVector3 _directions[] =
		{
			cbtVector3(1., 0., 0.),
			cbtVector3(0., 1., 0.),
			cbtVector3(0., 0., 1.),
			cbtVector3(-1., 0., 0.),
			cbtVector3(0., -1., 0.),
			cbtVector3(0., 0., -1.)};

	cbtVector3 _supporting[] =
		{
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.),
			cbtVector3(0., 0., 0.)};

	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

	for (int i = 0; i < 3; ++i)
	{
		m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
		m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
	}

#else

	for (int i = 0; i < 3; i++)
	{
		cbtVector3 vec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
		vec[i] = cbtScalar(1.);
		cbtVector3 tmp = localGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i];
		vec[i] = cbtScalar(-1.);
		tmp = localGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i];
	}
#endif
}
