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

#include "cbtConvexPolyhedron.h"
#include "LinearMath/cbtHashMap.h"

cbtConvexPolyhedron::cbtConvexPolyhedron()
{
}
cbtConvexPolyhedron::~cbtConvexPolyhedron()
{
}

inline bool IsAlmostZero1(const cbtVector3& v)
{
	if (cbtFabs(v.x()) > 1e-6 || cbtFabs(v.y()) > 1e-6 || cbtFabs(v.z()) > 1e-6) return false;
	return true;
}

struct cbtInternalVertexPair
{
	cbtInternalVertexPair(short int v0, short int v1)
		: m_v0(v0),
		  m_v1(v1)
	{
		if (m_v1 > m_v0)
			cbtSwap(m_v0, m_v1);
	}
	short int m_v0;
	short int m_v1;
	int getHash() const
	{
		return m_v0 + (m_v1 << 16);
	}
	bool equals(const cbtInternalVertexPair& other) const
	{
		return m_v0 == other.m_v0 && m_v1 == other.m_v1;
	}
};

struct cbtInternalEdge
{
	cbtInternalEdge()
		: m_face0(-1),
		  m_face1(-1)
	{
	}
	short int m_face0;
	short int m_face1;
};

//

#ifdef TEST_INTERNAL_OBJECTS
bool cbtConvexPolyhedron::testContainment() const
{
	for (int p = 0; p < 8; p++)
	{
		cbtVector3 LocalPt;
		if (p == 0)
			LocalPt = m_localCenter + cbtVector3(m_extents[0], m_extents[1], m_extents[2]);
		else if (p == 1)
			LocalPt = m_localCenter + cbtVector3(m_extents[0], m_extents[1], -m_extents[2]);
		else if (p == 2)
			LocalPt = m_localCenter + cbtVector3(m_extents[0], -m_extents[1], m_extents[2]);
		else if (p == 3)
			LocalPt = m_localCenter + cbtVector3(m_extents[0], -m_extents[1], -m_extents[2]);
		else if (p == 4)
			LocalPt = m_localCenter + cbtVector3(-m_extents[0], m_extents[1], m_extents[2]);
		else if (p == 5)
			LocalPt = m_localCenter + cbtVector3(-m_extents[0], m_extents[1], -m_extents[2]);
		else if (p == 6)
			LocalPt = m_localCenter + cbtVector3(-m_extents[0], -m_extents[1], m_extents[2]);
		else if (p == 7)
			LocalPt = m_localCenter + cbtVector3(-m_extents[0], -m_extents[1], -m_extents[2]);

		for (int i = 0; i < m_faces.size(); i++)
		{
			const cbtVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			const cbtScalar d = LocalPt.dot(Normal) + m_faces[i].m_plane[3];
			if (d > 0.0f)
				return false;
		}
	}
	return true;
}
#endif

void cbtConvexPolyhedron::initialize()
{
	cbtHashMap<cbtInternalVertexPair, cbtInternalEdge> edges;

	for (int i = 0; i < m_faces.size(); i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		int NbTris = numVertices;
		for (int j = 0; j < NbTris; j++)
		{
			int k = (j + 1) % numVertices;
			cbtInternalVertexPair vp(m_faces[i].m_indices[j], m_faces[i].m_indices[k]);
			cbtInternalEdge* edptr = edges.find(vp);
			cbtVector3 edge = m_vertices[vp.m_v1] - m_vertices[vp.m_v0];
			edge.normalize();

			bool found = false;

			for (int p = 0; p < m_uniqueEdges.size(); p++)
			{
				if (IsAlmostZero1(m_uniqueEdges[p] - edge) ||
					IsAlmostZero1(m_uniqueEdges[p] + edge))
				{
					found = true;
					break;
				}
			}

			if (!found)
			{
				m_uniqueEdges.push_back(edge);
			}

			if (edptr)
			{
				cbtAssert(edptr->m_face0 >= 0);
				cbtAssert(edptr->m_face1 < 0);
				edptr->m_face1 = i;
			}
			else
			{
				cbtInternalEdge ed;
				ed.m_face0 = i;
				edges.insert(vp, ed);
			}
		}
	}

#ifdef USE_CONNECTED_FACES
	for (int i = 0; i < m_faces.size(); i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		m_faces[i].m_connectedFaces.resize(numVertices);

		for (int j = 0; j < numVertices; j++)
		{
			int k = (j + 1) % numVertices;
			cbtInternalVertexPair vp(m_faces[i].m_indices[j], m_faces[i].m_indices[k]);
			cbtInternalEdge* edptr = edges.find(vp);
			cbtAssert(edptr);
			cbtAssert(edptr->m_face0 >= 0);
			cbtAssert(edptr->m_face1 >= 0);

			int connectedFace = (edptr->m_face0 == i) ? edptr->m_face1 : edptr->m_face0;
			m_faces[i].m_connectedFaces[j] = connectedFace;
		}
	}
#endif  //USE_CONNECTED_FACES

	initialize2();
}

void cbtConvexPolyhedron::initialize2()
{
	m_localCenter.setValue(0, 0, 0);
	cbtScalar TotalArea = 0.0f;
	for (int i = 0; i < m_faces.size(); i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		int NbTris = numVertices - 2;

		const cbtVector3& p0 = m_vertices[m_faces[i].m_indices[0]];
		for (int j = 1; j <= NbTris; j++)
		{
			int k = (j + 1) % numVertices;
			const cbtVector3& p1 = m_vertices[m_faces[i].m_indices[j]];
			const cbtVector3& p2 = m_vertices[m_faces[i].m_indices[k]];
			cbtScalar Area = ((p0 - p1).cross(p0 - p2)).length() * 0.5f;
			cbtVector3 Center = (p0 + p1 + p2) / 3.0f;
			m_localCenter += Area * Center;
			TotalArea += Area;
		}
	}
	m_localCenter /= TotalArea;

#ifdef TEST_INTERNAL_OBJECTS
	if (1)
	{
		m_radius = FLT_MAX;
		for (int i = 0; i < m_faces.size(); i++)
		{
			const cbtVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			const cbtScalar dist = cbtFabs(m_localCenter.dot(Normal) + m_faces[i].m_plane[3]);
			if (dist < m_radius)
				m_radius = dist;
		}

		cbtScalar MinX = FLT_MAX;
		cbtScalar MinY = FLT_MAX;
		cbtScalar MinZ = FLT_MAX;
		cbtScalar MaxX = -FLT_MAX;
		cbtScalar MaxY = -FLT_MAX;
		cbtScalar MaxZ = -FLT_MAX;
		for (int i = 0; i < m_vertices.size(); i++)
		{
			const cbtVector3& pt = m_vertices[i];
			if (pt.x() < MinX) MinX = pt.x();
			if (pt.x() > MaxX) MaxX = pt.x();
			if (pt.y() < MinY) MinY = pt.y();
			if (pt.y() > MaxY) MaxY = pt.y();
			if (pt.z() < MinZ) MinZ = pt.z();
			if (pt.z() > MaxZ) MaxZ = pt.z();
		}
		mC.setValue(MaxX + MinX, MaxY + MinY, MaxZ + MinZ);
		mE.setValue(MaxX - MinX, MaxY - MinY, MaxZ - MinZ);

		//		const cbtScalar r = m_radius / sqrtf(2.0f);
		const cbtScalar r = m_radius / sqrtf(3.0f);
		const int LargestExtent = mE.maxAxis();
		const cbtScalar Step = (mE[LargestExtent] * 0.5f - r) / 1024.0f;
		m_extents[0] = m_extents[1] = m_extents[2] = r;
		m_extents[LargestExtent] = mE[LargestExtent] * 0.5f;
		bool FoundBox = false;
		for (int j = 0; j < 1024; j++)
		{
			if (testContainment())
			{
				FoundBox = true;
				break;
			}

			m_extents[LargestExtent] -= Step;
		}
		if (!FoundBox)
		{
			m_extents[0] = m_extents[1] = m_extents[2] = r;
		}
		else
		{
			// Refine the box
			const cbtScalar Step = (m_radius - r) / 1024.0f;
			const int e0 = (1 << LargestExtent) & 3;
			const int e1 = (1 << e0) & 3;

			for (int j = 0; j < 1024; j++)
			{
				const cbtScalar Saved0 = m_extents[e0];
				const cbtScalar Saved1 = m_extents[e1];
				m_extents[e0] += Step;
				m_extents[e1] += Step;

				if (!testContainment())
				{
					m_extents[e0] = Saved0;
					m_extents[e1] = Saved1;
					break;
				}
			}
		}
	}
#endif
}
void cbtConvexPolyhedron::project(const cbtTransform& trans, const cbtVector3& dir, cbtScalar& minProj, cbtScalar& maxProj, cbtVector3& witnesPtMin, cbtVector3& witnesPtMax) const
{
	minProj = FLT_MAX;
	maxProj = -FLT_MAX;
	int numVerts = m_vertices.size();
	for (int i = 0; i < numVerts; i++)
	{
		cbtVector3 pt = trans * m_vertices[i];
		cbtScalar dp = pt.dot(dir);
		if (dp < minProj)
		{
			minProj = dp;
			witnesPtMin = pt;
		}
		if (dp > maxProj)
		{
			maxProj = dp;
			witnesPtMax = pt;
		}
	}
	if (minProj > maxProj)
	{
		cbtSwap(minProj, maxProj);
		cbtSwap(witnesPtMin, witnesPtMax);
	}
}
