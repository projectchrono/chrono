/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dgMeshEffect.h"
#include "dgConvexHull3d.h"
#include "dgStack.h"
#include "dgSphere.h"
#include <string.h>

#pragma warning(disable:4100)

class dgFlatClipEdgeAttr
{
	public:
	hacd::HaI32 m_rightIndex;
	hacd::HaI32 m_leftIndex;
	hacd::HaI32 m_leftEdgeAttr;
	hacd::HaI32 m_leftTwinAttr;
	hacd::HaI32 m_rightEdgeAttr;
	hacd::HaI32 m_rightTwinAttr;
	dgEdge* m_edge;
	dgEdge* m_twin;
};

dgMeshEffect::dgMeshEffect(bool preAllocaBuffers)
{
	Init(preAllocaBuffers);
}

dgMeshEffect::dgMeshEffect (const dgMatrix& planeMatrix, hacd::HaF32 witdth, hacd::HaF32 breadth, hacd::HaI32 material, const dgMatrix& textureMatrix0, const dgMatrix& textureMatrix1)
{
	hacd::HaI32 index[4];
	hacd::HaI64 attrIndex[4];
	dgBigVector face[4];

	Init(true);

	face[0] = dgBigVector (hacd::HaF32 (0.0f), -witdth, -breadth, hacd::HaF32 (0.0f));
	face[1] = dgBigVector (hacd::HaF32 (0.0f),  witdth, -breadth, hacd::HaF32 (0.0f));
	face[2] = dgBigVector (hacd::HaF32 (0.0f),  witdth,  breadth, hacd::HaF32 (0.0f));
	face[3] = dgBigVector (hacd::HaF32 (0.0f), -witdth,  breadth, hacd::HaF32 (0.0f));

	for (hacd::HaI32 i = 0; i < 4; i ++) {
		dgBigVector uv0 (textureMatrix0.TransformVector(face[i]));
		dgBigVector uv1 (textureMatrix1.TransformVector(face[i]));

		m_points[i] = planeMatrix.TransformVector(face[i]);

		m_attib[i].m_vertex.m_x = m_points[i].m_x;
		m_attib[i].m_vertex.m_y = m_points[i].m_y;
		m_attib[i].m_vertex.m_z = m_points[i].m_z;
		m_attib[i].m_vertex.m_w = hacd::HaF64 (0.0f);

		m_attib[i].m_normal_x = planeMatrix.m_front.m_x;
		m_attib[i].m_normal_y = planeMatrix.m_front.m_y;
		m_attib[i].m_normal_z = planeMatrix.m_front.m_z;
		
		m_attib[i].m_u0 = uv0.m_y;
		m_attib[i].m_v0 = uv0.m_z;

		m_attib[i].m_u1 = uv1.m_y;
		m_attib[i].m_v1 = uv1.m_z;

		m_attib[i].m_material = material;

		index[i] = i;
		attrIndex[i] = i;
	}

	m_pointCount = 4;
	m_atribCount = 4;
	BeginFace();
	AddFace (4, index, attrIndex);
	EndFace();
}


dgMeshEffect::dgMeshEffect(dgPolyhedra& mesh, const dgMeshEffect& source)
	:dgPolyhedra (mesh) 
{
	m_pointCount = source.m_pointCount;
	m_maxPointCount = source.m_maxPointCount;
	m_points = (dgBigVector*) HACD_ALLOC(hacd::HaI32 (m_maxPointCount * sizeof(dgBigVector)));
	memcpy (m_points, source.m_points, m_pointCount * sizeof(dgBigVector));

	m_atribCount = source.m_atribCount;
	m_maxAtribCount = source.m_maxAtribCount;
	m_attib = (dgVertexAtribute*) HACD_ALLOC(hacd::HaI32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
	memcpy (m_attib, source.m_attib, m_atribCount * sizeof(dgVertexAtribute));
}


dgMeshEffect::dgMeshEffect(const dgMeshEffect& source)
	:dgPolyhedra (source) 
{
	m_pointCount = source.m_pointCount;
	m_maxPointCount = source.m_maxPointCount;
	m_points = (dgBigVector*) HACD_ALLOC(hacd::HaI32 (m_maxPointCount * sizeof(dgBigVector)));
	memcpy (m_points, source.m_points, m_pointCount * sizeof(dgBigVector));

	m_atribCount = source.m_atribCount;
	m_maxAtribCount = source.m_maxAtribCount;
	m_attib = (dgVertexAtribute*) HACD_ALLOC(hacd::HaI32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
	memcpy (m_attib, source.m_attib, m_atribCount * sizeof(dgVertexAtribute));
}


dgMeshEffect::~dgMeshEffect(void)
{
	HACD_FREE (m_points);
	HACD_FREE (m_attib);
}

void dgMeshEffect::Init(bool preAllocaBuffers)
{
	m_pointCount = 0;
	m_atribCount = 0;
	m_maxPointCount = DG_MESH_EFFECT_INITIAL_VERTEX_SIZE;
	m_maxAtribCount = DG_MESH_EFFECT_INITIAL_VERTEX_SIZE;

	m_points = NULL;
	m_attib = NULL;
	if (preAllocaBuffers) {
		m_points = (dgBigVector*) HACD_ALLOC(hacd::HaI32 (m_maxPointCount * sizeof(dgBigVector)));
		m_attib = (dgVertexAtribute*) HACD_ALLOC(hacd::HaI32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
	}
}


void dgMeshEffect::Triangulate  ()
{
	dgPolyhedra polygon;

	hacd::HaI32 mark = IncLRU();
	polygon.BeginFace();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);

		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			hacd::HaI32	index[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			hacd::HaI32 indexCount = 0;
			do {
				hacd::HaI32 attribIndex = hacd::HaI32 (ptr->m_userData);
				m_attib[attribIndex].m_vertex.m_w = hacd::HaF64 (ptr->m_incidentVertex);
				ptr->m_mark = mark;
				index[indexCount] = attribIndex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, index);
		}
	}
	polygon.EndFace();


	dgPolyhedra leftOversOut;
	polygon.Triangulate(&m_attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &leftOversOut);
	HACD_ASSERT (leftOversOut.GetCount() == 0);


	RemoveAll();
	SetLRU (0);

	mark = polygon.IncLRU();
	BeginFace();
	dgPolyhedra::Iterator iter1 (polygon);
	for (iter1.Begin(); iter1; iter1 ++){
		dgEdge* const face = &(*iter1);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			hacd::HaI32	index[DG_MESH_EFFECT_POINT_SPLITED];
			hacd::HaI64	userData[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			hacd::HaI32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = hacd::HaI32 (m_attib[ptr->m_incidentVertex].m_vertex.m_w);

				userData[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();

	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) {
			hacd::HaI32 attribIndex = hacd::HaI32 (face->m_userData);
			m_attib[attribIndex].m_vertex.m_w = m_points[face->m_incidentVertex].m_w;
		}
	}

}

void dgMeshEffect::ConvertToPolygons ()
{
	dgPolyhedra polygon;

	hacd::HaI32 mark = IncLRU();
	polygon.BeginFace();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			hacd::HaI32	index[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			hacd::HaI32 indexCount = 0;
			do {
				hacd::HaI32 attribIndex = hacd::HaI32 (ptr->m_userData);

				m_attib[attribIndex].m_vertex.m_w = hacd::HaF32 (ptr->m_incidentVertex);
				ptr->m_mark = mark;
				index[indexCount] = attribIndex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, index);
		}
	}
	polygon.EndFace();

	dgPolyhedra leftOversOut;
	polygon.ConvexPartition (&m_attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &leftOversOut);
	HACD_ASSERT (leftOversOut.GetCount() == 0);

	RemoveAll();
	SetLRU (0);

	mark = polygon.IncLRU();
	BeginFace();
	dgPolyhedra::Iterator iter1 (polygon);
	for (iter1.Begin(); iter1; iter1 ++){
		dgEdge* const face = &(*iter1);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			hacd::HaI32	index[DG_MESH_EFFECT_POINT_SPLITED];
			hacd::HaI64	userData[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			hacd::HaI32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = hacd::HaI32 (m_attib[ptr->m_incidentVertex].m_vertex.m_w);

				userData[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();

	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) {
			hacd::HaI32 attribIndex = hacd::HaI32 (face->m_userData);
			m_attib[attribIndex].m_vertex.m_w = m_points[face->m_incidentVertex].m_w;
		}
	}

	RepairTJoints (false);
}

void dgMeshEffect::RemoveUnusedVertices(hacd::HaI32* const vertexMap)
{
	dgPolyhedra polygon;
	dgStack<hacd::HaI32>attrbMap(m_atribCount);

	memset(&vertexMap[0], -1, m_pointCount * sizeof (int));
	memset(&attrbMap[0], -1, m_atribCount * sizeof (int));

	int attribCount = 0;
	int vertexCount = 0;

	dgStack<dgBigVector>points (m_pointCount);
	dgStack<dgVertexAtribute>atributes (m_atribCount);

	hacd::HaI32 mark = IncLRU();
	polygon.BeginFace();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			hacd::HaI32	vertex[DG_MESH_EFFECT_POINT_SPLITED];
			hacd::HaI64	userData[DG_MESH_EFFECT_POINT_SPLITED];
			int indexCount = 0;
			dgEdge* ptr = face;
			do {
				ptr->m_mark = mark;

				int index = ptr->m_incidentVertex;
				if (vertexMap[index] == -1) {
					vertexMap[index] = vertexCount;
					points[vertexCount] = m_points[index];
					vertexCount ++;
				}
				vertex[indexCount] = vertexMap[index];

				index = int (ptr->m_userData);
				if (attrbMap[index] == -1) {
					attrbMap[index] = attribCount;
					atributes[attribCount] = m_attib[index];
					attribCount ++;
				}
				userData[indexCount] = attrbMap[index];
				indexCount ++;

				ptr = ptr->m_next;
			} while (ptr != face);
			polygon.AddFace(indexCount, vertex, userData);
		}
	}
	polygon.EndFace();

	m_pointCount = vertexCount;
	memcpy (&m_points[0].m_x, &points[0].m_x, m_pointCount * sizeof (dgBigVector));
	 
	m_atribCount = attribCount;
	memcpy (&m_attib[0].m_vertex.m_x, &atributes[0].m_vertex.m_x, m_atribCount * sizeof (dgVertexAtribute));


	RemoveAll();
	SetLRU (0);

	BeginFace();
	dgPolyhedra::Iterator iter1 (polygon);
	for (iter1.Begin(); iter1; iter1 ++){
		dgEdge* const face = &(*iter1);
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			hacd::HaI32	index[DG_MESH_EFFECT_POINT_SPLITED];
			hacd::HaI64	userData[DG_MESH_EFFECT_POINT_SPLITED];

			dgEdge* ptr = face;
			hacd::HaI32 indexCount = 0;
			do {
				ptr->m_mark = mark;
				index[indexCount] = ptr->m_incidentVertex;
				userData[indexCount] = hacd::HaI64 (ptr->m_userData);
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace(indexCount, index, userData);
		}
	}
	EndFace();
	PackVertexArrays ();
}

dgMatrix dgMeshEffect::CalculateOOBB (dgBigVector& size) const
{
	dgSphere sphere (CalculateSphere (&m_points[0].m_x, sizeof (dgBigVector), NULL));
	size = sphere.m_size;

	dgMatrix permuation (dgGetIdentityMatrix());
	permuation[0][0] = hacd::HaF32 (0.0f);
	permuation[0][1] = hacd::HaF32 (1.0f);
	permuation[1][1] = hacd::HaF32 (0.0f);
	permuation[1][2] = hacd::HaF32 (1.0f);
	permuation[2][2] = hacd::HaF32 (0.0f);
	permuation[2][0] = hacd::HaF32 (1.0f);

	while ((size.m_x < size.m_y) || (size.m_x < size.m_z)) {
		sphere = permuation * sphere;
		size = permuation.UnrotateVector(size);
	}

	return sphere;
}

void dgMeshEffect::CalculateAABB (dgBigVector& minBox, dgBigVector& maxBox) const
{
	dgBigVector minP ( hacd::HaF64 (1.0e15f),  hacd::HaF64 (1.0e15f),  hacd::HaF64 (1.0e15f), hacd::HaF64 (0.0f)); 
	dgBigVector maxP (-hacd::HaF64 (1.0e15f), -hacd::HaF64 (1.0e15f), -hacd::HaF64 (1.0e15f), hacd::HaF64 (0.0f)); 

	dgPolyhedra::Iterator iter (*this);
	const dgBigVector* const points = &m_points[0];
	for (iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		const dgBigVector& p (points[edge->m_incidentVertex]);

		minP.m_x = GetMin (p.m_x, minP.m_x); 
		minP.m_y = GetMin (p.m_y, minP.m_y); 
		minP.m_z = GetMin (p.m_z, minP.m_z); 

		maxP.m_x = GetMax (p.m_x, maxP.m_x); 
		maxP.m_y = GetMax (p.m_y, maxP.m_y); 
		maxP.m_z = GetMax (p.m_z, maxP.m_z); 
	}

	minBox = minP;
	maxBox = maxP;
}

void dgMeshEffect::EnumerateAttributeArray (dgVertexAtribute* const attib)
{
	hacd::HaI32 index = 0;
	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		HACD_ASSERT (index < GetCount());
		attib[index] = m_attib[hacd::HaI32 (edge->m_userData)];
		edge->m_userData = hacd::HaU64 (index);
		index ++;
	}
}

void dgMeshEffect::ApplyAttributeArray (dgVertexAtribute* const attib)
{
	dgStack<hacd::HaI32>indexMap (GetCount());

	m_atribCount = dgVertexListToIndexList (&attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof(hacd::HaF64), GetCount(), &indexMap[0], DG_VERTEXLIST_INDEXLIST_TOL);
	m_maxAtribCount = m_atribCount;

	HACD_FREE (m_attib);
	m_attib = (dgVertexAtribute*) HACD_ALLOC(hacd::HaI32 (m_atribCount * sizeof(dgVertexAtribute)));
	memcpy (m_attib, attib, m_atribCount * sizeof(dgVertexAtribute));

	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		hacd::HaI32 index = indexMap[hacd::HaI32 (edge->m_userData)];
		HACD_ASSERT (index >=0);
		HACD_ASSERT (index < m_atribCount);
		edge->m_userData = hacd::HaU64 (index);
	}
}

dgBigVector dgMeshEffect::GetOrigin ()const
{
	dgBigVector origin (hacd::HaF64 (0.0f), hacd::HaF64 (0.0f), hacd::HaF64 (0.0f), hacd::HaF64 (0.0f));	
	for (hacd::HaI32 i = 0; i < m_pointCount; i ++) {
		origin += m_points[i];
	}	
	return origin.Scale (hacd::HaF64 (1.0f) / m_pointCount);
}


void dgMeshEffect::FixCylindricalMapping (dgVertexAtribute* attribArray) const
{
	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		dgVertexAtribute& attrib0 = attribArray[hacd::HaI32 (edge->m_userData)];
		dgVertexAtribute& attrib1 = attribArray[hacd::HaI32 (edge->m_next->m_userData)];

		hacd::HaF64 error = fabs (attrib0.m_u0 - attrib1.m_u0);
		if (error > hacd::HaF32 (0.6f)) {
			if (attrib0.m_u0 < attrib1.m_u0) {
				attrib0.m_u0 += hacd::HaF32 (1.0f);
				attrib0.m_u1 = attrib0.m_u0;
			} else {
				attrib1.m_u0 += hacd::HaF32 (1.0f);
				attrib1.m_u1 = attrib1.m_u0;
			}
			
		}
	}

	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		dgVertexAtribute& attrib0 = attribArray[hacd::HaI32 (edge->m_userData)];
		dgVertexAtribute& attrib1 = attribArray[hacd::HaI32 (edge->m_next->m_userData)];

		hacd::HaF64 error = fabs (attrib0.m_u0 - attrib1.m_u0);
		if (error > hacd::HaF32 (0.6f)) {
			if (attrib0.m_u0 < attrib1.m_u0) {
				attrib0.m_u0 += hacd::HaF32 (1.0f);
				attrib0.m_u1 = attrib0.m_u0;
			} else {
				attrib1.m_u0 += hacd::HaF32 (1.0f);
				attrib1.m_u1 = attrib1.m_u0;
			}
		}
	}
}


void dgMeshEffect::SphericalMapping (hacd::HaI32 material)
{
	dgBigVector origin (GetOrigin());

	dgStack<dgBigVector>sphere (m_pointCount);
	for (hacd::HaI32 i = 0; i < m_pointCount; i ++) {
		dgBigVector point (m_points[i] - origin);
		point = point.Scale (1.0f / dgSqrt (point % point));

		hacd::HaF64 u = dgAsin (point.m_y);
		hacd::HaF64 v = dgAtan2 (point.m_x, point.m_z);

		u = (hacd::HaF64 (3.1416f/2.0f) - u) / hacd::HaF64 (3.1416f);
		v = (hacd::HaF64 (3.1416f) - v) / hacd::HaF64 (2.0f * 3.1416f);
		sphere[i].m_x = v;
		sphere[i].m_y = u;
	}


	dgStack<dgVertexAtribute>attribArray (GetCount());
	EnumerateAttributeArray (&attribArray[0]);

	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		dgVertexAtribute& attrib = attribArray[hacd::HaI32 (edge->m_userData)];
		attrib.m_u0 = sphere[edge->m_incidentVertex].m_x;
		attrib.m_v0 = sphere[edge->m_incidentVertex].m_y;
		attrib.m_u1 = sphere[edge->m_incidentVertex].m_x;
		attrib.m_v1 = sphere[edge->m_incidentVertex].m_y;
		attrib.m_material = material;
	}

	FixCylindricalMapping (&attribArray[0]);
	ApplyAttributeArray (&attribArray[0]);
}

void dgMeshEffect::CylindricalMapping (hacd::HaI32 cylinderMaterial, hacd::HaI32 capMaterial)
{
/*
	dgVector origin (GetOrigin());

	dgStack<dgVector>cylinder (m_pointCount);

	hacd::HaF32 xMax;
	hacd::HaF32 xMin;

	xMin= hacd::HaF32 (1.0e10f);
	xMax= hacd::HaF32 (-1.0e10f);
	for (hacd::HaI32 i = 0; i < m_pointCount; i ++) {
		cylinder[i] = m_points[i] - origin;
		xMin = GetMin (xMin, cylinder[i].m_x);
		xMax = GetMax (xMax, cylinder[i].m_x);
	}

	hacd::HaF32 xscale = hacd::HaF32 (1.0f)/ (xMax - xMin);
	for (hacd::HaI32 i = 0; i < m_pointCount; i ++) {
		hacd::HaF32 u;
		hacd::HaF32 v;
		hacd::HaF32 y;
		hacd::HaF32 z;
		y = cylinder[i].m_y;
		z = cylinder[i].m_z;
		u = dgAtan2 (z, y);
		if (u < hacd::HaF32 (0.0f)) {
			u += hacd::HaF32 (3.141592f * 2.0f);
		}
		v = (cylinder[i].m_x - xMin) * xscale;

		cylinder[i].m_x = hacd::HaF32 (1.0f) - u * hacd::HaF32 (1.0f / (2.0f * 3.141592f));
		cylinder[i].m_y = v;
	}

	dgStack<dgVertexAtribute>attribArray (GetCount());
	EnumerateAttributeArray (&attribArray[0]);

	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge;
		edge = &(*iter);
		dgVertexAtribute& attrib = attribArray[hacd::HaI32 (edge->m_userData)];
		attrib.m_u0 = cylinder[edge->m_incidentVertex].m_x;
		attrib.m_v0 = cylinder[edge->m_incidentVertex].m_y;
		attrib.m_u1 = cylinder[edge->m_incidentVertex].m_x;
		attrib.m_v1 = cylinder[edge->m_incidentVertex].m_y;
		attrib.m_material = cylinderMaterial;
	}

	FixCylindricalMapping (&attribArray[0]);

	hacd::HaI32 mark;
	mark = IncLRU();
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge;
		edge = &(*iter);
		if (edge->m_mark < mark){
			const dgVector& p0 = m_points[edge->m_incidentVertex];
			const dgVector& p1 = m_points[edge->m_next->m_incidentVertex];
			const dgVector& p2 = m_points[edge->m_prev->m_incidentVertex];

			edge->m_mark = mark;
			edge->m_next->m_mark = mark;
			edge->m_prev->m_mark = mark;

			dgVector e0 (p1 - p0);
			dgVector e1 (p2 - p0);
			dgVector n (e0 * e1);
			if ((n.m_x * n.m_x) > (hacd::HaF32 (0.99f) * (n % n))) {
				dgEdge* ptr;

				ptr = edge;
				do {
					dgVertexAtribute& attrib = attribArray[hacd::HaI32 (ptr->m_userData)];
					dgVector p (m_points[ptr->m_incidentVertex] - origin);
					p.m_x = hacd::HaF32 (0.0f);
					p = p.Scale (hacd::HaF32 (dgRsqrt(p % p)));
					attrib.m_u0 = hacd::HaF32 (0.5f) + p.m_y * hacd::HaF32 (0.5f);
					attrib.m_v0 = hacd::HaF32 (0.5f) + p.m_z * hacd::HaF32 (0.5f);
					attrib.m_u1 = hacd::HaF32 (0.5f) + p.m_y * hacd::HaF32 (0.5f);
					attrib.m_v1 = hacd::HaF32 (0.5f) + p.m_z * hacd::HaF32 (0.5f);
					attrib.m_material = capMaterial;

					ptr = ptr->m_next;
				}while (ptr !=  edge);
			}
		}
	}

	ApplyAttributeArray (&attribArray[0]);
*/

	dgBigVector origin (GetOrigin());
	dgStack<dgBigVector>cylinder (m_pointCount);

	dgBigVector pMin (hacd::HaF64 (1.0e10f), hacd::HaF64 (1.0e10f), hacd::HaF64 (1.0e10f), hacd::HaF64 (0.0f));
	dgBigVector pMax (hacd::HaF64 (-1.0e10f), hacd::HaF64 (-1.0e10f), hacd::HaF64 (-1.0e10f), hacd::HaF64 (0.0f));
	for (hacd::HaI32 i = 0; i < m_pointCount; i ++) {
		dgBigVector tmp (m_points[i] - origin);
		pMin.m_x = GetMin (pMin.m_x, tmp.m_x);
		pMax.m_x = GetMax (pMax.m_x, tmp.m_x);
		pMin.m_y = GetMin (pMin.m_y, tmp.m_y);
		pMax.m_y = GetMax (pMax.m_y, tmp.m_y);
		pMin.m_z = GetMin (pMin.m_z, tmp.m_z);
		pMax.m_z = GetMax (pMax.m_z, tmp.m_z);
	}

	dgBigVector scale (hacd::HaF64 (1.0f)/ (pMax.m_x - pMin.m_x), hacd::HaF64 (1.0f)/ (pMax.m_x - pMin.m_x), hacd::HaF64 (1.0f)/ (pMax.m_x - pMin.m_x), hacd::HaF64 (0.0f));
	for (hacd::HaI32 i = 0; i < m_pointCount; i ++) {
		dgBigVector point (m_points[i] - origin);
		hacd::HaF64 u = (point.m_x - pMin.m_x) * scale.m_x;

		point = point.Scale (1.0f / dgSqrt (point % point));
		hacd::HaF64 v = dgAtan2 (point.m_y, point.m_z);

		//u = (hacd::HaF64 (3.1416f/2.0f) - u) / hacd::HaF64 (3.1416f);
		v = (hacd::HaF64 (3.1416f) - v) / hacd::HaF64 (2.0f * 3.1416f);
		cylinder[i].m_x = v;
		cylinder[i].m_y = u;
	}


	dgStack<dgVertexAtribute>attribArray (GetCount());
	EnumerateAttributeArray (&attribArray[0]);

	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		dgVertexAtribute& attrib = attribArray[hacd::HaI32 (edge->m_userData)];
		attrib.m_u0 = cylinder[edge->m_incidentVertex].m_x;
		attrib.m_v0 = cylinder[edge->m_incidentVertex].m_y;
		attrib.m_u1 = cylinder[edge->m_incidentVertex].m_x;
		attrib.m_v1 = cylinder[edge->m_incidentVertex].m_y;
		attrib.m_material = cylinderMaterial;
	}

	FixCylindricalMapping (&attribArray[0]);

	// apply cap mapping
	hacd::HaI32 mark = IncLRU();
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if (edge->m_mark < mark){
			const dgVector& p0 = m_points[edge->m_incidentVertex];
			const dgVector& p1 = m_points[edge->m_next->m_incidentVertex];
			const dgVector& p2 = m_points[edge->m_prev->m_incidentVertex];

			edge->m_mark = mark;
			edge->m_next->m_mark = mark;
			edge->m_prev->m_mark = mark;

			dgVector e0 (p1 - p0);
			dgVector e1 (p2 - p0);
			dgVector n (e0 * e1);
			if ((n.m_x * n.m_x) > (hacd::HaF32 (0.99f) * (n % n))) {
				dgEdge* ptr = edge;
				do {
					dgVertexAtribute& attrib = attribArray[hacd::HaI32 (ptr->m_userData)];
					dgVector p (m_points[ptr->m_incidentVertex] - origin);
					hacd::HaF64 u = (p.m_y - pMin.m_y) * scale.m_y;
					hacd::HaF64 v = (p.m_z - pMin.m_z) * scale.m_z;
					attrib.m_u0 = u;
					attrib.m_v0 = v;
					attrib.m_u1 = u;
					attrib.m_v1 = v;
					attrib.m_material = capMaterial;

					ptr = ptr->m_next;
				}while (ptr !=  edge);
			}
		}
	}

	ApplyAttributeArray (&attribArray[0]);
}

void dgMeshEffect::BoxMapping (hacd::HaI32 front, hacd::HaI32 side, hacd::HaI32 top)
{
	dgBigVector minVal;
	dgBigVector maxVal;
	hacd::HaI32 materialArray[3];

	GetMinMax (minVal, maxVal, &m_points[0][0], m_pointCount, sizeof (dgBigVector));
	dgBigVector dist (maxVal - minVal);
	dgBigVector scale (hacd::HaF64 (1.0f)/ dist[0], hacd::HaF64 (1.0f)/ dist[1], hacd::HaF64 (1.0f)/ dist[2], hacd::HaF64 (0.0f));

	dgStack<dgVertexAtribute>attribArray (GetCount());
	EnumerateAttributeArray (&attribArray[0]);

	materialArray[0] = front;
	materialArray[1] = side;
	materialArray[2] = top;

	hacd::HaI32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if (edge->m_mark < mark){
			const dgBigVector& p0 = m_points[edge->m_incidentVertex];
			const dgBigVector& p1 = m_points[edge->m_next->m_incidentVertex];
			const dgBigVector& p2 = m_points[edge->m_prev->m_incidentVertex];

			edge->m_mark = mark;
			edge->m_next->m_mark = mark;
			edge->m_prev->m_mark = mark;

			dgBigVector e0 (p1 - p0);
			dgBigVector e1 (p2 - p0);
			dgBigVector n (e0 * e1);
			
			hacd::HaI32 index = 0;
			hacd::HaF64 maxProjection = hacd::HaF32 (0.0f);

			for (hacd::HaI32 i = 0; i < 3; i ++) {
				hacd::HaF64 proj = fabs (n[i]);
				if (proj > maxProjection) {
					index = i;
					maxProjection = proj;
				}
			}

			hacd::HaI32 u = (index + 1) % 3;
			hacd::HaI32 v = (u + 1) % 3;
			if (index == 1) {
				Swap (u, v);
			}
			dgEdge* ptr = edge;
			do {
				dgVertexAtribute& attrib = attribArray[hacd::HaI32 (ptr->m_userData)];
				dgBigVector p (scale.CompProduct(m_points[ptr->m_incidentVertex] - minVal));
				attrib.m_u0 = p[u];
				attrib.m_v0 = p[v];
				attrib.m_u1 = hacd::HaF64(0.0f);
				attrib.m_v1 = hacd::HaF64(0.0f);
				attrib.m_material = materialArray[index];

				ptr = ptr->m_next;
			}while (ptr !=  edge);
		}
	}

	ApplyAttributeArray (&attribArray[0]);
}

void dgMeshEffect::UniformBoxMapping (hacd::HaI32 material, const dgMatrix& textureMatrix)
{
	dgStack<dgVertexAtribute>attribArray (GetCount());
	EnumerateAttributeArray (&attribArray[0]);

	hacd::HaI32 mark = IncLRU();
	for (hacd::HaI32 i = 0; i < 3; i ++) {
		dgMatrix rotationMatrix (dgGetIdentityMatrix());
		if (i == 1) {
			rotationMatrix = dgYawMatrix(hacd::HaF32 (90.0f * 3.1416f / 180.0f));
		} else if (i == 2) {
			rotationMatrix = dgPitchMatrix(hacd::HaF32 (90.0f * 3.1416f / 180.0f));
		}

		dgPolyhedra::Iterator iter (*this);	

		for(iter.Begin(); iter; iter ++){
			dgEdge* const edge = &(*iter);
			if (edge->m_mark < mark){
				dgBigVector n (FaceNormal(edge, &m_points[0].m_x, sizeof (dgBigVector)));
				dgVector normal (rotationMatrix.RotateVector(dgVector (n.Scale (hacd::HaF64 (1.0f) / sqrt (n % n)))));
				normal.m_x = dgAbsf (normal.m_x);
				normal.m_y = dgAbsf (normal.m_y);
				normal.m_z = dgAbsf (normal.m_z);
				if ((normal.m_z >= (normal.m_x - hacd::HaF32 (1.0e-4f))) && (normal.m_z >= (normal.m_y - hacd::HaF32 (1.0e-4f)))) {
					dgEdge* ptr = edge;
					do {
						ptr->m_mark = mark;
						dgVertexAtribute& attrib = attribArray[hacd::HaI32 (ptr->m_userData)];
						dgVector p (textureMatrix.TransformVector(rotationMatrix.RotateVector(m_points[ptr->m_incidentVertex])));
						attrib.m_u0 = p.m_x;
						attrib.m_v0 = p.m_y;
						attrib.m_u1 = hacd::HaF32 (0.0f);
						attrib.m_v1 = hacd::HaF32 (0.0f);
						attrib.m_material = material;
						ptr = ptr->m_next;
					}while (ptr !=  edge);
				}
			}
		}
	}

	ApplyAttributeArray (&attribArray[0]);
}

void dgMeshEffect::CalculateNormals (hacd::HaF64 angleInRadians)
{
	dgStack<dgBigVector>faceNormal (GetCount());
	dgStack<dgVertexAtribute>attribArray (GetCount());
	EnumerateAttributeArray (&attribArray[0]);

	hacd::HaI32 faceIndex = 1;
	hacd::HaI32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);	
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
			dgEdge* ptr = edge;
			do {
				ptr->m_incidentFace = faceIndex;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr !=  edge);

			dgBigVector normal (FaceNormal (edge, &m_points[0].m_x, sizeof (m_points[0])));
			normal = normal.Scale (hacd::HaF32 (1.0f) / (sqrt(normal % normal) + hacd::HaF32(1.0e-16f)));
			faceNormal[faceIndex] = normal;
			faceIndex ++;
		}
	}


	hacd::HaF32 smoothValue = dgCos (angleInRadians); 
//smoothValue = -1;
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if (edge->m_incidentFace > 0) {
			dgBigVector normal0 (faceNormal[edge->m_incidentFace]);

			dgEdge* startEdge = edge;
			for (dgEdge* ptr = edge->m_prev->m_twin ; (ptr != edge) && (ptr->m_incidentFace > 0); ptr = ptr->m_prev->m_twin) {
				const dgBigVector& normal1 (faceNormal[ptr->m_incidentFace]);
				hacd::HaF64 dot = normal0 % normal1;
				if (dot < smoothValue) {
					break;
				}
				startEdge = ptr;
				normal0 = normal1;
			}

			dgBigVector normal (faceNormal[startEdge->m_incidentFace]);
			normal0 = normal;
			for (dgEdge* ptr = startEdge->m_twin->m_next; (ptr != startEdge) && (ptr->m_incidentFace > 0); ptr = ptr->m_twin->m_next) { 
				
				const dgBigVector& normal1 (faceNormal[ptr->m_incidentFace]);
				hacd::HaF64 dot = normal0 % normal1;
				if (dot < smoothValue)  {
					break;
				}
				normal += normal1;
				normal0 = normal1;
			} 

			normal = normal.Scale (hacd::HaF32 (1.0f) / (sqrt(normal % normal) + hacd::HaF32(1.0e-16f)));
			hacd::HaI32 edgeIndex = hacd::HaI32 (edge->m_userData);
			dgVertexAtribute& attrib = attribArray[edgeIndex];
			attrib.m_normal_x = normal.m_x;
			attrib.m_normal_y = normal.m_y;
			attrib.m_normal_z = normal.m_z;
		}
	}

	ApplyAttributeArray (&attribArray[0]);
}


void dgMeshEffect::BeginPolygon ()
{
	m_pointCount = 0;
	m_atribCount = 0;
	RemoveAll();
	BeginFace();
}


void dgMeshEffect::AddAtribute (const dgVertexAtribute& attib)
{
	if (m_atribCount >= m_maxAtribCount) {
		m_maxAtribCount *= 2;
		dgVertexAtribute* const attibArray = (dgVertexAtribute*) HACD_ALLOC(hacd::HaI32 (m_maxAtribCount * sizeof(dgVertexAtribute)));
		memcpy (attibArray, m_attib, m_atribCount * sizeof(dgVertexAtribute));
		HACD_FREE(m_attib);
		m_attib = attibArray;
	}

	m_attib[m_atribCount] = attib;
	m_attib[m_atribCount].m_vertex.m_x = QuantizeCordinade(m_attib[m_atribCount].m_vertex.m_x);
	m_attib[m_atribCount].m_vertex.m_y = QuantizeCordinade(m_attib[m_atribCount].m_vertex.m_y);
	m_attib[m_atribCount].m_vertex.m_z = QuantizeCordinade(m_attib[m_atribCount].m_vertex.m_z);
	m_atribCount ++;
}

void dgMeshEffect::AddVertex(const dgBigVector& vertex)
{
	if (m_pointCount >= m_maxPointCount) {
		m_maxPointCount *= 2;
		dgBigVector* const points = (dgBigVector*) HACD_ALLOC(hacd::HaI32 (m_maxPointCount * sizeof(dgBigVector)));
		memcpy (points, m_points, m_pointCount * sizeof(dgBigVector));
		HACD_FREE(m_points);
		m_points = points;
	}
	
	m_points[m_pointCount].m_x = QuantizeCordinade(vertex[0]);
	m_points[m_pointCount].m_y = QuantizeCordinade(vertex[1]);
	m_points[m_pointCount].m_z = QuantizeCordinade(vertex[2]);
	m_points[m_pointCount].m_w = vertex.m_w;
	m_pointCount ++;
}


void dgMeshEffect::AddPoint(const hacd::HaF64* vertex, hacd::HaI32 material)
{
	dgVertexAtribute attib;
	AddVertex(dgBigVector (vertex[0], vertex[1], vertex[2], vertex[3]));
	
	attib.m_vertex.m_x = m_points[m_pointCount - 1].m_x;
	attib.m_vertex.m_y = m_points[m_pointCount - 1].m_y;
	attib.m_vertex.m_z = m_points[m_pointCount - 1].m_z;
	attib.m_vertex.m_w = m_points[m_pointCount - 1].m_w;

	attib.m_normal_x = vertex[4];
	attib.m_normal_y = vertex[5];
	attib.m_normal_z = vertex[6];
	attib.m_u0 = vertex[7];
	attib.m_v0 = vertex[8];
	attib.m_u1 = vertex[9];
	attib.m_v1 = vertex[10];
	attib.m_material = material;

	AddAtribute (attib);
}

void dgMeshEffect::PackVertexArrays ()
{
	if (m_maxPointCount > m_pointCount) {
		dgBigVector* const points = (dgBigVector*) HACD_ALLOC(hacd::HaI32 (m_pointCount * sizeof(dgBigVector)));
		memcpy (points, m_points, m_pointCount * sizeof(dgBigVector));
		HACD_FREE(m_points);
		m_points = points;
		m_maxPointCount = m_pointCount;
	}


	if (m_maxAtribCount > m_atribCount) {
		dgVertexAtribute* const attibArray = (dgVertexAtribute*) HACD_ALLOC(hacd::HaI32 (m_atribCount * sizeof(dgVertexAtribute)));
		memcpy (attibArray, m_attib, m_atribCount * sizeof(dgVertexAtribute));
		HACD_FREE(m_attib);
		m_attib = attibArray;
		m_maxAtribCount = m_atribCount;
	}
}


void dgMeshEffect::AddPolygon (hacd::HaI32 count, const hacd::HaF64* const vertexList, hacd::HaI32 strideIndBytes, hacd::HaI32 material)
{
	hacd::HaI32 stride = hacd::HaI32 (strideIndBytes / sizeof (hacd::HaF64));

	if (count > 3) {
		dgPolyhedra polygon;
		hacd::HaI32 indexList[256];
		HACD_ASSERT (count < hacd::HaI32 (sizeof (indexList)/sizeof(indexList[0])));
		for (hacd::HaI32 i = 0; i < count; i ++) {
			indexList[i] = i;
		}

		polygon.BeginFace();
		polygon.AddFace(count, indexList, NULL);
		polygon.EndFace();
		polygon.Triangulate(vertexList, strideIndBytes, NULL);

		hacd::HaI32 mark = polygon.IncLRU();
		dgPolyhedra::Iterator iter (polygon);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) {
				hacd::HaI32 i0 = edge->m_incidentVertex;
				hacd::HaI32 i1 = edge->m_next->m_incidentVertex;
				hacd::HaI32 i2 = edge->m_next->m_next->m_incidentVertex;
				edge->m_mark = mark;
				edge->m_next->m_mark = mark;
				edge->m_next->m_next->m_mark = mark;

//				#ifdef _DEBUG
//					dgBigVector p0_ (&vertexList[i0 * stride]);
//					dgBigVector p1_ (&vertexList[i1 * stride]);
//					dgBigVector p2_ (&vertexList[i2 * stride]);
//					dgBigVector e1_ (p1_ - p0_);
//					dgBigVector e2_ (p2_ - p0_);
//					dgBigVector n_ (e1_ * e2_);
//					hacd::HaF64 mag2_ = n_ % n_;
//					HACD_ASSERT (mag2_ > hacd::HaF32 (DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)); 
//				#endif

				AddPoint(vertexList + i0 * stride, material);
				AddPoint(vertexList + i1 * stride, material);
				AddPoint(vertexList + i2 * stride, material);

				#ifdef _DEBUG
					const dgBigVector& p0 = m_points[m_pointCount - 3];
					const dgBigVector& p1 = m_points[m_pointCount - 2];
					const dgBigVector& p2 = m_points[m_pointCount - 1];
					dgBigVector e1 (p1 - p0);
					dgBigVector e2 (p2 - p0);
					dgBigVector n (e1 * e2);
					hacd::HaF64 mag3 = n % n;
					HACD_ASSERT (mag3 > hacd::HaF64 (DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV));
				#endif
			}
		}

	} else {

		AddPoint(vertexList, material);
		AddPoint(vertexList + stride, material);
		AddPoint(vertexList + stride + stride, material);

		const dgBigVector& p0 = m_points[m_pointCount - 3];
		const dgBigVector& p1 = m_points[m_pointCount - 2];
		const dgBigVector& p2 = m_points[m_pointCount - 1];
		dgBigVector e1 (p1 - p0);
		dgBigVector e2 (p2 - p0);
		dgBigVector n (e1 * e2);
		hacd::HaF64 mag3 = n % n;
		if (mag3 < hacd::HaF64 (DG_MESH_EFFECT_PRECISION_SCALE_INV * DG_MESH_EFFECT_PRECISION_SCALE_INV)) {
			m_pointCount -= 3;
			m_atribCount -= 3;
		}
	}
}


void dgMeshEffect::AddPolygon (hacd::HaI32 count, const hacd::HaF32* const vertexList, hacd::HaI32 strideIndBytes, hacd::HaI32 material)
{
	dgVertexAtribute points[256];
	HACD_ASSERT (count < hacd::HaI32 (sizeof (points)/sizeof (points[0])));

	hacd::HaI32 stride = strideIndBytes / sizeof (hacd::HaF32);
	for (hacd::HaI32 i = 0; i < count; i ++) 
	{
		points[i].m_vertex.m_x = vertexList[i * stride + 0];
		points[i].m_vertex.m_y = vertexList[i * stride + 1];
		points[i].m_vertex.m_z = vertexList[i * stride + 2];
		points[i].m_vertex.m_w = vertexList[i * stride + 3];

		points[i].m_normal_x = vertexList[i * stride + 4];
		points[i].m_normal_y = vertexList[i * stride + 5];
		points[i].m_normal_z = vertexList[i * stride + 6];

		points[i].m_u0 = vertexList[i * stride + 7];
		points[i].m_v0 = vertexList[i * stride + 8];

		points[i].m_u1 = vertexList[i * stride + 9];
		points[i].m_u1 = vertexList[i * stride + 10];
	}

	AddPolygon (count, &points[0].m_vertex.m_x, sizeof (dgVertexAtribute), material);
}

void dgMeshEffect::EndPolygon (hacd::HaF64 tol)
{
	dgStack<hacd::HaI32>indexMap(m_pointCount);
	dgStack<hacd::HaI32>attrIndexMap(m_atribCount);

#ifdef _DEBUG
	for (hacd::HaI32 i = 0; i < m_pointCount; i += 3) {
		dgBigVector p0 (m_points[i + 0]);
		dgBigVector p1 (m_points[i + 1]);
		dgBigVector p2 (m_points[i + 2]);
		dgBigVector e1 (p1 - p0);
		dgBigVector e2 (p2 - p0);
		dgBigVector n (e1 * e2);
		hacd::HaF64 mag2 = n % n;
//		HACD_ASSERT (mag2 > DG_MESH_EFFECT_TRIANGLE_MIN_AREA);
		HACD_ASSERT (mag2 > hacd::HaF32 (0.0f));
	}
#endif

	hacd::HaI32 triangCount = m_pointCount / 3;
	m_pointCount = dgVertexListToIndexList (&m_points[0].m_x, sizeof (dgBigVector), sizeof (dgBigVector)/sizeof (hacd::HaF64), m_pointCount, &indexMap[0], tol);
	m_atribCount = dgVertexListToIndexList (&m_attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute)/sizeof (hacd::HaF64), m_atribCount, &attrIndexMap[0], tol);

	for (hacd::HaI32 i = 0; i < triangCount; i ++) {
		hacd::HaI32 index[3];
		hacd::HaI64 userdata[3];

		index[0] = indexMap[i * 3 + 0];
		index[1] = indexMap[i * 3 + 1];
		index[2] = indexMap[i * 3 + 2];


		dgBigVector e1 (m_points[index[1]] - m_points[index[0]]);
		dgBigVector e2 (m_points[index[2]] - m_points[index[0]]);

		dgBigVector n (e1 * e2);
		hacd::HaF64 mag2 = n % n;
		if (mag2 > hacd::HaF64 (1.0e-12f)) {
			userdata[0] = attrIndexMap[i * 3 + 0];
			userdata[1] = attrIndexMap[i * 3 + 1];
			userdata[2] = attrIndexMap[i * 3 + 2];
			dgEdge* const edge = AddFace (3, index, userdata);
			if (!edge) {
				HACD_ASSERT ((m_pointCount + 3) <= m_maxPointCount);

				m_points[m_pointCount + 0] = m_points[index[0]];
				m_points[m_pointCount + 1] = m_points[index[1]];
				m_points[m_pointCount + 2] = m_points[index[2]];

				index[0] = m_pointCount + 0;
				index[1] = m_pointCount + 1;
				index[2] = m_pointCount + 2;

				m_pointCount += 3;

				#ifdef _DEBUG
					dgEdge* test = AddFace (3, index, userdata);
					HACD_ASSERT (test);
				#else 
					AddFace (3, index, userdata);
				#endif
			}
		}
	}
	EndFace();

	RepairTJoints (true);


#ifdef _DEBUG
	dgPolyhedra::Iterator iter (*this);	
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace > 0) {
			dgBigVector p0 (m_points[face->m_incidentVertex]);
			dgBigVector p1 (m_points[face->m_next->m_incidentVertex]);
			dgBigVector p2 (m_points[face->m_next->m_next->m_incidentVertex]);
			dgBigVector e1 (p1 - p0);
			dgBigVector e2 (p2 - p0);
			dgBigVector n (e1 * e2);
//			HACD_ASSERT (mag2 > DG_MESH_EFFECT_TRIANGLE_MIN_AREA);
			hacd::HaF64 mag2 = n % n;
			HACD_ASSERT (mag2 > hacd::HaF32 (0.0f));
		}
	}
#endif
}


void dgMeshEffect::BuildFromVertexListIndexList(
	hacd::HaI32 faceCount, const hacd::HaI32* const faceIndexCount, const hacd::HaI32* const faceMaterialIndex, 
	const hacd::HaF32* const vertex, hacd::HaI32 vertexStrideInBytes, const hacd::HaI32* const vertexIndex,
	const hacd::HaF32* const normal, hacd::HaI32  normalStrideInBytes, const hacd::HaI32* const normalIndex,
	const hacd::HaF32* const uv0, hacd::HaI32  uv0StrideInBytes, const hacd::HaI32* const uv0Index,
	const hacd::HaF32* const uv1, hacd::HaI32  uv1StrideInBytes, const hacd::HaI32* const uv1Index)
{
	BeginPolygon ();

	// calculate vertex Count
	hacd::HaI32 acc = 0;
	hacd::HaI32 vertexCount = 0;
	for (hacd::HaI32 j = 0; j < faceCount; j ++) {
		int count = faceIndexCount[j];
		for (int i = 0; i < count; i ++) {
			vertexCount = GetMax(vertexCount, vertexIndex[acc + i] + 1);
		}
		acc += count;
	}
		
	hacd::HaI32 layerCountBase = 0;
	hacd::HaI32 vertexStride = hacd::HaI32 (vertexStrideInBytes / sizeof (hacd::HaF32));
	for (int i = 0; i < vertexCount; i ++) {
		int index = i * vertexStride;
		AddVertex (dgBigVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], vertex[index + 3]));
		layerCountBase += (vertex[index + 3]) > hacd::HaF32(layerCountBase);
	}


	acc = 0;
	hacd::HaI32 normalStride = hacd::HaI32 (normalStrideInBytes / sizeof (hacd::HaF32));
	hacd::HaI32 uv0Stride = hacd::HaI32 (uv0StrideInBytes / sizeof (hacd::HaF32));
	hacd::HaI32 uv1Stride = hacd::HaI32 (uv1StrideInBytes / sizeof (hacd::HaF32));
	for (hacd::HaI32 j = 0; j < faceCount; j ++) {
		hacd::HaI32 indexCount = faceIndexCount[j];
		hacd::HaI32 materialIndex = faceMaterialIndex[j];
		for (hacd::HaI32 i = 0; i < indexCount; i ++) {
			dgVertexAtribute point;
			hacd::HaI32 index = vertexIndex[acc + i];
			point.m_vertex = m_points[index];
			
			index = normalIndex[(acc + i)] * normalStride;
			point.m_normal_x =  normal[index + 0];
			point.m_normal_y =  normal[index + 1];
			point.m_normal_z =  normal[index + 2];

			index = uv0Index[(acc + i)] * uv0Stride;
			point.m_u0 = uv0[index + 0];
			point.m_v0 = uv0[index + 1];
			
			index = uv1Index[(acc + i)] * uv1Stride;
			point.m_u1 = uv1[index + 0];
			point.m_v1 = uv1[index + 1];

			point.m_material = materialIndex;
			AddAtribute(point);
		}
		acc += indexCount;
	}


	dgStack<hacd::HaI32>attrIndexMap(m_atribCount);
	m_atribCount = dgVertexListToIndexList (&m_attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof (hacd::HaF64), m_atribCount, &attrIndexMap[0], DG_VERTEXLIST_INDEXLIST_TOL);

	bool hasFaces = true;
	dgStack<hacd::HaI8> faceMark (faceCount);
	memset (&faceMark[0], 1, size_t (faceMark.GetSizeInBytes()));
	
	hacd::HaI32 layerCount = 0;
	while (hasFaces) {
		acc = 0;
		hasFaces = false;
		hacd::HaI32 vertexBank = layerCount * vertexCount;
		for (hacd::HaI32 j = 0; j < faceCount; j ++) {
			hacd::HaI32 index[256];
			hacd::HaI64 userdata[256];

			int indexCount = faceIndexCount[j];
			HACD_ASSERT (indexCount < hacd::HaI32 (sizeof (index) / sizeof (index[0])));

			if (faceMark[j]) {
				for (int i = 0; i < indexCount; i ++) {
					index[i] = vertexIndex[acc + i] + vertexBank;
					userdata[i] = attrIndexMap[acc + i];
				}
				dgEdge* const edge = AddFace (indexCount, index, userdata);
				if (edge) {
					faceMark[j] = 0;
				} else {
					// check if the face is not degenerated
					bool degeneratedFace = false;
					for (int i = 0; i < indexCount - 1; i ++) {
						for (int k = i + 1; k < indexCount; k ++) {
							if (index[i] == index[k]) {
								degeneratedFace = true;		
							}
						}
					}
					if (degeneratedFace) {
						faceMark[j] = 0;
					} else {
						hasFaces = true;
					}
				}
			}
			acc += indexCount;
		}
		if (hasFaces) {
			layerCount ++;
			for (int i = 0; i < vertexCount; i ++) {
				int index = i * vertexStride;
				AddVertex (dgBigVector (vertex[index + 0], vertex[index + 1], vertex[index + 2], hacd::HaF64 (layerCount + layerCountBase)));
			}
		}
	}

	EndFace();
	PackVertexArrays ();
//	RepairTJoints (true);
}


hacd::HaI32 dgMeshEffect::GetTotalFaceCount() const
{
	return GetFaceCount();
}

hacd::HaI32 dgMeshEffect::GetTotalIndexCount() const
{
	Iterator iter (*this);
	hacd::HaI32 count = 0;
	hacd::HaI32 mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}
		
		dgEdge* ptr = edge;
		do {
			count ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}

void dgMeshEffect::GetFaces (hacd::HaI32* const facesIndex, hacd::HaI32* const materials, void** const faceNodeList) const
{
	Iterator iter (*this);

	hacd::HaI32 faces = 0;
	hacd::HaI32 indexCount = 0;
	hacd::HaI32 mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace < 0) {
			continue;
		}

		hacd::HaI32 faceCount = 0;
		dgEdge* ptr = edge;
		do {
//			indexList[indexCount] = hacd::HaI32 (ptr->m_userData);
			faceNodeList[indexCount] = GetNodeFromInfo (*ptr);
			indexCount ++;
			faceCount ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);

		facesIndex[faces] = faceCount;
		materials[faces] = dgFastInt(m_attib[hacd::HaI32 (edge->m_userData)].m_material);
		faces ++;
	}
}

void* dgMeshEffect::GetFirstVertex ()
{
	Iterator iter (*this);
	iter.Begin();

	dgTreeNode* node = NULL;
	if (iter) {
		hacd::HaI32 mark = IncLRU();
		node = iter.GetNode();

		dgEdge* const edge = &node->GetInfo();
		dgEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);
	}
	return node; 
}

void* dgMeshEffect::GetNextVertex (const void* const vertex)
{
	dgTreeNode* node = (dgTreeNode*) vertex;
	hacd::HaI32 mark = node->GetInfo().m_mark;

	Iterator iter (*this);
	iter.Set (node);
	for (iter ++; iter; iter ++) {
		dgTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			dgEdge* const edge = &node->GetInfo();
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			return node; 
		}
	}
	return NULL; 
}

hacd::HaI32 dgMeshEffect::GetVertexIndex (const void* const vertex) const
{
	dgTreeNode* const node = (dgTreeNode*) vertex;
	dgEdge* const edge = &node->GetInfo();
	return edge->m_incidentVertex;
}


void* dgMeshEffect::GetFirstPoint ()
{
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgTreeNode* const node = iter.GetNode();
		dgEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node;
		}
	}
	return NULL; 
}

void* dgMeshEffect::GetNextPoint (const void* const point)
{
	Iterator iter (*this);
	iter.Set ((dgTreeNode*) point);
	for (iter ++; iter; iter ++) {
		dgTreeNode* const node = iter.GetNode();
		dgEdge* const edge = &node->GetInfo();
		if (edge->m_incidentFace > 0) {
			return node; 
		}
	}
	return NULL; 
}

hacd::HaI32 dgMeshEffect::GetPointIndex (const void* const point) const
{
	dgTreeNode* const node = (dgTreeNode*) point;
	dgEdge* const edge = &node->GetInfo();
	return int (edge->m_userData);
}

hacd::HaI32 dgMeshEffect::GetVertexIndexFromPoint (const void* const point) const
{
	return GetVertexIndex (point);
}


dgEdge* dgMeshEffect::ConectVertex (dgEdge* const e0, dgEdge* const e1)
{
	dgEdge* const edge = AddHalfEdge(e1->m_incidentVertex, e0->m_incidentVertex);
	dgEdge* const twin = AddHalfEdge(e0->m_incidentVertex, e1->m_incidentVertex);
	HACD_ASSERT ((edge && twin) || !(edge || twin));
	if (edge) {
		edge->m_twin = twin;
		twin->m_twin = edge;

		edge->m_incidentFace = e0->m_incidentFace;
		twin->m_incidentFace = e1->m_incidentFace;

		edge->m_userData = e1->m_userData;
		twin->m_userData = e0->m_userData;

		edge->m_next = e0;
		edge->m_prev = e1->m_prev;

		twin->m_next = e1;
		twin->m_prev = e0->m_prev;

		e0->m_prev->m_next = twin;
		e0->m_prev = edge;

		e1->m_prev->m_next = edge;
		e1->m_prev = twin;
	}

	return edge;
}


//hacd::HaI32 dgMeshEffect::GetVertexAttributeIndex (const void* vertex) const
//{
//	dgTreeNode* node = (dgTreeNode*) vertex;
//	dgEdge* const edge = &node->GetInfo();
//	return int (edge->m_userData);
//}


void* dgMeshEffect::GetFirstEdge ()
{
	Iterator iter (*this);
	iter.Begin();

	dgTreeNode* node = NULL;
	if (iter) {
		hacd::HaI32 mark = IncLRU();

		node = iter.GetNode();

		dgEdge* const edge = &node->GetInfo();
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}
	return node; 
}

void* dgMeshEffect::GetNextEdge (const void* const edge)
{
	dgTreeNode* node = (dgTreeNode*) edge;
	hacd::HaI32 mark = node->GetInfo().m_mark;

	Iterator iter (*this);
	iter.Set (node);
	for (iter ++; iter; iter ++) {
		dgTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			node->GetInfo().m_mark = mark;
			node->GetInfo().m_twin->m_mark = mark;
			return node; 
		}
	}
	return NULL; 
}

void dgMeshEffect::GetEdgeIndex (const void* const edge, hacd::HaI32& v0, hacd::HaI32& v1) const
{
	dgTreeNode* node = (dgTreeNode*) edge;
	v0 = node->GetInfo().m_incidentVertex;
	v1 = node->GetInfo().m_twin->m_incidentVertex;
}

//void dgMeshEffect::GetEdgeAttributeIndex (const void* edge, hacd::HaI32& v0, hacd::HaI32& v1) const
//{
//	dgTreeNode* node = (dgTreeNode*) edge;
//	v0 = int (node->GetInfo().m_userData);
//	v1 = int (node->GetInfo().m_twin->m_userData);
//}


void* dgMeshEffect::GetFirstFace ()
{
	Iterator iter (*this);
	iter.Begin();

	dgTreeNode* node = NULL;
	if (iter) {
		hacd::HaI32 mark = IncLRU();
		node = iter.GetNode();

		dgEdge* const edge = &node->GetInfo();
		dgEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}

	return node;
}

void* dgMeshEffect::GetNextFace (const void* const face)
{
	dgTreeNode* node = (dgTreeNode*) face;
	hacd::HaI32 mark = node->GetInfo().m_mark;

	Iterator iter (*this);
	iter.Set (node);
	for (iter ++; iter; iter ++) {
		dgTreeNode* node = iter.GetNode();
		if (node->GetInfo().m_mark != mark) {
			dgEdge* const edge = &node->GetInfo();
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
			return node; 
		}
	}
	return NULL; 
}


hacd::HaI32 dgMeshEffect::IsFaceOpen (const void* const face) const
{
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	return (edge->m_incidentFace > 0) ? 0 : 1;
}

hacd::HaI32 dgMeshEffect::GetFaceMaterial (const void* const face) const
{
	dgTreeNode* const node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	return hacd::HaI32 (m_attib[edge->m_userData].m_material);
}

hacd::HaI32 dgMeshEffect::GetFaceIndexCount (const void* const face) const
{
	int count = 0;
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	dgEdge* ptr = edge;
	do {
		count ++;
		ptr = ptr->m_next;
	} while (ptr != edge);
	return count; 
}

void dgMeshEffect::GetFaceIndex (const void* const face, int* const indices) const
{
	int count = 0;
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	dgEdge* ptr = edge;
	do {
		indices[count] =  ptr->m_incidentVertex;
		count ++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}

void dgMeshEffect::GetFaceAttributeIndex (const void* const face, int* const indices) const
{
	int count = 0;
	dgTreeNode* node = (dgTreeNode*) face;
	dgEdge* const edge = &node->GetInfo();
	dgEdge* ptr = edge;
	do {
		indices[count] = int (ptr->m_userData);
		count ++;
		ptr = ptr->m_next;
	} while (ptr != edge);
}




/*
hacd::HaI32 GetTotalFaceCount() const;
{
	hacd::HaI32 mark;
	hacd::HaI32 count;
	hacd::HaI32 materialCount;
	hacd::HaI32 materials[256];
	hacd::HaI32 streamIndexMap[256];
	dgIndexArray* array; 

	count = 0;
	materialCount = 0;

	array = (dgIndexArray*) HACD_ALLOC (4 * sizeof (hacd::HaI32) * GetCount() + sizeof (dgIndexArray) + 2048);
	array->m_indexList = (hacd::HaI32*)&array[1];

	mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);	
	memset(streamIndexMap, 0, sizeof (streamIndexMap));
	for(iter.Begin(); iter; iter ++){

		dgEdge* const edge;
		edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark)) {
			dgEdge* ptr;
			hacd::HaI32 hashValue;
			hacd::HaI32 index0;
			hacd::HaI32 index1;

			ptr = edge;
			ptr->m_mark = mark;
			index0 = hacd::HaI32 (ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			index1 = hacd::HaI32 (ptr->m_userData);

			ptr = ptr->m_next;
			do {
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = hacd::HaI32 (ptr->m_userData);
				array->m_indexList[count * 4 + 3] = m_attib[hacd::HaI32 (edge->m_userData)].m_material;
				index1 = hacd::HaI32 (ptr->m_userData);

				hashValue = array->m_indexList[count * 4 + 3] & 0xff;
				streamIndexMap[hashValue] ++;
				materials[hashValue] = array->m_indexList[count * 4 + 3];
				count ++;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
*/




void dgMeshEffect::GetVertexStreams (hacd::HaI32 vetexStrideInByte, hacd::HaF32* const vertex, 
									 hacd::HaI32 normalStrideInByte, hacd::HaF32* const normal, 
									 hacd::HaI32 uvStrideInByte0, hacd::HaF32* const uv0, 
									 hacd::HaI32 uvStrideInByte1, hacd::HaF32* const uv1)
{
	uvStrideInByte0 /= sizeof (hacd::HaF32);
	uvStrideInByte1 /= sizeof (hacd::HaF32);
	vetexStrideInByte /= sizeof (hacd::HaF32);
	normalStrideInByte /= sizeof (hacd::HaF32);
	for (hacd::HaI32 i = 0; i < m_atribCount; i ++)	{
		hacd::HaI32 j = i * vetexStrideInByte;
		vertex[j + 0] = hacd::HaF32 (m_attib[i].m_vertex.m_x);
		vertex[j + 1] = hacd::HaF32 (m_attib[i].m_vertex.m_y);
		vertex[j + 2] = hacd::HaF32 (m_attib[i].m_vertex.m_z);

		j = i * normalStrideInByte;
		normal[j + 0] = hacd::HaF32 (m_attib[i].m_normal_x);
		normal[j + 1] = hacd::HaF32 (m_attib[i].m_normal_y);
		normal[j + 2] = hacd::HaF32 (m_attib[i].m_normal_z);

		j = i * uvStrideInByte1;
		uv1[j + 0] = hacd::HaF32 (m_attib[i].m_u1);
		uv1[j + 1] = hacd::HaF32 (m_attib[i].m_v1);

		j = i * uvStrideInByte0;
		uv0[j + 0] = hacd::HaF32 (m_attib[i].m_u0);
		uv0[j + 1] = hacd::HaF32 (m_attib[i].m_v0);
	}
}


void dgMeshEffect::GetIndirectVertexStreams(
	hacd::HaI32 vetexStrideInByte, hacd::HaF64* const vertex, hacd::HaI32* const vertexIndices, hacd::HaI32* const vertexCount,
	hacd::HaI32 normalStrideInByte, hacd::HaF64* const normal, hacd::HaI32* const normalIndices, hacd::HaI32* const normalCount,
	hacd::HaI32 uvStrideInByte0, hacd::HaF64* const uv0, hacd::HaI32* const uvIndices0, hacd::HaI32* const uvCount0,
	hacd::HaI32 uvStrideInByte1, hacd::HaF64* const uv1, hacd::HaI32* const uvIndices1, hacd::HaI32* const uvCount1)
{
/*
	GetVertexStreams (vetexStrideInByte, vertex, normalStrideInByte, normal, uvStrideInByte0, uv0, uvStrideInByte1, uv1);

	*vertexCount = dgVertexListToIndexList(vertex, vetexStrideInByte, vetexStrideInByte, 0, m_atribCount, vertexIndices, hacd::HaF32 (0.0f));
	*normalCount = dgVertexListToIndexList(normal, normalStrideInByte, normalStrideInByte, 0, m_atribCount, normalIndices, hacd::HaF32 (0.0f));

	dgTriplex* const tmpUV = (dgTriplex*) HACD_ALLOC (hacd::HaI32 (sizeof (dgTriplex) * m_atribCount));
	hacd::HaI32 stride = hacd::HaI32 (uvStrideInByte1 /sizeof (hacd::HaF32));
	for (hacd::HaI32 i = 0; i < m_atribCount; i ++){
		tmpUV[i].m_x = uv1[i * stride + 0];
		tmpUV[i].m_y = uv1[i * stride + 1];
		tmpUV[i].m_z = hacd::HaF32 (0.0f);
	}

	hacd::HaI32 count = dgVertexListToIndexList(&tmpUV[0].m_x, sizeof (dgTriplex), sizeof (dgTriplex), 0, m_atribCount, uvIndices1, hacd::HaF32 (0.0f));
	for (hacd::HaI32 i = 0; i < count; i ++){
		uv1[i * stride + 0] = tmpUV[i].m_x;
		uv1[i * stride + 1] = tmpUV[i].m_y;
	}
	*uvCount1 = count;

	stride = hacd::HaI32 (uvStrideInByte0 /sizeof (hacd::HaF32));
	for (hacd::HaI32 i = 0; i < m_atribCount; i ++){
		tmpUV[i].m_x = uv0[i * stride + 0];
		tmpUV[i].m_y = uv0[i * stride + 1];
		tmpUV[i].m_z = hacd::HaF32 (0.0f);
	}
	count = dgVertexListToIndexList(&tmpUV[0].m_x, sizeof (dgTriplex), sizeof (dgTriplex), 0, m_atribCount, uvIndices0, hacd::HaF32 (0.0f));
	for (hacd::HaI32 i = 0; i < count; i ++){
		uv0[i * stride + 0] = tmpUV[i].m_x;
		uv0[i * stride + 1] = tmpUV[i].m_y;
	}
	*uvCount0 = count;

	HACD_FREE (tmpUV);
*/
}

dgMeshEffect::dgIndexArray* dgMeshEffect::MaterialGeometryBegin()
{
	hacd::HaI32 materials[256];
	hacd::HaI32 streamIndexMap[256];

	hacd::HaI32 count = 0;
	hacd::HaI32 materialCount = 0;
	
	dgIndexArray* const array = (dgIndexArray*) HACD_ALLOC (hacd::HaI32 (4 * sizeof (hacd::HaI32) * GetCount() + sizeof (dgIndexArray) + 2048));
	array->m_indexList = (hacd::HaI32*)&array[1];
	
	hacd::HaI32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);	
	memset(streamIndexMap, 0, sizeof (streamIndexMap));
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark)) {
			dgEdge* ptr = edge;
			ptr->m_mark = mark;
			hacd::HaI32 index0 = hacd::HaI32 (ptr->m_userData);

			ptr = ptr->m_next;
			ptr->m_mark = mark;
			hacd::HaI32 index1 = hacd::HaI32 (ptr->m_userData);

			ptr = ptr->m_next;
			do {
				ptr->m_mark = mark;

				array->m_indexList[count * 4 + 0] = index0;
				array->m_indexList[count * 4 + 1] = index1;
				array->m_indexList[count * 4 + 2] = hacd::HaI32 (ptr->m_userData);
				array->m_indexList[count * 4 + 3] = hacd::HaI32 (m_attib[hacd::HaI32 (edge->m_userData)].m_material);
				index1 = hacd::HaI32 (ptr->m_userData);

				hacd::HaI32 hashValue = array->m_indexList[count * 4 + 3] & 0xff;
				streamIndexMap[hashValue] ++;
				materials[hashValue] = array->m_indexList[count * 4 + 3];
				count ++;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	array->m_indexCount = count;
	array->m_materialCount = materialCount;

	count = 0;
	for (hacd::HaI32 i = 0; i < 256;i ++) {
		if (streamIndexMap[i]) {
			array->m_materials[count] = materials[i];
			array->m_materialsIndexCount[count] = streamIndexMap[i] * 3;
			count ++;
		}
	}

	array->m_materialCount = count;

	return array;
}

void dgMeshEffect::MaterialGeomteryEnd(dgIndexArray* const handle)
{
	HACD_FREE (handle);
}


hacd::HaI32 dgMeshEffect::GetFirstMaterial (dgIndexArray* const handle)
{
	return GetNextMaterial (handle, -1);
}

hacd::HaI32 dgMeshEffect::GetNextMaterial (dgIndexArray* const handle, hacd::HaI32 materialId)
{
	materialId ++;
	if(materialId >= handle->m_materialCount) {
		materialId = -1;
	}
	return materialId;
}

void dgMeshEffect::GetMaterialGetIndexStream (dgIndexArray* const handle, hacd::HaI32 materialHandle, hacd::HaI32* const indexArray)
{
	hacd::HaI32 index;
	hacd::HaI32 textureID;

	index = 0;
	textureID = handle->m_materials[materialHandle];
	for (hacd::HaI32 j = 0; j < handle->m_indexCount; j ++) {
		if (handle->m_indexList[j * 4 + 3] == textureID) {
			indexArray[index + 0] = handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = handle->m_indexList[j * 4 + 2];

			index += 3;
		}
	}
}

void dgMeshEffect::GetMaterialGetIndexStreamShort (dgIndexArray* const handle, hacd::HaI32 materialHandle, hacd::HaI16* const indexArray)
{
	hacd::HaI32 index;
	hacd::HaI32 textureID;

	index = 0;
	textureID = handle->m_materials[materialHandle];
	for (hacd::HaI32 j = 0; j < handle->m_indexCount; j ++) {
		if (handle->m_indexList[j * 4 + 3] == textureID) {
			indexArray[index + 0] = (hacd::HaI16)handle->m_indexList[j * 4 + 0];
			indexArray[index + 1] = (hacd::HaI16)handle->m_indexList[j * 4 + 1];
			indexArray[index + 2] = (hacd::HaI16)handle->m_indexList[j * 4 + 2];
			index += 3;
		}
	}
}

/*
hacd::HaI32 dgMeshEffect::GetEffectiveVertexCount() const
{
	hacd::HaI32 mark;
	hacd::HaI32 count;

	count = 0;
	mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* vertex;
	
		vertex = &(*iter);
		if (vertex->m_mark != mark) {
			dgEdge* ptr;

			ptr = vertex;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != vertex);
			count ++;
		}
	}
	return count;
}
*/

dgConvexHull3d * dgMeshEffect::CreateConvexHull(hacd::HaF64 tolerance,hacd::HaI32 maxVertexCount) const
{
	dgConvexHull3d *ret = NULL;

	dgStack<dgBigVector> poolPtr(m_pointCount * 2); 
	dgBigVector* const pool = &poolPtr[0];

	hacd::HaI32 count = 0;
	hacd::HaI32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++)
	{
		dgEdge* const vertex = &(*iter);
		if (vertex->m_mark != mark) 
		{
			dgEdge* ptr = vertex;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != vertex);

			if (count < hacd::HaI32 (poolPtr.GetElementsCount())) 
			{
				const dgBigVector p = m_points[vertex->m_incidentVertex];
				pool[count] = p;
				count++;
			}
		}
	}

	ret = HACD_NEW(dgConvexHull3d)((const hacd::HaF64 *)pool,sizeof(dgBigVector),count,tolerance,maxVertexCount);

	return ret;
}


void dgMeshEffect::TransformMesh (const dgMatrix& matrix)
{
	dgMatrix normalMatrix (matrix);
	normalMatrix.m_posit = dgVector (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (1.0f));

	matrix.TransformTriplex (&m_points->m_x, sizeof (dgBigVector), &m_points->m_x, sizeof (dgBigVector), m_pointCount);
	matrix.TransformTriplex (&m_attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), &m_attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), m_atribCount);
	normalMatrix.TransformTriplex (&m_attib[0].m_normal_x, sizeof (dgVertexAtribute), &m_attib[0].m_normal_x, sizeof (dgVertexAtribute), m_atribCount);
}


dgMeshEffect::dgVertexAtribute dgMeshEffect::InterpolateEdge (dgEdge* const edge, hacd::HaF64 param) const
{
	dgVertexAtribute attrEdge;
	hacd::HaF64 t1 = param;
	hacd::HaF64 t0 = hacd::HaF64 (1.0f) - t1;
	HACD_ASSERT (t1 >= hacd::HaF64(0.0f));
	HACD_ASSERT (t1 <= hacd::HaF64(1.0f));

	const dgVertexAtribute& attrEdge0 = m_attib[edge->m_userData];
	const dgVertexAtribute& attrEdge1 = m_attib[edge->m_next->m_userData];

	attrEdge.m_vertex.m_x = attrEdge0.m_vertex.m_x * t0 + attrEdge1.m_vertex.m_x * t1;
	attrEdge.m_vertex.m_y = attrEdge0.m_vertex.m_y * t0 + attrEdge1.m_vertex.m_y * t1;
	attrEdge.m_vertex.m_z = attrEdge0.m_vertex.m_z * t0 + attrEdge1.m_vertex.m_z * t1;
	attrEdge.m_vertex.m_w = hacd::HaF32(0.0f);
	attrEdge.m_normal_x = attrEdge0.m_normal_x * t0 +  attrEdge1.m_normal_x * t1; 
	attrEdge.m_normal_y = attrEdge0.m_normal_y * t0 +  attrEdge1.m_normal_y * t1; 
	attrEdge.m_normal_z = attrEdge0.m_normal_z * t0 +  attrEdge1.m_normal_z * t1; 
	attrEdge.m_u0 = attrEdge0.m_u0 * t0 +  attrEdge1.m_u0 * t1;
	attrEdge.m_v0 = attrEdge0.m_v0 * t0 +  attrEdge1.m_v0 * t1;
	attrEdge.m_u1 = attrEdge0.m_u1 * t0 +  attrEdge1.m_u1 * t1;
	attrEdge.m_v1 = attrEdge0.m_v1 * t0 +  attrEdge1.m_v1 * t1;
	attrEdge.m_material = attrEdge0.m_material;
	return attrEdge;
}

bool dgMeshEffect::Sanity () const
{
	HACD_ASSERT (0);
return false;
/*
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		const dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_incidentFace > 0) {
			const dgVertexAtribute& attrEdge0 = m_attib[edge->m_userData];
			dgVector p0 (m_points[edge->m_incidentVertex]);
			dgVector q0 (attrEdge0.m_vertex);
			dgVector delta0 (p0 - q0);
			hacd::HaF32 error0 = delta0 % delta0;
			if (error0 > hacd::HaF32 (1.0e-15f)) {
				return false;
			}

			const dgVertexAtribute& attrEdge1 = m_attib[edge->m_next->m_userData];
			dgVector p1 (m_points[edge->m_next->m_incidentVertex]);
			dgVector q1 (attrEdge1.m_vertex);
			dgVector delta1 (p1 - q1);
			hacd::HaF32 error1 = delta1 % delta1;
			if (error1 > hacd::HaF32 (1.0e-15f)) {
				return false;
			}
		}
	}
	return true;
*/
}


dgEdge* dgMeshEffect::InsertEdgeVertex (dgEdge* const edge, hacd::HaF64 param)
{

	dgEdge* const twin = edge->m_twin;
	dgVertexAtribute attrEdge (InterpolateEdge (edge, param));
	dgVertexAtribute attrTwin (InterpolateEdge (twin, hacd::HaF32 (1.0f) - param));

	attrTwin.m_vertex = attrEdge.m_vertex;
	AddPoint(&attrEdge.m_vertex.m_x, dgFastInt (attrEdge.m_material));
	AddAtribute (attrTwin);

	hacd::HaI32 edgeAttrV0 = hacd::HaI32 (edge->m_userData);
	hacd::HaI32 twinAttrV0 = hacd::HaI32 (twin->m_userData);

	dgEdge* const faceA0 = edge->m_next;
	dgEdge* const faceA1 = edge->m_prev;
	dgEdge* const faceB0 = twin->m_next;
	dgEdge* const faceB1 = twin->m_prev;

//	SpliteEdgeAndTriangulate (m_pointCount - 1, edge);
	SpliteEdge (m_pointCount - 1, edge);

	faceA0->m_prev->m_userData = hacd::HaU64 (m_atribCount - 2);
	faceA1->m_next->m_userData = hacd::HaU64 (edgeAttrV0);

	faceB0->m_prev->m_userData = hacd::HaU64 (m_atribCount - 1);
	faceB1->m_next->m_userData = hacd::HaU64 (twinAttrV0);

	return faceA1->m_next;
}



dgMeshEffect::dgVertexAtribute dgMeshEffect::InterpolateVertex (const dgBigVector& srcPoint, dgEdge* const face) const
{
	//this should use Googol extended precision floats, because some face coming from Voronoi decomposition and booleans
	//clipping has extreme aspect ratios, for now just use float64
	const dgBigVector point (srcPoint);

	dgVertexAtribute attribute;
	memset (&attribute, 0, sizeof (attribute));
	hacd::HaF64 tol = hacd::HaF32 (1.0e-4f);
	for (hacd::HaI32 i = 0; i < 4; i ++) {
		dgEdge* ptr = face;
		dgEdge* const edge0 = ptr;
		dgBigVector q0 (m_points[ptr->m_incidentVertex]);

		ptr = ptr->m_next;
		const dgEdge* edge1 = ptr;
		dgBigVector q1 (m_points[ptr->m_incidentVertex]);

		ptr = ptr->m_next;
		const dgEdge* edge2 = ptr;
		do {
			const dgBigVector q2 (m_points[ptr->m_incidentVertex]);

			dgBigVector p10 (q1 - q0);
			dgBigVector p20 (q2 - q0);
#ifdef	_DEBUG		
			hacd::HaF64 dot = p20 % p10;
			hacd::HaF64 mag1 = p10 % p10;
			hacd::HaF64 mag2 = p20 % p20;
			hacd::HaF64 collinear = dot * dot - mag2 * mag1;
			HACD_ASSERT (fabs (collinear) > hacd::HaF64 (1.0e-8f));
#endif

			dgBigVector p_p0 (point - q0);
			dgBigVector p_p1 (point - q1);
			dgBigVector p_p2 (point - q2);

			hacd::HaF64 alpha1 = p10 % p_p0;
			hacd::HaF64 alpha2 = p20 % p_p0;
			hacd::HaF64 alpha3 = p10 % p_p1;
			hacd::HaF64 alpha4 = p20 % p_p1;
			hacd::HaF64 alpha5 = p10 % p_p2;
			hacd::HaF64 alpha6 = p20 % p_p2;

			hacd::HaF64 vc = alpha1 * alpha4 - alpha3 * alpha2;
			hacd::HaF64 vb = alpha5 * alpha2 - alpha1 * alpha6;
			hacd::HaF64 va = alpha3 * alpha6 - alpha5 * alpha4;
			hacd::HaF64 den = va + vb + vc;
			hacd::HaF64 minError = den * (-tol);
			hacd::HaF64 maxError = den * (hacd::HaF32 (1.0f) + tol);
			if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {
				edge2 = ptr;

				den = hacd::HaF64 (1.0f) / (va + vb + vc);

				hacd::HaF64 alpha0 = hacd::HaF32 (va * den);
				hacd::HaF64 alpha1 = hacd::HaF32 (vb * den);
				hacd::HaF64 alpha2 = hacd::HaF32 (vc * den);

				const dgVertexAtribute& attr0 = m_attib[edge0->m_userData];
				const dgVertexAtribute& attr1 = m_attib[edge1->m_userData];
				const dgVertexAtribute& attr2 = m_attib[edge2->m_userData];
				dgBigVector normal (attr0.m_normal_x * alpha0 + attr1.m_normal_x * alpha1 + attr2.m_normal_x * alpha2,
									attr0.m_normal_y * alpha0 + attr1.m_normal_y * alpha1 + attr2.m_normal_y * alpha2,
									attr0.m_normal_z * alpha0 + attr1.m_normal_z * alpha1 + attr2.m_normal_z * alpha2, hacd::HaF32 (0.0f));
				normal = normal.Scale (hacd::HaF64 (1.0f) / sqrt (normal % normal));

	#ifdef _DEBUG
				dgBigVector testPoint (attr0.m_vertex.m_x * alpha0 + attr1.m_vertex.m_x * alpha1 + attr2.m_vertex.m_x * alpha2,
									   attr0.m_vertex.m_y * alpha0 + attr1.m_vertex.m_y * alpha1 + attr2.m_vertex.m_y * alpha2,
									   attr0.m_vertex.m_z * alpha0 + attr1.m_vertex.m_z * alpha1 + attr2.m_vertex.m_z * alpha2, hacd::HaF32 (0.0f));
				HACD_ASSERT (fabs (testPoint.m_x - point.m_x) < hacd::HaF32 (1.0e-2f));
				HACD_ASSERT (fabs (testPoint.m_y - point.m_y) < hacd::HaF32 (1.0e-2f));
				HACD_ASSERT (fabs (testPoint.m_z - point.m_z) < hacd::HaF32 (1.0e-2f));
	#endif


				attribute.m_vertex.m_x = point.m_x;
				attribute.m_vertex.m_y = point.m_y;
				attribute.m_vertex.m_z = point.m_z;
				attribute.m_vertex.m_w = point.m_w;
				attribute.m_normal_x = normal.m_x;
				attribute.m_normal_y = normal.m_y;
				attribute.m_normal_z = normal.m_z;
				attribute.m_u0 = attr0.m_u0 * alpha0 +  attr1.m_u0 * alpha1 + attr2.m_u0 * alpha2;
				attribute.m_v0 = attr0.m_v0 * alpha0 +  attr1.m_v0 * alpha1 + attr2.m_v0 * alpha2;
				attribute.m_u1 = attr0.m_u1 * alpha0 +  attr1.m_u1 * alpha1 + attr2.m_u1 * alpha2;
				attribute.m_v1 = attr0.m_v1 * alpha0 +  attr1.m_v1 * alpha1 + attr2.m_v1 * alpha2;

				attribute.m_material = attr0.m_material;
				HACD_ASSERT (attr0.m_material == attr1.m_material);
				HACD_ASSERT (attr0.m_material == attr2.m_material);
				return attribute; 
			}
			
			q1 = q2;
			edge1 = ptr;

			ptr = ptr->m_next;
		} while (ptr != face);
		tol *= hacd::HaF64 (2.0f);
	}
	// this should never happens
	HACD_ASSERT (0);
	return attribute;
}




void dgMeshEffect::MergeFaces (const dgMeshEffect* const source)
{
	hacd::HaI32 mark = source->IncLRU();
	dgPolyhedra::Iterator iter (*source);
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) {
			dgVertexAtribute face[DG_MESH_EFFECT_POINT_SPLITED];

			hacd::HaI32 count = 0;
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				face[count] = source->m_attib[ptr->m_userData];
				count ++;
				HACD_ASSERT (count < hacd::HaI32 (sizeof (face) / sizeof (face[0])));
				ptr = ptr->m_next;
			} while (ptr != edge);
			AddPolygon(count, &face[0].m_vertex.m_x, sizeof (dgVertexAtribute), dgFastInt (face[0].m_material));
		}
	}
}


void dgMeshEffect::ReverseMergeFaces (dgMeshEffect* const source)
{
	hacd::HaI32 mark = source->IncLRU();
	dgPolyhedra::Iterator iter (*source);
	for(iter.Begin(); iter; iter ++){
		dgEdge* const edge = &(*iter);
		if ((edge->m_incidentFace > 0) && (edge->m_mark < mark)) {
			dgVertexAtribute face[DG_MESH_EFFECT_POINT_SPLITED];

			hacd::HaI32 count = 0;
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				face[count] = source->m_attib[ptr->m_userData];
				face[count].m_normal_x *= hacd::HaF32 (-1.0f);
				face[count].m_normal_y *= hacd::HaF32 (-1.0f);
				face[count].m_normal_z *= hacd::HaF32 (-1.0f);
				count ++;
				HACD_ASSERT (count < hacd::HaI32 (sizeof (face) / sizeof (face[0])));
				ptr = ptr->m_prev;
			} while (ptr != edge);
			AddPolygon(count, &face[0].m_vertex.m_x, sizeof (dgVertexAtribute), dgFastInt (face[0].m_material));
		}
	}
}



void dgMeshEffect::FilterCoplanarFaces (const dgMeshEffect* const coplanarFaces, hacd::HaF32 sign)
{
	const hacd::HaF64 tol = hacd::HaF64 (1.0e-5f);
	const hacd::HaF64 tol2 = tol * tol;

	hacd::HaI32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dgEdge* const face = &(*iter);
		iter ++;
		if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
			dgEdge* ptr = face;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);

			dgBigVector normal (FaceNormal(face, &m_points[0].m_x, sizeof (dgBigVector)));
			normal = normal.Scale (sign);
			dgBigVector origin (m_points[face->m_incidentVertex]);

			hacd::HaF64 error2 = (normal % normal) * tol2;
			hacd::HaI32 capMark = coplanarFaces->IncLRU();

			Iterator capIter (*coplanarFaces);
			for (capIter.Begin (); capIter; capIter ++) {
				dgEdge* const capFace = &(*capIter);
				if ((capFace->m_mark != capMark) && (capFace->m_incidentFace > 0)) {
					dgEdge* ptr = capFace;
					do {
						ptr->m_mark = capMark;
						ptr = ptr->m_next;
					} while (ptr != capFace);

					dgBigVector capNormal (coplanarFaces->FaceNormal(capFace, &coplanarFaces->m_points[0].m_x, sizeof (dgBigVector)));

					if ((capNormal % normal) > hacd::HaF64 (0.0f)) {
						dgBigVector capOrigin (coplanarFaces->m_points[capFace->m_incidentVertex]);
						hacd::HaF64 dist = normal % (capOrigin - origin);
						if ((dist * dist) < error2) {
							DeleteFace(face);
							iter.Begin();
							break;
						}
					}
				}
			}
		}
	}
}


bool dgMeshEffect::HasOpenEdges () const
{
	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++){
		dgEdge* const face = &(*iter);
		if (face->m_incidentFace < 0){
			return true;
		}
	}
	return false;
}



bool dgMeshEffect::SeparateDuplicateLoops (dgEdge* const face)
{
	for (dgEdge* ptr0 = face; ptr0 != face->m_prev; ptr0 = ptr0->m_next) {
		hacd::HaI32 index = ptr0->m_incidentVertex;

		dgEdge* ptr1 = ptr0->m_next; 
		do {
			if (ptr1->m_incidentVertex == index) {
				dgEdge* const ptr00 = ptr0->m_prev;
				dgEdge* const ptr11 = ptr1->m_prev;

				ptr00->m_next = ptr1;
				ptr1->m_prev = ptr00;

				ptr11->m_next = ptr0;
				ptr0->m_prev = ptr11;

				return true;
			}

			ptr1 = ptr1->m_next;
		} while (ptr1 != face);
	}

	return false;
}


dgMeshEffect* dgMeshEffect::GetNextLayer (hacd::HaI32 mark)
{
	Iterator iter(*this);
	dgEdge* edge = NULL;
	for (iter.Begin (); iter; iter ++) {
		edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
			break;
		}
	}

	if (!edge) {
		return NULL;
	}

	hacd::HaI32 layer = hacd::HaI32 (m_points[edge->m_incidentVertex].m_w);
	dgPolyhedra polyhedra;

	polyhedra.BeginFace ();
	for (iter.Begin (); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
			hacd::HaI32 thislayer = hacd::HaI32 (m_points[edge->m_incidentVertex].m_w);
			if (thislayer == layer) {
				dgEdge* ptr = edge;
				hacd::HaI32 count = 0;
				hacd::HaI32 faceIndex[256];
				hacd::HaI64 faceDataIndex[256];
				do {
					ptr->m_mark = mark;
					faceIndex[count] = ptr->m_incidentVertex;
					faceDataIndex[count] = ptr->m_userData;
					count ++;
					HACD_ASSERT (count < hacd::HaI32 (sizeof (faceIndex)/ sizeof(faceIndex[0])));
					ptr = ptr->m_next;
				} while (ptr != edge);
				polyhedra.AddFace (count, &faceIndex[0], &faceDataIndex[0]);
			}
		}
	}
	polyhedra.EndFace ();

	dgMeshEffect* solid = NULL;
	if (polyhedra.GetCount()) {
		solid = HACD_NEW(dgMeshEffect)(polyhedra, *this);
		solid->SetLRU(mark);
	}
	return solid;
}


void dgMeshEffect::RepairTJoints (bool triangulate)
{
	hacd::HaI32 mark = IncLRU();
	dgPolyhedra::Iterator iter (*this);
#ifdef _DEBUG
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const face = &(*iter);
		if ((face->m_incidentFace < 0) && (face->m_mark != mark)) {
			for (dgEdge* ptr = face; ptr != face->m_prev; ptr = ptr->m_next) {
				dgBigVector p0 (m_points[ptr->m_incidentVertex]);
				for (dgEdge* ptr1 = ptr->m_next; ptr1 != face; ptr1 = ptr1->m_next) {
					if (ptr->m_incidentVertex != ptr1->m_incidentVertex) {
						dgBigVector p1 (m_points[ptr1->m_incidentVertex]);
						dgBigVector dp (p1 - p0);
						hacd::HaF64 err2 (dp % dp);
						if (err2 < hacd::HaF64 (1.0e-16f)) {
//							HACD_ASSERT (0);
						}
					}
				} 
			}
		}
	}
	mark = IncLRU();
#endif



	for (iter.Begin(); iter; ) {
		dgEdge* const face = &(*iter);
		iter ++;


		if ((face->m_incidentFace < 0) && (face->m_mark != mark)) {
			// vertices project 

			while (SeparateDuplicateLoops (face));

			dgBigVector dir (hacd::HaF64 (0.0f), hacd::HaF64 (0.0f), hacd::HaF64 (0.0f), hacd::HaF64 (0.0f));
			hacd::HaF64 lengh2 = hacd::HaF64 (0.0f);
			dgEdge* ptr = face;
			do {
				dgBigVector dir1 (m_points[ptr->m_next->m_incidentVertex] - m_points[ptr->m_incidentVertex]);
				hacd::HaF64 val = dir1 % dir1;
				if (val > lengh2) {
					lengh2 = val;
					dir = dir1;
				}
				ptr = ptr->m_next;
			} while (ptr != face);

			HACD_ASSERT (lengh2 > hacd::HaF32 (0.0f));

			dgEdge* lastEdge = NULL;
			dgEdge* firstEdge = NULL;
			hacd::HaF64 minVal = hacd::HaF64 (-1.0e10f);
			hacd::HaF64 maxVal = hacd::HaF64 (-1.0e10f);
			ptr = face;
			do {
				const dgBigVector& p = m_points[ptr->m_incidentVertex];
				hacd::HaF64 val = p % dir;
				if (val > maxVal) {
					maxVal = val;
					lastEdge = ptr;
				}
				val *= hacd::HaF64 (-1.0f);
				if (val > minVal) {
					minVal = val;
					firstEdge = ptr;
				}

				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);

			HACD_ASSERT (firstEdge);
			HACD_ASSERT (lastEdge);

			bool isTJoint = true;
			dgBigVector p0 (m_points[firstEdge->m_incidentVertex]);
			dgBigVector p1 (m_points[lastEdge->m_incidentVertex]);
			dgBigVector p1p0 (p1 - p0);
			hacd::HaF64 den = p1p0 % p1p0;
			ptr = firstEdge->m_next;
			do {
				dgBigVector p2 (m_points[ptr->m_incidentVertex]);
				hacd::HaF64 num = (p2 - p0) % p1p0;
				dgBigVector q (p0 + p1p0.Scale (num / den));
				dgBigVector dist (p2 - q);
				hacd::HaF64 err2 = dist % dist;
				isTJoint &= (err2 < (hacd::HaF64 (1.0e-4f) * hacd::HaF64 (1.0e-4f)));
				ptr = ptr->m_next;
			} while (isTJoint && (ptr != firstEdge));

			if (isTJoint) {
				do {
					dgEdge* next = NULL;

					const dgBigVector p0 = m_points[firstEdge->m_incidentVertex];
					const dgBigVector p1 = m_points[firstEdge->m_next->m_incidentVertex];
					const dgBigVector p2 = m_points[firstEdge->m_prev->m_incidentVertex];

					dgBigVector p1p0 (p1 - p0);
					dgBigVector p2p0 (p2 - p0);
					hacd::HaF64 dist10 = p1p0 % p1p0;
					hacd::HaF64 dist20 = p2p0 % p2p0;

					dgEdge* begin = NULL;
					dgEdge* last = NULL;
					if (dist20 > dist10) {
						hacd::HaF64 t = (p1p0 % p2p0) / dist20;
						HACD_ASSERT (t > hacd::HaF32 (0.0f));
						HACD_ASSERT (t < hacd::HaF32 (1.0f));

						if (firstEdge->m_next->m_next->m_next != firstEdge) {
							ConectVertex (firstEdge->m_prev, firstEdge->m_next);
							next = firstEdge->m_next->m_twin->m_next;
						}
						HACD_ASSERT (firstEdge->m_next->m_next->m_next == firstEdge);

#ifdef _DEBUG
						dgEdge* tmp = firstEdge->m_twin;
						do {
							HACD_ASSERT (tmp->m_incidentFace > 0);
							tmp = tmp->m_next;
						} while (tmp != firstEdge->m_twin); 
#endif

						begin = firstEdge->m_next;
						last = firstEdge;
						firstEdge->m_userData = firstEdge->m_prev->m_twin->m_userData;
						firstEdge->m_incidentFace = firstEdge->m_prev->m_twin->m_incidentFace;
						dgVertexAtribute attrib (InterpolateEdge (firstEdge->m_prev->m_twin, t));
						attrib.m_vertex = m_points[firstEdge->m_next->m_incidentVertex];
						AddAtribute(attrib);
						firstEdge->m_next->m_incidentFace = firstEdge->m_prev->m_twin->m_incidentFace;
						firstEdge->m_next->m_userData = hacd::HaU64 (m_atribCount - 1);

						bool restart = false;
						if ((firstEdge->m_prev == &(*iter)) || (firstEdge->m_prev->m_twin == &(*iter))) {
							restart = true;
						}
						DeleteEdge(firstEdge->m_prev);
						if (restart) {
							iter.Begin();
						}

					} else {

						HACD_ASSERT (dist20 < dist10);

						hacd::HaF64 t = (p1p0 % p2p0) / dist10;
						HACD_ASSERT (t > hacd::HaF32 (0.0f));
						HACD_ASSERT (t < hacd::HaF32 (1.0f));

						if (firstEdge->m_next->m_next->m_next != firstEdge) {
							ConectVertex (firstEdge->m_next, firstEdge->m_prev);
							next = firstEdge->m_next->m_twin;
						}
						HACD_ASSERT (firstEdge->m_next->m_next->m_next == firstEdge);

#ifdef _DEBUG
						dgEdge* tmp = firstEdge->m_twin;
						do {
							HACD_ASSERT (tmp->m_incidentFace > 0);
							tmp = tmp->m_next;
						} while (tmp != firstEdge->m_twin); 
#endif

						begin = firstEdge->m_prev;
						last = firstEdge->m_next;
						firstEdge->m_next->m_userData = firstEdge->m_twin->m_userData;
						firstEdge->m_next->m_incidentFace = firstEdge->m_twin->m_incidentFace;
						dgVertexAtribute attrib (InterpolateEdge (firstEdge->m_twin, hacd::HaF64 (1.0f) - t));
						attrib.m_vertex = m_points[firstEdge->m_prev->m_incidentVertex];
						AddAtribute(attrib);
						firstEdge->m_prev->m_incidentFace = firstEdge->m_twin->m_incidentFace;
						firstEdge->m_prev->m_userData = hacd::HaU64 (m_atribCount - 1);

						bool restart = false;
						if ((firstEdge == &(*iter)) || (firstEdge->m_twin == &(*iter))) {
							restart = true;
						}
						DeleteEdge(firstEdge);
						if (restart) {
							iter.Begin();
						}
					}

					if (triangulate) {
						HACD_ASSERT (begin);
						HACD_ASSERT (last);
						for (dgEdge* ptr = begin->m_next->m_next; ptr != last; ptr = ptr->m_next) {
							dgEdge* const e = AddHalfEdge (begin->m_incidentVertex, ptr->m_incidentVertex);
							dgEdge* const t = AddHalfEdge (ptr->m_incidentVertex, begin->m_incidentVertex);
							if (e && t) {
								HACD_ASSERT (e);
								HACD_ASSERT (t);
								e->m_twin = t;
								t->m_twin = e;

								e->m_incidentFace = ptr->m_incidentFace;
								t->m_incidentFace = ptr->m_incidentFace;

								e->m_userData = last->m_next->m_userData;
								t->m_userData = ptr->m_userData;

								t->m_prev = ptr->m_prev;
								ptr->m_prev->m_next = t;
								e->m_next = ptr;
								ptr->m_prev = e;
								t->m_next = last->m_next;
								e->m_prev = last;
								last->m_next->m_prev = t;
								last->m_next = e;
							}
						}
					}

					firstEdge = next;
				} while (firstEdge);
			}
		}
	}

	DeleteDegenerateFaces(&m_points[0].m_x, sizeof (m_points[0]), hacd::HaF64 (1.0e-7f));
}

// create a convex hull
dgMeshEffect::dgMeshEffect (const hacd::HaF64* const vertexCloud, hacd::HaI32 count, hacd::HaI32 strideInByte, hacd::HaF64 distTol)
:dgPolyhedra()
{
	Init(true);
	if (count >= 4) {
		dgConvexHull3d convexHull (vertexCloud, strideInByte, count, distTol);
		if (convexHull.GetCount()) {

			hacd::HaI32 vertexCount = convexHull.GetVertexCount();
			dgStack<dgVector> pointsPool (convexHull.GetVertexCount());
			dgVector* const points = &pointsPool[0];
			for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
				points[i] = convexHull.GetVertex(i);
			}
			dgVector uv(hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
			dgVector normal (hacd::HaF32 (0.0f), hacd::HaF32 (1.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));

			hacd::HaI32 triangleCount = convexHull.GetCount();
			dgStack<hacd::HaI32> faceCountPool (triangleCount);
			dgStack<hacd::HaI32> materialsPool (triangleCount);
			dgStack<hacd::HaI32> vertexIndexListPool (triangleCount * 3);
			dgStack<hacd::HaI32> normalIndexListPool (triangleCount * 3);


			memset (&materialsPool[0], 0, triangleCount * sizeof (hacd::HaI32));
			memset (&normalIndexListPool[0], 0, 3 * triangleCount * sizeof (hacd::HaI32));

			hacd::HaI32 index = 0;
			hacd::HaI32* const faceCount = &faceCountPool[0];
			hacd::HaI32* const vertexIndexList = &vertexIndexListPool[0];
			for (dgConvexHull3d::dgListNode* faceNode = convexHull.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
				dgConvexHull3DFace& face = faceNode->GetInfo();
				faceCount[index] = 3;
				vertexIndexList[index * 3 + 0] = face.m_index[0]; 
				vertexIndexList[index * 3 + 1] = face.m_index[1]; 
				vertexIndexList[index * 3 + 2] = face.m_index[2]; 
				index ++;
			}

			BuildFromVertexListIndexList(triangleCount, faceCount, &materialsPool[0], 
				&points[0].m_x, sizeof (dgVector), vertexIndexList,
				&normal.m_x, sizeof (dgVector), &normalIndexListPool[0],
				&uv.m_x, sizeof (dgVector), &normalIndexListPool[0],
				&uv.m_x, sizeof (dgVector), &normalIndexListPool[0]);
		}
	}
}

//**** Note, from this point out is a copy of 'MeshEffect3.cpp' from the Newton Physics Engine


#define USE_NEW_METHOD 0

#if USE_NEW_METHOD

// based of the paper Hierarchical Approximate Convex Decomposition by Khaled Mamou 
// with his permission to adapt his algorithm so be more efficient.
// available http://sourceforge.net/projects/hacd/
// for the details http://kmamou.blogspot.com/


class dgClusterFace
{
	public:
	dgClusterFace()
	{
	}
	~dgClusterFace()
	{
	}

	dgEdge* m_edge;
	hacd::HaF64 m_area;
	hacd::HaF64 m_perimeter;
	dgBigVector m_normal;
};

class dgPairProxi
{
	public:
	dgPairProxi()
		:m_edgeA(NULL)
		,m_edgeB(NULL)
		,m_area(hacd::HaF64(0.0f))
		,m_perimeter(hacd::HaF64(0.0f))
	{
	}

	~dgPairProxi()
	{
	}

	dgEdge* m_edgeA;
	dgEdge* m_edgeB;
	hacd::HaF64 m_area;
	hacd::HaF64 m_perimeter;
};

class dgClusterList: public dgList<dgClusterFace>
{
	public:
	dgClusterList(void) 
		: dgList<dgClusterFace>()
		,m_area (hacd::HaF32 (0.0f))
		,m_perimeter (hacd::HaF32 (0.0f))
	  {
	  }

	  ~dgClusterList()
	  {
	  }

	  hacd::HaI32 AddVertexToPool(const dgMeshEffect& mesh, dgBigVector* const vertexPool, hacd::HaI32* const vertexMarks, hacd::HaI32 vertexMark)
	  {
		  hacd::HaI32 count = 0;

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& face = node->GetInfo();

			  dgEdge* edge = face.m_edge;
			  do {
				  hacd::HaI32 index = edge->m_incidentVertex;
				  if (vertexMarks[index] != vertexMark)
				  {
					  vertexMarks[index] = vertexMark;
					  vertexPool[count] = points[index];
					  count++;
				  }
				  edge = edge->m_next;
			  } while (edge != face.m_edge);
		  }
		  return count;
	  }

	  hacd::HaF64 CalculateTriangleConcavity2(const dgConvexHull3d& convexHull, dgClusterFace& info, hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, const dgBigVector* const points) const
	  {
		  hacd::HaU32 head = 1;
		  hacd::HaU32 tail = 0;
		  dgBigVector pool[1<<8][3];

		  pool[0][0] = points[i0];
		  pool[0][1] = points[i1];
		  pool[0][2] = points[i2];

		  const dgBigVector step(info.m_normal.Scale(hacd::HaF64(4.0f) * convexHull.GetDiagonal()));

		  hacd::HaF64 concavity = hacd::HaF32(0.0f);
		  hacd::HaF64 minArea = hacd::HaF32(0.125f);
		  hacd::HaF64 minArea2 = minArea * minArea * 0.5f;

		  // weight the area by the area of the face 
		  //dgBigVector edge10(pool[0][1] - pool[0][0]);
		  //dgBigVector edge20(pool[0][2] - pool[0][0]);
		  //dgBigVector triangleArea = edge10 * edge20;
		  //hacd::HaF64 triangleArea2 = triangleArea % triangleArea;
		  //if ((triangleArea2 / minArea2)> hacd::HaF32 (64.0f)) {
			// minArea2 = triangleArea2 / hacd::HaF32 (64.0f);
		  //}

		  hacd::HaI32 maxCount = 4;
		  hacd::HaU32 mask = (sizeof (pool) / (3 * sizeof (pool[0][0]))) - 1;
		  while ((tail != head) && (maxCount >= 0)) {
			  //stack--;
			  maxCount --;
			  dgBigVector p0(pool[tail][0]);
			  dgBigVector p1(pool[tail][1]);
			  dgBigVector p2(pool[tail][2]);
			  tail =  (tail + 1) & mask;

			  dgBigVector q1((p0 + p1 + p2).Scale(hacd::HaF64(1.0f / 3.0f)));
			  dgBigVector q0(q1 + step);

			  hacd::HaF64 param = convexHull.RayCast(q0, q1);
			  if (param > hacd::HaF64(1.0f)) {
				  param = hacd::HaF64(1.0f);
			  }
			  dgBigVector dq(step.Scale(hacd::HaF32(1.0f) - param));
			  hacd::HaF64 length2 = dq % dq;
			  if (length2 > concavity) {
				  concavity = length2;
			  }

			  if (((head + 1) & mask) != tail) {
				  dgBigVector edge10(p1 - p0);
				  dgBigVector edge20(p2 - p0);
				  dgBigVector n(edge10 * edge20);
				  hacd::HaF64 area2 = n % n;
				  if (area2 > minArea2) {
					  dgBigVector p01((p0 + p1).Scale(hacd::HaF64(0.5f)));
					  dgBigVector p12((p1 + p2).Scale(hacd::HaF64(0.5f)));
					  dgBigVector p20((p2 + p0).Scale(hacd::HaF64(0.5f)));

					  pool[head][0] = p0;
					  pool[head][1] = p01;
					  pool[head][2] = p20;
					  head = (head + 1) & mask;

					  if (((head + 1) & mask) != tail) {
						  pool[head][0] = p1;
						  pool[head][1] = p12;
						  pool[head][2] = p01;
						  head = (head + 1) & mask;

						  if (((head + 1) & mask) != tail)	{
							  pool[head][0] = p2;
							  pool[head][1] = p20;
							  pool[head][2] = p12;
							  head = (head + 1) & mask;
						  }
					  }
				  }
			  }
		  }
		  return concavity;
	  }

	  hacd::HaF64 CalculateConcavity2(const dgConvexHull3d& convexHull, const dgMeshEffect& mesh)
	  {
		  hacd::HaF64 concavity = hacd::HaF32(0.0f);

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();

		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& info = node->GetInfo();
			  hacd::HaI32 i0 = info.m_edge->m_incidentVertex;
			  hacd::HaI32 i1 = info.m_edge->m_next->m_incidentVertex;
			  for (dgEdge* edge = info.m_edge->m_next->m_next; edge != info.m_edge; edge = edge->m_next) {
				  hacd::HaI32 i2 = edge->m_incidentVertex;
				  hacd::HaF64 val = CalculateTriangleConcavity2(convexHull, info, i0, i1, i2, points);
				  if (val > concavity) {
					  concavity = val;
				  }
				  i1 = i2;
			  }
		  }

		  return concavity;
	  }

	  bool IsClusterCoplanar(const dgBigPlane& plane,
		  const dgMeshEffect& mesh) const
	  {
		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& info = node->GetInfo();

			  dgEdge* ptr = info.m_edge;
			  do {
				  const dgBigVector& p = points[ptr->m_incidentVertex];
				  hacd::HaF64 dist = fabs(plane.Evalue(p));
				  if (dist > hacd::HaF64(1.0e-5f)) {
					  return false;
				  }
				  ptr = ptr->m_next;
			  } while (ptr != info.m_edge);
		  }

		  return true;
	  }

	  bool IsEdgeConvex(const dgBigPlane& plane, const dgMeshEffect& mesh,
		  dgEdge* const edge) const
	  {
		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  dgEdge* const edge0 = edge->m_next;
		  dgEdge* ptr = edge0->m_twin->m_next;
		  do {
			  if (ptr->m_twin->m_incidentFace == edge->m_twin->m_incidentFace) {
				  HACD_ASSERT(edge0->m_incidentVertex == ptr->m_incidentVertex);
				  dgBigVector e0(points[edge0->m_twin->m_incidentVertex] - points[edge0->m_incidentVertex]);
				  dgBigVector e1(points[ptr->m_twin->m_incidentVertex] - points[edge0->m_incidentVertex]);
				  dgBigVector normal(e0 * e1);
				  return (normal % plane) > hacd::HaF64(0.0f);
			  }
			  ptr = ptr->m_twin->m_next;
		  } while (ptr != edge->m_twin);

		  HACD_ASSERT(0);
		  return true;
	  }

	  // calculate the convex hull of a conched group of faces,
	  // and measure the concavity, according to Khaled Mamou convexity criteria, which is basically
	  // has two components, 
	  // the first is ratio  between the the perimeter of the group of faces
	  // and the second the largest distance from any of the face to the surface of the hull 
	  // when the faces are are a strip of a convex hull the perimeter ratio components is 1.0 and the distance to the hull is zero
	  // this is the ideal concavity.
	  // when the face are no part of the hull, then the worse distance to the hull is dominate the the metric
	  // this matrix is used to place all possible combination of this cluster with any adjacent cluster into a priority heap and determine 
	  // which pair of two adjacent cluster is the best selection for combining the into a larger cluster.
	  void CalculateNodeCost(dgMeshEffect& mesh, hacd::HaI32 meshMask,
		  dgBigVector* const vertexPool, hacd::HaI32* const vertexMarks,
		  hacd::HaI32& vertexMark, dgClusterList* const clusters, hacd::HaF64 diagonalInv,
		  hacd::HaF64 aspectRatioCoeficent, dgList<dgPairProxi>& proxyList,
		  dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64>& heap)
	  {
		  hacd::HaI32 faceIndex = GetFirst()->GetInfo().m_edge->m_incidentFace;

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();

		  bool flatStrip = true;
		  dgBigPlane plane(GetFirst()->GetInfo().m_normal, -(points[GetFirst()->GetInfo().m_edge->m_incidentVertex] % GetFirst()->GetInfo().m_normal));
		  if (GetCount() > 1) {
			  flatStrip = IsClusterCoplanar(plane, mesh);
		  }

		  vertexMark++;
		  hacd::HaI32 vertexCount = AddVertexToPool(mesh, vertexPool, vertexMarks, vertexMark);
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  //dgClusterFace& clusterFaceA = GetFirst()->GetInfo();
			  dgClusterFace& clusterFaceA = node->GetInfo();

			  dgEdge* edge = clusterFaceA.m_edge;
			  do {
				  hacd::HaI32 twinFaceIndex = edge->m_twin->m_incidentFace;
				  if ((edge->m_mark != meshMask) && (twinFaceIndex != faceIndex) && (twinFaceIndex > 0)) {

					  dgClusterList& clusterListB = clusters[twinFaceIndex];

					  vertexMark++;
					  hacd::HaI32 extraCount = clusterListB.AddVertexToPool(mesh, &vertexPool[vertexCount], &vertexMarks[0], vertexMark);

					  hacd::HaI32 count = vertexCount + extraCount;
					  dgConvexHull3d convexHull(&vertexPool[0].m_x, sizeof(dgBigVector), count, 0.0);

					  hacd::HaF64 concavity = hacd::HaF64(0.0f);
					  if (convexHull.GetVertexCount()) {
						  concavity = sqrt(GetMax(CalculateConcavity2(convexHull, mesh), clusterListB.CalculateConcavity2(convexHull, mesh)));
						  if (concavity < hacd::HaF64(1.0e-3f)) {
							  concavity = hacd::HaF64(0.0f);
						  }
					  }

					  if ((concavity == hacd::HaF64(0.0f)) && flatStrip)  {
						  if (clusterListB.IsClusterCoplanar(plane, mesh)) {
							  bool concaveEdge = !(IsEdgeConvex(plane, mesh, edge) && IsEdgeConvex(plane, mesh, edge->m_twin));
							  if (concaveEdge) {
								  concavity += 1000.0f;
							  }
						  }
					  }

					  dgBigVector p1p0(points[edge->m_twin->m_incidentVertex] - points[edge->m_incidentVertex]);
					  hacd::HaF64 edgeLength = hacd::HaF64(2.0f) * sqrt(p1p0 % p1p0);

					  hacd::HaF64 area = m_area + clusterListB.m_area;
					  hacd::HaF64 perimeter = m_perimeter + clusterListB.m_perimeter - edgeLength;
					  hacd::HaF64 edgeCost = perimeter * perimeter / (hacd::HaF64(4.0f * 3.141592f) * area);
					  hacd::HaF64 cost = diagonalInv * (concavity + edgeCost * aspectRatioCoeficent);

					  if ((heap.GetCount() + 20) > heap.GetMaxCount()) {
						  for (hacd::HaI32 i = heap.GetCount() - 1; i >= 0; i--) {
							  dgList<dgPairProxi>::dgListNode* emptyNode = heap[i];
							  dgPairProxi& emptyPair = emptyNode->GetInfo();
							  if ((emptyPair.m_edgeA == NULL) && (emptyPair.m_edgeB == NULL)) {
								  heap.Remove(i);
							  }
						  }
					  }

					  dgList<dgPairProxi>::dgListNode* pairNode = proxyList.Append();
					  dgPairProxi& pair = pairNode->GetInfo();
					  pair.m_edgeA = edge;
					  pair.m_edgeB = edge->m_twin;
					  pair.m_area = area;
					  pair.m_perimeter = perimeter;
					  edge->m_userData = hacd::HaU64(pairNode);
					  edge->m_twin->m_userData = hacd::HaU64(pairNode);
					  heap.Push(pairNode, cost);
				  }

				  edge->m_mark = meshMask;
				  edge->m_twin->m_mark = meshMask;
				  edge = edge->m_next;
			  } while (edge != clusterFaceA.m_edge);
		  }
	  }


	  static hacd::HaF64 CollapseClusters (
		  dgMeshEffect& mesh, 
		  dgClusterList* const clusters,
		  dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64>& head, 
		  hacd::HaF32 normalizedConcavity, 
		  hacd::HaI32 maxClustersCount, 
		  hacd::HaI32& essentialClustersCount,
		  hacd::HaI32& vertexMark,
		  dgBigVector* const vertexPool,
		  hacd::HaI32* const vertexMarks,
		  hacd::HaF64 diagonalInv,
		  hacd::HaF64 aspectRatioCoeficent,
		  dgList<dgPairProxi>& proxyList)
	  {
		  hacd::HaF64 concavity = hacd::HaF32 (0.0f);

		  while (head.GetCount() && ((head.Value() < normalizedConcavity) || (essentialClustersCount > maxClustersCount))) 
		  {
			  dgList<dgPairProxi>::dgListNode* const pairNode = head[0];
			  head.Pop();
			  dgPairProxi& pair = pairNode->GetInfo();

			  HACD_ASSERT((pair.m_edgeA && pair.m_edgeA) || (!pair.m_edgeA && !pair.m_edgeA));
			  if (pair.m_edgeA && pair.m_edgeB) 
			  {
				  HACD_ASSERT(pair.m_edgeA->m_incidentFace != pair.m_edgeB->m_incidentFace);
				  // merge two clusters
				  hacd::HaI32 faceIndexA = pair.m_edgeA->m_incidentFace;
				  hacd::HaI32 faceIndexB = pair.m_edgeB->m_incidentFace;
				  dgClusterList* listA = &clusters[faceIndexA];
				  dgClusterList* listB = &clusters[faceIndexB];
				  if (pair.m_edgeA->m_incidentFace > pair.m_edgeB->m_incidentFace) 
				  {
					  Swap(faceIndexA, faceIndexB);
					  Swap(listA, listB);
				  }

				  while (listB->GetFirst()) 
				  {
					  dgClusterList::dgListNode* const nodeB = listB->GetFirst();
					  listB->Unlink(nodeB);
					  dgClusterFace& faceB = nodeB->GetInfo();

					  dgEdge* ptr = faceB.m_edge;
					  do 
					  {
						  ptr->m_incidentFace = faceIndexA;
						  ptr = ptr->m_next;
					  } while (ptr != faceB.m_edge);
					  listA->Append(nodeB);
				  }

				  essentialClustersCount --;

				  listB->m_area = hacd::HaF32 (0.0f);
				  listB->m_perimeter = hacd::HaF32 (0.0f);
				  listA->m_area = pair.m_area;
				  listA->m_perimeter = pair.m_perimeter;

				  // recalculated the new metric for the new cluster, and tag the used cluster as invalid, so that 
				  // other potential selection do not try merge with this this one, producing convex that re use a face more than once  
				  hacd::HaI32 mark = mesh.IncLRU();
				  for (dgClusterList::dgListNode* node = listA->GetFirst(); node; node = node->GetNext()) 
				  {
					  dgClusterFace& face = node->GetInfo();
					  dgEdge* ptr = face.m_edge;
					  do 
					  {
						  if (ptr->m_userData != hacd::HaU64 (-1)) 
						  {
							  dgList<dgPairProxi>::dgListNode* const pairNode = (dgList<dgPairProxi>::dgListNode*) ptr->m_userData;
							  dgPairProxi& pairProxy = pairNode->GetInfo();
							  pairProxy.m_edgeA = NULL;
							  pairProxy.m_edgeB = NULL;
						  }
						  ptr->m_userData = hacd::HaU64 (-1);
						  ptr->m_twin->m_userData = hacd::HaU64 (-1);

						  if ((ptr->m_twin->m_incidentFace == faceIndexA) || (ptr->m_twin->m_incidentFace < 0)) 
						  {
							  ptr->m_mark = mark;
							  ptr->m_twin->m_mark = mark;
						  }

						  if (ptr->m_mark != mark) 
						  {
							  dgClusterList& adjacentList = clusters[ptr->m_twin->m_incidentFace];
							  for (dgClusterList::dgListNode* adjacentNode = adjacentList.GetFirst(); adjacentNode; adjacentNode = adjacentNode->GetNext()) 
							  {
								  dgClusterFace& adjacentFace = adjacentNode->GetInfo();
								  dgEdge* adjacentEdge = adjacentFace.m_edge;
								  do 
								  {
									  if (adjacentEdge->m_twin->m_incidentFace == faceIndexA)	
									  {
										  adjacentEdge->m_twin->m_mark = mark;
									  }
									  adjacentEdge = adjacentEdge->m_next;
								  } while (adjacentEdge != adjacentFace.m_edge);
							  }
							  ptr->m_mark = mark - 1;
						  }
						  ptr = ptr->m_next;
					  } while (ptr != face.m_edge);
				  }

				  // re generated the cost of merging this new all its adjacent clusters, that are still alive. 
				  vertexMark++;
				  listA->CalculateNodeCost(mesh, mark, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, head);
			  }

			  proxyList.Remove(pairNode);
		  }

		  while (head.GetCount()) 
		  {
			  concavity = head.Value();
			  dgList<dgPairProxi>::dgListNode* const pairNode = head[0];
			  dgPairProxi& pair = pairNode->GetInfo();
			  HACD_ASSERT((pair.m_edgeA && pair.m_edgeA) || (!pair.m_edgeA && !pair.m_edgeA));
			  if (pair.m_edgeA && pair.m_edgeB) 
			  {
				  break;
			  }
			  head.Pop();
		  }
		  return concavity;
	  }

	  hacd::HaF64 m_area;
	  hacd::HaF64 m_perimeter;
};

dgMeshEffect::dgMeshEffect(const dgMeshEffect& source, hacd::HaF32 normalizedConcavity, hacd::HaI32 maxCount, hacd::ICallback* callback) 
	:dgPolyhedra()
{

	if (callback)
	{
		callback->ReportProgress("+ Calculating initial cost\n", 0.0);
	}

	Init(true);

	dgMeshEffect mesh(source);
	hacd::HaI32 faceCount = mesh.GetTotalFaceCount() + 1;
	dgStack<dgClusterList> clusterPool(faceCount);
	dgClusterList* const clusters = &clusterPool[0];

	for (hacd::HaI32 i = 0; i < faceCount; i++) 
	{
		clusters[i] = dgClusterList();
	}


	hacd::HaI32 meshMask = mesh.IncLRU();
	const dgBigVector* const points = mesh.m_points;

	// enumerate all faces, and initialize cluster pool
	dgMeshEffect::Iterator iter(mesh);

	hacd::HaI32 clusterIndex = 1;
	for (iter.Begin(); iter; iter++) 
	{
		dgEdge* const edge = &(*iter);
		edge->m_userData = hacd::HaU64 (-1);
		if ((edge->m_mark != meshMask) && (edge->m_incidentFace > 0)) 
		{
			hacd::HaF64 perimeter = hacd::HaF64(0.0f);
			dgEdge* ptr = edge;
			do 
			{
				dgBigVector p1p0(points[ptr->m_incidentVertex] - points[ptr->m_prev->m_incidentVertex]);
				perimeter += sqrt(p1p0 % p1p0);
				ptr->m_incidentFace = clusterIndex;
				ptr->m_mark = meshMask;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dgBigVector normal = mesh.FaceNormal(edge, &points[0][0], sizeof(dgBigVector));
			hacd::HaF64 mag = sqrt(normal % normal);

			dgClusterFace& faceInfo = clusters[clusterIndex].Append()->GetInfo();

			faceInfo.m_edge = edge;
			faceInfo.m_perimeter = perimeter;
			faceInfo.m_area = hacd::HaF64(0.5f) * mag;
			faceInfo.m_normal = normal.Scale(hacd::HaF64(1.0f) / mag);

			clusters[clusterIndex].m_perimeter = perimeter;
			clusters[clusterIndex].m_area = faceInfo.m_area;

			clusterIndex++;
		}
	}

	HACD_ASSERT(faceCount == clusterIndex);

	// recalculate all edge cost
	dgStack<hacd::HaI32> vertexMarksArray(mesh.GetVertexCount());
	dgStack<dgBigVector> vertexArray(mesh.GetVertexCount() * 2);
	
	dgBigVector* const vertexPool = &vertexArray[0];
	hacd::HaI32* const vertexMarks = &vertexMarksArray[0];
	memset(&vertexMarks[0], 0, vertexMarksArray.GetSizeInBytes());

	dgList<dgPairProxi> proxyList;
	dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64> heap(mesh.GetCount() + 1000);

	hacd::HaI32 vertexMark = 0;

	dgBigVector size;
	dgMatrix matrix (mesh.CalculateOOBB (size));
	hacd::HaF64 diagonalLength =  hacd::HaF64 (2.0f) * dgSqrt (size % size);
	hacd::HaF64 diagonalInv = hacd::HaF64 (1.0f) / diagonalLength;
	
	// use same weight coefficient used bu Khaled Mamou  
	hacd::HaF64 aspectRatioCoeficent = normalizedConcavity * diagonalLength / hacd::HaF32(10.0f);
	meshMask = mesh.IncLRU();

	// calculate all the initial cost of all clusters, which at this time are all a single faces
	for (hacd::HaI32 faceIndex = 1; faceIndex < faceCount; faceIndex++) 
	{
		vertexMark++;
		dgClusterList& clusterList = clusters[faceIndex];
		HACD_ASSERT(clusterList.GetFirst()->GetInfo().m_edge->m_incidentFace == faceIndex);
		clusterList.CalculateNodeCost(mesh, meshMask, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);
	}

	hacd::HaF32 percent = 0;

	if (callback)
	{
		callback->ReportProgress("+ Merging Clusters\n",percent);
	}
	
	hacd::HaI32 essentialClustersCount = faceCount - 1;
	hacd::HaI32 oldCount = essentialClustersCount;
	hacd::HaF64 maxConcavity = dgClusterList::CollapseClusters (mesh, clusters, heap, normalizedConcavity, maxCount, essentialClustersCount,
															  vertexMark, vertexPool, vertexMarks, diagonalInv, aspectRatioCoeficent, proxyList);



	while ((maxConcavity > normalizedConcavity) && (essentialClustersCount < oldCount)) 
	{
		percent+=5;
		if ( percent > 90 )
		{
			percent = 90;
		}
		if ( callback )
		{
			char msg[512];
			HACD_SPRINTF_S(msg,512, "%3.2f %% \t \t \r", percent);
			callback->ReportProgress(msg, percent);
		}
		// this means that Khaled Mamou flat (perimeter^ / area) is not really a gradient function
		// which mean the each time two collapse were fused the need on  the they combined concavity could increase 
		// the concavity of the mesh and render, what this means is that some of the clusters still active can actually have lower 
		// lower concavity than what eh combine distance concavity + perimeter/area indicate.
		// this is similar to a hill climbing algorithm or steepest descend function that has small local minimal.
		// the algorithm can terminate on a local minimal.
		// the solution to this si to use an annealing that  increased the temperature 
		// of one of the parameter as each time the function fail. I am no going to use any complex annealing function 
		// I will use the spirit of annealing, basically I will double the perimeter/area part of the concavity 
		//and try again, an continue doing as long as at least one pair of cluster is collapsed in the next pass.

		essentialClustersCount = 0;
		for (hacd::HaI32 faceIndex = 1; faceIndex < faceCount; faceIndex++) 
		{
			dgClusterList& clusterList = clusters[faceIndex];
			if (clusterList.GetCount())	
			{
				essentialClustersCount ++;
			}
		}
		normalizedConcavity *= hacd::HaF64 (1.1f);
		//normalizedConcavity *= hacd::HaF64(2.0f);
		oldCount = essentialClustersCount;
		maxConcavity = dgClusterList::CollapseClusters (mesh, clusters, heap, normalizedConcavity, maxCount, essentialClustersCount,
														vertexMark, vertexPool, vertexMarks, diagonalInv, aspectRatioCoeficent, proxyList);
	}

	BeginPolygon();
	hacd::HaF32 layer = hacd::HaF32(0.0f);

	dgVertexAtribute polygon[256];
	memset(polygon, 0, sizeof(polygon));
	dgArray<dgBigVector> convexVertexBuffer(1024);
	for (hacd::HaI32 i = 0; i < faceCount; i++) 
	{
		dgClusterList& clusterList = clusters[i];

		if (callback)
			callback->ReportProgress("+ Generating Hulls", (i / (float)faceCount) * 100.0f);


		if (clusterList.GetCount())	
		{
			hacd::HaI32 count = 0;
			for (dgClusterList::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) 
			{
				dgClusterFace& face = node->GetInfo();
				dgEdge* edge = face.m_edge;

				dgEdge* sourceEdge = source.FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
				do 
				{
					hacd::HaI32 index = edge->m_incidentVertex;
					convexVertexBuffer[count] = points[index];

					count++;
					sourceEdge = sourceEdge->m_next;
					edge = edge->m_next;
				} while (edge != face.m_edge);
			}

			dgConvexHull3d convexHull(&convexVertexBuffer[0].m_x, sizeof(dgBigVector), count, 0.0);

			if (convexHull.GetCount()) 
			{
				const dgBigVector* const vertex = convexHull.GetVertexPool();
				for (dgConvexHull3d::dgListNode* node = convexHull.GetFirst(); node; node = node->GetNext()) 
				{
					const dgConvexHull3DFace* const face = &node->GetInfo();
					hacd::HaI32 i0 = face->m_index[0];
					hacd::HaI32 i1 = face->m_index[1];
					hacd::HaI32 i2 = face->m_index[2];
					polygon[0].m_vertex = vertex[i0];
					polygon[0].m_vertex.m_w = layer;
					polygon[1].m_vertex = vertex[i1];
					polygon[1].m_vertex.m_w = layer;
					polygon[2].m_vertex = vertex[i2];
					polygon[2].m_vertex.m_w = layer;
					AddPolygon(3, &polygon[0].m_vertex.m_x, sizeof(dgVertexAtribute), 0);
				}
				layer += hacd::HaF32(1.0f);
			}
		}
	}
	EndPolygon(1.0e-5f);

	for (hacd::HaI32 i = 0; i < faceCount; i++) 
	{
		clusters[i].RemoveAll();
	}
}


dgMeshEffect* dgMeshEffect::CreateConvexApproximation(hacd::HaF32 maxConcavity, hacd::HaI32 maxCount, hacd::ICallback* callback) const
{
	if (maxCount <= 1) {
		maxCount = 1;
	}
	if (maxConcavity <= hacd::HaF32 (1.0e-5f)) 
	{
		maxConcavity = hacd::HaF32 (1.0e-5f);
	}
	dgMeshEffect* const convexPartion = HACD_NEW(dgMeshEffect)(*this, maxConcavity, maxCount,callback );
	return convexPartion;
}

#else
// based of the paper Hierarchical Approximate Convex Decomposition by Khaled Mamou 
// with his permission to adapt his algorithm so be more efficient.
// available http://sourceforge.net/projects/hacd/
// for the details http://kmamou.blogspot.com/


class dgClusterFace
{
	public:
	dgClusterFace()
	{
	}
	~dgClusterFace()
	{
	}

	dgEdge* m_edge;
	hacd::HaF64 m_area;
	hacd::HaF64 m_perimeter;
	dgBigVector m_normal;
};

class dgPairProxi
{
	public:
	dgPairProxi()
		:m_edgeA(NULL)
		,m_edgeB(NULL)
		,m_area(hacd::HaF64(0.0f))
		,m_perimeter(hacd::HaF64(0.0f))
	{
	}

	~dgPairProxi()
	{
	}

	dgEdge* m_edgeA;
	dgEdge* m_edgeB;
	hacd::HaF64 m_area;
	hacd::HaF64 m_perimeter;
};

class dgClusterList: public dgList<dgClusterFace>
{
	public:
	dgClusterList() 
		: dgList<dgClusterFace>()
		,m_area (hacd::HaF32 (0.0f))
		,m_perimeter (hacd::HaF32 (0.0f))
	  {
	  }

	  ~dgClusterList()
	  {
	  }

	  hacd::HaI32 AddVertexToPool(const dgMeshEffect& mesh, dgBigVector* const vertexPool, hacd::HaI32* const vertexMarks, hacd::HaI32 vertexMark)
	  {
		  hacd::HaI32 count = 0;

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& face = node->GetInfo();

			  dgEdge* edge = face.m_edge;
			  do {
				  hacd::HaI32 index = edge->m_incidentVertex;
				  if (vertexMarks[index] != vertexMark)
				  {
					  vertexMarks[index] = vertexMark;
					  vertexPool[count] = points[index];
					  count++;
				  }
				  edge = edge->m_next;
			  } while (edge != face.m_edge);
		  }
		  return count;
	  }

	  hacd::HaF64 CalculateTriangleConcavity2(const dgConvexHull3d& convexHull, dgClusterFace& info, hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, const dgBigVector* const points) const
	  {
		  hacd::HaU32 head = 1;
		  hacd::HaU32 tail = 0;
		  dgBigVector pool[1<<8][3];

		  pool[0][0] = points[i0];
		  pool[0][1] = points[i1];
		  pool[0][2] = points[i2];

		  const dgBigVector step(info.m_normal.Scale(hacd::HaF64(4.0f) * convexHull.GetDiagonal()));

		  hacd::HaF64 concavity = hacd::HaF32(0.0f);
		  hacd::HaF64 minArea = hacd::HaF32(0.125f);
		  hacd::HaF64 minArea2 = minArea * minArea * 0.5f;

		  // weight the area by the area of the face 
		  //dgBigVector edge10(pool[0][1] - pool[0][0]);
		  //dgBigVector edge20(pool[0][2] - pool[0][0]);
		  //dgBigVector triangleArea = edge10 * edge20;
		  //hacd::HaF64 triangleArea2 = triangleArea % triangleArea;
		  //if ((triangleArea2 / minArea2)> hacd::HaF32 (64.0f)) {
			// minArea2 = triangleArea2 / hacd::HaF32 (64.0f);
		  //}

		  hacd::HaI32 maxCount = 4;
		  hacd::HaU32 mask = (sizeof (pool) / (3 * sizeof (pool[0][0]))) - 1;
		  while ((tail != head) && (maxCount >= 0)) {
			  //stack--;
			  maxCount --;
			  dgBigVector p0(pool[tail][0]);
			  dgBigVector p1(pool[tail][1]);
			  dgBigVector p2(pool[tail][2]);
			  tail =  (tail + 1) & mask;

			  dgBigVector q1((p0 + p1 + p2).Scale(hacd::HaF64(1.0f / 3.0f)));
			  dgBigVector q0(q1 + step);

			  hacd::HaF64 param = convexHull.RayCast(q0, q1);
			  if (param > hacd::HaF64(1.0f)) {
				  param = hacd::HaF64(1.0f);
			  }
			  dgBigVector dq(step.Scale(hacd::HaF32(1.0f) - param));
			  hacd::HaF64 length2 = dq % dq;
			  if (length2 > concavity) {
				  concavity = length2;
			  }

			  if (((head + 1) & mask) != tail) {
				  dgBigVector edge10(p1 - p0);
				  dgBigVector edge20(p2 - p0);
				  dgBigVector n(edge10 * edge20);
				  hacd::HaF64 area2 = n % n;
				  if (area2 > minArea2) {
					  dgBigVector p01((p0 + p1).Scale(hacd::HaF64(0.5f)));
					  dgBigVector p12((p1 + p2).Scale(hacd::HaF64(0.5f)));
					  dgBigVector p20((p2 + p0).Scale(hacd::HaF64(0.5f)));

					  pool[head][0] = p0;
					  pool[head][1] = p01;
					  pool[head][2] = p20;
					  head = (head + 1) & mask;

					  if (((head + 1) & mask) != tail) {
						  pool[head][0] = p1;
						  pool[head][1] = p12;
						  pool[head][2] = p01;
						  head = (head + 1) & mask;

						  if (((head + 1) & mask) != tail)	{
							  pool[head][0] = p2;
							  pool[head][1] = p20;
							  pool[head][2] = p12;
							  head = (head + 1) & mask;
						  }
					  }
				  }
			  }
		  }
		  return concavity;
	  }

	  hacd::HaF64 CalculateConcavity2(const dgConvexHull3d& convexHull, const dgMeshEffect& mesh)
	  {
		  hacd::HaF64 concavity = hacd::HaF32(0.0f);

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();

		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& info = node->GetInfo();
			  hacd::HaI32 i0 = info.m_edge->m_incidentVertex;
			  hacd::HaI32 i1 = info.m_edge->m_next->m_incidentVertex;
			  for (dgEdge* edge = info.m_edge->m_next->m_next; edge != info.m_edge; edge = edge->m_next) {
				  hacd::HaI32 i2 = edge->m_incidentVertex;
				  hacd::HaF64 val = CalculateTriangleConcavity2(convexHull, info, i0, i1, i2, points);
				  if (val > concavity) {
					  concavity = val;
				  }
				  i1 = i2;
			  }
		  }

		  return concavity;
	  }

	  bool IsClusterCoplanar(const dgBigPlane& plane,
		  const dgMeshEffect& mesh) const
	  {
		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  dgClusterFace& info = node->GetInfo();

			  dgEdge* ptr = info.m_edge;
			  do {
				  const dgBigVector& p = points[ptr->m_incidentVertex];
				  hacd::HaF64 dist = fabs(plane.Evalue(p));
				  if (dist > hacd::HaF64(1.0e-5f)) {
					  return false;
				  }
				  ptr = ptr->m_next;
			  } while (ptr != info.m_edge);
		  }

		  return true;
	  }

	  bool IsEdgeConvex(const dgBigPlane& plane, const dgMeshEffect& mesh,
		  dgEdge* const edge) const
	  {
		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();
		  dgEdge* const edge0 = edge->m_next;
		  dgEdge* ptr = edge0->m_twin->m_next;
		  do {
			  if (ptr->m_twin->m_incidentFace == edge->m_twin->m_incidentFace) {
				  HACD_ASSERT(edge0->m_incidentVertex == ptr->m_incidentVertex);
				  dgBigVector e0(points[edge0->m_twin->m_incidentVertex] - points[edge0->m_incidentVertex]);
				  dgBigVector e1(points[ptr->m_twin->m_incidentVertex] - points[edge0->m_incidentVertex]);
				  dgBigVector normal(e0 * e1);
				  return (normal % plane) > hacd::HaF64(0.0f);
			  }
			  ptr = ptr->m_twin->m_next;
		  } while (ptr != edge->m_twin);

		  HACD_ASSERT(0);
		  return true;
	  }

	  // calculate the convex hull of a conched group of faces,
	  // and measure the concavity, according to Khaled convexity criteria, which is basically
	  // has two components, 
	  // the first is ratio  between the the perimeter of the group of faces
	  // and the second the largest distance from any of the face to the surface of the hull 
	  // when the faces are are a strip of a convex hull the perimeter ratio components is 1.0 and the distance to the hull is zero
	  // this is the ideal concavity.
	  // when the face are no part of the hull, then the worse distance to the hull is dominate the the metric
	  // this matrix is used to place all possible combination of this cluster with any adjacent cluster into a priority heap and determine 
	  // which pair of two adjacent cluster is the best selection for combining the into a larger cluster.
	  void CalculateNodeCost(dgMeshEffect& mesh, hacd::HaI32 meshMask,
		  dgBigVector* const vertexPool, hacd::HaI32* const vertexMarks,
		  hacd::HaI32& vertexMark, dgClusterList* const clusters, hacd::HaF64 diagonalInv,
		  hacd::HaF64 aspectRatioCoeficent, dgList<dgPairProxi>& proxyList,
		  dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64>& heap)
	  {
		  hacd::HaI32 faceIndex = GetFirst()->GetInfo().m_edge->m_incidentFace;

		  const dgBigVector* const points = (dgBigVector*) mesh.GetVertexPool();

		  bool flatStrip = true;
		  dgBigPlane plane(GetFirst()->GetInfo().m_normal, -(points[GetFirst()->GetInfo().m_edge->m_incidentVertex] % GetFirst()->GetInfo().m_normal));
		  if (GetCount() > 1) {
			  flatStrip = IsClusterCoplanar(plane, mesh);
		  }

		  vertexMark++;
		  hacd::HaI32 vertexCount = AddVertexToPool(mesh, vertexPool, vertexMarks, vertexMark);
		  for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
			  //dgClusterFace& clusterFaceA = GetFirst()->GetInfo();
			  dgClusterFace& clusterFaceA = node->GetInfo();

			  dgEdge* edge = clusterFaceA.m_edge;
			  do {
				  hacd::HaI32 twinFaceIndex = edge->m_twin->m_incidentFace;
				  if ((edge->m_mark != meshMask) && (twinFaceIndex != faceIndex) && (twinFaceIndex > 0)) {

					  dgClusterList& clusterListB = clusters[twinFaceIndex];

					  vertexMark++;
					  hacd::HaI32 extraCount = clusterListB.AddVertexToPool(mesh, &vertexPool[vertexCount], &vertexMarks[0], vertexMark);

					  hacd::HaI32 count = vertexCount + extraCount;
					  dgConvexHull3d convexHull(&vertexPool[0].m_x, sizeof(dgBigVector), count, 0.0);

					  hacd::HaF64 concavity = hacd::HaF64(0.0f);
					  if (convexHull.GetVertexCount()) {
						  concavity = sqrt(GetMax(CalculateConcavity2(convexHull, mesh), clusterListB.CalculateConcavity2(convexHull, mesh)));
						  if (concavity < hacd::HaF64(1.0e-3f)) {
							  concavity = hacd::HaF64(0.0f);
						  }
					  }

					  if ((concavity == hacd::HaF64(0.0f)) && flatStrip)  {
						  if (clusterListB.IsClusterCoplanar(plane, mesh)) {
							  bool concaveEdge = !(IsEdgeConvex(plane, mesh, edge) && IsEdgeConvex(plane, mesh, edge->m_twin));
							  if (concaveEdge) {
								  concavity += 1000.0f;
							  }
						  }
					  }

					  dgBigVector p1p0(points[edge->m_twin->m_incidentVertex] - points[edge->m_incidentVertex]);
					  hacd::HaF64 edgeLength = hacd::HaF64(2.0f) * sqrt(p1p0 % p1p0);

					  hacd::HaF64 area = m_area + clusterListB.m_area;
					  hacd::HaF64 perimeter = m_perimeter + clusterListB.m_perimeter - edgeLength;
					  hacd::HaF64 edgeCost = perimeter * perimeter / (hacd::HaF64(4.0f * 3.141592f) * area);
					  hacd::HaF64 cost = diagonalInv * (concavity + edgeCost * aspectRatioCoeficent);

					  if ((heap.GetCount() + 20) > heap.GetMaxCount()) {
						  for (hacd::HaI32 i = heap.GetCount() - 1; i >= 0; i--) {
							  dgList<dgPairProxi>::dgListNode* emptyNode = heap[i];
							  dgPairProxi& emptyPair = emptyNode->GetInfo();
							  if ((emptyPair.m_edgeA == NULL) && (emptyPair.m_edgeB == NULL)) {
								  heap.Remove(i);
							  }
						  }
					  }

					  dgList<dgPairProxi>::dgListNode* pairNode = proxyList.Append();
					  dgPairProxi& pair = pairNode->GetInfo();
					  pair.m_edgeA = edge;
					  pair.m_edgeB = edge->m_twin;
					  pair.m_area = area;
					  pair.m_perimeter = perimeter;
					  edge->m_userData = hacd::HaU64(pairNode);
					  edge->m_twin->m_userData = hacd::HaU64(pairNode);
					  heap.Push(pairNode, cost);
				  }

				  edge->m_mark = meshMask;
				  edge->m_twin->m_mark = meshMask;
				  edge = edge->m_next;
			  } while (edge != clusterFaceA.m_edge);
		  }
	  }


	  hacd::HaF64 m_area;
	  hacd::HaF64 m_perimeter;
};

dgMeshEffect::dgMeshEffect(const dgMeshEffect& source, hacd::HaF32 absoluteconcavity, hacd::HaI32 maxCount, hacd::ICallback* callback) 
	:dgPolyhedra()
{
	Init(true);

	dgMeshEffect mesh(source);
	hacd::HaI32 faceCount = mesh.GetTotalFaceCount() + 1;
	dgStack<dgClusterList> clusterPool(faceCount);
	dgClusterList* const clusters = &clusterPool[0];

	for (hacd::HaI32 i = 0; i < faceCount; i++) {
		clusters[i] = dgClusterList();
	}

	hacd::HaI32 meshMask = mesh.IncLRU();
	const dgBigVector* const points = mesh.m_points;

	// enumerate all faces, and initialize cluster pool
	dgMeshEffect::Iterator iter(mesh);

	hacd::HaI32 clusterIndex = 1;
	for (iter.Begin(); iter; iter++) {
		dgEdge* const edge = &(*iter);
		edge->m_userData = hacd::HaU64 (-1);
		if ((edge->m_mark != meshMask) && (edge->m_incidentFace > 0)) {
			hacd::HaF64 perimeter = hacd::HaF64(0.0f);
			dgEdge* ptr = edge;
			do {
				dgBigVector p1p0(points[ptr->m_incidentVertex] - points[ptr->m_prev->m_incidentVertex]);
				perimeter += sqrt(p1p0 % p1p0);
				ptr->m_incidentFace = clusterIndex;

				ptr->m_mark = meshMask;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dgBigVector normal = mesh.FaceNormal(edge, &points[0][0], sizeof(dgBigVector));
			hacd::HaF64 mag = sqrt(normal % normal);

			dgClusterFace& faceInfo = clusters[clusterIndex].Append()->GetInfo();

			faceInfo.m_edge = edge;
			faceInfo.m_perimeter = perimeter;
			faceInfo.m_area = hacd::HaF64(0.5f) * mag;
			faceInfo.m_normal = normal.Scale(hacd::HaF64(1.0f) / mag);

			clusters[clusterIndex].m_perimeter = perimeter;
			clusters[clusterIndex].m_area = faceInfo.m_area;

			clusterIndex++;
		}
	}

	HACD_ASSERT(faceCount == clusterIndex);

	// recalculate all edge cost
	dgStack<hacd::HaI32> vertexMarksArray(mesh.GetVertexCount());
	dgStack<dgBigVector> vertexArray(mesh.GetVertexCount() * 2);
	
	dgBigVector* const vertexPool = &vertexArray[0];
	hacd::HaI32* const vertexMarks = &vertexMarksArray[0];
	memset(&vertexMarks[0], 0, vertexMarksArray.GetSizeInBytes());

	dgList<dgPairProxi> proxyList;
	dgUpHeap<dgList<dgPairProxi>::dgListNode*, hacd::HaF64> heap(mesh.GetCount() + 1000);

	hacd::HaI32 vertexMark = 0;

	hacd::HaF64 diagonalInv = hacd::HaF32(1.0f);
	hacd::HaF64 aspectRatioCoeficent = absoluteconcavity / hacd::HaF32(10.0f);
	meshMask = mesh.IncLRU();

	if (callback)
		callback->ReportProgress("+ Calculating initial cost\n", 0.0);

	// calculate all the initial cost of all clusters, which at this time are all a single faces
	for (hacd::HaI32 faceIndex = 1; faceIndex < faceCount; faceIndex++) {
		vertexMark++;
		dgClusterList& clusterList = clusters[faceIndex];
		HACD_ASSERT(clusterList.GetFirst()->GetInfo().m_edge->m_incidentFace == faceIndex);
		clusterList.CalculateNodeCost(mesh, meshMask, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);
	}
	
	hacd::HaF32 progress, progressOld;
	progress = progressOld = 0.0f;
	hacd::HaF32 ptgStep = 1.0f;
	hacd::HaF32 startConcavity = (hacd::HaF32)heap.Value();
	hacd::HaF32 targetRange = absoluteconcavity - startConcavity;
	if (targetRange == 0.0f)
		targetRange = 1.0f;

	if (callback)
		callback->ReportProgress("+ Merging Clusters\n", 0.0);

	// calculate all essential convex clusters by merging the all possible clusters according 
	// which combined concavity es lower that the max absolute concavity 
	// select the pair with the smaller concavity and fuse then into a larger cluster
	hacd::HaI32 essencialClustersCount = faceCount - 1;
	char msg[1024];
	while (heap.GetCount() && ((heap.Value() < absoluteconcavity) || (essencialClustersCount > maxCount))) 
	{
		progress = (((hacd::HaF32)heap.Value() - startConcavity) / targetRange) * 100.0f;
		if ((progress-progressOld) > ptgStep && callback)
		{
			HACD_SPRINTF_S(msg, "%3.2f %% \t \t \r", progress);
			callback->ReportProgress(msg, progress);
			progressOld = progress;
		}

		// Bail out if we receive a cancel.
		if (callback->Cancelled())
		{
			for (hacd::HaI32 i = 0; i < faceCount; i++) 
			{
				clusters[i].RemoveAll();
			}
			return;
		}	

		dgList<dgPairProxi>::dgListNode* const pairNode = heap[0];
		heap.Pop();
		dgPairProxi& pair = pairNode->GetInfo();

		HACD_ASSERT((pair.m_edgeA && pair.m_edgeA) || (!pair.m_edgeA && !pair.m_edgeA));
		if (pair.m_edgeA && pair.m_edgeB) {

			HACD_ASSERT(pair.m_edgeA->m_incidentFace != pair.m_edgeB->m_incidentFace);

			// merge two clusters
			hacd::HaI32 faceIndexA = pair.m_edgeA->m_incidentFace;
			hacd::HaI32 faceIndexB = pair.m_edgeB->m_incidentFace;
			dgClusterList* listA = &clusters[faceIndexA];
			dgClusterList* listB = &clusters[faceIndexB];
			if (pair.m_edgeA->m_incidentFace > pair.m_edgeB->m_incidentFace) {
				Swap(faceIndexA, faceIndexB);
				Swap(listA, listB);
			}
			
			while (listB->GetFirst()) {
				dgClusterList::dgListNode* const nodeB = listB->GetFirst();
				listB->Unlink(nodeB);
				dgClusterFace& faceB = nodeB->GetInfo();

				dgEdge* ptr = faceB.m_edge;
				do {
					ptr->m_incidentFace = faceIndexA;
					ptr = ptr->m_next;
				} while (ptr != faceB.m_edge);
				listA->Append(nodeB);
			}
			essencialClustersCount --;

			listB->m_area = hacd::HaF32 (0.0f);
			listB->m_perimeter = hacd::HaF32 (0.0f);
			listA->m_area = pair.m_area;
			listA->m_perimeter = pair.m_perimeter;

			// recalculated the new metric for the new cluster, and tag the used cluster as invalid, so that 
			// other potential selection do not try merge with this this one, producing convex that re use a face more than once  
			hacd::HaI32 mark = mesh.IncLRU();
			for (dgClusterList::dgListNode* node = listA->GetFirst(); node; node = node->GetNext()) {
				dgClusterFace& face = node->GetInfo();
				dgEdge* ptr = face.m_edge;
				do {
					if (ptr->m_userData != hacd::HaU64 (-1)) {
						dgList<dgPairProxi>::dgListNode* const pairNode = (dgList<dgPairProxi>::dgListNode*) ptr->m_userData;
						dgPairProxi& pairProxy = pairNode->GetInfo();
						pairProxy.m_edgeA = NULL;
						pairProxy.m_edgeB = NULL;
					}
					ptr->m_userData = hacd::HaU64 (-1);
					ptr->m_twin->m_userData = hacd::HaU64 (-1);

					if ((ptr->m_twin->m_incidentFace == faceIndexA) || (ptr->m_twin->m_incidentFace < 0)) {
						ptr->m_mark = mark;
						ptr->m_twin->m_mark = mark;
					}

					if (ptr->m_mark != mark) {
						dgClusterList& adjacentList = clusters[ptr->m_twin->m_incidentFace];
						for (dgClusterList::dgListNode* adjacentNode = adjacentList.GetFirst(); adjacentNode; adjacentNode = adjacentNode->GetNext()) {
							dgClusterFace& adjacentFace = adjacentNode->GetInfo();
							dgEdge* adjacentEdge = adjacentFace.m_edge;
							do {
								if (adjacentEdge->m_twin->m_incidentFace == faceIndexA)	{
									adjacentEdge->m_twin->m_mark = mark;
								}
								adjacentEdge = adjacentEdge->m_next;
							} while (adjacentEdge != adjacentFace.m_edge);
						}
						ptr->m_mark = mark - 1;
					}
					ptr = ptr->m_next;
				} while (ptr != face.m_edge);
			}

			// re generated the cost of merging this new all its adjacent clusters, that are still alive. 
			vertexMark++;
			listA->CalculateNodeCost(mesh, mark, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);
		}

		proxyList.Remove(pairNode);
	}

	/*
	if (callback)
	{
		HACD_SPRINTF_S(msg, "# clusters =  %lu \t C = %f\n", static_cast<unsigned long>(essencialClustersCount), heap.Value());
		(*callback)(msg, 0.0f);
	}*/

	// if the essential convex cluster count is larger than the the maximum specified by the user 
	// then resuming the cluster again to the heap and start merging then by the the worse merging criteria
	// also at this time add the distance heuristic to combine disjoint cluster as well.
	// this is where I disagree with Khaled Mamore, he uses a brute force approach by adding extra points.
	// I do this to the first partition and then connect disjoint face only on the perimeter
/*
maxCount = 1;
	while (essencialClustersCount > maxCount) {

		heap.Flush();
		meshMask = mesh.IncLRU();

		// color code each face on each cluster with it cluster index
		for (hacd::HaI32 faceIndex = 0; faceIndex < faceCount; faceIndex++) {
			dgClusterList& clusterList = clusters[faceIndex];
			if (clusterList.GetCount()) {
				for (dgClusterList::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) {
					dgClusterFace& face = node->GetInfo();
					dgEdge* ptr = face.m_edge;
					do {
						ptr->m_incidentFace = faceIndex;
						ptr = ptr->m_next;
					} while (ptr != face.m_edge);
				}
			}
		}
			
		for (hacd::HaI32 faceIndex = 0; faceIndex < faceCount; faceIndex++) {
			dgClusterList& clusterList = clusters[faceIndex];

			// note: add the disjoint cluster criteria here, but for now just ignore

			// calculate the cost again
			if (clusterList.GetCount()) {

				// note: something is wrong with my color coding that is not marking the perimeter corrently	

				vertexMark++;
				clusterList.CalculateNodeCost(mesh, meshMask, &vertexPool[0], &vertexMarks[0], vertexMark, &clusters[0], diagonalInv, aspectRatioCoeficent, proxyList, heap);	
			}
			
		}
	}
*/
	if (callback)
		callback->ReportProgress("+ Generating hulls\n", 0.0);


	BeginPolygon();
	hacd::HaF32 layer = hacd::HaF32(0.0f);

	dgVertexAtribute polygon[256];
	memset(polygon, 0, sizeof(polygon));
	dgArray<dgBigVector> convexVertexBuffer(1024);
	for (hacd::HaI32 i = 0; i < faceCount; i++) 
	{
		dgClusterList& clusterList = clusters[i];

		if (callback)
		{
#if defined( _WINDOWS )
			sprintf_s(msg, "%3.2f %% \t \t \r", (i / (float)faceCount) * 100.0f);
#else
			sprintf(msg, "%3.2f %% \t \t \r", (i / (float)faceCount) * 100.0f);
#endif
			callback->ReportProgress(msg, progress);
		}

		if (clusterList.GetCount())	{
			hacd::HaI32 count = 0;
			for (dgClusterList::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) {
				dgClusterFace& face = node->GetInfo();
				dgEdge* edge = face.m_edge;

				dgEdge* sourceEdge = source.FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
				do {
					hacd::HaI32 index = edge->m_incidentVertex;
					convexVertexBuffer[count] = points[index];

					count++;
					sourceEdge = sourceEdge->m_next;
					edge = edge->m_next;
				} while (edge != face.m_edge);
			}

			dgConvexHull3d convexHull(&convexVertexBuffer[0].m_x, sizeof(dgBigVector), count, 0.0);

			if (convexHull.GetCount()) {
				const dgBigVector* const vertex = convexHull.GetVertexPool();
				for (dgConvexHull3d::dgListNode* node = convexHull.GetFirst(); node; node = node->GetNext()) {
					const dgConvexHull3DFace* const face = &node->GetInfo();

					hacd::HaI32 i0 = face->m_index[0];
					hacd::HaI32 i1 = face->m_index[1];
					hacd::HaI32 i2 = face->m_index[2];

					polygon[0].m_vertex = vertex[i0];
					polygon[0].m_vertex.m_w = layer;

					polygon[1].m_vertex = vertex[i1];
					polygon[1].m_vertex.m_w = layer;

					polygon[2].m_vertex = vertex[i2];
					polygon[2].m_vertex.m_w = layer;

					AddPolygon(3, &polygon[0].m_vertex.m_x, sizeof(dgVertexAtribute), 0);
				}

				layer += hacd::HaF32(1.0f);
				//break;
			}
		}
	}
	EndPolygon(1.0e-5f);

	for (hacd::HaI32 i = 0; i < faceCount; i++) {
		clusters[i].RemoveAll();
	}
}


dgMeshEffect* dgMeshEffect::CreateConvexApproximation(hacd::HaF32 maxConcavity, hacd::HaI32 maxCount, hacd::ICallback* callback) const
{
	dgMeshEffect triangleMesh(*this);
	if (maxCount <= 1) {
		maxCount = 1;
	}
	if (maxConcavity <= hacd::HaF32 (1.0e-5f)) {
		maxConcavity = hacd::HaF32 (1.0e-5f);
	}
	dgMeshEffect* const convexPartion = HACD_NEW(dgMeshEffect)(triangleMesh, maxConcavity, maxCount, callback);
	return convexPartion;
}
#endif
