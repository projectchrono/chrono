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

#include "dgTypes.h"
#include "dgHeap.h"
#include "dgStack.h"
#include "dgSphere.h"
#include "dgPolyhedra.h"
#include "dgConvexHull3d.h"
#include "dgSmallDeterminant.h"
#include <string.h>


#pragma warning(disable:4100)
//#define DG_MIN_EDGE_ASPECT_RATIO  hacd::HaF64 (0.02f)

class dgDiagonalEdge
{
	public:
	dgDiagonalEdge (dgEdge* const edge)
		:m_i0(edge->m_incidentVertex), m_i1(edge->m_twin->m_incidentVertex)
	{
	}
	hacd::HaI32 m_i0;
	hacd::HaI32 m_i1;
};


struct dgEdgeCollapseEdgeHandle
{
	dgEdgeCollapseEdgeHandle (dgEdge* const newEdge)
		:m_inList(false), m_edge(newEdge)
	{
	}

	dgEdgeCollapseEdgeHandle (const dgEdgeCollapseEdgeHandle &dataHandle)
		:m_inList(true), m_edge(dataHandle.m_edge)
	{
		dgEdgeCollapseEdgeHandle* const handle = (dgEdgeCollapseEdgeHandle *)IntToPointer (m_edge->m_userData);
		if (handle) {
			HACD_ASSERT (handle != this);
			handle->m_edge = NULL;
		}
		m_edge->m_userData = hacd::HaU64 (PointerToInt(this));
	}

	~dgEdgeCollapseEdgeHandle ()
	{
		if (m_inList) {
			if (m_edge) {
				dgEdgeCollapseEdgeHandle* const handle = (dgEdgeCollapseEdgeHandle *)IntToPointer (m_edge->m_userData);
				if (handle == this) {
					m_edge->m_userData = PointerToInt (NULL);
				}
			}
		}
		m_edge = NULL;
	}

	hacd::HaU32 m_inList;
	dgEdge* m_edge;
};


class dgVertexCollapseVertexMetric
{
	public:
	hacd::HaF64 elem[10];

	dgVertexCollapseVertexMetric (const dgBigPlane &plane) 
	{
		elem[0] = plane.m_x * plane.m_x;  
		elem[1] = plane.m_y * plane.m_y;  
		elem[2] = plane.m_z * plane.m_z;  
		elem[3] = plane.m_w * plane.m_w;  
		elem[4] = hacd::HaF64 (2.0) * plane.m_x * plane.m_y;  
		elem[5] = hacd::HaF64 (2.0) * plane.m_x * plane.m_z;  
		elem[6] = hacd::HaF64 (2.0) * plane.m_x * plane.m_w;  
		elem[7] = hacd::HaF64 (2.0) * plane.m_y * plane.m_z;  
		elem[8] = hacd::HaF64 (2.0) * plane.m_y * plane.m_w;  
		elem[9] = hacd::HaF64 (2.0) * plane.m_z * plane.m_w;  
	}

	void Clear ()
	{
		memset (elem, 0, 10 * sizeof (hacd::HaF64));
	}

	void Accumulate (const dgVertexCollapseVertexMetric& p) 
	{
		elem[0] += p.elem[0]; 
		elem[1] += p.elem[1]; 
		elem[2] += p.elem[2]; 
		elem[3] += p.elem[3]; 
		elem[4] += p.elem[4]; 
		elem[5] += p.elem[5]; 
		elem[6] += p.elem[6]; 
		elem[7] += p.elem[7]; 
		elem[8] += p.elem[8]; 
		elem[9] += p.elem[9]; 
	}

	void Accumulate (const dgBigPlane& plane) 
	{
		elem[0] += plane.m_x * plane.m_x;  
		elem[1] += plane.m_y * plane.m_y;  
		elem[2] += plane.m_z * plane.m_z;  
		elem[3] += plane.m_w * plane.m_w;  

		elem[4] += hacd::HaF64 (2.0f) * plane.m_x * plane.m_y;  
		elem[5] += hacd::HaF64 (2.0f) * plane.m_x * plane.m_z;  
		elem[7] += hacd::HaF64 (2.0f) * plane.m_y * plane.m_z;  

		elem[6] += hacd::HaF64 (2.0f) * plane.m_x * plane.m_w;  
		elem[8] += hacd::HaF64 (2.0f) * plane.m_y * plane.m_w;  
		elem[9] += hacd::HaF64 (2.0f) * plane.m_z * plane.m_w;  
	}


	hacd::HaF64 Evalue (const dgVector &p) const 
	{
		hacd::HaF64 acc = elem[0] * p.m_x * p.m_x + elem[1] * p.m_y * p.m_y + elem[2] * p.m_z * p.m_z + 
						elem[4] * p.m_x * p.m_y + elem[5] * p.m_x * p.m_z + elem[7] * p.m_y * p.m_z + 
						elem[6] * p.m_x + elem[8] * p.m_y + elem[9] * p.m_z + elem[3];  
		return fabs (acc);
	}
};



#if 0
namespace InternalPolyhedra
{
	

	struct VertexCache: public dgList<dgEdge*>
	{
		hacd::HaI32 size;

		VertexCache (hacd::HaI32 t, dgMemoryAllocator* const allocator)
			:dgList<dgEdge*>(allocator)
		{
			size   = t;
		} 


		hacd::HaI32 IsInCache (dgEdge *edge)   const
		{
			hacd::HaI32 score;
			dgEdge *ptr;

			score = GetCount() + 2;
			Iterator iter (*this);
			for (iter.End(); iter; iter --) {
				ptr = *iter;
				if (ptr->m_incidentVertex == edge->m_incidentVertex) {
					return score;
				}
				score --;
			}
			return 0;
		}

		hacd::HaI32 AddEdge (dgEdge *edge) 
		{
			if (IsInCache (edge) == 0)	{
				Addtop (edge);
				if (GetCount() > size) {
					Remove(GetLast());
				}
				return 1;
			}
			return 0;
		}


		dgEdge *GetEdge (hacd::HaI32 mark) const 
		{
			dgEdge *ptr;
			dgEdge *edge;

			if (GetCount()) {
				Iterator iter (*this);
				for (iter.End(); iter; iter --) {
					ptr = *iter;
					edge = ptr;
					do {
						if (edge->m_incidentFace > 0) {
							if (edge->m_mark != mark) {
								return edge;
							}
						}
						edge = edge->m_twin->m_next;
					} while (ptr != edge);
				}
			}
			return NULL;
		}
	};
















	/*
	static bool CheckIfCoplanar (
	const dgBigPlane& plane, 
	dgEdge *face, 
	const hacd::HaF32* const pool, 
	hacd::HaI32 stride) 
	{
	dgEdge* ptr;
	hacd::HaF64 dist;

	ptr = face;
	do {
	dgBigVector p (&pool[ptr->m_incidentVertex * stride]);
	dist = fabs (plane.Evalue (p));
	if (dist > hacd::HaF64(0.08)) {
	return false;
	}
	ptr = ptr->m_next;
	} while (ptr != face);

	return true;
	}
	*/















	static void GetAdjacentCoplanarFacesPerimeter (
		dgPolyhedra& perimeter,
		const dgPolyhedra& polyhedra,
		dgEdge* const face, 
		const hacd::HaF32* const pool,  
		hacd::HaI32 strideInBytes,
		dgEdge** const stack,
		hacd::HaI32* const faceIndex)
	{
		const hacd::HaF32 normalDeviation = hacd::HaF32 (0.9999f);
		dgStack<hacd::HaI32>facesIndex (4096);

		HACD_ASSERT (face->m_incidentFace > 0);

		polyhedra.IncLRU();
		hacd::HaI32 faceMark = polyhedra.IncLRU();

		dgVector normal (polyhedra.FaceNormal (face, pool, strideInBytes));
		hacd::HaF32 dot = normal % normal;
		if (dot < hacd::HaF32 (1.0e-12f)) {
			dgEdge* ptr = face;
			hacd::HaI32 faceIndexCount	= 0;
			do {
				faceIndex[faceIndexCount] = ptr->m_incidentVertex;
				faceIndexCount	++;
				ptr->m_mark = faceMark;
				ptr = ptr->m_next;
			} while (ptr != face);
			perimeter.AddFace (faceIndexCount, faceIndex);
			return;
		}
		normal = normal.Scale (hacd::HaF32 (1.0f) / dgSqrt (dot));

		stack[0] = face;
		hacd::HaI32 index = 1;
		perimeter.BeginFace();
		while (index) {
			index --;
			dgEdge* edge = stack[index];

			if (edge->m_mark == faceMark) {
				continue;
			}

			dgVector normal1 (polyhedra.FaceNormal (edge, pool, strideInBytes));
			dot = normal1 % normal1;
			if (dot < hacd::HaF32 (1.0e-12f)) {
				dot = hacd::HaF32 (1.0e-12f);
			}
			normal1 = normal1.Scale (hacd::HaF32 (1.0f) / dgSqrt (dot));

			dot = normal1 % normal;
			if (dot >= normalDeviation) {
				dgEdge* ptr = edge;
				hacd::HaI32 faceIndexCount	= 0;
				do {
					faceIndex[faceIndexCount] = ptr->m_incidentVertex;
					faceIndexCount	++;
					ptr->m_mark = faceMark;
					if ((ptr->m_twin->m_incidentFace > 0) && (ptr->m_twin->m_mark != faceMark)) {
						stack[index] = ptr->m_twin;
						index ++;
						HACD_ASSERT (index < polyhedra.GetCount() / 2);
					}
					ptr = ptr->m_next;
				} while (ptr != edge);
				perimeter.AddFace (faceIndexCount, faceIndex);
			}
		}
		perimeter.EndFace();

		dgPolyhedra::Iterator iter (perimeter);
		for (iter.Begin(); iter; ) {
			dgEdge* edge = &(*iter);
			iter ++;
			if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
				if ( perimeter.GetNodeFromInfo (*edge->m_twin)	== iter.GetNode()) {
					iter ++;
				}
				perimeter.DeleteEdge (edge);
			}
		}
	}
}


dgPolyhedraDescriptor::dgPolyhedraDescriptor (const dgPolyhedra& Polyhedra)
	:m_unboundedLoops ()
{
	Update (Polyhedra);
}

dgPolyhedraDescriptor::~dgPolyhedraDescriptor ()
{
}

void dgPolyhedraDescriptor::Update (const dgPolyhedra& srcPolyhedra)
{
	hacd::HaI32 saveMark;
	hacd::HaI32 faceCountLocal;
	hacd::HaI32 edgeCountLocal;
	hacd::HaI32 vertexCountLocal;
	hacd::HaI32 maxVertexIndexLocal;
	dgEdge *ptr;
	dgEdge *edge;
	dgPolyhedra* polyhedra;

	polyhedra = (dgPolyhedra*) &srcPolyhedra; 

	faceCountLocal = 0;
	edgeCountLocal = 0;
	vertexCountLocal	= 0;
	maxVertexIndexLocal = -1;

	saveMark = polyhedra->m_edgeMark;
	if (saveMark < 8) {
		saveMark	= 8;
	}
	polyhedra->m_edgeMark = 8;
	dgPolyhedra::Iterator iter(*polyhedra);
	for (iter.Begin(); iter; iter ++) {
		edge = &(*iter);
		edge->m_mark = 0;
		edgeCountLocal ++;
		if (edge->m_incidentVertex > maxVertexIndexLocal) {
			maxVertexIndexLocal = edge->m_incidentVertex;
		}
	}

	m_unboundedLoops.RemoveAll();
	for (iter.Begin(); iter; iter ++) {
		edge = &(*iter);

		if (edge->m_incidentFace < 0) {
			if (~edge->m_mark & 1) {
				m_unboundedLoops.Append (edge);
				ptr = edge;
				do {
					ptr->m_mark |= 1;
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		} 

		if (~edge->m_mark & 2) {
			vertexCountLocal ++; 
			ptr = edge;
			do {
				ptr->m_mark |= 2;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}

		if (~edge->m_mark & 4) {
			ptr = edge;
			faceCountLocal ++; 
			do {
				ptr->m_mark |= 4;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	m_edgeCount = edgeCountLocal;
	m_faceCount = faceCountLocal;
	m_vertexCount = vertexCountLocal;
	m_maxVertexIndex = maxVertexIndexLocal + 1;

	polyhedra->m_edgeMark = saveMark;
}







void dgPolyhedra::DeleteAllFace()
{
	m_baseMark	= 0;
	m_edgeMark = 0;
	m_faceSecuence = 0;
	RemoveAll();
}


bool dgPolyhedra::SanityCheck () const
{
	//HACD_ASSERT (0);
	return true;
	/*
	hacd::HaI32 i;
	hacd::HaI32 coorCount;
	dgEdge *ptr;
	dgEdge *edge;
	dgTreeNode *node;
	dgStack<hacd::HaI32> coor(65536);

	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
	node = iter.GetNode();
	if (!node->IsAlive()) {
	return false;
	}

	edge = &(*iter);

	if ((edge->m_incidentFace < 0) && (edge->m_twin->m_incidentFace < 0)) {
	return false;
	}


	if (edge->m_mark > m_edgeMark) {
	return false;
	}

	node = iter.GetNode();
	dgPairKey key (edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
	if (key.GetVal() != node->GetKey()) {
	return false;
	}

	ptr = edge;
	do {
	if (ptr->m_incidentVertex != edge->m_incidentVertex) {
	return false;
	}
	ptr = ptr->m_twin->m_next;
	} while (ptr != edge);

	ptr = edge;
	coorCount = 0;
	do {
	if (coorCount  * sizeof (hacd::HaI32) > coor.GetSizeInBytes()) {
	return false;
	}
	if (ptr->m_incidentFace != edge->m_incidentFace) {
	return false;
	}
	coor [coorCount] = ptr->m_incidentVertex;
	coorCount ++;

	ptr = ptr->m_next;
	} while (ptr != edge);

	ptr = edge->m_prev;
	i = 0;
	do {
	if (i * sizeof (hacd::HaI32) > coor.GetSizeInBytes()) {
	return false;
	}
	if (ptr->m_incidentFace != edge->m_incidentFace) {
	return false;
	}

	if (coor [coorCount - i - 1] != ptr->m_incidentVertex) {
	return false;
	}

	i ++;
	ptr = ptr->m_prev;
	} while (ptr != edge->m_prev);

	if (edge->m_twin->m_twin != edge) {
	return false;
	}

	if (edge->m_prev->m_next != edge) {
	return false;
	}

	if (edge->m_next->m_prev != edge) {
	return false;
	}
	}

	return dgTree <dgEdge, hacd::HaI64>::SanityCheck();
	*/
}











dgEdge *dgPolyhedra::FindVertexNode (hacd::HaI32 v) const
{
	dgEdge *edge;
	dgTreeNode *node;

	dgPairKey key (0, v);
	node = FindGreaterEqual (key.GetVal());
	edge = NULL;
	if (node) {
		dgEdge *ptr;
		ptr = node->GetInfo().m_twin;
		if (ptr->m_incidentVertex == v) {
			edge = ptr;
		}
	}

	return edge;
}

















dgEdge *dgPolyhedra::SpliteEdgeAndTriangulate (hacd::HaI32 newIndex, dgEdge* srcEdge)
{
	dgEdge* ankle = srcEdge->m_next;
	dgEdge* edge = SpliteEdge (newIndex, srcEdge);
	HACD_ASSERT (edge == ankle->m_prev);
	edge = ankle->m_prev;
	ankle = edge;

	do {
		dgEdge* const faceEdge = edge->m_twin;
		if (faceEdge->m_incidentFace > 0)	{
			if (faceEdge->m_next->m_next->m_next != faceEdge) {
				dgEdge* const edge0 = AddHalfEdge (newIndex, faceEdge->m_prev->m_incidentVertex);
				dgEdge* const twin0 = AddHalfEdge (faceEdge->m_prev->m_incidentVertex, newIndex);

				twin0->m_incidentFace = faceEdge->m_incidentFace;
				faceEdge->m_prev->m_incidentFace = m_faceSecuence;
				faceEdge->m_incidentFace = m_faceSecuence;
				edge0->m_incidentFace = m_faceSecuence;
				m_faceSecuence ++;

				edge0->m_twin = twin0;
				twin0->m_twin = edge0;

				twin0->m_next = faceEdge->m_next;
				faceEdge->m_next->m_prev = twin0;

				twin0->m_prev = faceEdge->m_prev->m_prev;
				faceEdge->m_prev->m_prev->m_next = twin0;

				edge0->m_prev = faceEdge;
				faceEdge->m_next = edge0;

				edge0->m_next = faceEdge->m_prev;
				faceEdge->m_prev->m_prev = edge0;
			}
		}

		edge = edge->m_twin->m_next;
	} while (edge != ankle);

#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck ());
#endif

	return ankle;
}




hacd::HaI32 dgPolyhedra::GetMaxIndex() const
{
	hacd::HaI32 maxIndex;
	dgEdge *edge;

	maxIndex = -1;

	dgPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) {
		edge = &(*iter);
		if (edge->m_incidentVertex > maxIndex) {
			maxIndex = edge->m_incidentVertex;
		}
	}
	return (maxIndex + 1);
}



hacd::HaI32 dgPolyhedra::GetUnboundedFaceCount () const
{
	hacd::HaI32 count;
	hacd::HaI32 mark;
	dgEdge *ptr;
	dgEdge *edge;
	Iterator iter (*this);

	count = 0;
	mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}

		if (edge->m_incidentFace > 0) {
			continue;
		}

		count ++;
		ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}



hacd::HaI32 dgPolyhedra::PackVertex (hacd::HaF32* const destArray, const hacd::HaF32* const unpackArray, hacd::HaI32 strideInBytes)
{
	HACD_ASSERT (0);
	return 0;
	/*
	hacd::HaI32 i;
	hacd::HaI32 mark;
	hacd::HaI32 stride;
	hacd::HaI32 maxCount;
	hacd::HaI32 edgeCount;
	hacd::HaI32 vertexCount;
	dgEdge *ptr;
	dgEdge *edge;
	dgTreeNode *node;
	dgTreeNode **tree;

	mark = IncLRU();

	stride = strideInBytes / sizeof (hacd::HaF32);

	maxCount	= GetCount();
	dgStack<dgTreeNode *> treePool(GetCount());
	tree = &treePool[0];

	edgeCount = 0;
	vertexCount = 0;
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
	node = iter.GetNode();
	iter ++;

	tree[edgeCount] = node;

	HACD_ASSERT (edgeCount < maxCount);
	edgeCount ++;

	edge = &node->GetInfo();
	if (edge->m_mark != mark) {
	hacd::HaI32 index;
	ptr = edge;
	index = edge->m_incidentVertex;
	memcpy (&destArray[vertexCount * stride], &unpackArray[index * stride], stride * sizeof (hacd::HaF32)); 
	do {
	ptr->m_mark = mark;
	ptr->m_incidentVertex = vertexCount;
	ptr = ptr->m_twin->m_next;

	} while (ptr != edge);
	vertexCount ++;
	}	
	}

	RemoveAll ();
	for (i = 0; i < edgeCount; i ++) {
	node = tree[i];
	node->Unkill();
	edge = &node->GetInfo();
	dgPairKey key(edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
	Insert (node, key.GetVal());
	node->Release();
	}

	return vertexCount;
	*/
}





void dgPolyhedra::GetBadEdges (dgList<dgEdge*>& faceList, const hacd::HaF32* const pool, hacd::HaI32 strideInBytes) const 
{
	dgStack<char> memPool ((4096 + 256) * (sizeof (hacd::HaF32) + sizeof(dgEdge))); 
	dgDownHeap<dgEdge*, hacd::HaF32> heap(&memPool[0], memPool.GetSizeInBytes());

	dgPolyhedra tmp (*this);
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF32));

	hacd::HaI32 mark = tmp.IncLRU();
	Iterator iter (tmp);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);

		if (edge->m_mark == mark) {
			continue;
		}
		if (edge->m_incidentFace < 0) {
			continue;
		}

		hacd::HaI32 count = 0;
		dgEdge* ptr = edge;
		do {
			count ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);

		if (count > 3) {
			dgEdge* const badEdge = InternalPolyhedra::TriangulateFace (tmp, edge, pool, stride, heap, NULL);
			if (badEdge) {
				dgEdge* const edge = FindEdge (badEdge->m_incidentVertex, badEdge->m_twin->m_incidentVertex);
				dgEdge* ptr = edge;
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != edge);

				HACD_ASSERT (edge);
				HACD_ASSERT(0);
				faceList.Append(edge);
			}
		}
	}
}




/*
void dgPolyhedra::CollapseDegenerateFaces (
dgPolyhedraDescriptor &desc, 
const hacd::HaF32* const pool, 
hacd::HaI32 strideInBytes, 
hacd::HaF32 area)
{
hacd::HaI32 i0;
hacd::HaI32 i1;
hacd::HaI32 i2;
hacd::HaI32 mark;
hacd::HaI32 stride;
hacd::HaF32 cost;
hacd::HaF32 area2;
hacd::HaF32 e10Len;
hacd::HaF32 e21Len;
hacd::HaF32 e02Len;
hacd::HaF32 faceArea;
bool someChanges;
dgEdge *ptr;
dgEdge *face;
dgEdge *edge;


#ifdef __ENABLE_SANITY_CHECK 
HACD_ASSERT (SanityCheck ());
#endif

stride = strideInBytes / sizeof (hacd::HaF32);
area2 = area * area;
dgStack<char> heapPool (desc.m_faceCount * (sizeof (hacd::HaF32) + sizeof (dgPairKey) + sizeof (hacd::HaI32))); 
HACD_ASSERT (0);
dgDownHeap<dgPairKey, hacd::HaF32> bigHeapArray(&heapPool[0], heapPool.GetSizeInBytes());

Iterator iter (*this);
do {
someChanges	= false;
mark = IncLRU();
for (iter.Begin(); iter; iter ++) {
edge = &(*iter);

if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) {
HACD_ASSERT (edge->m_next->m_next->m_next == edge);

edge->m_mark = mark;
edge->m_next->m_mark = mark;
edge->m_prev->m_mark = mark;

i0 = edge->m_incidentVertex * stride;
i1 = edge->m_next->m_incidentVertex * stride;
i2 = edge->m_prev->m_incidentVertex * stride;

dgVector p0 (&pool[i0]);
dgVector p1 (&pool[i1]);
dgVector p2 (&pool[i2]);

dgVector normal ((p2 - p0) * (p1 - p0));
faceArea = normal % normal;
if (faceArea < area2) {
someChanges	= true;
dgPairKey key (edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
bigHeapArray.Push (key, area2 - faceArea);
}
}
}

while (bigHeapArray.GetCount()) {
// collapse this edge
cost = area2 - bigHeapArray.Value();
dgPairKey key (bigHeapArray[0]);
bigHeapArray.Pop();

//			face = FindEdge (key.i0, key.i1);
face = FindEdge (key.GetLowKey(), key.GetHighKey());
if (face) {
i0 = face->m_incidentVertex * stride;
i1 = face->m_next->m_incidentVertex * stride;
i2 = face->m_prev->m_incidentVertex * stride;

dgVector p0 (&pool[i0]);
dgVector p1 (&pool[i1]);
dgVector p2 (&pool[i2]);

dgVector e10 (p1 - p0);
dgVector e21 (p2 - p1);
dgVector e02 (p0 - p2);

e10Len = e10 % e10;
e21Len = e21 % e21;
e02Len = e02 % e02;


edge = face;
if ((e21Len < e10Len) && (e21Len < e02Len)){
edge = face->m_next;
}
if ((e02Len < e10Len) && (e02Len < e21Len)){
edge = face->m_prev;
}
ptr = CollapseEdge(edge);
if (!ptr) {
ptr = CollapseEdge(edge->m_twin);
if (!ptr) {
ptr = CollapseEdge(edge->m_next);
if (!ptr) {
ptr = CollapseEdge(edge->m_next->m_twin);
if (!ptr) {
ptr = CollapseEdge(edge->m_prev->m_next);
if (!ptr) {
ptr = CollapseEdge(edge->m_prev->m_twin);
if (!ptr) {
DeleteFace (edge);
}
}
}
}
}
}

#ifdef __ENABLE_SANITY_CHECK 
HACD_ASSERT (SanityCheck ());
#endif

}
}
} while (someChanges);

desc.Update(*this);
}
*/

/*
void dgPolyhedra::GetCoplanarFaces (dgList<dgEdge*>& faceList, dgEdge *startFace, const hacd::HaF32* const pool, hacd::HaI32 strideInBytes, hacd::HaF32 normalDeviation) const
{
	hacd::HaI32 mark;
	hacd::HaI32 index;
//	hacd::HaF64 dot;

	if (!GetCount()) {
		return;
	}

	dgStack<dgEdge*> stackPool (GetCount() / 2); 
	dgEdge **const stack = &stackPool[0];

	if (startFace->m_incidentFace < 0) {
		startFace = startFace->m_twin;
	}

	HACD_ASSERT (startFace->m_incidentFace > 0);
	mark = IncLRU();

	dgBigVector normal (FaceNormal (startFace, pool, strideInBytes));
	hacd::HaF64 dot = normal % normal;
	if (dot < hacd::HaF64 (1.0e-12f)) {
		dgEdge* ptr = startFace;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != startFace);

		HACD_ASSERT (0);
		faceList.Append (startFace);
		return;
	}
	normal = normal.Scale (hacd::HaF64 (1.0) / sqrt (dot));


	stack[0] = startFace;
	index = 1;
	while (index) {
		index --;
		dgEdge* const edge = stack[index];

		if (edge->m_mark == mark) {
			HACD_ASSERT (0u);
			continue;
		}

		dgBigVector normal1 (FaceNormal (edge, pool, strideInBytes));
		dot = normal1 % normal1;
		if (dot > hacd::HaF64 (1.0e-12f)) {
			normal1 = normal1.Scale (hacd::HaF64 (1.0) / sqrt (dot));
		}

		dot = normal1 % normal;
		if (dot >= normalDeviation) {
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;

				if ((ptr->m_twin->m_incidentFace > 0) && (ptr->m_twin->m_mark != mark)) {
					stack[index] = ptr->m_twin;
					index ++;
					HACD_ASSERT (index < GetCount() / 2);
				}
				ptr = ptr->m_next;
			} while (ptr != edge);

			HACD_ASSERT (0);
			faceList.Append (edge);
		}
	}
}



void dgPolyhedra::DeleteOverlapingFaces (
	const hacd::HaF32* const pool, 
	hacd::HaI32 strideInBytes, 
	hacd::HaF32 distTol__)
{
	hacd::HaI32 i;
	hacd::HaI32 mark;
	hacd::HaI32 perimeterCount;
	dgEdge *edge;
	dgPolyhedra *perimeters;

	if (!GetCount()) {
		return;
	}

	dgStack<hacd::HaI32>faceIndexPool (4096);
	dgStack<dgEdge*> stackPool (GetCount() / 2 + 100);
	dgStack<dgPolyhedra> flatPerimetersPool (GetCount() / 3 + 100);

	perimeterCount = 0;
	perimeters = &flatPerimetersPool[0];

	mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; iter++) {
		edge = &(*iter);
		if (edge->m_incidentFace < 0) {
			continue;
		}

		if (edge->m_mark >= mark) {
			continue;
		}

		dgPolyhedra dommy;
		perimeters[perimeterCount] = dommy;
		InternalPolyhedra::GetAdjacentCoplanarFacesPerimeter (perimeters[perimeterCount], *this, edge, pool, strideInBytes, &stackPool[0], &faceIndexPool[0]);
		perimeterCount ++;
	}

	// write code here


	for (i = 0 ; i < perimeterCount; i ++) {
		perimeters[i].DeleteAllFace();
	}
}


void dgPolyhedra::InvertWinding ()
{
	dgStack<hacd::HaI32> vertexData(1024 * 4);
	dgStack<hacd::HaI64> userData(1024 * 4);

	dgPolyhedra tmp (*this);

	RemoveAll();

	BeginFace();
	hacd::HaI32 mark = tmp.IncLRU();
	Iterator iter (tmp);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);

		if (edge->m_incidentFace < 0) {
			continue;
		}
		if (edge->m_mark == mark) {
			continue;
		}

		hacd::HaI32 count = 0;
		dgEdge* ptr = edge;
		do {
			userData[count] = hacd::HaI32 (ptr->m_userData);
			vertexData[count] = ptr->m_incidentVertex;
			count ++;
			HACD_ASSERT (count < 1024 * 4);

			ptr->m_mark = mark;
			ptr = ptr->m_prev;
		} while (ptr != edge);

		AddFace(count, &vertexData[0], &userData[0]);
	}
	EndFace();

	HACD_ASSERT (SanityCheck());

}
*/

/*

void dgPolyhedra::Quadrangulate (const hacd::HaF32* const vertex, hacd::HaI32 strideInBytes)
{
hacd::HaI32 mark;
hacd::HaI32 stride;
hacd::HaU32 cost;
hacd::HaU32 maxCost;
dgTree<dgEdge*, hacd::HaI64> essensialEdge;

Iterator iter (*this);
for (iter.Begin(); iter; iter ++) {
dgEdge *edge;
edge = &(*iter);
dgPairKey code (edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
essensialEdge.Insert (edge, code.GetVal());
}

Triangulate (vertex, strideInBytes);

stride = strideInBytes / sizeof (hacd::HaF32);
dgHeap<dgEdge*, hacd::HaU32> heapCost (GetCount(), 0xffffffff);
maxCost = 1<<30;
mark = IncLRU();
for (iter.Begin(); iter; iter ++) {
dgEdge *edge;
dgEdge *twin;

hacd::HaU32 edgeCost;
hacd::HaU32 twinCost;
hacd::HaU32 shapeCost;
hacd::HaU32 normalCost;
hacd::HaF32 normalDot;
hacd::HaF32 edgeAngle0;
hacd::HaF32 edgeAngle1;
hacd::HaF32 twinAngle0;
hacd::HaF32 twinAngle1;
hacd::HaF32 shapeFactor;
hacd::HaF32 medianAngle;

dgTree<dgEdge*, hacd::HaI64>::dgTreeNode *essencial;

edge = &(*iter);
twin = edge->m_twin;

if (edge->m_mark == mark) {
continue;
}

if ((edge->m_incidentFace < 0) || (twin->m_incidentFace < 0)) {
continue;
}

edge->m_mark = mark;
twin->m_mark = mark;

dgVector edgeNormal (FaceNormal (edge, vertex, strideInBytes));
if ((edgeNormal % edgeNormal) < 1.0e-8) {
continue;
}
dgVector twinNormal (FaceNormal (twin, vertex, strideInBytes));

if ((twinNormal % twinNormal) < 1.0e-8) {
continue;
}

edgeNormal = edgeNormal.Scale (dgRsqrt (edgeNormal % edgeNormal));
twinNormal = twinNormal.Scale (dgRsqrt (twinNormal % twinNormal));

normalDot = edgeNormal % twinNormal;
normalCost = ClampValue (2048 - (hacd::HaI32)(2048.0f * normalDot), 0, 2048);
if (normalCost	> 600) {
normalCost = 4000000;
}

dgVector p0 (&vertex[edge->m_incidentVertex * stride]);
dgVector p1 (&vertex[edge->m_twin->m_incidentVertex * stride]);
dgVector p2 (&vertex[edge->m_prev->m_incidentVertex * stride]);
dgVector p3 (&vertex[twin->m_prev->m_incidentVertex * stride]);

dgVector e10 (p1 - p0);
dgVector e20 (p2 - p0);
dgVector e30 (p3 - p0);

e10 = e10.Scale (dgRsqrt ((e10 % e10) + 1.e-10f));
e20 = e20.Scale (dgRsqrt ((e20 % e20) + 1.e-10f));
e30 = e30.Scale (dgRsqrt ((e30 % e30) + 1.e-10f));

edgeAngle0 = dgRAD2DEG * dgAtan2 (edgeNormal % (e10 * e20), e20 % e10);
edgeAngle1 = dgRAD2DEG * dgAtan2 (twinNormal % (e30 * e10), e10 % e30);

if ((edgeAngle0 + edgeAngle1) < 160.0f) {
HACD_ASSERT ((edgeAngle0 + edgeAngle1) > 0.0f);
medianAngle = 4.0f * edgeAngle0 * edgeAngle1 / (edgeAngle0 + edgeAngle1);

HACD_ASSERT (medianAngle > 0.0f);
HACD_ASSERT (medianAngle < 360.0f);
edgeCost = abs (ClampValue (90 - (hacd::HaI32)medianAngle, -90, 90));
} else {
edgeCost	= 4000000;
}

dgVector t10 (p0 - p1);
dgVector t20 (p3 - p1);
dgVector t30 (p2 - p1);

t10 = t10.Scale (dgRsqrt ((t10 % t10) + 1.e-10f));
t20 = t20.Scale (dgRsqrt ((t20 % t20) + 1.e-10f));
t30 = t30.Scale (dgRsqrt ((t30 % t30) + 1.e-10f));

twinAngle0 = dgRAD2DEG * dgAtan2 (twinNormal % (t10 * t20), t20 % t10);
twinAngle1 = dgRAD2DEG * dgAtan2 (edgeNormal % (t30 * t10), t10 % t30);

if ((twinAngle0 + twinAngle1) < 160.0f) {
HACD_ASSERT ((twinAngle0 + twinAngle1) > 0.0f);
medianAngle = 4.0f * twinAngle0 * twinAngle1 / (twinAngle0 + twinAngle1);

HACD_ASSERT (medianAngle > 0.0f);
HACD_ASSERT (medianAngle < 360.0f);
twinCost = abs (ClampValue (90 - (hacd::HaI32)medianAngle, -90, 90));
} else {
twinCost	= 4000000;
}


hacd::HaF32 a0;
hacd::HaF32 a1;
hacd::HaF32 a2;
hacd::HaF32 a3;
hacd::HaF32 angleSum;
hacd::HaF32 angleSum2;

a0 = edgeAngle0 + edgeAngle1;
a1 = twinAngle0 + twinAngle1;

dgVector oe10 (p0 - p2);
dgVector oe20 (p1 - p2);

oe10 = oe10.Scale (dgRsqrt ((oe10 % oe10) + 1.e-10f));
oe20 = oe20.Scale (dgRsqrt ((oe20 % oe20) + 1.e-10f));
a2 = dgRAD2DEG * dgAtan2 (edgeNormal % (oe10 * oe20), oe20 % oe10);

dgVector ot10 (p1 - p3);
dgVector ot20 (p0 - p3);

ot10 = ot10.Scale (dgRsqrt ((ot10 % ot10) + 1.e-10f));
ot20 = ot20.Scale (dgRsqrt ((ot20 % ot20) + 1.e-10f));
a3 = dgRAD2DEG * dgAtan2 (twinNormal % (ot10 * ot20), ot20 % ot10);

angleSum	= (a0 + a1 + a2 + a3) * 0.25f;
angleSum2 = (a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3) * 0.25f;
shapeFactor = powf (dgSqrt (angleSum2 - angleSum * angleSum), 1.25f);

shapeCost = ClampValue ((hacd::HaI32)(shapeFactor * 4.0f), 0, 4096);

cost = normalCost + edgeCost + twinCost + shapeCost;
dgPairKey code (edge->m_incidentVertex, edge->m_twin->m_incidentVertex);
essencial = essensialEdge.Find(code.GetVal());
if (essencial) {
cost += 1000;
}
heapCost.Push (edge, maxCost - cost);
}


mark = IncLRU();
while (heapCost.GetCount ()) {
hacd::HaU32 cost;
dgEdge *edge;

edge = heapCost[0];
cost = maxCost - heapCost.Value ();
if (cost	> 100000) {
break;
}

heapCost.Pop();

if (edge->m_mark != mark) {
edge->m_mark = mark;
edge->m_twin->m_mark = mark;

edge->m_next->m_mark = mark;
edge->m_next->m_twin->m_mark = mark;

edge->m_prev->m_mark = mark;
edge->m_prev->m_twin->m_mark = mark;

edge->m_twin->m_next->m_mark = mark;
edge->m_twin->m_next->m_twin->m_mark = mark;

edge->m_twin->m_prev->m_mark = mark;
edge->m_twin->m_prev->m_twin->m_mark = mark;

DeleteEdge (edge);
}

}

//#ifdef _DEBUG
//mark = IncLRU();
//for (iter.Begin(); iter; iter ++) {
//	dgEdge *edge;
//	edge = &(*iter);
//
//	if (edge->m_incidentFace > 0) {
//		if (edge->m_mark != mark) {
//			dgEdge *ptr;
//			ptr = edge;
//			do {
//				ptr->m_mark = mark;
//				dgTrace (("%d ", ptr->m_incidentVertex));
//				ptr = ptr->m_next;
//			} while(ptr != edge);
//			dgTrace (("\n"));
//
//		}
//	}
//}
//HACD_ASSERT (0);
//#endif

}

void dgPolyhedra::OptimalTriangulation (const hacd::HaF32* const vertex, hacd::HaI32 strideInBytes)
{
hacd::HaI32 color;
dgEdge *edge;
dgList<dgEdge*> edgeStart;

Quadrangulate (vertex, strideInBytes);

color = IncLRU();
dgTree<dgEdge*, hacd::HaI64> faceList;
Iterator iter (*this);
for (iter.Begin(); iter; iter ++) {
hacd::HaI32 min;
hacd::HaI32 count;
dgEdge *ptr;
dgEdge *start;

edge = &(*iter);
if (edge->m_mark == color) {
continue;
}
if (edge->m_incidentFace < 0) {
edge->m_mark = color + 1;
continue;
}

count	= 0;
min = 0x7fffffff;
start	= edge;
ptr = edge;
do {
count	++;
ptr->m_mark = color;
if (ptr->m_incidentVertex < min) {
start	= ptr;
min = ptr->m_incidentVertex;
}
ptr = ptr->m_next;
} while (ptr != edge);
if (count == 4) {
dgPairKey	key (start->m_incidentVertex, start->m_twin->m_incidentVertex);
faceList.Insert (start, key.GetVal());
}
}

color = IncLRU();
for (edge = InternalPolyhedra::FindQuadStart(faceList, color); edge; edge = InternalPolyhedra::FindQuadStart(faceList, color)) {
InternalPolyhedra::TriangleQuadStrips (*this, faceList, edge, color);
}

#ifdef _DEBUG
for (iter.Begin(); iter; iter ++) {
edge = &(*iter);
if (edge->m_incidentFace > 0)
HACD_ASSERT (edge->m_next->m_next->m_next == edge);
}
#endif
}


hacd::HaI32 dgPolyhedra::TriangleStrips (
hacd::HaU32 outputBuffer[], 
hacd::HaI32 maxBufferSize, 
hacd::HaI32 vertexCacheSize) const
{
hacd::HaI32 setMark;
hacd::HaI32 indexCount;
hacd::HaI32 stripsIndex;
hacd::HaI32 faceColorMark;
hacd::HaI32 debugFaceCount; 
hacd::HaI32 debugIndexCount;

dgEdge *edge;
InternalPolyhedra::VertexCache vertexCache(vertexCacheSize);

dgPolyhedra tmp(*this);

tmp.IncLRU();
faceColorMark = tmp.IncLRU();
tmp.IncLRU();
setMark = tmp.IncLRU();

debugFaceCount = 0;
debugIndexCount = 0;

indexCount = 0;

for (edge = InternalPolyhedra::FindStripStart(tmp, faceColorMark, vertexCache); edge; edge = InternalPolyhedra::FindStripStart(tmp, faceColorMark, vertexCache)) {
stripsIndex = InternalPolyhedra::TriangleStrips (edge, setMark, &outputBuffer[indexCount + 1], vertexCache);

debugFaceCount	+= stripsIndex	- 2;
debugIndexCount += stripsIndex;

if (stripsIndex > 0)	{
outputBuffer[indexCount] = stripsIndex;
indexCount += stripsIndex + 1;
if (indexCount > maxBufferSize) {
break;
}
}
}

dgTrace (("index per triangles %f\n", hacd::HaF32 (debugIndexCount) / hacd::HaF32 (debugFaceCount)));

return indexCount;
}
*/

/*
hacd::HaI32 dgPolyhedra::TriangleList (
hacd::HaU32 outputBuffer[], 
hacd::HaI32 maxSize,
hacd::HaI32 vertexCacheSize) const
{
hacd::HaI32 mark;
hacd::HaI32 cost;
hacd::HaI32 count;
hacd::HaI32 vertex;
hacd::HaI32 cacheHit;
hacd::HaI32 tmpVertex;
hacd::HaI32 cacheMiss;
hacd::HaI32 twinVertex;
hacd::HaI32 lowestPrize;
hacd::HaI64 key;
dgEdge *ptr;
dgEdge *edge;
dgList<hacd::HaI32> vertexCache;
dgTree<dgEdge*, hacd::HaI64> edgeList;
dgTree<dgEdge*, hacd::HaI64>::dgTreeNode *node;
dgTree<dgEdge*, hacd::HaI64>::dgTreeNode *bestEdge;


cacheHit = 0;
cacheMiss = 0;

Iterator iter (*this);
for (iter.Begin(); iter; iter ++) {
edge = &(*iter);
if (edge->m_incidentFace > 0) {
edgeList.Insert (edge, iter.GetNode()->GetKey());
}
}

count = 0;
mark = IncLRU();
while (edgeList) {

node = edgeList.Minimum();
edge = node->GetInfo();
ptr = edge; 
do {
ptr->m_mark = mark;

vertex = ptr->m_incidentVertex;
if (count < maxSize) {
outputBuffer[count] = vertex;
count ++;
}
cacheMiss ++;
vertexCache.Append (vertex);
if (vertexCache.GetCount() > vertexCacheSize) {
vertexCache.Remove (vertexCache.GetFirst());
}
edgeList.Remove(GetNodeFromInfo(*ptr)->GetKey());

ptr = ptr->m_next;
} while (ptr != edge);

dgList<hacd::HaI32>::Iterator vertexIter(vertexCache);
for (vertexIter.Begin(); vertexIter; ) {

vertex = *vertexIter;
vertexIter ++;

key = vertex;
key <<= 32;

bestEdge	= NULL;
lowestPrize = 100000;

node = edgeList.FindGreaterEqual (key);
dgTree<dgEdge *, hacd::HaI64>::Iterator iter(edgeList);
for (iter.Set(node); iter; iter ++) {
node = iter.GetNode();
key = node->GetKey();
key >>= 32;
if (key > vertex) {
break;
}

ptr = node->GetInfo();

HACD_ASSERT (ptr->m_mark != mark);
HACD_ASSERT (ptr->m_twin->m_incidentVertex == vertex);


twinVertex = ptr->m_twin->m_incidentVertex;
dgList<hacd::HaI32>::Iterator vertexIter(vertexCache);
cost = 0;
for (vertexIter.Begin(); vertexIter; vertexIter ++) {
tmpVertex = *vertexIter;
if (twinVertex == tmpVertex) {
break;
};
cost ++;
}
if (cost < lowestPrize) {
lowestPrize	= cost;
bestEdge	= node;
}
}

if (bestEdge) {
edge = bestEdge->GetInfo();

ptr = edge;
do {
ptr->m_mark = mark;
vertex = ptr->m_incidentVertex;
if (count < maxSize) {
outputBuffer[count] = vertex;
count ++;
}

edgeList.Remove(GetNodeFromInfo(*ptr)->GetKey());

dgList<hacd::HaI32>::Iterator vertexIter(vertexCache);
for (vertexIter.Begin(); vertexIter; vertexIter++) {
tmpVertex = *vertexIter;
if (vertex == tmpVertex) {
cacheHit ++;
break;
}
}

if (!vertexIter) {
cacheMiss ++;
vertexCache.Append (vertex);
if (vertexCache.GetCount() > vertexCacheSize) {
vertexCache.Remove (vertexCache.GetFirst());
}
}

ptr = ptr->m_next;
} while (ptr != edge);

vertexIter.Begin();
}
}		
}	

//	dgTrace ("cacheHit = %d, cacheMiss = %d, total = %d\n", cacheHit, cacheMiss, cacheMiss + cacheHit);

return count;
}

*/


hacd::HaI32 dgPolyhedra::TriangleList (hacd::HaU32 outputBuffer[], hacd::HaI32 maxSize__, hacd::HaI32 vertexCacheSize__) const
{
	hacd::HaI32 mark;
	hacd::HaI32 count;
	hacd::HaI32 cacheMiss;
	hacd::HaI32 score;
	hacd::HaI32 bestScore;
	dgEdge *ptr;
	dgEdge *edge;
	dgEdge *face;
	dgTree<dgEdge*, hacd::HaI32> vertexIndex;
	InternalPolyhedra::VertexCache vertexCache (16);


	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		edge = &(*iter);
		vertexIndex.Insert (edge, edge->m_incidentVertex);
	}
	count = 0;
	cacheMiss = 0;;

	mark = IncLRU();
	while (vertexIndex.GetCount()) {
		edge = vertexCache.GetEdge(mark);
		if (!edge) {
			dgTree<dgEdge*, hacd::HaI32>::dgTreeNode *node;
			dgTree<dgEdge*, hacd::HaI32>::Iterator iter (vertexIndex);
			for (iter.Begin (); iter;  ) {
				node = iter.GetNode();
				iter ++;
				ptr = node->GetInfo();;
				edge = ptr;

				do {
					if (edge->m_incidentFace > 0) {
						if (edge->m_mark != mark) {
							goto newEdge;
						}
					}
					edge = edge->m_twin->m_next;
				} while (edge != ptr);
				vertexIndex.Remove (node);
			}
			edge = NULL;
		}
newEdge:

		face = NULL;
		bestScore = -1;
		if (edge) {
			ptr = edge;
			do {
				if (ptr->m_incidentFace > 0) {
					if (ptr->m_mark != mark) {
						score =  vertexCache.IsInCache (ptr->m_next) + vertexCache.IsInCache(ptr->m_prev); 
						if (score > bestScore) {
							bestScore = score;
							face = ptr;
						}
					}
				}

				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);

			HACD_ASSERT (face);
			ptr = face;
			do {
				outputBuffer[count] = hacd::HaU32 (ptr->m_incidentVertex);
				count ++;
				cacheMiss += vertexCache.AddEdge (ptr);
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != face);
		}
	}

	// add all legacy faces
	for (iter.Begin(); iter; iter ++) {
		edge = &(*iter);
		if (edge->m_mark != mark) {
			if (edge->m_incidentFace > 0) {
				ptr = edge;
				do {
					outputBuffer[count] = hacd::HaU32 (ptr->m_incidentVertex);
					count ++;
					cacheMiss ++;
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}
	}

	dgTrace (("fifo efficiency %f\n", hacd::HaF32 (cacheMiss) * 3.0f / hacd::HaF32 (count)));

	return count; 
}



void dgPolyhedra::SwapInfo (dgPolyhedra& source)
{
	dgTree<dgEdge, dgEdgeKey>::SwapInfo((dgTree<dgEdge, dgEdgeKey>&)source);

	Swap (m_baseMark, source.m_baseMark);
	Swap (m_edgeMark, source.m_edgeMark);
	Swap (m_faceSecuence, source.m_faceSecuence);

}

void dgPolyhedra::GetOpenFaces (dgList<dgEdge*>& faceList) const
{
	hacd::HaI32 mark = IncLRU();
	//	dgList<dgEdge*> openfaces;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace < 0)) {
			dgEdge* ptr = edge;
			faceList.Append(edge);
			do {
				ptr->m_mark = mark;

				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
}


/*
bool dgPolyhedra::TriangulateFace (dgEdge* face, const hacd::HaF32* const vertex, hacd::HaI32 strideInBytes, dgVector& normal)
{
	hacd::HaI32 memPool [2048]; 
	dgDownHeap<dgEdge*, hacd::HaF32> heap(&memPool[0], sizeof (memPool));


	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF32));
	return InternalPolyhedra::TriangulateFace (*this, face, vertex, stride, heap, &normal) ? false : true;
}
*/





#endif



dgPolyhedra::dgPolyhedra (void)
	:dgTree <dgEdge, hacd::HaI64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
}

dgPolyhedra::dgPolyhedra (const dgPolyhedra &polyhedra)
	:dgTree <dgEdge, hacd::HaI64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
	dgStack<hacd::HaI32> indexPool (1024 * 16);
	dgStack<hacd::HaU64> userPool (1024 * 16);
	hacd::HaI32* const index = &indexPool[0];
	hacd::HaU64* const user = &userPool[0];

	BeginFace ();
	Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) {
			continue;
		}

		if (!FindEdge(edge->m_incidentVertex, edge->m_twin->m_incidentVertex))	{
			hacd::HaI32 indexCount = 0;
			dgEdge* ptr = edge;
			do {
				user[indexCount] = ptr->m_userData;
				index[indexCount] = ptr->m_incidentVertex;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dgEdge* const face = AddFace (indexCount, index, (hacd::HaI64*) user);
			ptr = face;
			do {
				ptr->m_incidentFace = edge->m_incidentFace;
				ptr = ptr->m_next;
			} while (ptr != face);
		}
	}
	EndFace();

	m_faceSecuence = polyhedra.m_faceSecuence;

#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck());
#endif
}

dgPolyhedra::~dgPolyhedra ()
{
}


hacd::HaI32 dgPolyhedra::GetFaceCount() const
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

		count ++;
		dgEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != edge);
	}
	return count;
}



dgEdge* dgPolyhedra::AddFace ( hacd::HaI32 count, const hacd::HaI32* const index, const hacd::HaI64* const userdata)
{
	class IntersectionFilter
	{
		public:
		IntersectionFilter ()
		{
			m_count = 0;
		}

		bool Insert (hacd::HaI32 dummy, hacd::HaI64 value)
		{
			hacd::HaI32 i;				
			for (i = 0 ; i < m_count; i ++) {
				if (m_array[i] == value) {
					return false;
				}
			}
			m_array[i] = value;
			m_count ++;
			return true;
		}

		hacd::HaI32 m_count;
		hacd::HaI64 m_array[2048];
	};

	IntersectionFilter selfIntersectingFaceFilter;

	hacd::HaI32 dummyValues = 0;
	hacd::HaI32 i0 = index[count-1];
	for (hacd::HaI32 i = 0; i < count; i ++) {
		hacd::HaI32 i1 = index[i];
		dgPairKey code0 (i0, i1);

		if (!selfIntersectingFaceFilter.Insert (dummyValues, code0.GetVal())) {
			return NULL;
		}

		dgPairKey code1 (i1, i0);
		if (!selfIntersectingFaceFilter.Insert (dummyValues, code1.GetVal())) {
			return NULL;
		}


		if (i0 == i1) {
			return NULL;
		}
		if (FindEdge (i0, i1)) {
			return NULL;
		}
		i0 = i1;
	}

	m_faceSecuence ++;

	i0 = index[count-1];
	hacd::HaI32 i1 = index[0];
	hacd::HaU64 udata0 = 0;
	hacd::HaU64 udata1 = 0;
	if (userdata) {
		udata0 = hacd::HaU64 (userdata[count-1]);
		udata1 = hacd::HaU64 (userdata[0]);
	} 

	bool state;
	dgPairKey code (i0, i1);
	dgEdge tmpEdge (i0, m_faceSecuence, udata0);
	dgTreeNode* node = Insert (tmpEdge, code.GetVal(), state); 
	HACD_ASSERT (!state);
	dgEdge* edge0 = &node->GetInfo();
	dgEdge* const first = edge0;

	for (hacd::HaI32 i = 1; i < count; i ++) {
		i0 = i1;
		i1 = index[i];
		udata0 = udata1;
		udata1 = hacd::HaU64 (userdata ? userdata[i] : 0);

		dgPairKey code (i0, i1);
		dgEdge tmpEdge (i0, m_faceSecuence, udata0);
		node = Insert (tmpEdge, code.GetVal(), state); 
		HACD_ASSERT (!state);

		dgEdge* const edge1 = &node->GetInfo();
		edge0->m_next = edge1;
		edge1->m_prev = edge0;
		edge0 = edge1;
	}

	first->m_prev = edge0;
	edge0->m_next = first;

	return first->m_next;
}


void dgPolyhedra::EndFace ()
{
	dgPolyhedra::Iterator iter (*this);

	// Connect all twin edge
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (!edge->m_twin) {
			edge->m_twin = FindEdge (edge->m_next->m_incidentVertex, edge->m_incidentVertex);
			if (edge->m_twin) {
				edge->m_twin->m_twin = edge; 
			}
		}
	}

#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck());
#endif
	dgStack<dgEdge*> edgeArrayPool(GetCount() * 2 + 256);

	hacd::HaI32 edgeCount = 0;
	dgEdge** const edgeArray = &edgeArrayPool[0];
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (!edge->m_twin) {
			bool state;
			dgPolyhedra::dgPairKey code (edge->m_next->m_incidentVertex, edge->m_incidentVertex);
			dgEdge tmpEdge (edge->m_next->m_incidentVertex, -1);
			tmpEdge.m_incidentFace = -1; 
			dgPolyhedra::dgTreeNode* const node = Insert (tmpEdge, code.GetVal(), state); 
			HACD_ASSERT (!state);
			edge->m_twin = &node->GetInfo();
			edge->m_twin->m_twin = edge; 
			edgeArray[edgeCount] = edge->m_twin;
			edgeCount ++;
		}
	}

	for (hacd::HaI32 i = 0; i < edgeCount; i ++) {
		dgEdge* const edge = edgeArray[i];
		HACD_ASSERT (!edge->m_prev);
		dgEdge *ptr = edge->m_twin;
		for (; ptr->m_next; ptr = ptr->m_next->m_twin){}
		ptr->m_next = edge;
		edge->m_prev = ptr;
	}

#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck ());
#endif
}


void dgPolyhedra::DeleteFace(dgEdge* const face)
{
	dgEdge* edgeList[1024 * 16];

	if (face->m_incidentFace > 0) {
		hacd::HaI32 count = 0;
		dgEdge* ptr = face;
		do {
			ptr->m_incidentFace = -1;
			hacd::HaI32 i = 0;
			for (; i < count; i ++) {
				if ((edgeList[i] == ptr) || (edgeList[i]->m_twin == ptr)) {
					break;
				}
			}
			if (i == count) {
				edgeList[count] = ptr;
				count ++;
			}
			ptr = ptr->m_next;
		} while (ptr != face);


		for (hacd::HaI32 i = 0; i < count; i ++) {
			dgEdge* const ptr = edgeList[i];
			if (ptr->m_twin->m_incidentFace < 0) {
				DeleteEdge (ptr);
			}
		}
	}
}



dgBigVector dgPolyhedra::FaceNormal (dgEdge* const face, const hacd::HaF64* const pool, hacd::HaI32 strideInBytes) const
{
	hacd::HaI32 stride = strideInBytes / sizeof (hacd::HaF64);
	dgEdge* edge = face;
	dgBigVector p0 (&pool[edge->m_incidentVertex * stride]);
	edge = edge->m_next;
	dgBigVector p1 (&pool[edge->m_incidentVertex * stride]);
	dgBigVector e1 (p1 - p0);

	dgBigVector normal (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
	for (edge = edge->m_next; edge != face; edge = edge->m_next) {
		dgBigVector p2 (&pool[edge->m_incidentVertex * stride]);
		dgBigVector e2 (p2 - p0);
		normal += e1 * e2;
		e1 = e2;
	} 
	return normal;
}


dgEdge* dgPolyhedra::AddHalfEdge (hacd::HaI32 v0, hacd::HaI32 v1)
{
	if (v0 != v1) {
		dgPairKey pairKey (v0, v1);
		dgEdge tmpEdge (v0, -1);

		dgTreeNode* node = Insert (tmpEdge, pairKey.GetVal()); 
		return node ? &node->GetInfo() : NULL;
	} else {
		return NULL;
	}
}


void dgPolyhedra::DeleteEdge (dgEdge* const edge)
{
	dgEdge *const twin = edge->m_twin;

	edge->m_prev->m_next = twin->m_next;
	twin->m_next->m_prev = edge->m_prev;
	edge->m_next->m_prev = twin->m_prev;
	twin->m_prev->m_next = edge->m_next;

	dgTreeNode *const nodeA = GetNodeFromInfo (*edge);
	dgTreeNode *const nodeB = GetNodeFromInfo (*twin);

	HACD_ASSERT (&nodeA->GetInfo() == edge);
	HACD_ASSERT (&nodeB->GetInfo() == twin);

	Remove (nodeA);
	Remove (nodeB);
}


dgEdge* dgPolyhedra::SpliteEdge (hacd::HaI32 newIndex,	dgEdge* const edge)
{
	dgEdge* const edge00 = edge->m_prev;
	dgEdge* const edge01 = edge->m_next;
	dgEdge* const twin00 = edge->m_twin->m_next;
	dgEdge* const twin01 = edge->m_twin->m_prev;

	hacd::HaI32 i0 = edge->m_incidentVertex;
	hacd::HaI32 i1 = edge->m_twin->m_incidentVertex;

	hacd::HaI32 f0 = edge->m_incidentFace;
	hacd::HaI32 f1 = edge->m_twin->m_incidentFace;

	DeleteEdge (edge);

	dgEdge* const edge0 = AddHalfEdge (i0, newIndex);
	dgEdge* const edge1 = AddHalfEdge (newIndex, i1);

	dgEdge* const twin0 = AddHalfEdge (newIndex, i0);
	dgEdge* const twin1 = AddHalfEdge (i1, newIndex);
	HACD_ASSERT (edge0);
	HACD_ASSERT (edge1);
	HACD_ASSERT (twin0);
	HACD_ASSERT (twin1);

	edge0->m_twin = twin0;
	twin0->m_twin = edge0;

	edge1->m_twin = twin1;
	twin1->m_twin = edge1;

	edge0->m_next = edge1;
	edge1->m_prev = edge0;

	twin1->m_next = twin0;
	twin0->m_prev = twin1;

	edge0->m_prev = edge00;
	edge00 ->m_next = edge0;

	edge1->m_next = edge01;
	edge01->m_prev = edge1;

	twin0->m_next = twin00;
	twin00->m_prev = twin0;

	twin1->m_prev = twin01;
	twin01->m_next = twin1;

	edge0->m_incidentFace = f0;
	edge1->m_incidentFace = f0;

	twin0->m_incidentFace = f1;
	twin1->m_incidentFace = f1;

#ifdef __ENABLE_SANITY_CHECK 
	//	HACD_ASSERT (SanityCheck ());
#endif

	return edge0;
}



bool dgPolyhedra::FlipEdge (dgEdge* const edge)
{
	//	dgTreeNode *node;
	if (edge->m_next->m_next->m_next != edge) {
		return false;
	}

	if (edge->m_twin->m_next->m_next->m_next != edge->m_twin) {
		return false;
	}

	if (FindEdge(edge->m_prev->m_incidentVertex, edge->m_twin->m_prev->m_incidentVertex)) {
		return false;
	}

	dgEdge *const prevEdge = edge->m_prev;
	dgEdge *const prevTwin = edge->m_twin->m_prev;

	dgPairKey edgeKey (prevTwin->m_incidentVertex, prevEdge->m_incidentVertex);
	dgPairKey twinKey (prevEdge->m_incidentVertex, prevTwin->m_incidentVertex);

	ReplaceKey (GetNodeFromInfo (*edge), edgeKey.GetVal());
	//	HACD_ASSERT (node);

	ReplaceKey (GetNodeFromInfo (*edge->m_twin), twinKey.GetVal());
	//	HACD_ASSERT (node);

	edge->m_incidentVertex = prevTwin->m_incidentVertex;
	edge->m_twin->m_incidentVertex = prevEdge->m_incidentVertex;

	edge->m_userData = prevTwin->m_userData;
	edge->m_twin->m_userData = prevEdge->m_userData;

	prevEdge->m_next = edge->m_twin->m_next;
	prevTwin->m_prev->m_prev = edge->m_prev;

	prevTwin->m_next = edge->m_next;
	prevEdge->m_prev->m_prev = edge->m_twin->m_prev;

	edge->m_prev = prevTwin->m_prev;
	edge->m_next = prevEdge;

	edge->m_twin->m_prev = prevEdge->m_prev;
	edge->m_twin->m_next = prevTwin;

	prevTwin->m_prev->m_next = edge;
	prevTwin->m_prev = edge->m_twin;

	prevEdge->m_prev->m_next = edge->m_twin;
	prevEdge->m_prev = edge;

	edge->m_next->m_incidentFace = edge->m_incidentFace;
	edge->m_prev->m_incidentFace = edge->m_incidentFace;

	edge->m_twin->m_next->m_incidentFace = edge->m_twin->m_incidentFace;
	edge->m_twin->m_prev->m_incidentFace = edge->m_twin->m_incidentFace;


#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck ());
#endif

	return true;
}



bool dgPolyhedra::GetConectedSurface (dgPolyhedra &polyhedra) const
{
	if (!GetCount()) {
		return false;
	}

	dgEdge* edge = NULL;
	Iterator iter(*this);
	for (iter.Begin (); iter; iter ++) {
		edge = &(*iter);
		if ((edge->m_mark < m_baseMark) && (edge->m_incidentFace > 0)) {
			break;
		}
	}

	if (!iter) {
		return false;
	}

	hacd::HaI32 faceIndex[4096];
	hacd::HaI64 faceDataIndex[4096];
	dgStack<dgEdge*> stackPool (GetCount()); 
	dgEdge** const stack = &stackPool[0];

	hacd::HaI32 mark = IncLRU();

	polyhedra.BeginFace ();
	stack[0] = edge;
	hacd::HaI32 index = 1;
	while (index) {
		index --;
		dgEdge* const edge = stack[index];

		if (edge->m_mark == mark) {
			continue;
		}

		hacd::HaI32 count = 0;
		dgEdge* ptr = edge;
		do {
			ptr->m_mark = mark;
			faceIndex[count] = ptr->m_incidentVertex;
			faceDataIndex[count] = hacd::HaI64 (ptr->m_userData);
			count ++;
			HACD_ASSERT (count <  hacd::HaI32 ((sizeof (faceIndex)/sizeof(faceIndex[0]))));

			if ((ptr->m_twin->m_incidentFace > 0) && (ptr->m_twin->m_mark != mark)) {
				stack[index] = ptr->m_twin;
				index ++;
				HACD_ASSERT (index < GetCount());
			}

			ptr = ptr->m_next;
		} while (ptr != edge);

		polyhedra.AddFace (count, &faceIndex[0], &faceDataIndex[0]);
	}

	polyhedra.EndFace ();

	return true;
}


void dgPolyhedra::ChangeEdgeIncidentVertex (dgEdge* const edge, hacd::HaI32 newIndex)
{
	dgEdge* ptr = edge;
	do {
		dgTreeNode* node = GetNodeFromInfo(*ptr);
		dgPairKey Key0 (newIndex, ptr->m_twin->m_incidentVertex);
		ReplaceKey (node, Key0.GetVal());

		node = GetNodeFromInfo(*ptr->m_twin);
		dgPairKey Key1 (ptr->m_twin->m_incidentVertex, newIndex);
		ReplaceKey (node, Key1.GetVal());

		ptr->m_incidentVertex = newIndex;

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);
}


void dgPolyhedra::DeleteDegenerateFaces (const hacd::HaF64* const pool, hacd::HaI32 strideInBytes, hacd::HaF64 area)
{
	if (!GetCount()) {
		return;
	}

#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck ());
#endif
	dgStack <dgPolyhedra::dgTreeNode*> faceArrayPool(GetCount() / 2 + 100);

	hacd::HaI32 count = 0;
	dgPolyhedra::dgTreeNode** const faceArray = &faceArrayPool[0];
	hacd::HaI32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);

		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) {
			faceArray[count] = iter.GetNode();
			count ++;
			dgEdge* ptr = edge;
			do	{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	hacd::HaF64 area2 = area * area;
	area2 *= hacd::HaF64 (4.0f);

	for (hacd::HaI32 i = 0; i < count; i ++) {
		dgPolyhedra::dgTreeNode* const faceNode = faceArray[i];
		dgEdge* const edge = &faceNode->GetInfo();

		dgBigVector normal (FaceNormal (edge, pool, strideInBytes));

		hacd::HaF64 faceArea = normal % normal;
		if (faceArea < area2) {
			DeleteFace (edge);
		}
	}

#ifdef __ENABLE_SANITY_CHECK 
	mark = IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace > 0)) {
			//HACD_ASSERT (edge->m_next->m_next->m_next == edge);
			dgEdge* ptr = edge;
			do	{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dgBigVector normal (FaceNormal (edge, pool, strideInBytes));

			hacd::HaF64 faceArea = normal % normal;
			HACD_ASSERT (faceArea >= area2);
		}
	}
	HACD_ASSERT (SanityCheck ());
#endif
}


static void NormalizeVertex (hacd::HaI32 count, dgBigVector* const dst, const hacd::HaF64* const src, hacd::HaI32 stride)
{
//	dgBigVector min;
//	dgBigVector max;
//	GetMinMax (min, max, src, count, hacd::HaI32 (stride * sizeof (hacd::HaF64)));
//	dgBigVector centre (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
	for (hacd::HaI32 i = 0; i < count; i ++) {
//		dst[i].m_x = centre.m_x + src[i * stride + 0];
//		dst[i].m_y = centre.m_y + src[i * stride + 1];
//		dst[i].m_z = centre.m_z + src[i * stride + 2];
		dst[i].m_x = src[i * stride + 0];
		dst[i].m_y = src[i * stride + 1];
		dst[i].m_z = src[i * stride + 2];
		dst[i].m_w= hacd::HaF64 (0.0f);
	}
}

static dgBigPlane EdgePlane (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, const dgBigVector* const pool)
{
	const dgBigVector& p0 = pool[i0];
	const dgBigVector& p1 = pool[i1];
	const dgBigVector& p2 = pool[i2];

	dgBigPlane plane (p0, p1, p2);
	hacd::HaF64 mag = sqrt (plane % plane);
	if (mag < hacd::HaF64 (1.0e-12f)) {
		mag = hacd::HaF64 (1.0e-12f);
	}
	mag = hacd::HaF64 (1.0f) / mag;

	plane.m_x *= mag;
	plane.m_y *= mag;
	plane.m_z *= mag;
	plane.m_w *= mag;

	return plane;
}


static dgBigPlane UnboundedLoopPlane (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2, const dgBigVector* const pool)
{
	const dgBigVector p0 = pool[i0];
	const dgBigVector p1 = pool[i1];
	const dgBigVector p2 = pool[i2];
	dgBigVector E0 (p1 - p0); 
	dgBigVector E1 (p2 - p0); 

	dgBigVector N ((E0 * E1) * E0); 
	hacd::HaF64 dist = - (N % p0);
	dgBigPlane plane (N, dist);

	hacd::HaF64 mag = sqrt (plane % plane);
	if (mag < hacd::HaF64 (1.0e-12f)) {
		mag = hacd::HaF64 (1.0e-12f);
	}
	mag = hacd::HaF64 (10.0f) / mag;

	plane.m_x *= mag;
	plane.m_y *= mag;
	plane.m_z *= mag;
	plane.m_w *= mag;

	return plane;
}


static void CalculateAllMetrics (const dgPolyhedra* const polyhedra, dgVertexCollapseVertexMetric* const table, const dgBigVector* const pool)
{
	hacd::HaI32 edgeMark = polyhedra->IncLRU();
	dgPolyhedra::Iterator iter (*polyhedra);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);

		HACD_ASSERT (edge);
		if (edge->m_mark != edgeMark) {

			if (edge->m_incidentFace > 0) {
				hacd::HaI32 i0 = edge->m_incidentVertex;
				hacd::HaI32 i1 = edge->m_next->m_incidentVertex;
				hacd::HaI32 i2 = edge->m_prev->m_incidentVertex;

				dgBigPlane constrainPlane (EdgePlane (i0, i1, i2, pool));
				dgVertexCollapseVertexMetric tmp (constrainPlane);

				dgEdge* ptr = edge;
				do {
					ptr->m_mark = edgeMark;
					i0 = ptr->m_incidentVertex;
					table[i0].Accumulate(tmp);

					ptr = ptr->m_next;
				} while (ptr != edge);

			} else {
				HACD_ASSERT (edge->m_twin->m_incidentFace > 0);
				hacd::HaI32 i0 = edge->m_twin->m_incidentVertex;
				hacd::HaI32 i1 = edge->m_twin->m_next->m_incidentVertex;
				hacd::HaI32 i2 = edge->m_twin->m_prev->m_incidentVertex;

				edge->m_mark = edgeMark;
				dgBigPlane constrainPlane (UnboundedLoopPlane (i0, i1, i2, pool));
				dgVertexCollapseVertexMetric tmp (constrainPlane);

				i0 = edge->m_incidentVertex;
				table[i0].Accumulate(tmp);

				i0 = edge->m_twin->m_incidentVertex;
				table[i0].Accumulate(tmp);
			}
		}
	}
}


hacd::HaF64 dgPolyhedra::EdgePenalty (const dgBigVector* const pool, dgEdge* const edge) const
{
	hacd::HaI32 i0 = edge->m_incidentVertex;
	hacd::HaI32 i1 = edge->m_next->m_incidentVertex;

	const dgBigVector& p0 = pool[i0];
	const dgBigVector& p1 = pool[i1];
	dgBigVector dp (p1 - p0);

	hacd::HaF64 dot = dp % dp;
	if (dot < hacd::HaF64(1.0e-6f)) {
		return hacd::HaF64 (-1.0f);
	}

	if ((edge->m_incidentFace > 0) && (edge->m_twin->m_incidentFace > 0)) {
		dgBigVector edgeNormal (FaceNormal (edge, &pool[0].m_x, sizeof (dgBigVector)));
		dgBigVector twinNormal (FaceNormal (edge->m_twin, &pool[0].m_x, sizeof (dgBigVector)));

		hacd::HaF64 mag0 = edgeNormal % edgeNormal;
		hacd::HaF64 mag1 = twinNormal % twinNormal;
		if ((mag0 < hacd::HaF64 (1.0e-24f)) || (mag1 < hacd::HaF64 (1.0e-24f))) {
			return hacd::HaF64 (-1.0f);
		}

		edgeNormal = edgeNormal.Scale (hacd::HaF64 (1.0f) / sqrt(mag0));
		twinNormal = twinNormal.Scale (hacd::HaF64 (1.0f) / sqrt(mag1));

		dot = edgeNormal % twinNormal;
		if (dot < hacd::HaF64 (-0.9f)) {
			return hacd::HaF32 (-1.0f);
		}

		dgEdge* ptr = edge;
		do {
			if ((ptr->m_incidentFace <= 0) || (ptr->m_twin->m_incidentFace <= 0)){
				dgEdge* const adj = edge->m_twin;
				ptr = edge;
				do {
					if ((ptr->m_incidentFace <= 0) || (ptr->m_twin->m_incidentFace <= 0)){
						return hacd::HaF32 (-1.0);
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != adj);
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != edge);
	}

	hacd::HaI32 faceA = edge->m_incidentFace;
	hacd::HaI32 faceB = edge->m_twin->m_incidentFace;

	i0 = edge->m_twin->m_incidentVertex;
	dgBigVector p (pool[i0].m_x, pool[i0].m_y, pool[i0].m_z, hacd::HaF32 (0.0f));

	bool penalty = false;
	dgEdge* ptr = edge;
	do {
		dgEdge* const adj = ptr->m_twin;

		hacd::HaI32 face = adj->m_incidentFace;
		if ((face != faceB) && (face != faceA) && (face >= 0) && (adj->m_next->m_incidentFace == face) && (adj->m_prev->m_incidentFace == face)){

			hacd::HaI32 i0 = adj->m_next->m_incidentVertex;
			const dgBigVector& p0 = pool[i0];

			hacd::HaI32 i1 = adj->m_incidentVertex;
			const dgBigVector& p1 = pool[i1];

			hacd::HaI32 i2 = adj->m_prev->m_incidentVertex;
			const dgBigVector& p2 = pool[i2];

			dgBigVector n0 ((p1 - p0) * (p2 - p0));
			dgBigVector n1 ((p1 - p) * (p2 - p));

//			hacd::HaF64 mag0 = n0 % n0;
//			HACD_ASSERT (mag0 > hacd::HaF64(1.0e-16f));
//			mag0 = sqrt (mag0);

//			hacd::HaF64 mag1 = n1 % n1;
//			mag1 = sqrt (mag1);

			hacd::HaF64 dot = n0 % n1;
			if (dot < hacd::HaF64 (0.0f)) {
//			if (dot <= (mag0 * mag1 * hacd::HaF32 (0.707f)) || (mag0 > (hacd::HaF64(16.0f) * mag1))) {
				penalty = true;
				break;
			}
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);

	hacd::HaF64 aspect = hacd::HaF32 (-1.0f);
	if (!penalty) {
		hacd::HaI32 i0 = edge->m_twin->m_incidentVertex;
		dgBigVector p0 (pool[i0]);

		aspect = hacd::HaF32 (1.0f);
		for (dgEdge* ptr = edge->m_twin->m_next->m_twin->m_next; ptr != edge; ptr = ptr->m_twin->m_next) {
			if (ptr->m_incidentFace > 0) {
				hacd::HaI32 i0 = ptr->m_next->m_incidentVertex;
				const dgBigVector& p1 = pool[i0];

				hacd::HaI32 i1 = ptr->m_prev->m_incidentVertex;
				const dgBigVector& p2 = pool[i1];

				dgBigVector e0 (p1 - p0);
				dgBigVector e1 (p2 - p1);
				dgBigVector e2 (p0 - p2);

				hacd::HaF64 mag0 = e0 % e0;
				hacd::HaF64 mag1 = e1 % e1;
				hacd::HaF64 mag2 = e2 % e2;
				hacd::HaF64 maxMag = GetMax (mag0, mag1, mag2);
				hacd::HaF64 minMag = GetMin (mag0, mag1, mag2);
				hacd::HaF64 ratio = minMag / maxMag;

				if (ratio < aspect) {
					aspect = ratio;
				}
			}
		}
		aspect = sqrt (aspect);
		//aspect = 1.0f;
	}

	return aspect;
}

static void CalculateVertexMetrics (dgVertexCollapseVertexMetric table[], const dgBigVector* const pool, dgEdge* const edge)
{
	hacd::HaI32 i0 = edge->m_incidentVertex;

//	const dgBigVector& p0 = pool[i0];
	table[i0].Clear ();
	dgEdge* ptr = edge;
	do {

		if (ptr->m_incidentFace > 0) {
			hacd::HaI32 i1 = ptr->m_next->m_incidentVertex;
			hacd::HaI32 i2 = ptr->m_prev->m_incidentVertex;
			dgBigPlane constrainPlane (EdgePlane (i0, i1, i2, pool));
			table[i0].Accumulate (constrainPlane);

		} else {
			hacd::HaI32 i1 = ptr->m_twin->m_incidentVertex;
			hacd::HaI32 i2 = ptr->m_twin->m_prev->m_incidentVertex;
			dgBigPlane constrainPlane (UnboundedLoopPlane (i0, i1, i2, pool));
			table[i0].Accumulate (constrainPlane);

			i1 = ptr->m_prev->m_incidentVertex;
			i2 = ptr->m_prev->m_twin->m_prev->m_incidentVertex;
			constrainPlane = UnboundedLoopPlane (i0, i1, i2, pool);
			table[i0].Accumulate (constrainPlane);
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != edge);
}

static void RemoveHalfEdge (dgPolyhedra* const polyhedra, dgEdge* const edge)
{
	dgEdgeCollapseEdgeHandle* const handle = (dgEdgeCollapseEdgeHandle *) IntToPointer (edge->m_userData);
	if (handle) { 
		handle->m_edge = NULL;
	}

	dgPolyhedra::dgTreeNode* const node = polyhedra->GetNodeFromInfo(*edge);
	HACD_ASSERT (node);
	polyhedra->Remove (node);
}


static dgEdge* CollapseEdge(dgPolyhedra* const polyhedra, dgEdge* const edge)
{
	hacd::HaI32 v0 = edge->m_incidentVertex;
	hacd::HaI32 v1 = edge->m_twin->m_incidentVertex;

#ifdef __ENABLE_SANITY_CHECK 
	dgPolyhedra::dgPairKey TwinKey (v1, v0);
	dgPolyhedra::dgTreeNode* const node = polyhedra->Find (TwinKey.GetVal());
	dgEdge* const twin1 = node ? &node->GetInfo() : NULL;
	HACD_ASSERT (twin1);
	HACD_ASSERT (edge->m_twin == twin1);
	HACD_ASSERT (twin1->m_twin == edge);
	HACD_ASSERT (edge->m_incidentFace != 0);
	HACD_ASSERT (twin1->m_incidentFace != 0);
#endif


	dgEdge* retEdge = edge->m_twin->m_prev->m_twin;
	if (retEdge	== edge->m_twin->m_next) {
		return NULL;
	}
	if (retEdge	== edge->m_twin) {
		return NULL;
	}
	if (retEdge	== edge->m_next) {
		retEdge = edge->m_prev->m_twin;
		if (retEdge	== edge->m_twin->m_next) {
			return NULL;
		}
		if (retEdge	== edge->m_twin) {
			return NULL;
		}
	}

	dgEdge* lastEdge = NULL;
	dgEdge* firstEdge = NULL;
	if ((edge->m_incidentFace >= 0)	&& (edge->m_twin->m_incidentFace >= 0)) {	
		lastEdge = edge->m_prev->m_twin;
		firstEdge = edge->m_twin->m_next->m_twin->m_next;
	} else if (edge->m_twin->m_incidentFace >= 0) {
		firstEdge = edge->m_twin->m_next->m_twin->m_next;
		lastEdge = edge;
	} else {
		lastEdge = edge->m_prev->m_twin;
		firstEdge = edge->m_twin->m_next;
	}

	for (dgEdge* ptr = firstEdge; ptr != lastEdge; ptr = ptr->m_twin->m_next) {
		dgEdge* badEdge = polyhedra->FindEdge (edge->m_twin->m_incidentVertex, ptr->m_twin->m_incidentVertex);
		if (badEdge) {
			return NULL;
		}
	} 

	dgEdge* const twin = edge->m_twin;
	if (twin->m_next == twin->m_prev->m_prev) {
		twin->m_prev->m_twin->m_twin = twin->m_next->m_twin;
		twin->m_next->m_twin->m_twin = twin->m_prev->m_twin;

		RemoveHalfEdge (polyhedra, twin->m_prev);
		RemoveHalfEdge (polyhedra, twin->m_next);
	} else {
		twin->m_next->m_prev = twin->m_prev;
		twin->m_prev->m_next = twin->m_next;
	}

	if (edge->m_next == edge->m_prev->m_prev) {
		edge->m_next->m_twin->m_twin = edge->m_prev->m_twin;
		edge->m_prev->m_twin->m_twin = edge->m_next->m_twin;
		RemoveHalfEdge (polyhedra, edge->m_next);
		RemoveHalfEdge (polyhedra, edge->m_prev);
	} else {
		edge->m_next->m_prev = edge->m_prev;
		edge->m_prev->m_next = edge->m_next;
	}

	HACD_ASSERT (twin->m_twin->m_incidentVertex == v0);
	HACD_ASSERT (edge->m_twin->m_incidentVertex == v1);
	RemoveHalfEdge (polyhedra, twin);
	RemoveHalfEdge (polyhedra, edge);

	dgEdge* ptr = retEdge;
	do {
		dgPolyhedra::dgPairKey pairKey (v0, ptr->m_twin->m_incidentVertex);

		dgPolyhedra::dgTreeNode* node = polyhedra->Find (pairKey.GetVal());
		if (node) {
			if (&node->GetInfo() == ptr) {
				dgPolyhedra::dgPairKey key (v1, ptr->m_twin->m_incidentVertex);
				ptr->m_incidentVertex = v1;
				node = polyhedra->ReplaceKey (node, key.GetVal());
				HACD_ASSERT (node);
			} 
		}

		dgPolyhedra::dgPairKey TwinKey (ptr->m_twin->m_incidentVertex, v0);
		node = polyhedra->Find (TwinKey.GetVal());
		if (node) {
			if (&node->GetInfo() == ptr->m_twin) {
				dgPolyhedra::dgPairKey key (ptr->m_twin->m_incidentVertex, v1);
				node = polyhedra->ReplaceKey (node, key.GetVal());
				HACD_ASSERT (node);
			}
		}

		ptr = ptr->m_twin->m_next;
	} while (ptr != retEdge);

	return retEdge;
}



void dgPolyhedra::Optimize (const hacd::HaF64* const array, hacd::HaI32 strideInBytes, hacd::HaF64 tol)
{
	dgList <dgEdgeCollapseEdgeHandle>::dgListNode *handleNodePtr;

	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));

#ifdef __ENABLE_SANITY_CHECK 
	HACD_ASSERT (SanityCheck ());
#endif

	hacd::HaI32 edgeCount = GetEdgeCount() * 4 + 1024 * 16;
	hacd::HaI32 maxVertexIndex = GetLastVertexIndex();

	dgStack<dgBigVector> vertexPool (maxVertexIndex); 
	dgStack<dgVertexCollapseVertexMetric> vertexMetrics (maxVertexIndex + 512); 

	dgList <dgEdgeCollapseEdgeHandle> edgeHandleList;
	dgStack<char> heapPool (2 * edgeCount * hacd::HaI32 (sizeof (hacd::HaF64) + sizeof (dgEdgeCollapseEdgeHandle*) + sizeof (hacd::HaI32))); 
	dgDownHeap<dgList <dgEdgeCollapseEdgeHandle>::dgListNode* , hacd::HaF64> bigHeapArray(&heapPool[0], heapPool.GetSizeInBytes());

	NormalizeVertex (maxVertexIndex, &vertexPool[0], array, stride);
	memset (&vertexMetrics[0], 0, maxVertexIndex * sizeof (dgVertexCollapseVertexMetric));
	CalculateAllMetrics (this, &vertexMetrics[0], &vertexPool[0]);


	hacd::HaF64 tol2 = tol * tol;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);

		edge->m_userData = 0;
		hacd::HaI32 index0 = edge->m_incidentVertex;
		hacd::HaI32 index1 = edge->m_twin->m_incidentVertex;

		dgVertexCollapseVertexMetric &metric = vertexMetrics[index0];
		dgVector p	(&vertexPool[index1].m_x);
		hacd::HaF64 cost = metric.Evalue (p); 
		if (cost < tol2) {
			cost = EdgePenalty (&vertexPool[0], edge);

			if (cost > hacd::HaF64 (0.0f)) {
				dgEdgeCollapseEdgeHandle handle (edge);
				handleNodePtr = edgeHandleList.Addtop (handle);
				bigHeapArray.Push (handleNodePtr, cost);
			}
		}
	}


	while (bigHeapArray.GetCount()) {
		handleNodePtr = bigHeapArray[0];

		dgEdge* edge = handleNodePtr->GetInfo().m_edge;
		bigHeapArray.Pop();
		edgeHandleList.Remove (handleNodePtr);

		if (edge) {
			CalculateVertexMetrics (&vertexMetrics[0], &vertexPool[0], edge);

			hacd::HaI32 index0 = edge->m_incidentVertex;
			hacd::HaI32 index1 = edge->m_twin->m_incidentVertex;
			dgVertexCollapseVertexMetric &metric = vertexMetrics[index0];
			dgBigVector p (vertexPool[index1]);

			if ((metric.Evalue (p) < tol2) && (EdgePenalty (&vertexPool[0], edge)  > hacd::HaF64 (0.0f))) {

#ifdef __ENABLE_SANITY_CHECK 
				HACD_ASSERT (SanityCheck ());
#endif

				edge = CollapseEdge(this, edge);

#ifdef __ENABLE_SANITY_CHECK 
				HACD_ASSERT (SanityCheck ());
#endif
				if (edge) {
					// Update vertex metrics
					CalculateVertexMetrics (&vertexMetrics[0], &vertexPool[0], edge);

					// Update metrics for all surrounding vertex
					dgEdge* ptr = edge;
					do {
						CalculateVertexMetrics (&vertexMetrics[0], &vertexPool[0], ptr->m_twin);
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);

					// calculate edge cost of all incident edges
					hacd::HaI32 mark = IncLRU();
					ptr = edge;
					do {
						HACD_ASSERT (ptr->m_mark != mark);
						ptr->m_mark = mark;

						index0 = ptr->m_incidentVertex;
						index1 = ptr->m_twin->m_incidentVertex;

						dgVertexCollapseVertexMetric &metric = vertexMetrics[index0];
						dgBigVector p (vertexPool[index1]);

						hacd::HaF64 cost = hacd::HaF32 (-1.0f);
						if (metric.Evalue (p) < tol2) {
							cost = EdgePenalty (&vertexPool[0], ptr);
						}

						if (cost  > hacd::HaF64 (0.0f)) {
							dgEdgeCollapseEdgeHandle handle (ptr);
							handleNodePtr = edgeHandleList.Addtop (handle);
							bigHeapArray.Push (handleNodePtr, cost);
						} else {
							dgEdgeCollapseEdgeHandle* const handle = (dgEdgeCollapseEdgeHandle*)IntToPointer (ptr->m_userData);
							if (handle) {
								handle->m_edge = NULL;
							}
							ptr->m_userData = hacd::HaU64 (NULL);

						}

						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);


					// calculate edge cost of all incident edges to a surrounding vertex
					ptr = edge;
					do {
						dgEdge* const incidentEdge = ptr->m_twin;		

						dgEdge* ptr1 = incidentEdge;
						do {
							index0 = ptr1->m_incidentVertex;
							index1 = ptr1->m_twin->m_incidentVertex;

							if (ptr1->m_mark != mark) {
								ptr1->m_mark = mark;
								dgVertexCollapseVertexMetric &metric = vertexMetrics[index0];
								dgBigVector p (vertexPool[index1]);

								hacd::HaF64 cost = hacd::HaF32 (-1.0f);
								if (metric.Evalue (p) < tol2) {
									cost = EdgePenalty (&vertexPool[0], ptr1);
								}

								if (cost  > hacd::HaF64 (0.0f)) {
									HACD_ASSERT (cost > hacd::HaF64(0.0f));
									dgEdgeCollapseEdgeHandle handle (ptr1);
									handleNodePtr = edgeHandleList.Addtop (handle);
									bigHeapArray.Push (handleNodePtr, cost);
								} else {
									dgEdgeCollapseEdgeHandle *handle;
									handle = (dgEdgeCollapseEdgeHandle*)IntToPointer (ptr1->m_userData);
									if (handle) {
										handle->m_edge = NULL;
									}
									ptr1->m_userData = hacd::HaU64 (NULL);

								}
							}

							if (ptr1->m_twin->m_mark != mark) {
								ptr1->m_twin->m_mark = mark;
								dgVertexCollapseVertexMetric &metric = vertexMetrics[index1];
								dgBigVector p (vertexPool[index0]);

								hacd::HaF64 cost = hacd::HaF32 (-1.0f);
								if (metric.Evalue (p) < tol2) {
									cost = EdgePenalty (&vertexPool[0], ptr1->m_twin);
								}

								if (cost  > hacd::HaF64 (0.0f)) {
									HACD_ASSERT (cost > hacd::HaF64(0.0f));
									dgEdgeCollapseEdgeHandle handle (ptr1->m_twin);
									handleNodePtr = edgeHandleList.Addtop (handle);
									bigHeapArray.Push (handleNodePtr, cost);
								} else {
									dgEdgeCollapseEdgeHandle *handle;
									handle = (dgEdgeCollapseEdgeHandle*) IntToPointer (ptr1->m_twin->m_userData);
									if (handle) {
										handle->m_edge = NULL;
									}
									ptr1->m_twin->m_userData = hacd::HaU64 (NULL);

								}
							}

							ptr1 = ptr1->m_twin->m_next;
						} while (ptr1 != incidentEdge);

						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);
				}
			}
		}
	}
}


dgEdge* dgPolyhedra::FindEarTip (dgEdge* const face, const hacd::HaF64* const pool, hacd::HaI32 stride, dgDownHeap<dgEdge*, hacd::HaF64>& heap, const dgBigVector &normal) const
{
	dgEdge* ptr = face;
	dgBigVector p0 (&pool[ptr->m_prev->m_incidentVertex * stride]);
	dgBigVector p1 (&pool[ptr->m_incidentVertex * stride]);
	dgBigVector d0 (p1 - p0);
	hacd::HaF64 f = sqrt (d0 % d0);
	if (f < hacd::HaF64 (1.0e-10f)) {
		f = hacd::HaF64 (1.0e-10f);
	}
	d0 = d0.Scale (hacd::HaF64 (1.0f) / f);

	hacd::HaF64 minAngle = hacd::HaF32 (10.0f);
	do {
		dgBigVector p2 (&pool [ptr->m_next->m_incidentVertex * stride]);
		dgBigVector d1 (p2 - p1);
		hacd::HaF32 f = dgSqrt (d1 % d1);
		if (f < hacd::HaF32 (1.0e-10f)) {
			f = hacd::HaF32 (1.0e-10f);
		}
		d1 = d1.Scale (hacd::HaF32 (1.0f) / f);
		dgBigVector n (d0 * d1);

		hacd::HaF64 angle = normal %  n;
		if (angle >= hacd::HaF64 (0.0f)) {
			heap.Push (ptr, angle);
		}

		if (angle < minAngle) {
			minAngle = angle;
		}

		d0 = d1;
		p1 = p2;
		ptr = ptr->m_next;
	} while (ptr != face);

	if (minAngle > hacd::HaF32 (0.1f)) {
		return heap[0];
	}

	dgEdge* ear = NULL;
	while (heap.GetCount()) {
		ear = heap[0];
		heap.Pop();

		if (FindEdge (ear->m_prev->m_incidentVertex, ear->m_next->m_incidentVertex)) {
			continue;
		}

		dgBigVector p0 (&pool [ear->m_prev->m_incidentVertex * stride]);
		dgBigVector p1 (&pool [ear->m_incidentVertex * stride]);
		dgBigVector p2 (&pool [ear->m_next->m_incidentVertex * stride]);

		dgBigVector p10 (p1 - p0);
		dgBigVector p21 (p2 - p1);
		dgBigVector p02 (p0 - p2);

		for (ptr = ear->m_next->m_next; ptr != ear->m_prev; ptr = ptr->m_next) {
			dgBigVector p (&pool [ptr->m_incidentVertex * stride]);

			hacd::HaF64 side = ((p - p0) * p10) % normal;
			if (side < hacd::HaF64 (0.05f)) {
				side = ((p - p1) * p21) % normal;
				if (side < hacd::HaF64 (0.05f)) {
					side = ((p - p2) * p02) % normal;
					if (side < hacd::HaF32 (0.05f)) {
						break;
					}
				}
			}
		}

		if (ptr == ear->m_prev) {
			break;
		}
	}

	return ear;
}





//dgEdge* TriangulateFace (dgPolyhedra& polyhedra, dgEdge* face, const hacd::HaF32* const pool, hacd::HaI32 stride, dgDownHeap<dgEdge*, hacd::HaF32>& heap, dgVector* const faceNormalOut)
dgEdge* dgPolyhedra::TriangulateFace (dgEdge* face, const hacd::HaF64* const pool, hacd::HaI32 stride, dgDownHeap<dgEdge*, hacd::HaF64>& heap, dgBigVector* const faceNormalOut)
{
	dgEdge* perimeter [1024 * 16]; 
	dgEdge* ptr = face;
	hacd::HaI32 perimeterCount = 0;
	do {
		perimeter[perimeterCount] = ptr;
		perimeterCount ++;
		HACD_ASSERT (perimeterCount < hacd::HaI32 (sizeof (perimeter) / sizeof (perimeter[0])));
		ptr = ptr->m_next;
	} while (ptr != face);
	perimeter[perimeterCount] = face;
	HACD_ASSERT ((perimeterCount + 1) < hacd::HaI32 (sizeof (perimeter) / sizeof (perimeter[0])));

	dgBigVector normal (FaceNormal (face, pool, hacd::HaI32 (stride * sizeof (hacd::HaF64))));

	hacd::HaF64 dot = normal % normal;
	if (dot < hacd::HaF64 (1.0e-12f)) {
		if (faceNormalOut) {
			*faceNormalOut = dgBigVector (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f)); 
		}
		return face;
	}
	normal = normal.Scale (hacd::HaF64 (1.0f) / sqrt (dot));
	if (faceNormalOut) {
		*faceNormalOut = normal;
	}


	while (face->m_next->m_next->m_next != face) {
		dgEdge* const ear = FindEarTip (face, pool, stride, heap, normal); 
		if (!ear) {
			return face;
		}
		if ((face == ear)	|| (face == ear->m_prev)) {
			face = ear->m_prev->m_prev;
		}
		dgEdge* const edge = AddHalfEdge (ear->m_next->m_incidentVertex, ear->m_prev->m_incidentVertex);
		if (!edge) {
			return face;
		}
		dgEdge* const twin = AddHalfEdge (ear->m_prev->m_incidentVertex, ear->m_next->m_incidentVertex);
		if (!twin) {
			return face;
		}
		HACD_ASSERT (twin);


		edge->m_mark = ear->m_mark;
		edge->m_userData = ear->m_next->m_userData;
		edge->m_incidentFace = ear->m_incidentFace;

		twin->m_mark = ear->m_mark;
		twin->m_userData = ear->m_prev->m_userData;
		twin->m_incidentFace = ear->m_incidentFace;

		edge->m_twin = twin;
		twin->m_twin = edge;

		twin->m_prev = ear->m_prev->m_prev;
		twin->m_next = ear->m_next;
		ear->m_prev->m_prev->m_next = twin;
		ear->m_next->m_prev = twin;

		edge->m_next = ear->m_prev;
		edge->m_prev = ear;
		ear->m_prev->m_prev = edge;
		ear->m_next = edge;

		heap.Flush ();
	}
	return NULL;
}


void dgPolyhedra::MarkAdjacentCoplanarFaces (dgPolyhedra& polyhedraOut, dgEdge* const face, const hacd::HaF64* const pool, hacd::HaI32 strideInBytes)
{
	const hacd::HaF64 normalDeviation = hacd::HaF64 (0.9999f);
	const hacd::HaF64 distanceFromPlane = hacd::HaF64 (1.0f / 128.0f);

	hacd::HaI32 faceIndex[1024 * 4];
	dgEdge* stack[1024 * 4];
	dgEdge* deleteEdge[1024 * 4];

	hacd::HaI32 deleteCount = 1;
	deleteEdge[0] = face;
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));

	HACD_ASSERT (face->m_incidentFace > 0);

	dgBigVector normalAverage (FaceNormal (face, pool, strideInBytes));
	hacd::HaF64 dot = normalAverage % normalAverage;
	if (dot > hacd::HaF64 (1.0e-12f)) {
		hacd::HaI32 testPointsCount = 1;
		dot = hacd::HaF64 (1.0f) / sqrt (dot);
		dgBigVector normal (normalAverage.Scale (dot));

		dgBigVector averageTestPoint (&pool[face->m_incidentVertex * stride]);
		dgBigPlane testPlane(normal, - (averageTestPoint % normal));

		polyhedraOut.BeginFace();

		IncLRU();
		hacd::HaI32 faceMark = IncLRU();

		hacd::HaI32 faceIndexCount = 0;
		dgEdge* ptr = face;
		do {
			ptr->m_mark = faceMark;
			faceIndex[faceIndexCount] = ptr->m_incidentVertex;
			faceIndexCount ++;
			HACD_ASSERT (faceIndexCount < hacd::HaI32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
			ptr = ptr ->m_next;
		} while (ptr != face);
		polyhedraOut.AddFace(faceIndexCount, faceIndex);

		hacd::HaI32 index = 1;
		deleteCount = 0;
		stack[0] = face;
		while (index) {
			index --;
			dgEdge* const face = stack[index];
			deleteEdge[deleteCount] = face;
			deleteCount ++;
			HACD_ASSERT (deleteCount < hacd::HaI32 (sizeof (deleteEdge) / sizeof (deleteEdge[0])));
			HACD_ASSERT (face->m_next->m_next->m_next == face);

			dgEdge* edge = face;
			do {
				dgEdge* const ptr = edge->m_twin;
				if (ptr->m_incidentFace > 0) {
					if (ptr->m_mark != faceMark) {
						dgEdge* ptr1 = ptr;
						faceIndexCount = 0;
						do {
							ptr1->m_mark = faceMark;
							faceIndex[faceIndexCount] = ptr1->m_incidentVertex;
							HACD_ASSERT (faceIndexCount < hacd::HaI32 (sizeof (faceIndex) / sizeof (faceIndex[0])));
							faceIndexCount ++;
							ptr1 = ptr1 ->m_next;
						} while (ptr1 != ptr);

						dgBigVector normal1 (FaceNormal (ptr, pool, strideInBytes));
						dot = normal1 % normal1;
						if (dot < hacd::HaF64 (1.0e-12f)) {
							deleteEdge[deleteCount] = ptr;
							deleteCount ++;
							HACD_ASSERT (deleteCount < hacd::HaI32 (sizeof (deleteEdge) / sizeof (deleteEdge[0])));
						} else {
							//normal1 = normal1.Scale (hacd::HaF64 (1.0f) / sqrt (dot));
							dgBigVector testNormal (normal1.Scale (hacd::HaF64 (1.0f) / sqrt (dot)));
							dot = normal % testNormal;
							if (dot >= normalDeviation) {
								dgBigVector testPoint (&pool[ptr->m_prev->m_incidentVertex * stride]);
								hacd::HaF64 dist = fabs (testPlane.Evalue (testPoint));
								if (dist < distanceFromPlane) {
									testPointsCount ++;

									averageTestPoint += testPoint;
									testPoint = averageTestPoint.Scale (hacd::HaF64 (1.0f) / hacd::HaF64(testPointsCount));

									normalAverage += normal1;
									testNormal = normalAverage.Scale (hacd::HaF64 (1.0f) / sqrt (normalAverage % normalAverage));
									testPlane = dgBigPlane (testNormal, - (testPoint % testNormal));

									polyhedraOut.AddFace(faceIndexCount, faceIndex);;
									stack[index] = ptr;
									index ++;
									HACD_ASSERT (index < hacd::HaI32 (sizeof (stack) / sizeof (stack[0])));
								}
							}
						}
					}
				}

				edge = edge->m_next;
			} while (edge != face);
		}
		polyhedraOut.EndFace();
	}

	for (hacd::HaI32 index = 0; index < deleteCount; index ++) {
		DeleteFace (deleteEdge[index]);
	}
}


void dgPolyhedra::RefineTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 stride, dgBigVector* const normal, hacd::HaI32 perimeterCount, dgEdge** const perimeter)
{
	dgList<dgDiagonalEdge> dignonals;

	for (hacd::HaI32 i = 1; i <= perimeterCount; i ++) {
		dgEdge* const last = perimeter[i - 1];
		for (dgEdge* ptr = perimeter[i]->m_prev; ptr != last; ptr = ptr->m_twin->m_prev) {
			dgList<dgDiagonalEdge>::dgListNode* node = dignonals.GetFirst();
			for (; node; node = node->GetNext()) {
				const dgDiagonalEdge& key = node->GetInfo();
				if (((key.m_i0 == ptr->m_incidentVertex) && (key.m_i1 == ptr->m_twin->m_incidentVertex)) ||
					((key.m_i1 == ptr->m_incidentVertex) && (key.m_i0 == ptr->m_twin->m_incidentVertex))) {
						break;
				}
			}
			if (!node) {
				dgDiagonalEdge key (ptr);
				dignonals.Append(key);
			}
		}
	}

	dgEdge* const face = perimeter[0];
	hacd::HaI32 i0 = face->m_incidentVertex * stride;
	hacd::HaI32 i1 = face->m_next->m_incidentVertex * stride;
	dgBigVector p0 (vertex[i0], vertex[i0 + 1], vertex[i0 + 2], hacd::HaF32 (0.0f));
	dgBigVector p1 (vertex[i1], vertex[i1 + 1], vertex[i1 + 2], hacd::HaF32 (0.0f));

	dgBigVector p1p0 (p1 - p0);
	hacd::HaF64 mag2 = p1p0 % p1p0;
	for (dgEdge* ptr = face->m_next->m_next; mag2 < hacd::HaF32 (1.0e-12f); ptr = ptr->m_next) {
		hacd::HaI32 i1 = ptr->m_incidentVertex * stride;
		dgBigVector p1 (vertex[i1], vertex[i1 + 1], vertex[i1 + 2], hacd::HaF32 (0.0f));
		p1p0 = p1 - p0;
		mag2 = p1p0 % p1p0;
	}

	dgMatrix matrix (dgGetIdentityMatrix());
	matrix.m_posit = p0;
	matrix.m_front = dgVector (p1p0.Scale (hacd::HaF64 (1.0f) / sqrt (mag2)));
	matrix.m_right = dgVector (normal->Scale (hacd::HaF64 (1.0f) / sqrt (*normal % *normal)));
	matrix.m_up = matrix.m_right * matrix.m_front;
	matrix = matrix.Inverse();
	matrix.m_posit.m_w = hacd::HaF32 (1.0f);

	hacd::HaI32 maxCount = dignonals.GetCount() * dignonals.GetCount();
	while (dignonals.GetCount() && maxCount) {
		maxCount --;
		dgList<dgDiagonalEdge>::dgListNode* const node = dignonals.GetFirst();
		dgDiagonalEdge key (node->GetInfo());
		dignonals.Remove(node);
		dgEdge* const edge = FindEdge(key.m_i0, key.m_i1);
		if (edge) {
			hacd::HaI32 i0 = edge->m_incidentVertex * stride;
			hacd::HaI32 i1 = edge->m_next->m_incidentVertex * stride;
			hacd::HaI32 i2 = edge->m_next->m_next->m_incidentVertex * stride;
			hacd::HaI32 i3 = edge->m_twin->m_prev->m_incidentVertex * stride;

			dgBigVector p0 (vertex[i0], vertex[i0 + 1], vertex[i0 + 2], hacd::HaF64 (0.0f));
			dgBigVector p1 (vertex[i1], vertex[i1 + 1], vertex[i1 + 2], hacd::HaF64 (0.0f));
			dgBigVector p2 (vertex[i2], vertex[i2 + 1], vertex[i2 + 2], hacd::HaF64 (0.0f));
			dgBigVector p3 (vertex[i3], vertex[i3 + 1], vertex[i3 + 2], hacd::HaF64 (0.0f));

			p0 = matrix.TransformVector(p0);
			p1 = matrix.TransformVector(p1);
			p2 = matrix.TransformVector(p2);
			p3 = matrix.TransformVector(p3);

			hacd::HaF64 circleTest[3][3];
			circleTest[0][0] = p0[0] - p3[0];
			circleTest[0][1] = p0[1] - p3[1];
			circleTest[0][2] = circleTest[0][0] * circleTest[0][0] + circleTest[0][1] * circleTest[0][1];

			circleTest[1][0] = p1[0] - p3[0];
			circleTest[1][1] = p1[1] - p3[1];
			circleTest[1][2] = circleTest[1][0] * circleTest[1][0] + circleTest[1][1] * circleTest[1][1];

			circleTest[2][0] = p2[0] - p3[0];
			circleTest[2][1] = p2[1] - p3[1];
			circleTest[2][2] = circleTest[2][0] * circleTest[2][0] + circleTest[2][1] * circleTest[2][1];

			hacd::HaF64 error;
			hacd::HaF64 det = Determinant3x3 (circleTest, &error);
			if (det < hacd::HaF32 (0.0f)) {
				dgEdge* frontFace0 = edge->m_prev;
				dgEdge* backFace0 = edge->m_twin->m_prev;

				FlipEdge(edge);

				if (perimeterCount > 4) {
					dgEdge* backFace1 = backFace0->m_next;
					dgEdge* frontFace1 = frontFace0->m_next;
					for (hacd::HaI32 i = 0; i < perimeterCount; i ++) {
						if (frontFace0 == perimeter[i]) {
							frontFace0 = NULL;
						}
						if (frontFace1 == perimeter[i]) {
							frontFace1 = NULL;
						}

						if (backFace0 == perimeter[i]) {
							backFace0 = NULL;
						}
						if (backFace1 == perimeter[i]) {
							backFace1 = NULL;
						}
					}

					if (backFace0 && (backFace0->m_incidentFace > 0) && (backFace0->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key (backFace0);
						dignonals.Append(key);
					}
					if (backFace1 && (backFace1->m_incidentFace > 0) && (backFace1->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key (backFace1);
						dignonals.Append(key);
					}

					if (frontFace0 && (frontFace0->m_incidentFace > 0) && (frontFace0->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key (frontFace0);
						dignonals.Append(key);
					}

					if (frontFace1 && (frontFace1->m_incidentFace > 0) && (frontFace1->m_twin->m_incidentFace > 0)) {
						dgDiagonalEdge key (frontFace1);
						dignonals.Append(key);
					}
				}
			}
		}
	}
}


void dgPolyhedra::RefineTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 stride)
{
	dgEdge* edgePerimeters[1024 * 16];
	hacd::HaI32 perimeterCount = 0;

	dgPolyhedra::Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) {
			dgEdge* ptr = edge;
			do {
				edgePerimeters[perimeterCount] = ptr->m_twin;
				perimeterCount ++;
				HACD_ASSERT (perimeterCount < hacd::HaI32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
				ptr = ptr->m_prev;
			} while (ptr != edge);
			break;
		}
	}
	HACD_ASSERT (perimeterCount);
	HACD_ASSERT (perimeterCount < hacd::HaI32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
	edgePerimeters[perimeterCount] = edgePerimeters[0];

	dgBigVector normal (FaceNormal(edgePerimeters[0], vertex, hacd::HaI32 (stride * sizeof (hacd::HaF64))));
	if ((normal % normal) > hacd::HaF32 (1.0e-12f)) {
		RefineTriangulation (vertex, stride, &normal, perimeterCount, edgePerimeters);
	}
}


void dgPolyhedra::OptimizeTriangulation (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes)
{
	hacd::HaI32 polygon[1024 * 8];
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));

	dgPolyhedra leftOver;
	dgPolyhedra buildConvex;

	buildConvex.BeginFace();
	dgPolyhedra::Iterator iter (*this);

	for (iter.Begin(); iter; ) {
		dgEdge* const edge = &(*iter);
		iter++;

		if (edge->m_incidentFace > 0) {
			dgPolyhedra flatFace;
			MarkAdjacentCoplanarFaces (flatFace, edge, vertex, strideInBytes);
			//HACD_ASSERT (flatFace.GetCount());

			if (flatFace.GetCount()) {
				//flatFace.Triangulate (vertex, strideInBytes, &leftOver);
				//HACD_ASSERT (!leftOver.GetCount());
				flatFace.RefineTriangulation (vertex, stride);

				hacd::HaI32 mark = flatFace.IncLRU();
				dgPolyhedra::Iterator iter (flatFace);
				for (iter.Begin(); iter; iter ++) {
					dgEdge* const edge = &(*iter);
					if (edge->m_mark != mark) {
						if (edge->m_incidentFace > 0) {
							dgEdge* ptr = edge;
							hacd::HaI32 vertexCount = 0;
							do {
								polygon[vertexCount] = ptr->m_incidentVertex;				
								vertexCount ++;
								HACD_ASSERT (vertexCount < hacd::HaI32 (sizeof (polygon) / sizeof (polygon[0])));
								ptr->m_mark = mark;
								ptr = ptr->m_next;
							} while (ptr != edge);
							if (vertexCount >= 3) {
								buildConvex.AddFace (vertexCount, polygon);
							}
						}
					}
				}
			}
			iter.Begin();
		}
	}
	buildConvex.EndFace();
	HACD_ASSERT (GetCount() == 0);
	SwapInfo(buildConvex);
}


void dgPolyhedra::Triangulate (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, dgPolyhedra* const leftOver)
{
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));

	hacd::HaI32 count = GetCount() / 2;
	dgStack<char> memPool (hacd::HaI32 ((count + 512) * (2 * sizeof (hacd::HaF64)))); 
	dgDownHeap<dgEdge*, hacd::HaF64> heap(&memPool[0], memPool.GetSizeInBytes());

	hacd::HaI32 mark = IncLRU();
	Iterator iter (*this);
	for (iter.Begin(); iter; ) {
		dgEdge* const thisEdge = &(*iter);
		iter ++;

		if (thisEdge->m_mark == mark) {
			continue;
		}
		if (thisEdge->m_incidentFace < 0) {
			continue;
		}

		count = 0;
		dgEdge* ptr = thisEdge;
		do {
			count ++;
			ptr->m_mark = mark;
			ptr = ptr->m_next;
		} while (ptr != thisEdge);

		if (count > 3) {
			dgEdge* const edge = TriangulateFace (thisEdge, vertex, stride, heap, NULL);
			heap.Flush ();

			if (edge) {
				HACD_ASSERT (edge->m_incidentFace > 0);

				if (leftOver) {
					hacd::HaI32* const index = (hacd::HaI32 *) &heap[0];
					hacd::HaI64* const data = (hacd::HaI64 *)&index[count];
					hacd::HaI32 i = 0;
					dgEdge* ptr = edge;
					do {
						index[i] = ptr->m_incidentVertex;
						data[i] = hacd::HaI64 (ptr->m_userData);
						i ++;
						ptr = ptr->m_next;
					} while (ptr != edge);
					leftOver->AddFace(i, index, data);

				} 
				else 
				{
//					dgTrace (("Deleting face:"));					
//					ptr = edge;
//					do {
//						dgTrace (("%d ", ptr->m_incidentVertex));
//					} while (ptr != edge);
////					dgTrace (("\n"));					
				}

				DeleteFace (edge);
				iter.Begin();
			}
		}
	}

	OptimizeTriangulation (vertex, strideInBytes);

	mark = IncLRU();
	m_faceSecuence = 1;
	for (iter.Begin(); iter; iter ++) {
		dgEdge* edge = &(*iter);
		if (edge->m_mark == mark) {
			continue;
		}
		if (edge->m_incidentFace < 0) {
			continue;
		}
		HACD_ASSERT (edge == edge->m_next->m_next->m_next);

		for (hacd::HaI32 i = 0; i < 3; i ++) { 
			edge->m_incidentFace = m_faceSecuence; 
			edge->m_mark = mark;
			edge = edge->m_next;
		}
		m_faceSecuence ++;
	}
}


static void RemoveColinearVertices (dgPolyhedra& flatFace, const hacd::HaF64* const vertex, hacd::HaI32 stride)
{
	dgEdge* edgePerimeters[1024];

	hacd::HaI32 perimeterCount = 0;
	hacd::HaI32 mark = flatFace.IncLRU();
	dgPolyhedra::Iterator iter (flatFace);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if ((edge->m_incidentFace < 0) && (edge->m_mark != mark)) {
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
			edgePerimeters[perimeterCount] = edge;
			perimeterCount ++;
			HACD_ASSERT (perimeterCount < hacd::HaI32 (sizeof (edgePerimeters) / sizeof (edgePerimeters[0])));
		}
	}

	for (hacd::HaI32 i = 0; i < perimeterCount; i ++) {
		dgEdge* edge = edgePerimeters[i];
		dgEdge* ptr = edge;
		dgVector p0 (&vertex[ptr->m_incidentVertex * stride]);
		dgVector p1 (&vertex[ptr->m_next->m_incidentVertex * stride]);
		dgVector e0 (p1 - p0) ;
		e0 = e0.Scale (hacd::HaF32 (1.0f) / (dgSqrt (e0 % e0) + hacd::HaF32 (1.0e-12f)));
		hacd::HaI32 ignoreTest = 1;
		do {
			ignoreTest = 0;
			dgVector p2 (&vertex[ptr->m_next->m_next->m_incidentVertex * stride]);
			dgVector e1 (p2 - p1);
			e1 = e1.Scale (hacd::HaF32 (1.0f) / (dgSqrt (e1 % e1) + hacd::HaF32 (1.0e-12f)));
			hacd::HaF32 dot = e1 % e0;
			if (dot > hacd::HaF32 (hacd::HaF32 (0.9999f))) {

				for (dgEdge* interiorEdge = ptr->m_next->m_twin->m_next; interiorEdge != ptr->m_twin; interiorEdge = ptr->m_next->m_twin->m_next) {
					flatFace.DeleteEdge (interiorEdge);
				} 

				if (ptr->m_twin->m_next->m_next->m_next == ptr->m_twin) {
					HACD_ASSERT (ptr->m_twin->m_next->m_incidentFace > 0);
					flatFace.DeleteEdge (ptr->m_twin->m_next);
				}

				HACD_ASSERT (ptr->m_next->m_twin->m_next->m_twin == ptr);
				edge = ptr->m_next;

				if (!flatFace.FindEdge (ptr->m_incidentVertex, edge->m_twin->m_incidentVertex) && 
					!flatFace.FindEdge (edge->m_twin->m_incidentVertex, ptr->m_incidentVertex)) {
						ptr->m_twin->m_prev = edge->m_twin->m_prev;
						edge->m_twin->m_prev->m_next = ptr->m_twin;

						edge->m_next->m_prev = ptr;
						ptr->m_next = edge->m_next;

						edge->m_next = edge->m_twin;
						edge->m_prev = edge->m_twin;
						edge->m_twin->m_next = edge;
						edge->m_twin->m_prev = edge;
						flatFace.DeleteEdge (edge);								
						flatFace.ChangeEdgeIncidentVertex (ptr->m_twin, ptr->m_next->m_incidentVertex);

						e1 = e0;
						p1 = p2;
						edge = ptr;
						ignoreTest = 1;
						continue;
				}
			}

			e0 = e1;
			p1 = p2;
			ptr = ptr->m_next;
		} while ((ptr != edge) || ignoreTest);
	}
}


static hacd::HaI32 GetInteriorDiagonals (dgPolyhedra& polyhedra, dgEdge** const diagonals, hacd::HaI32 maxCount)
{
	hacd::HaI32 count = 0;
	hacd::HaI32 mark = polyhedra.IncLRU();
	dgPolyhedra::Iterator iter (polyhedra);
	for (iter.Begin(); iter; iter++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark != mark) { 
			if (edge->m_incidentFace > 0) {
				if (edge->m_twin->m_incidentFace > 0) {
					edge->m_twin->m_mark = mark;
					if (count < maxCount){
						diagonals[count] = edge;
						count ++;
					}
					HACD_ASSERT (count <= maxCount);
				}
			}
		}
		edge->m_mark = mark;
	}

	return count;
}

static bool IsEssensialPointDiagonal (dgEdge* const diagonal, const dgBigVector& normal, const hacd::HaF64* const pool, hacd::HaI32 stride)
{
	hacd::HaF64 dot;
	dgBigVector p0 (&pool[diagonal->m_incidentVertex * stride]);
	dgBigVector p1 (&pool[diagonal->m_twin->m_next->m_twin->m_incidentVertex * stride]);
	dgBigVector p2 (&pool[diagonal->m_prev->m_incidentVertex * stride]);

	dgBigVector e1 (p1 - p0);
	dot = e1 % e1;
	if (dot < hacd::HaF64 (1.0e-12f)) {
		return false;
	}
	e1 = e1.Scale (hacd::HaF64 (1.0f) / sqrt(dot));

	dgBigVector e2 (p2 - p0);
	dot = e2 % e2;
	if (dot < hacd::HaF64 (1.0e-12f)) {
		return false;
	}
	e2 = e2.Scale (hacd::HaF64 (1.0f) / sqrt(dot));

	dgBigVector n1 (e1 * e2); 

	dot = normal % n1;
	//if (dot > hacd::HaF64 (hacd::HaF32 (0.1f)f)) {
	//if (dot >= hacd::HaF64 (-1.0e-6f)) {
	if (dot >= hacd::HaF64 (0.0f)) {
		return false;
	}
	return true;
}


static bool IsEssensialDiagonal (dgEdge* const diagonal, const dgBigVector& normal, const hacd::HaF64* const pool,  hacd::HaI32 stride)
{
	return IsEssensialPointDiagonal (diagonal, normal, pool, stride) || IsEssensialPointDiagonal (diagonal->m_twin, normal, pool, stride); 
}


void dgPolyhedra::ConvexPartition (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, dgPolyhedra* const leftOversOut)
{
	if (GetCount()) {
		Triangulate (vertex, strideInBytes, leftOversOut);
		DeleteDegenerateFaces (vertex, strideInBytes, hacd::HaF32 (1.0e-5f));
		Optimize (vertex, strideInBytes, hacd::HaF32 (1.0e-4f));
		DeleteDegenerateFaces (vertex, strideInBytes, hacd::HaF32 (1.0e-5f));

		if (GetCount()) {
			hacd::HaI32 removeCount = 0;
			hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));

			hacd::HaI32 polygon[1024 * 8];
			dgEdge* diagonalsPool[1024 * 8];
			dgPolyhedra buildConvex;

			buildConvex.BeginFace();
			dgPolyhedra::Iterator iter (*this);
			for (iter.Begin(); iter; ) {
				dgEdge* edge = &(*iter);
				iter++;
				if (edge->m_incidentFace > 0) {

					dgPolyhedra flatFace;
					MarkAdjacentCoplanarFaces (flatFace, edge, vertex, strideInBytes);

					if (flatFace.GetCount()) {
						flatFace.RefineTriangulation (vertex, stride);
						RemoveColinearVertices (flatFace, vertex, stride);

						hacd::HaI32 diagonalCount = GetInteriorDiagonals (flatFace, diagonalsPool, sizeof (diagonalsPool) / sizeof (diagonalsPool[0]));
						if (diagonalCount) {
							edge = &flatFace.GetRoot()->GetInfo();
							if (edge->m_incidentFace < 0) {
								edge = edge->m_twin;
							}
							HACD_ASSERT (edge->m_incidentFace > 0);

							dgBigVector normal (FaceNormal (edge, vertex, strideInBytes));
							normal = normal.Scale (hacd::HaF64 (1.0f) / sqrt (normal % normal));

							edge = NULL;
							dgPolyhedra::Iterator iter (flatFace);
							for (iter.Begin(); iter; iter ++) {
								edge = &(*iter);
								if (edge->m_incidentFace < 0) {
									break;
								}
							}
							HACD_ASSERT (edge);

							hacd::HaI32 isConvex = 1;
							dgEdge* ptr = edge;
							hacd::HaI32 mark = flatFace.IncLRU();

							dgBigVector normal2 (normal);
							dgBigVector p0 (&vertex[ptr->m_prev->m_incidentVertex * stride]);
							dgBigVector p1 (&vertex[ptr->m_incidentVertex * stride]);
							dgBigVector e0 (p1 - p0);
							e0 = e0.Scale (hacd::HaF32 (1.0f) / (dgSqrt (e0 % e0) + hacd::HaF32 (1.0e-14f)));
							do {
								dgBigVector p2 (&vertex[ptr->m_next->m_incidentVertex * stride]);
								dgBigVector e1 (p2 - p1);
								e1 = e1.Scale (hacd::HaF32 (1.0f) / (sqrt (e1 % e1) + hacd::HaF32 (1.0e-14f)));
								hacd::HaF64 dot = (e0 * e1) % normal2;
								//if (dot > hacd::HaF32 (0.0f)) {
								if (dot > hacd::HaF32 (5.0e-3f)) {
									isConvex = 0;
									break;
								}
								ptr->m_mark = mark;
								e0 = e1;
								p1 = p2;
								ptr = ptr->m_next;
							} while (ptr != edge);

							if (isConvex) {
								dgPolyhedra::Iterator iter (flatFace);
								for (iter.Begin(); iter; iter ++) {
									ptr = &(*iter);
									if (ptr->m_incidentFace < 0) {
										if (ptr->m_mark < mark) {
											isConvex = 0;
											break;
										}
									}
								}
							}

							if (isConvex) {
								if (diagonalCount > 2) {
									hacd::HaI32 count = 0;
									ptr = edge;
									do {
										polygon[count] = ptr->m_incidentVertex;				
										count ++;
										HACD_ASSERT (count < hacd::HaI32 (sizeof (polygon) / sizeof (polygon[0])));
										ptr = ptr->m_next;
									} while (ptr != edge);

									for (hacd::HaI32 i = 0; i < count - 1; i ++) {
										for (hacd::HaI32 j = i + 1; j < count; j ++) {
											if (polygon[i] == polygon[j]) {
												i = count;
												isConvex = 0;
												break ;
											}
										}
									}
								}
							}

							if (isConvex) {
								for (hacd::HaI32 j = 0; j < diagonalCount; j ++) {
									dgEdge* const diagonal = diagonalsPool[j];
									removeCount ++;
									flatFace.DeleteEdge (diagonal);
								}
							} else {
								for (hacd::HaI32 j = 0; j < diagonalCount; j ++) {
									dgEdge* const diagonal = diagonalsPool[j];
									if (!IsEssensialDiagonal(diagonal, normal, vertex, stride)) {
										removeCount ++;
										flatFace.DeleteEdge (diagonal);
									}
								}
							}
						}

						hacd::HaI32 mark = flatFace.IncLRU();
						dgPolyhedra::Iterator iter (flatFace);
						for (iter.Begin(); iter; iter ++) {
							dgEdge* const edge = &(*iter);
							if (edge->m_mark != mark) {
								if (edge->m_incidentFace > 0) {
									dgEdge* ptr = edge;
									hacd::HaI32 diagonalCount = 0;
									do {
										polygon[diagonalCount] = ptr->m_incidentVertex;				
										diagonalCount ++;
										HACD_ASSERT (diagonalCount < hacd::HaI32 (sizeof (polygon) / sizeof (polygon[0])));
										ptr->m_mark = mark;
										ptr = ptr->m_next;
									} while (ptr != edge);
									if (diagonalCount >= 3) {
										buildConvex.AddFace (diagonalCount, polygon);
									}
								}
							}
						}
					}
					iter.Begin();
				}
			}

			buildConvex.EndFace();
			HACD_ASSERT (GetCount() == 0);
			SwapInfo(buildConvex);
		}
	}
}


dgSphere dgPolyhedra::CalculateSphere (const hacd::HaF64* const vertex, hacd::HaI32 strideInBytes, const dgMatrix* const basis) const
{
/*
		// this si a degenerate mesh of a flat plane calculate OOBB by discrete rotations
		dgStack<hacd::HaI32> pool (GetCount() * 3 + 6); 
		hacd::HaI32* const indexList = &pool[0]; 

		dgMatrix axis (dgGetIdentityMatrix());
		dgBigVector p0 (hacd::HaF32 ( 1.0e10f), hacd::HaF32 ( 1.0e10f), hacd::HaF32 ( 1.0e10f), hacd::HaF32 (0.0f));
		dgBigVector p1 (hacd::HaF32 (-1.0e10f), hacd::HaF32 (-1.0e10f), hacd::HaF32 (-1.0e10f), hacd::HaF32 (0.0f));

		hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));
		hacd::HaI32 indexCount = 0;
		hacd::HaI32 mark = IncLRU();
		dgPolyhedra::Iterator iter(*this);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			if (edge->m_mark != mark) {
				dgEdge *ptr = edge;
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				hacd::HaI32 index = edge->m_incidentVertex;
				indexList[indexCount + 6] = edge->m_incidentVertex;
				dgBigVector point (vertex[index * stride + 0], vertex[index * stride + 1], vertex[index * stride + 2], hacd::HaF32 (0.0f));
				for (hacd::HaI32 i = 0; i < 3; i ++) {
					if (point[i] < p0[i]) {
						p0[i] = point[i];
						indexList[i * 2 + 0] = index;
					}
					if (point[i] > p1[i]) {
						p1[i] = point[i];
						indexList[i * 2 + 1] = index;
					}
				}
				indexCount ++;
			}
		}
		indexCount += 6;


		dgBigVector size (p1 - p0);
		hacd::HaF64 volume = size.m_x * size.m_y * size.m_z;


		for (hacd::HaF32 pitch = hacd::HaF32 (0.0f); pitch < hacd::HaF32 (90.0f); pitch += hacd::HaF32 (10.0f)) {
			dgMatrix pitchMatrix (dgPitchMatrix(pitch * hacd::HaF32 (3.1416f) / hacd::HaF32 (180.0f)));
			for (hacd::HaF32 yaw = hacd::HaF32 (0.0f); yaw  < hacd::HaF32 (90.0f); yaw  += hacd::HaF32 (10.0f)) {
				dgMatrix yawMatrix (dgYawMatrix(yaw * hacd::HaF32 (3.1416f) / hacd::HaF32 (180.0f)));
				for (hacd::HaF32 roll = hacd::HaF32 (0.0f); roll < hacd::HaF32 (90.0f); roll += hacd::HaF32 (10.0f)) {
					hacd::HaI32 tmpIndex[6];
					dgMatrix rollMatrix (dgRollMatrix(roll * hacd::HaF32 (3.1416f) / hacd::HaF32 (180.0f)));
					dgMatrix tmp (pitchMatrix * yawMatrix * rollMatrix);
					dgBigVector q0 (hacd::HaF32 ( 1.0e10f), hacd::HaF32 ( 1.0e10f), hacd::HaF32 ( 1.0e10f), hacd::HaF32 (0.0f));
					dgBigVector q1 (hacd::HaF32 (-1.0e10f), hacd::HaF32 (-1.0e10f), hacd::HaF32 (-1.0e10f), hacd::HaF32 (0.0f));

					hacd::HaF32 volume1 = hacd::HaF32 (1.0e10f);
					for (hacd::HaI32 i = 0; i < indexCount; i ++) {
						hacd::HaI32 index = indexList[i];
						dgBigVector point (vertex[index * stride + 0], vertex[index * stride + 1], vertex[index * stride + 2], hacd::HaF32 (0.0f));
						point = tmp.UnrotateVector(point);

						for (hacd::HaI32 j = 0; j < 3; j ++) {
							if (point[j] < q0[j]) {
								q0[j] = point[j];
								tmpIndex[j * 2 + 0] = index;
							}
							if (point[j] > q1[j]) {
								q1[j] = point[j];
								tmpIndex[j * 2 + 1] = index;
							}
						}


						dgVector size1 (q1 - q0);
						volume1 = size1.m_x * size1.m_y * size1.m_z;
						if (volume1 >= volume) {
							break;
						}
					}

					if (volume1 < volume) {
						p0 = q0;
						p1 = q1;
						axis = tmp;
						volume = volume1;
						memcpy (indexList, tmpIndex, sizeof (tmpIndex));
					}
				}
			}
		}

		HACD_ASSERT (0);
		dgSphere sphere (axis);
		dgVector q0 (p0);
		dgVector q1 (p1);
		sphere.m_posit = axis.RotateVector((q1 + q0).Scale (hacd::HaF32 (0.5f)));
		sphere.m_size = (q1 - q0).Scale (hacd::HaF32 (0.5f));
		return sphere;
*/

	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));	

	hacd::HaI32 vertexCount = 0;
	hacd::HaI32 mark = IncLRU();
	dgPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark != mark) {
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			vertexCount ++;
		}
	}
	HACD_ASSERT (vertexCount);

	mark = IncLRU();
	hacd::HaI32 vertexCountIndex = 0;
	dgStack<dgBigVector> pool (vertexCount);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark != mark) {
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			hacd::HaI32 incidentVertex = edge->m_incidentVertex * stride;
			pool[vertexCountIndex] = dgBigVector (vertex[incidentVertex + 0], vertex[incidentVertex + 1], vertex[incidentVertex + 2], hacd::HaF32 (0.0f));
			vertexCountIndex ++;
		}
	}
	HACD_ASSERT (vertexCountIndex <= vertexCount);

	dgMatrix axis (dgGetIdentityMatrix());
	dgSphere sphere (axis);
	dgConvexHull3d convexHull (&pool[0].m_x, sizeof (dgBigVector), vertexCountIndex, 0.0f);
	if (convexHull.GetCount()) {
		dgStack<hacd::HaI32> triangleList (convexHull.GetCount() * 3); 				
		hacd::HaI32 trianglesCount = 0;
		for (dgConvexHull3d::dgListNode* node = convexHull.GetFirst(); node; node = node->GetNext()) {
			dgConvexHull3DFace* const face = &node->GetInfo();
			triangleList[trianglesCount * 3 + 0] = face->m_index[0];
			triangleList[trianglesCount * 3 + 1] = face->m_index[1];
			triangleList[trianglesCount * 3 + 2] = face->m_index[2];
			trianglesCount ++;
			HACD_ASSERT ((trianglesCount * 3) <= triangleList.GetElementsCount());
		}

		dgVector* const dst = (dgVector*) &pool[0].m_x;
		for (hacd::HaI32 i = 0; i < convexHull.GetVertexCount(); i ++) {
			dst[i] = convexHull.GetVertex(i);
		}
		sphere.SetDimensions (&dst[0].m_x, sizeof (dgVector), &triangleList[0], trianglesCount * 3, NULL);

	} else if (vertexCountIndex >= 3) {
		dgStack<hacd::HaI32> triangleList (GetCount() * 3 * 2); 
		hacd::HaI32 mark = IncLRU();
		hacd::HaI32 trianglesCount = 0;
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			if (edge->m_mark != mark) {
				dgEdge* ptr = edge;
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);

				ptr = edge->m_next->m_next;
				do {
					triangleList[trianglesCount * 3 + 0] = edge->m_incidentVertex;
					triangleList[trianglesCount * 3 + 1] = ptr->m_prev->m_incidentVertex;
					triangleList[trianglesCount * 3 + 2] = ptr->m_incidentVertex;
					trianglesCount ++;
					HACD_ASSERT ((trianglesCount * 3) <= triangleList.GetElementsCount());
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);

				dgVector* const dst = (dgVector*) &pool[0].m_x;
				for (hacd::HaI32 i = 0; i < vertexCountIndex; i ++) {
					dst[i] = pool[i];
				}
				sphere.SetDimensions (&dst[0].m_x, sizeof (dgVector), &triangleList[0], trianglesCount * 3, NULL);
			}
		}
	}
	return sphere;

}
